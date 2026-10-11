"""RViz scene replay, adjustable target ROI and clickable candidate selection."""
import json
from pathlib import Path
import textwrap

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MarkerArray
from rak_interfaces.msg import GraspCandidate, GraspCandidates
from rak_interfaces.srv import SelectGrasp
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster

from .core import gripper_lines, load_snapshot, scene_cloud, validate_result, roi_mask


def candidate_message(item, meta):
    msg = GraspCandidate()
    msg.header.frame_id = meta["depth_frame"]
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(meta["depth_stamp_ns"], 10**9)
    msg.id, msg.width_m, msg.depth_m, msg.score = item["id"], item["width_m"], item["depth_m"], item["score"]
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = item["position"]
    q = msg.pose.orientation
    q.x, q.y, q.z, q.w = item["quaternion_xyzw"]
    return msg


class Viewer(Node):
    def __init__(self):
        super().__init__("grasp_replay")
        snapshot = Path(self.declare_parameter("snapshot", "").value)
        result_path = self.declare_parameter("result", "").value
        self.roi_path = Path(self.declare_parameter("roi_output", "roi.json").value)
        self.meta, rgb, depth = load_snapshot(snapshot)
        # Replay factory internal calibration; this is not a robot mounting TF.
        self.camera_tf = StaticTransformBroadcaster(self)
        color_frame = self.meta.get("color_frame", "camera_color_optical_frame")
        if color_frame != self.meta["depth_frame"]:
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = self.meta["depth_frame"], color_frame
            tf.header.stamp = self.get_clock().now().to_msg()
            matrix = np.linalg.inv(np.asarray(self.meta["depth_to_color"]))
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = matrix[:3, 3].tolist()
            q = tf.transform.rotation
            q.x, q.y, q.z, q.w = Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()
            self.camera_tf.sendTransform(tf)
        self.xyz, colors = scene_cloud(self.meta, rgb, depth)
        self.scene_top_y = float(np.quantile(self.xyz[:, 1], 0.02))
        self.result = {"candidates": []}
        if result_path:
            self.result = validate_result(json.loads(Path(result_path).read_text()), self.meta)
        self.items = {item["id"]: item for item in self.result["candidates"]}
        roi = self.result.get("roi")
        if roi is None:
            center = np.median(self.xyz, axis=0)
            roi = {"min": (center-0.15).tolist(), "max": (center+0.15).tolist()}
        roi_mask(self.xyz[:1], roi)
        self.center = (np.array(roi["min"])+np.array(roi["max"]))/2
        self.size = np.array(roi["max"])-np.array(roi["min"])
        self.selected = None
        self.sim_status = "simulation planner not selected"
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(PointCloud2, "/grasp/scene", qos)
        self.candidate_pub = self.create_publisher(GraspCandidates, "/grasp/candidates", qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/grasp/markers", qos)
        self.selected_pub = self.create_publisher(GraspCandidate, "/grasp/selected", 10)
        self.status_pub = self.create_publisher(String, "/grasp/status", qos)
        self.create_subscription(String, "/grasp/sim_status", self.planning_status, 10)
        self.create_service(SelectGrasp, "/grasp/select", self.select_service)
        self.server = InteractiveMarkerServer(self, "grasp_interactive")
        self.menu = MenuHandler()
        self.menu.insert("Save target ROI", callback=self.save_roi)
        self.menu.insert("Increase ROI size 25%", callback=lambda f: self.resize_roi(1.25))
        self.menu.insert("Decrease ROI size 20%", callback=lambda f: self.resize_roi(0.8))
        self.menu.insert("夹持力未测量 / Force unmeasured",
                         callback=lambda f: self.status("夹持力未测量 / Force unmeasured"))
        self.roi_marker()
        self.add_buttons()
        self.server.applyChanges()
        self.cloud = self.cloud_message(self.xyz[::4], colors[::4])
        self.timer = self.create_timer(1.0, self.publish_scene)
        self.publish_scene()
        self.status("force unmeasured / 夹持力未测量; click a grasp with RViz Interact")

    def status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def planning_status(self, message):
        self.sim_status = message.data
        self.publish_scene()

    def cloud_message(self, points, rgb):
        rgba = (rgb[:, 0].astype(np.uint32) << 16) | (rgb[:, 1].astype(np.uint32) << 8) | rgb[:, 2]
        payload = np.empty((len(points), 4), dtype="<f4")
        payload[:, :3], payload[:, 3] = points, rgba.astype("<u4").view("<f4")
        cloud = PointCloud2()
        cloud.header.frame_id = self.meta["depth_frame"]
        cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(self.meta["depth_stamp_ns"], 10**9)
        cloud.height, cloud.width, cloud.point_step = 1, len(points), 16
        cloud.row_step, cloud.is_dense, cloud.is_bigendian = len(points)*16, True, False
        cloud.fields = [PointField(name=name, offset=i*4,
            datatype=PointField.UINT32 if name == "rgb" else PointField.FLOAT32, count=1)
            for i, name in enumerate(("x", "y", "z", "rgb"))]
        cloud.data = payload.tobytes()
        return cloud

    def line_marker(self, item):
        marker = Marker()
        marker.header.frame_id, marker.ns, marker.id = self.meta["depth_frame"], "grasps", item["id"]
        marker.type, marker.action = Marker.LINE_LIST, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.004 if self.selected == item["id"] else 0.002
        # Hue differentiates candidates, never implies force or probability.
        import colorsys
        r, g, b = map(float, colorsys.hsv_to_rgb((item["id"] * 0.618) % 1, 0.8, 1.0))
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = r, g, b, 1.0
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in gripper_lines(item)]
        return marker

    def add_buttons(self):
        for item in self.items.values():
            interactive = InteractiveMarker()
            interactive.header.frame_id = self.meta["depth_frame"]
            interactive.name, interactive.description = f"grasp_{item['id']}", f"Select #{item['id']}"
            interactive.pose.position.x, interactive.pose.position.y, interactive.pose.position.z = item["position"]
            interactive.pose.orientation.w, interactive.scale = 1.0, 0.05
            control = InteractiveMarkerControl()
            control.name, control.interaction_mode = "select", InteractiveMarkerControl.BUTTON
            control.always_visible = True
            dot = Marker(type=Marker.SPHERE)
            dot.pose.orientation.w = 1.0
            dot.scale.x = dot.scale.y = dot.scale.z = 0.008
            dot.color.r, dot.color.g, dot.color.b, dot.color.a = 0.8, 0.2, 1.0, 1.0
            control.markers.append(dot)
            interactive.controls.append(control)
            self.server.insert(interactive, feedback_callback=self.button_feedback)

    def button_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.select(int(feedback.marker_name.split("_")[-1]))

    def select_service(self, request, response):
        response.accepted, response.reason = self.select(request.id)
        return response

    def select(self, ident):
        if ident not in self.items:
            return False, "unknown candidate ID"
        self.selected = ident
        item = self.items[ident]
        self.selected_pub.publish(candidate_message(item, self.meta))
        self.status(f"selected #{ident}: position={item['position']} "
                    f"quaternion_xyzw={item['quaternion_xyzw']} width={item['width_m']:.4f}m "
                    f"score={item['score']:.4f}; force unmeasured / 夹持力未测量")
        self.publish_scene()
        return True, "selected; simulation planner must be launched separately"

    def roi_marker(self):
        interactive = InteractiveMarker()
        interactive.header.frame_id = self.meta["depth_frame"]
        interactive.name, interactive.description, interactive.scale = "target_roi", "Target ROI (right click to save/resize)", 0.2
        interactive.pose.position.x, interactive.pose.position.y, interactive.pose.position.z = self.center.tolist()
        interactive.pose.orientation.w = 1.0
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MENU
        cube = Marker(type=Marker.CUBE)
        cube.pose.orientation.w = 1.0
        cube.scale.x, cube.scale.y, cube.scale.z = self.size.tolist()
        cube.color.g, cube.color.a = 1.0, 0.15
        control.markers.append(cube)
        interactive.controls.append(control)
        for axis, orientation in (("x", (1., 0., 0.)), ("y", (0., 0., 1.)), ("z", (0., 1., 0.))):
            move = InteractiveMarkerControl()
            move.name, move.interaction_mode = "move_"+axis, InteractiveMarkerControl.MOVE_AXIS
            move.orientation.w = 1.0
            move.orientation.x, move.orientation.y, move.orientation.z = orientation
            interactive.controls.append(move)
        self.server.insert(interactive, feedback_callback=self.roi_feedback)
        self.menu.apply(self.server, interactive.name)
        self.server.applyChanges()

    def roi_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            p = feedback.pose.position
            self.center = np.array([p.x, p.y, p.z])

    def resize_roi(self, factor):
        self.size = np.clip(self.size * factor, 0.01, 5.0)
        self.roi_marker()

    def save_roi(self, feedback):
        self.roi_path.parent.mkdir(parents=True, exist_ok=True)
        value = {"min": (self.center-self.size/2).tolist(), "max": (self.center+self.size/2).tolist()}
        self.roi_path.write_text(json.dumps(value, indent=2))
        self.status(f"saved ROI: {self.roi_path}; rerun offline inference to update candidates")

    def publish_scene(self):
        self.cloud_pub.publish(self.cloud)
        self.candidate_pub.publish(GraspCandidates(candidates=[candidate_message(i, self.meta) for i in self.items.values()]))
        markers = [self.line_marker(i) for i in self.items.values()]
        label = Marker()
        label.header.frame_id, label.ns, label.id = self.meta["depth_frame"], "selection_status", 0
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.orientation.w = 1.0
        label.scale.z = 0.04
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        position = self.center.copy()
        if self.selected is not None:
            item = self.items[self.selected]
            position = np.asarray(item["position"])
            label.text = (f"#{self.selected} pos={np.round(position, 3).tolist()}\n"
                          f"q_xyzw={np.round(item['quaternion_xyzw'], 3).tolist()}\n"
                          f"width={item['width_m']:.3f}m score={item['score']:.3f}\n")
        label.text += self.result.get("backend", "ROI selection; no inference result")+"\n"
        # Ogre's 3D text is byte-oriented and keeps an oversized default space
        # width. Keep the label ASCII with visible separators; full bilingual
        # status remains available in the Qt menu, /grasp/status and logs.
        label.text += "Force unmeasured\n"+self.sim_status
        counts = self.result.get("metrics", {}).get("filter_counts", {})
        if counts:
            label.text += "\nfilters: "+str(counts)
        label.text = "\n".join(part.replace(" ", "_")
            for line in label.text.splitlines() for part in textwrap.wrap(line, width=64))
        # Keep the multi-line status above the scene in the default optical
        # view instead of obscuring the selected object's point cloud.
        label_position = position.copy()
        label_position[1] = min(position[1], self.scene_top_y)-0.35
        label.pose.position.x, label.pose.position.y, label.pose.position.z = label_position.tolist()
        markers.append(label)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = Viewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
