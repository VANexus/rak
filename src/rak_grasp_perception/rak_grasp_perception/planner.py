"""Plan-only MoveIt pick preview in isolated domain 77; no execution clients."""
import copy
import json
import os
from pathlib import Path
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints,
    DisplayTrajectory, JointConstraint, MotionPlanRequest,
    PlanningScene, PlanningSceneComponents, RobotState, RobotTrajectory)
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetMotionPlan,
    GetPositionFK, GetPositionIK, GetStateValidity, GetPlanningScene)
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from rak_interfaces.msg import GraspCandidate
from rak_arm_simulation.controllers import JOINT_BOUNDS
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml

from .core import check_transform, load_snapshot, roi_mask, scene_cloud
from .collision import occupied_boxes


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    value = np.eye(4)
    value[:3, 3] = [p.x, p.y, p.z]
    value[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return value


def matrix_pose(value):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = value[:3, 3].tolist()
    q = pose.orientation
    q.x, q.y, q.z, q.w = Rotation.from_matrix(value[:3, :3]).as_quat().tolist()
    return pose


def end_state(state, trajectory):
    result = copy.deepcopy(state)
    values = dict(zip(result.joint_state.name, result.joint_state.position))
    values.update(zip(trajectory.joint_trajectory.joint_names,
                      trajectory.joint_trajectory.points[-1].positions))
    result.joint_state.name, result.joint_state.position = list(values), list(values.values())
    return result


class Planner(Node):
    def __init__(self):
        super().__init__("grasp_sim_planner")
        # [AI] 隔离默认仍是 77，但不再硬拦（工作区统一域 47；见 grasp_sim.launch.py）
        if os.environ.get("ROS_DOMAIN_ID") != "77":
            self.get_logger().warning(
                f"grasp simulation prefers ROS_DOMAIN_ID=77, current="
                f"{os.environ.get('ROS_DOMAIN_ID')!r}"
            )
        mount = yaml.safe_load(Path(self.declare_parameter("simulation_mount", "").value).read_text())
        if mount.get("simulation_only") is not True or mount.get("target_frame") != "base_link":
            raise ValueError("explicit simulation-only base_link mount is required")
        self.mount = check_transform(mount["matrix"])
        # Optional fictional start for plan-only previews; never changes joint
        # feedback or commands the fixture. MoveIt validates it below.
        self.fixture_start = mount.get("fixture_start_positions")
        self.meta, rgb, depth = load_snapshot(self.declare_parameter("snapshot", "").value)
        if mount["source_frame"] != self.meta["depth_frame"]:
            raise ValueError("mount optical frame does not match snapshot")
        self.points, _ = scene_cloud(self.meta, rgb, depth)
        self.roi = json.loads(Path(self.declare_parameter("roi", "").value).read_text())
        self.inside = roi_mask(self.points, self.roi)
        self.points = self.points @ self.mount[:3, :3].T + self.mount[:3, 3]
        self.busy, self.joints = False, None
        self.status_pub = self.create_publisher(String, "/grasp/sim_status", 10)
        # MoveIt's default topic can contain a pregrasp plan even when a later
        # phase fails. Only publish the complete validated preview here.
        self.display = self.create_publisher(DisplayTrajectory, "/grasp/display_planned_path", 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state, 10)
        self.create_subscription(GraspCandidate, "/grasp/selected", self.selected, 10)
        self.planning_clients = {
            "scene": self.create_client(ApplyPlanningScene, "/apply_planning_scene"),
            "plan": self.create_client(GetMotionPlan, "/plan_kinematic_path"),
            "cart": self.create_client(GetCartesianPath, "/compute_cartesian_path"),
            "fk": self.create_client(GetPositionFK, "/compute_fk"),
            "ik": self.create_client(GetPositionIK, "/compute_ik"),
            "valid": self.create_client(GetStateValidity, "/check_state_validity"),
            "get_scene": self.create_client(GetPlanningScene, "/get_planning_scene"),
        }
        self.tf = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "base_link", self.meta["depth_frame"]
        tf.header.stamp = self.get_clock().now().to_msg()
        p = matrix_pose(self.mount)
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p.position.x, p.position.y, p.position.z
        tf.transform.rotation = p.orientation
        self.tf.sendTransform(tf)
        self.status("SIMULATION ONLY: virtual mount, no measured force, no execution")

    def status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def joint_state(self, message):
        self.joints = message

    def call(self, name, request):
        client = self.planning_clients[name]
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f"MoveIt service unavailable: {name}")
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: event.set())
        if not event.wait(20):
            client.remove_pending_request(future)
            raise RuntimeError(f"MoveIt {name} timeout")
        return future.result()

    def selected(self, message):
        if self.busy:
            self.status("BUSY: selection ignored; select again after planning")
            return
        self.busy = True
        threading.Thread(target=self.plan, args=(message,), daemon=True).start()

    def scene_object(self, ident, points):
        obj = CollisionObject()
        obj.header.frame_id, obj.id, obj.operation = "base_link", ident, CollisionObject.ADD
        obj.pose.orientation.w = 1.0
        # Build in the optical frame, where the target ROI is axis aligned.
        # Splitting mixed cells avoids artificial target/obstacle penetration.
        points = points[np.linalg.norm(points, axis=1) <= 0.9]
        optical = (points-self.mount[:3, 3]) @ self.mount[:3, :3]
        for center, dimensions in occupied_boxes(optical, self.roi, ident == 'grasp_target'):
            obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX,
                                                dimensions=dimensions.tolist()))
            transform = self.mount.copy()
            transform[:3, 3] = self.mount[:3, :3] @ center + self.mount[:3, 3]
            obj.primitive_poses.append(matrix_pose(transform))
        return obj

    def apply_objects(self, objects):
        scene = PlanningScene(is_diff=True)
        # Contact with the target is only permitted for virtual gripper links.
        query = GetPlanningScene.Request()
        query.components.components = (PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                                       | PlanningSceneComponents.WORLD_OBJECT_NAMES)
        current = self.call("get_scene", query).scene
        existing = {obj.id for obj in current.world.collision_objects}
        scene.world.collision_objects = [obj for obj in objects
            if obj.operation != CollisionObject.REMOVE or obj.id in existing]
        matrix = current.allowed_collision_matrix
        from moveit_msgs.msg import AllowedCollisionEntry
        for name in ("grasp_target", "tool0", "gripper_link"):
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        target = matrix.entry_names.index("grasp_target")
        for name in ("tool0", "gripper_link"):
            link = matrix.entry_names.index(name)
            matrix.entry_values[target].enabled[link] = True
            matrix.entry_values[link].enabled[target] = True
        scene.allowed_collision_matrix = matrix
        if not self.call("scene", ApplyPlanningScene.Request(scene=scene)).success:
            raise RuntimeError("planning scene rejected")

    def arm_plan(self, state, target):
        # Resolve an explicit IK goal from the current state. A loose pose goal
        # can end on a different wrist branch or away from the exact pregrasp,
        # causing the following Cartesian segment to fail its jump checks.
        ik = GetPositionIK.Request()
        ik.ik_request.group_name, ik.ik_request.ik_link_name = "arm", "tool0"
        ik.ik_request.robot_state, ik.ik_request.avoid_collisions = state, True
        ik.ik_request.pose_stamped.header.frame_id = "base_link"
        ik.ik_request.pose_stamped.pose = target
        ik.ik_request.timeout.sec = 2
        solution = self.call("ik", ik)
        if solution.error_code.val != 1:
            raise RuntimeError(f"pregrasp IK/planning rejected: {solution.error_code.val}")
        constraint = Constraints(joint_constraints=[JointConstraint(
            joint_name=name, position=value, tolerance_above=0.0001,
            tolerance_below=0.0001, weight=1.0)
            for name, value in zip(solution.solution.joint_state.name,
                                   solution.solution.joint_state.position)
            if name != "gripper_joint"])
        if len(constraint.joint_constraints) != 6:
            raise RuntimeError("IK solution does not match the six-axis software fixture")
        request = MotionPlanRequest(group_name="arm", start_state=state,
            goal_constraints=[constraint], num_planning_attempts=3, allowed_planning_time=5.0,
            max_velocity_scaling_factor=0.1, max_acceleration_scaling_factor=0.1)
        response = self.call("plan", GetMotionPlan.Request(motion_plan_request=request)).motion_plan_response
        if response.error_code.val != 1 or not response.trajectory.joint_trajectory.points:
            raise RuntimeError(f"pregrasp IK/planning rejected: {response.error_code.val}")
        return response.trajectory

    def cartesian(self, state, target):
        request = GetCartesianPath.Request()
        request.header.frame_id, request.group_name, request.link_name = "base_link", "arm", "tool0"
        request.start_state, request.waypoints = state, [target]
        request.max_step, request.jump_threshold, request.avoid_collisions = 0.005, 2.0, True
        request.revolute_jump_threshold = 0.3
        request.prismatic_jump_threshold = 0.01
        response = self.call("cart", request)
        if response.error_code.val != 1 or response.fraction < 0.999 or not response.solution.joint_trajectory.points:
            raise RuntimeError(f"Cartesian collision/reachability failure: fraction={response.fraction:.3f}")
        # Bound preview speed even if this MoveIt version returns untimed Cartesian points.
        for i, point in enumerate(response.solution.joint_trajectory.points):
            ns = (i+1)*100_000_000
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
            point.velocities, point.accelerations = [], []
        return response.solution

    def gripper(self, state, width):
        result = RobotTrajectory()
        result.joint_trajectory.joint_names = ["gripper_joint"]
        current = dict(zip(state.joint_state.name, state.joint_state.position))["gripper_joint"]
        for seconds, value in ((0, current), (1, width)):
            point = JointTrajectoryPoint(positions=[float(value)])
            point.time_from_start.sec = seconds
            result.joint_trajectory.points.append(point)
        return result

    def plan(self, message):
        original_scene = None
        terminal = "REJECTED: planning interrupted"
        try:
            if self.joints is None or len(self.joints.name) != 7:
                raise RuntimeError("waiting for seven-joint software fixture state")
            stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
            if message.header.frame_id != self.meta["depth_frame"] or stamp != self.meta["depth_stamp_ns"]:
                raise ValueError("selected grasp belongs to a different snapshot")
            if (not np.isfinite([message.width_m, message.depth_m, message.score]).all()
                    or not 0 < message.width_m <= 0.05 or message.depth_m <= 0):
                raise ValueError("candidate opening exceeds virtual fixture range (0,0.05] m")
            q = message.pose.orientation
            if not np.isfinite([q.x, q.y, q.z, q.w]).all() or not np.isclose(
                    np.linalg.norm([q.x, q.y, q.z, q.w]), 1, atol=1e-5):
                raise ValueError("selected grasp has an invalid orientation")
            point = message.pose.position
            if not roi_mask(np.array([[point.x, point.y, point.z]]), self.roi)[0]:
                raise ValueError("selected grasp lies outside the target ROI")
            grasp = self.mount @ check_transform(pose_matrix(message.pose))
            initial = RobotState(joint_state=copy.deepcopy(self.joints))
            if self.fixture_start is not None:
                if (set(self.fixture_start) != set(initial.joint_state.name)
                        or not np.isfinite(list(self.fixture_start.values())).all()):
                    raise ValueError("fictional start must specify all seven finite fixture joints")
                initial.joint_state.position = [float(self.fixture_start[name])
                                               for name in initial.joint_state.name]
                if any(not JOINT_BOUNDS[name][0] <= value <= JOINT_BOUNDS[name][1]
                       for name, value in zip(initial.joint_state.name, initial.joint_state.position)):
                    raise ValueError("fictional start exceeds software fixture joint limits")
            original_scene = [self.scene_object("grasp_obstacles", self.points[~self.inside]),
                              self.scene_object("grasp_target", self.points[self.inside])]
            if not original_scene[1].primitives:
                original_scene = None
                raise ValueError("target has no points in the virtual arm workspace")
            # Empty obstacle sets remove stale obstacles rather than adding an invalid shape.
            if not original_scene[0].primitives:
                original_scene[0].operation = CollisionObject.REMOVE
            self.apply_objects(original_scene)
            validity = self.call("valid", GetStateValidity.Request(robot_state=initial, group_name="arm"))
            if not validity.valid:
                contacts = [(c.contact_body_1, c.contact_body_2) for c in validity.contacts]
                raise RuntimeError(f"collision/constraints in virtual starting state: {contacts}")
            state, phases = copy.deepcopy(initial), []
            opened = self.gripper(state, 0.05)
            phases.append(opened)
            state = end_state(state, opened)
            validity = self.call("valid", GetStateValidity.Request(robot_state=state, group_name="arm"))
            if not validity.valid:
                contacts = [(c.contact_body_1, c.contact_body_2) for c in validity.contacts]
                raise RuntimeError(f"collision in virtual opening state: {contacts}")
            # Parallel jaws have the same grasp after a 180-degree roll about
            # +X. NMS uses this symmetry too. Test both exact orientations with
            # the unchanged scene and fixture collision geometry.
            failures = []
            for roll_degrees in (0, 180):
                oriented = grasp.copy()
                if roll_degrees:
                    oriented[:3, :3] = grasp[:3, :3] @ np.diag([1., -1., -1.])
                # Explicit virtual TCP: grasp center is 0.08 m along tool0 +X.
                tool = oriented.copy()
                tool[:3, 3] -= oriented[:3, 0]*0.08
                pre = tool.copy()
                pre[:3, 3] -= tool[:3, 0]*0.05
                self.status(f"#{message.id}: planning pregrasp, equivalent roll={roll_degrees}deg")
                try:
                    pregrasp = self.arm_plan(state, matrix_pose(pre))
                    approached = end_state(state, pregrasp)
                    self.status(f"#{message.id}: planning collision-checked approach")
                    approach = self.cartesian(approached, matrix_pose(tool))
                    approached = end_state(approached, approach)
                    close = self.gripper(approached, max(0.0, message.width_m-0.002))
                    closed = end_state(approached, close)
                    validity = self.call("valid", GetStateValidity.Request(
                        robot_state=closed, group_name="arm"))
                    if not validity.valid:
                        raise RuntimeError("virtual closure collides with scene")
                except RuntimeError as error:
                    failures.append(f"{roll_degrees}deg: {error}")
                    continue
                phases.extend([pregrasp, approach, close])
                state = closed
                break
            else:
                raise RuntimeError("both parallel-jaw orientations rejected; "+"; ".join(failures))
            fk_req = GetPositionFK.Request(robot_state=state, fk_link_names=["tool0"])
            fk_req.header.frame_id = "base_link"
            fk = self.call("fk", fk_req)
            if fk.error_code.val != 1 or not fk.pose_stamped:
                raise RuntimeError("fixture FK unavailable")
            inverse_tool = np.linalg.inv(pose_matrix(fk.pose_stamped[0].pose))
            attached = AttachedCollisionObject(link_name="tool0", touch_links=["tool0", "gripper_link"])
            attached.object = copy.deepcopy(original_scene[1])
            attached.object.header.frame_id = "tool0"
            attached.object.primitive_poses = [matrix_pose(inverse_tool @ pose_matrix(p))
                                             for p in attached.object.primitive_poses]
            state.attached_collision_objects = [attached]
            remove = CollisionObject(id="grasp_target", operation=CollisionObject.REMOVE)
            self.apply_objects([remove])
            validity = self.call("valid", GetStateValidity.Request(robot_state=state, group_name="arm"))
            if not validity.valid:
                contacts = [(c.contact_body_1, c.contact_body_2) for c in validity.contacts]
                raise RuntimeError(f"attached virtual target collides before lift: {contacts}")
            lift = tool.copy()
            lift[2, 3] += 0.08
            self.status(f"#{message.id}: planning lift with attached virtual target")
            phases.append(self.cartesian(state, matrix_pose(lift)))
            display = DisplayTrajectory(model_id="rak_six_axis_sim", trajectory_start=initial, trajectory=phases)
            self.display.publish(display)
            terminal = (f"#{message.id}: PLANNED_ONLY open/pregrasp/approach/close/lift; "
                        f"equivalent_roll={roll_degrees}deg; force unmeasured")
        except Exception as error:
            terminal = f"REJECTED: {error}"
        finally:
            if original_scene is not None:
                try:
                    self.apply_objects(original_scene)
                except Exception as error:
                    terminal = f"REJECTED: scene restore failed: {error}"
            self.busy = False
            self.status(terminal)


def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
