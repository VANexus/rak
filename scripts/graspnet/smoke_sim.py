"""Synthetic ROS/MoveIt integration test. No camera, model output or hardware motion.

Run on Jazzy with ROS_DOMAIN_ID=77 and a sourced grasp overlay. --output must be a
new directory outside the repository, e.g. /home/xrak/tools/graspnet/sim-smoke.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, DisplayTrajectory, PlanningScene, PlanningSceneWorld
from moveit_msgs.srv import ApplyPlanningScene
from rak_interfaces.msg import GraspCandidate
from rak_interfaces.srv import SelectGrasp
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--rviz", action="store_true", help="Render fixture and trajectory on an available X display")
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "77":
        raise SystemExit("Requires ROS_DOMAIN_ID=77")
    if args.rviz and not os.environ.get("DISPLAY"):
        raise SystemExit("--rviz requires DISPLAY (use xvfb-run for headless validation)")
    args.output.mkdir(parents=True, exist_ok=False)
    info = {"width": 3, "height": 3, "k": [100, 0, 1, 0, 100, 1, 0, 0, 1], "d": []}
    meta = {"schema_version": 1, "depth_unit": "mm", "color_stamp_ns": 1000,
            "depth_stamp_ns": 1000, "depth_frame": "camera_depth_optical_frame",
            "depth_info": info, "color_info": info, "depth_to_color": np.eye(4).tolist(),
            "source": "synthetic integration fixture, NOT camera or model output"}
    (args.output/"snapshot.json").write_text(json.dumps(meta))
    cv2.imwrite(str(args.output/"color.png"), np.full((3, 3, 3), 255, np.uint8))
    cv2.imwrite(str(args.output/"depth-mm.png"), np.full((3, 3), 1000, np.uint16))
    (args.output/"roi.json").write_text(json.dumps({"min": [-.1, -.1, .9], "max": [.5, .1, 1.1]}))
    (args.output/"mount.yaml").write_text(
        'simulation_only: true\nsource_frame: camera_depth_optical_frame\ntarget_frame: base_link\n'
        'matrix: [[1,0,0,0.38],[0,1,0,0],[0,0,1,-0.75],[0,0,0,1]]\n')
    log = (args.output/"launch.log").open("w")
    process = subprocess.Popen(["ros2", "launch", "rak_moveit_config", "grasp_sim.launch.py",
        f"snapshot:={args.output}", f"roi:={args.output/'roi.json'}",
        f"simulation_mount:={args.output/'mount.yaml'}", f"rviz:={str(args.rviz).lower()}"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    viewer_process = None
    if args.rviz:
        item = {"id": 1, "position": [0., 0., 1.], "quaternion_xyzw": [0., 0., 0., 1.],
                "width_m": .04, "depth_m": .03, "score": .8}
        (args.output/'result.json').write_text(json.dumps({"schema_version": 1,
            "frame_id": meta['depth_frame'], "stamp_ns": 1000,
            "backend": "SYNTHETIC FIXTURE - NOT MODEL OUTPUT", "candidates": [item]}))
        viewer_process = subprocess.Popen(['ros2', 'run', 'rak_grasp_perception', 'replay_grasps',
            '--ros-args', '-p', f'snapshot:={args.output}', '-p', f'result:={args.output}/result.json',
            '-p', f'roi_output:={args.output}/viewer-roi.json'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node("grasp_sim_smoke")
    statuses, trajectories, results = [], [], {}
    node.create_subscription(String, "/grasp/sim_status", lambda m: statuses.append(m.data), 10)
    node.create_subscription(DisplayTrajectory, "/grasp/display_planned_path", trajectories.append, 10)
    publisher = node.create_publisher(GraspCandidate, "/grasp/selected", 10)
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    selection_client = node.create_client(SelectGrasp, "/grasp/select") if args.rviz else None

    def wait(predicate, timeout=40):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("simulation launch exited; inspect launch.log")
            rclpy.spin_once(node, timeout_sec=.1)
            if predicate():
                return
        raise TimeoutError(str(statuses[-5:]))

    def selected(ident, x=0.):
        statuses.clear()
        message = GraspCandidate(id=ident, width_m=.04, depth_m=.03, score=.8)
        message.header.frame_id = meta["depth_frame"]
        message.header.stamp.nanosec = 1000
        message.pose.position.x, message.pose.position.z = x, 1.
        message.pose.orientation.w = 1.
        if selection_client is not None and ident == 1:
            wait(selection_client.service_is_ready, 10)
            future = selection_client.call_async(SelectGrasp.Request(id=ident))
            wait(future.done, 10)
            assert future.result().accepted, future.result().reason
        else:
            publisher.publish(message)
        wait(lambda: any("PLANNED_ONLY" in s or s.startswith("REJECTED:") for s in statuses))
        return next(s for s in statuses if "PLANNED_ONLY" in s or s.startswith("REJECTED:"))

    def scene_object(operation):
        obj = CollisionObject(id="smoke_blocker", operation=operation)
        obj.header.frame_id = "base_link"
        obj.pose.orientation.w = 1.
        if operation == CollisionObject.ADD:
            pose = Pose()
            pose.orientation.w, pose.position.x, pose.position.z = 1., .25, .25
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.5, .5, .5])]
            obj.primitive_poses = [pose]
        future = client.call_async(ApplyPlanningScene.Request(
            scene=PlanningScene(is_diff=True, world=PlanningSceneWorld(collision_objects=[obj]))))
        wait(future.done, 10)
        assert future.result().success

    try:
        wait(lambda: client.service_is_ready() and publisher.get_subscription_count() > 0, 30)
        if args.rviz:
            wait(lambda: any(name.startswith('rviz') for name in node.get_node_names()), 30)
        # Also allow the fake joint state subscription to become ready.
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        results["reachable"] = selected(1)
        assert "PLANNED_ONLY" in results["reachable"], results
        wait(lambda: bool(trajectories), 5)
        assert len(trajectories[-1].trajectory) == 5
        if args.rviz:
            from PIL import ImageGrab
            time.sleep(8)
            if viewer_process.poll() is not None:
                raise RuntimeError('Viewer exited; inspect launch.log')
            screenshot = ImageGrab.grab()
            assert screenshot.convert('L').getextrema()[1] > 10, 'RViz screenshot is blank'
            screenshot.save(args.output/'rviz-synthetic-trajectory.png')
        results["unreachable"] = selected(2, .4)
        assert results["unreachable"].startswith("REJECTED:"), results
        accepted_count = len(trajectories)
        scene_object(CollisionObject.ADD)
        results["collision"] = selected(3)
        assert results["collision"].startswith("REJECTED:") and "collision" in results["collision"], results
        assert len(trajectories) == accepted_count == 1, 'rejected candidate published a preview'
        scene_object(CollisionObject.REMOVE)
        results["trajectory_phases"] = len(trajectories[-1].trajectory)
        if args.rviz:
            errors = [line for line in (args.output/'launch.log').read_text().splitlines()
                      if '[rviz2-' in line and '[ERROR]' in line]
            if errors:
                raise RuntimeError('RViz visualization errors: '+str(errors))
        (args.output/"results.json").write_text(json.dumps(results, indent=2))
        print(json.dumps(results, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()
        if viewer_process is not None and viewer_process.poll() is None:
            os.killpg(viewer_process.pid, signal.SIGINT)
            try:
                viewer_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(viewer_process.pid, signal.SIGTERM)
                viewer_process.wait(timeout=5)


if __name__ == "__main__":
    main()
