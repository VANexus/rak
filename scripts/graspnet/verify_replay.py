"""Verify saved model candidates through ROS selection and RViz, without motion.

Requires the Jazzy grasp overlay, ROS_DOMAIN_ID=77 and an X display (Xvfb works).
Outputs evidence in a new directory; never creates or alters model candidates.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from PIL import ImageGrab
import rclpy
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from rak_interfaces.msg import GraspCandidate, GraspCandidates
from rak_interfaces.srv import SelectGrasp
from rak_grasp_perception.core import load_snapshot, validate_result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidate-id', type=int)
    parser.add_argument('--simulation-mount', type=Path,
                        help='Optional explicit fictional mount for five-phase planning verification')
    parser.add_argument('--roi', type=Path, help='Required with --simulation-mount')
    parser.add_argument('--rviz-config', type=Path, help='Optional viewpoint configuration')
    parser.add_argument('--expect-rejection', help='Require this substring in a simulation rejection')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '77' or not os.environ.get('DISPLAY'):
        parser.error('requires ROS_DOMAIN_ID=77 and DISPLAY')
    meta, _, _ = load_snapshot(args.snapshot)
    saved = validate_result(json.loads(args.result.read_text()), meta)
    if not saved['candidates'] or saved.get('backend') != 'graspnet-baseline':
        parser.error('requires nonempty actual GraspNet output')
    if args.simulation_mount and not args.roi:
        parser.error('--simulation-mount requires --roi')
    if args.simulation_mount and json.loads(args.roi.read_text()) != saved.get('roi'):
        parser.error('simulation ROI must match the saved model inference ROI')
    if args.expect_rejection and not args.simulation_mount:
        parser.error('--expect-rejection requires --simulation-mount')
    args.output.mkdir(parents=True, exist_ok=False)
    log = (args.output/'replay.log').open('w')
    processes = []
    rclpy.init()
    node = Node('verify_saved_grasp_replay')
    clouds, arrays, selected, statuses, sim_statuses, trajectories = [], [], [], [], [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(PointCloud2, '/grasp/scene', clouds.append, qos)
    node.create_subscription(GraspCandidates, '/grasp/candidates', arrays.append, qos)
    node.create_subscription(GraspCandidate, '/grasp/selected', selected.append, 10)
    node.create_subscription(String, '/grasp/status', lambda m: statuses.append(m.data), qos)
    node.create_subscription(String, '/grasp/sim_status', lambda m: sim_statuses.append(m.data), 10)
    node.create_subscription(DisplayTrajectory, '/grasp/display_planned_path', trajectories.append, 10)
    client = node.create_client(SelectGrasp, '/grasp/select')

    def wait(predicate, timeout=45):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('replay or RViz exited; inspect replay.log')
            rclpy.spin_once(node, timeout_sec=.1)
            if predicate():
                return
        raise TimeoutError('ROS replay did not become ready')

    try:
        visualizer = (['ros2', 'launch', 'rak_moveit_config', 'grasp_sim.launch.py',
                       f'snapshot:={args.snapshot}', f'roi:={args.roi}',
                       f'simulation_mount:={args.simulation_mount}', 'rviz:=true']
                      if args.simulation_mount else
                      ['rviz2', '-d', str(Path(get_package_share_directory('rak_grasp_perception'))/
                                         'config/grasp_scene.rviz')])
        if args.rviz_config:
            if args.simulation_mount:
                visualizer.append(f'rviz_config:={args.rviz_config}')
            else:
                visualizer[-1] = str(args.rviz_config)
        for command in (
            ['ros2', 'launch', 'rak_grasp_perception', 'replay.launch.py',
             f'snapshot:={args.snapshot}', f'result:={args.result}',
             f'roi_output:={args.output}/roi.json'],
            visualizer,
        ):
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        wait(lambda: clouds and arrays and client.service_is_ready())
        assert clouds[-1].width > 0
        assert len(arrays[-1].candidates) == len(saved['candidates'])
        if args.simulation_mount:
            wait(lambda: 'grasp_sim_planner' in node.get_node_names()
                 and any(name == '/compute_ik' for name, _ in node.get_service_names_and_types()))
            # The planner's software joint-state subscription must also settle.
            deadline = time.monotonic()+3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
        ident = args.candidate_id if args.candidate_id is not None else saved['candidates'][0]['id']
        future = client.call_async(SelectGrasp.Request(id=ident))
        wait(future.done)
        assert future.result().accepted, future.result().reason
        wait(lambda: selected and selected[-1].id == ident)
        wait(lambda: any('selected #' in status for status in statuses))
        if args.simulation_mount:
            wait(lambda: any('PLANNED_ONLY' in s or s.startswith('REJECTED:')
                             for s in sim_statuses), 60)
            if args.expect_rejection:
                assert sim_statuses[-1].startswith('REJECTED:') and args.expect_rejection in sim_statuses[-1], sim_statuses
                assert not trajectories, 'rejected candidate published a complete preview'
            else:
                assert 'PLANNED_ONLY' in sim_statuses[-1], sim_statuses
                wait(lambda: bool(trajectories))
                assert len(trajectories[-1].trajectory) == 5
        time.sleep(8)
        ImageGrab.grab().save(args.output/'rviz-model-candidates.png')
        log.flush()
        errors = [line for line in (args.output/'replay.log').read_text().splitlines()
                  if '[ERROR]' in line and (not args.expect_rejection or '[rviz2-' in line)]
        assert not errors, errors
        evidence = {'backend': saved['backend'], 'result': str(args.result.resolve()),
                    'candidate_count': len(saved['candidates']),
                    'displayed_cloud_points': clouds[-1].width,
                    'selected_id': ident, 'status': statuses[-1],
                    'metrics': saved.get('metrics'), 'hardware_motion': False}
        if args.simulation_mount:
            evidence.update(simulation_mount=str(args.simulation_mount.resolve()),
                            simulation_status=sim_statuses[-1],
                            expected_rejection=args.expect_rejection,
                            trajectory_phases=len(trajectories[-1].trajectory) if trajectories else 0)
        (args.output/'results.json').write_text(json.dumps(evidence, indent=2))
        print(json.dumps(evidence, indent=2))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    main()
