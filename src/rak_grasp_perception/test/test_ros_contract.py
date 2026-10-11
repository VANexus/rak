"""Real ROS serialization, transient replay, selection and stride decoding."""
import json
import os
import time

import cv2
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from rak_interfaces.srv import SelectGrasp
from rak_grasp_perception.capture import image_array
from rak_grasp_perception.viewer import Viewer


def test_capture_padded_bigendian_and_rgb():
    image = Image(height=1, width=2, encoding="16UC1", is_bigendian=1, step=6,
                  data=bytes([0x03, 0xe8, 0x07, 0xd0, 0, 0]))
    np.testing.assert_array_equal(image_array(image), [[1000, 2000]])
    image = Image(height=1, width=1, encoding="rgb8", step=4, data=bytes([1, 2, 3, 0]))
    np.testing.assert_array_equal(image_array(image, True), [[[3, 2, 1]]])
    image.step = 1
    with pytest.raises(ValueError, match="stride"):
        image_array(image, True)


def test_replay_and_selection_over_ros(tmp_path):
    if os.environ.get("ROS_DOMAIN_ID") != "77":
        pytest.skip("ROS integration requires isolated ROS_DOMAIN_ID=77")
    info = {"width": 3, "height": 3, "k": [100, 0, 1, 0, 100, 1, 0, 0, 1], "d": []}
    meta = {"schema_version": 1, "depth_unit": "mm", "color_stamp_ns": 1000,
            "depth_stamp_ns": 1000, "depth_frame": "camera_depth_optical_frame",
            "depth_info": info, "color_info": info, "depth_to_color": np.eye(4).tolist()}
    (tmp_path/"snapshot.json").write_text(json.dumps(meta))
    cv2.imwrite(str(tmp_path/"color.png"), np.full((3, 3, 3), 255, np.uint8))
    cv2.imwrite(str(tmp_path/"depth-mm.png"), np.full((3, 3), 1000, np.uint16))
    item = {"id": 7, "position": [0., 0., 1.], "quaternion_xyzw": [0., 0., 0., 1.],
            "width_m": 0.04, "depth_m": 0.03, "score": 0.8}
    result = {"schema_version": 1, "stamp_ns": 1000, "frame_id": meta["depth_frame"],
              "backend": "synthetic-contract-test", "candidates": [item]}
    (tmp_path/"result.json").write_text(json.dumps(result))
    rclpy.init(args=["--ros-args", "-p", f"snapshot:={tmp_path}",
                     "-p", f"result:={tmp_path/'result.json'}"])
    viewer, observer, executor = None, None, SingleThreadedExecutor()
    try:
        viewer, observer = Viewer(), Node("grasp_contract_observer")
        executor.add_node(viewer)
        executor.add_node(observer)
        messages = []
        observer.create_subscription(PointCloud2, "/grasp/scene", messages.append, 10)
        client = observer.create_client(SelectGrasp, "/grasp/select")
        assert client.wait_for_service(timeout_sec=5)
        for ident, expected in ((99, False), (7, True)):
            future = client.call_async(SelectGrasp.Request(id=ident))
            deadline = time.monotonic()+5
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
            assert future.done()
            assert future.result().accepted is expected
        deadline = time.monotonic()+3
        while not messages and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert messages and messages[0].point_step == 16
        assert messages[0].header.frame_id == meta["depth_frame"]
        assert viewer.selected == 7
    finally:
        executor.shutdown()
        if viewer:
            viewer.server.shutdown()
            viewer.destroy_node()
        if observer:
            observer.destroy_node()
        rclpy.shutdown()
