from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions


LAUNCH_DIR = Path(__file__).parents[1] / "launch"


def load_launch_module(filename):
    spec = spec_from_file_location(filename.replace(".", "_"), LAUNCH_DIR / filename)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_context(**values):
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def test_selector_defaults_to_gemini_and_exposes_only_supported_backends():
    module = load_launch_module("depth_camera.launch.py")
    arguments = {
        action.name: action
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert perform_substitutions(
        LaunchContext(), arguments["camera"].default_value
    ) == "gemini335l"
    assert arguments["camera"].choices == ["gemini335l", "astra"]
    assert perform_substitutions(
        LaunchContext(), arguments["serial_number"].default_value
    ) == "CP2G853000G0"


def test_selector_dispatches_to_astra_without_gemini_arguments():
    module = load_launch_module("depth_camera.launch.py")
    context = launch_context(camera="astra", serial_number="ignored")

    with (
        patch.object(
            module, "get_package_share_directory", return_value="/share/rak_bringup"
        ),
        patch.object(module, "PythonLaunchDescriptionSource") as source_factory,
        patch.object(module, "IncludeLaunchDescription") as include_factory,
    ):
        module._launch_selected_camera(context)

    assert source_factory.call_args.args[0] == (
        "/share/rak_bringup/launch/astra.launch.py"
    )
    assert dict(include_factory.call_args.kwargs["launch_arguments"]) == {}


def test_selector_dispatches_to_gemini_and_forwards_serial_number():
    module = load_launch_module("depth_camera.launch.py")
    context = launch_context(camera="gemini335l", serial_number="camera-serial")

    with (
        patch.object(
            module, "get_package_share_directory", return_value="/share/rak_bringup"
        ),
        patch.object(module, "PythonLaunchDescriptionSource") as source_factory,
        patch.object(module, "IncludeLaunchDescription") as include_factory,
    ):
        module._launch_selected_camera(context)

    assert source_factory.call_args.args[0] == (
        "/share/rak_bringup/launch/gemini335l.launch.py"
    )
    assert dict(include_factory.call_args.kwargs["launch_arguments"]) == {
        "serial_number": "camera-serial",
    }


def test_gemini_adapter_targets_official_330_launch_with_sensor_contract():
    module = load_launch_module("gemini335l.launch.py")
    context = launch_context(
        camera_name="camera",
        serial_number="camera-serial",
        enable_point_cloud="false",
        config_file_path="",
    )

    with (
        patch.object(
            module,
            "get_package_share_directory",
            side_effect=lambda package: f"/share/{package}",
        ),
        patch.object(module, "PythonLaunchDescriptionSource") as source_factory,
        patch.object(module, "IncludeLaunchDescription") as include_factory,
    ):
        module._include_orbbec_driver(context)

    assert source_factory.call_args.args[0] == (
        "/share/orbbec_camera/launch/gemini_330_series.launch.py"
    )
    assert dict(include_factory.call_args.kwargs["launch_arguments"]) == {
        "camera_name": "camera",
        "serial_number": "camera-serial",
        "enable_point_cloud": "false",
        "enable_colored_point_cloud": "false",
        "config_file_path": "/share/rak_bringup/config/gemini335l.yaml",
    }


def test_gemini_profile_matches_verified_335l_capabilities():
    config = yaml.safe_load(
        (LAUNCH_DIR.parent / "config" / "gemini335l.yaml").read_text()
    )

    # The upstream launch merges YAML over launch arguments.
    assert "enable_point_cloud" not in config
    assert config["uvc_backend"] == "v4l2"
    assert config["color_width"] == 1280
    assert config["color_height"] == 800
    assert config["color_fps"] == 30
    assert config["color_format"] == "RGB"
    assert config["color.image_raw.enable_pub_plugins"] == ["image_transport/raw"]
    assert config["color_preset"] == ""
    assert config["depth_width"] == 1280
    assert config["depth_height"] == 800
    assert config["depth_fps"] == 30
    assert config["depth_format"] == "Y16"
    assert config["depth.image_raw.enable_pub_plugins"] == ["image_transport/raw"]
    assert config["depth_registration"] is False
    assert config["enable_hardware_noise_removal_filter"] is True
    assert config["enable_noise_removal_filter"] is False
    assert config["enable_sync_output_accel_gyro"] is True
