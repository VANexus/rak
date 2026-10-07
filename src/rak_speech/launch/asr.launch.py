"""Offline speech recognition. See rak_speech/asr_node.py for the model search."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "source", default_value="arecord",
                description="wav (a file) or arecord (a live capture device).",
            ),
            DeclareLaunchArgument("wav_path", default_value=""),
            DeclareLaunchArgument(
                "device", default_value="default",
                description="ALSA capture device. The iFlytek array needs vendor init first.",
            ),
            DeclareLaunchArgument(
                "model_dir", default_value="",
                description="Vosk model. Empty searches brain/data/models/vosk-model-*.",
            ),
            Node(
                package="rak_speech",
                executable="asr_node",
                name="speech_asr",
                output="screen",
                parameters=[{
                    "source": LaunchConfiguration("source"),
                    "wav_path": LaunchConfiguration("wav_path"),
                    "device": LaunchConfiguration("device"),
                    "model_dir": LaunchConfiguration("model_dir"),
                }],
            ),
        ]
    )
