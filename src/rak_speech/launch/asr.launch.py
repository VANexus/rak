"""Offline speech recognition. See rak_speech/asr_node.py for the model search.

Standalone: starts only the recogniser. For the standard microphone-array path
(audio_capture -> /audio -> this node) use `ros2 launch rak_bringup
speech.launch.py`, which composes both; this file accepts the same arguments so
a file, a capture device or a topic can be swapped in by hand.

Lifecycle (ADR-0020 §5): `SpeechAsr` is a `ManagedNode`, so a plain `Node(...)`
would leave the process silently stuck in `unconfigured` -- no model, no
subscription, no transcript, no error. This file therefore starts it through
`rak_orchestration.launch_helpers.managed`, exactly like `speech.launch.py`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from rak_orchestration.launch_helpers import managed

#: iFlytek XFM-DP six-mic array: its UAC interface gives 16 kHz mono only, and
#: the card *id* (not index) keeps the device stable across replug.
#: Measured detail: docs/reference/microphone_array.md.
ARRAY_DEVICE = "hw:CARD=XFMDPV0018"


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "source",
                default_value="arecord",
                description=(
                    "topic (audio_common AudioData), arecord (a capture device) "
                    "or wav (a file)."
                ),
            ),
            DeclareLaunchArgument("wav_path", default_value=""),
            DeclareLaunchArgument(
                "device",
                default_value=ARRAY_DEVICE,
                description="ALSA capture device for source:=arecord.",
            ),
            DeclareLaunchArgument(
                "audio_topic",
                default_value="/audio",
                description="AudioData topic for source:=topic.",
            ),
            DeclareLaunchArgument(
                "model_dir",
                default_value="",
                description=(
                    "ASR model dir. Empty searches acoustic/models/"
                    "sherpa-onnx-streaming-zipformer-* (zipformer) or "
                    "data/models/vosk-model-* (vosk)."
                ),
            ),
            DeclareLaunchArgument(
                "engine",
                default_value="zipformer",
                description="ASR engine: zipformer (streaming 14M) | vosk.",
            ),
            DeclareLaunchArgument(
                "supervised",
                default_value="false",
                description=(
                    "true = leave configure/activate to system_manager; "
                    "false = this file self-starts the managed node."
                ),
            ),
            *managed(
                package="rak_speech",
                executable="asr_node",
                name="speech_asr",
                parameters=[{
                    "source": LaunchConfiguration("source"),
                    "wav_path": LaunchConfiguration("wav_path"),
                    "device": LaunchConfiguration("device"),
                    "audio_topic": LaunchConfiguration("audio_topic"),
                    "model_dir": LaunchConfiguration("model_dir"),
                    "engine": LaunchConfiguration("engine"),
                }],
            ),
        ]
    )
