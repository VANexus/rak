"""Standard microphone-array speech pipeline (capture, wake, ASR, TTS, playback).

    ros2 launch rak_bringup speech.launch.py
    ros2 topic echo /speech/transcript
    ros2 topic echo /speech/wake
    ros2 service call /speech/say rak_interfaces/srv/Say "{text: 你好}"

Four components, each individually switchable (capture/wake/tts/playback args):

- `audio_capture` (audio_common, the standard ROS 2 audio stack) reads the
  iFlytek XFM-DP six-mic array through GStreamer/ALSA straight from the hardware
  -- deliberately *not* through PipeWire, whose default source is the built-in
  microphone and whose routing is a desktop concern. `format=wave` with
  `dst=appsink` publishes **raw** 16 kHz mono S16 PCM (no WAV header) as
  `audio_common_msgs/AudioData` on `/audio`; `/audio_info` states that contract
  and `/audio_stamped` carries the measured rate for diagnostics.
- `asr_node` (rak_speech, Vosk, offline) consumes that topic and publishes
  `/speech/transcript` plus the deterministic `/speech/intent` the task layer
  parses. It never sends a goal: rak_task_executor owns execution.
- `vtn_node` (rak_speech) bridges the array's on-board wake/DOA engine (R818
  降噪板,讯飞 VTN, adb-forwarded unix socket) to `/speech/wake`
  (rak_interfaces/WakeEvent) and latched `/speech/vtn_ready`. The board is the
  only wake-word authority; it reports sound-source angle **only together with
  a wake event** (firmware behaviour, docs/microphone_array.md).
- `tts_node` (rak_speech, sherpa-onnx, offline) turns `/speech/say` requests
  into raw 24 kHz mono S16 PCM on `/audio_out`; `audio_play` (audio_common)
  feeds that to the USB speaker through `plughw:CARD=<id>` (device *id*, not
  index). Synthesis is queued: the service callback only enqueues.

The array does beamforming on board; its UAC interface already carries the
processed single channel, so no host-side beamforming is involved. Measured
device detail and verification commands: docs/microphone_array.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: Card *id*, not index: replug must not renumber the array out from under us.
ARRAY_DEVICE = "hw:CARD=XFMDPV0018"
#: USB speaker (C-Media). plughw lets ALSA resample/rate-convert for us.
SPEAKER_DEVICE = "plughw:CARD=Device"
#: kokoro-multi-lang-v1_1 emits 24 kHz; audio_play must declare the same rate.
TTS_SAMPLE_RATE = "24000"


def generate_launch_description():
    device = LaunchConfiguration("device")
    audio_topic = LaunchConfiguration("audio_topic")
    model_dir = LaunchConfiguration("model_dir")
    tts_model_dir = LaunchConfiguration("tts_model_dir")
    tts_sample_rate = LaunchConfiguration("tts_sample_rate")
    speaker_device = LaunchConfiguration("speaker_device")
    vtn_push_keywords = LaunchConfiguration("vtn_push_keywords")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device",
                default_value=ARRAY_DEVICE,
                description="ALSA capture device of the microphone array.",
            ),
            DeclareLaunchArgument(
                "audio_topic",
                default_value="/audio",
                description="Raw-PCM AudioData topic between capture and ASR.",
            ),
            DeclareLaunchArgument(
                "model_dir",
                default_value="",
                description="Vosk model. Empty searches <repo>/data/models/vosk-model-*.",
            ),
            DeclareLaunchArgument(
                "capture",
                default_value="true",
                description="false: consume an already-published audio topic.",
            ),
            DeclareLaunchArgument(
                "wake",
                default_value="true",
                description="false: no vtn_node (no /speech/wake from the array).",
            ),
            DeclareLaunchArgument(
                "tts",
                default_value="true",
                description="false: no tts_node (no /speech/say synthesis).",
            ),
            DeclareLaunchArgument(
                "playback",
                default_value="true",
                description="false: no audio_play (nothing plays /audio_out).",
            ),
            DeclareLaunchArgument(
                "tts_model_dir",
                default_value="",
                description=(
                    "sherpa-onnx TTS model. Empty searches "
                    "<repo>/acoustic/models/{kokoro-*,vits-*}."
                ),
            ),
            DeclareLaunchArgument(
                "tts_sample_rate",
                default_value=TTS_SAMPLE_RATE,
                description=(
                    "Sample rate tts_node publishes; audio_play declares it so "
                    "GStreamer can negotiate. kokoro=24000, vits-aishell3=8000."
                ),
            ),
            DeclareLaunchArgument(
                "speaker_device",
                default_value=SPEAKER_DEVICE,
                description="ALSA playback device of the speaker (by card id).",
            ),
            DeclareLaunchArgument(
                "vtn_push_keywords",
                default_value="",
                description=(
                    "Optional JSON control pushed to the board once after "
                    "handshake, e.g. {\"type\":\"wakeup_keywords\",\"content\":"
                    "{\"keyword\":\"hi rak\",\"threshold\":\"500\"}}. "
                    "Empty = keep the board's own configuration."
                ),
            ),
            Node(
                package="audio_capture",
                executable="audio_capture_node",
                name="audio_capture",
                output="screen",
                condition=IfCondition(LaunchConfiguration("capture")),
                parameters=[
                    {
                        "src": "alsasrc",
                        "dst": "appsink",  # publish to ROS instead of a file
                        "format": "wave",  # with appsink: raw PCM, no WAV header
                        "device": device,
                        "sample_rate": 16000,  # what the Vosk model expects
                        "channels": 1,
                        "depth": 16,
                        "sample_format": "S16LE",
                    }
                ],
                remappings=[("audio", audio_topic)],
            ),
            Node(
                package="rak_speech",
                executable="asr_node",
                name="speech_asr",
                output="screen",
                parameters=[
                    {
                        "source": "topic",
                        "audio_topic": audio_topic,
                        "model_dir": model_dir,
                    }
                ],
            ),
            Node(
                package="rak_speech",
                executable="vtn_node",
                name="speech_vtn",
                output="screen",
                condition=IfCondition(LaunchConfiguration("wake")),
                parameters=[
                    {
                        "manage_forward": True,  # adb tunnel is owned by the node
                        "push_keywords": vtn_push_keywords,
                    }
                ],
            ),
            Node(
                package="rak_speech",
                executable="tts_node",
                name="speech_tts",
                output="screen",
                condition=IfCondition(LaunchConfiguration("tts")),
                parameters=[
                    {
                        "model_dir": tts_model_dir,
                        "audio_topic": "/audio_out",
                    }
                ],
            ),
            Node(
                package="audio_play",
                executable="audio_play_node",
                name="audio_play",
                output="screen",
                condition=IfCondition(LaunchConfiguration("playback")),
                parameters=[
                    {
                        "format": "wave",  # raw PCM in (no WAV header, no mp3)
                        "device": speaker_device,
                        "sample_rate": tts_sample_rate,
                        "channels": 1,
                        "depth": 16,
                        "sample_format": "S16LE",
                        "dst": "alsasink",
                    }
                ],
                remappings=[("audio", "/audio_out")],
            ),
        ]
    )
