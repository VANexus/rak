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
- `tts_node` (rak_speech, offline) turns `/speech/say` requests into raw 24 kHz
  mono S16 PCM on `/audio_out`, in three layers (ADR-0018): a **instant layer**
  plays pre-synthesized wav assets (`acoustic/models/quick/*.wav`, keyed by the
  persona templates; also driven by `/speech/quick`) with no inference at all; a
  **content layer** synthesizes everything else with `qwentts.cpp`
  (`backend:=qwentts`, speaker `serena`, resident local server, streaming PCM);
  and a **response budget** (first frame <= `tts_first_frame_budget`) drops an
  over-budget sentence and falls back to an instant asset. `sherpa` (kokoro/vits
  CPU) and `cosyvoice` (GPU) remain selectable backends. `audio_play`
  (audio_common) feeds `/audio_out` to the USB speaker through
  `plughw:CARD=<id>` (device *id*, not index). Synthesis is queued: the service
  callback only enqueues (the instant layer is the exception -- it must fire now).

The array does beamforming on board; its UAC interface already carries the
processed single channel, so no host-side beamforming is involved. Measured
device detail and verification commands: docs/microphone_array.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#: Card *id*, not index: replug must not renumber the array out from under us.
ARRAY_DEVICE = "hw:CARD=XFMDPV0018"
#: USB speaker (C-Media). plughw lets ALSA resample/rate-convert for us.
SPEAKER_DEVICE = "plughw:CARD=Device"
#: kokoro-multi-lang-v1_1 / qwentts.cpp both emit 24 kHz; audio_play must
#: declare the same rate.
TTS_SAMPLE_RATE = "24000"
#: Repo-local qwentts.cpp runtime (gitignored tools/). Empty = node discovers it.
QWENTTS_SERVER = ""


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
                "engine",
                default_value="zipformer",
                description="ASR engine: zipformer (streaming 14M, sherpa-onnx) | vosk.",
            ),
            DeclareLaunchArgument(
                "tts_backend",
                default_value="qwentts",
                description=(
                    "TTS backend: qwentts (qwentts.cpp + serena, offline, 默认) | "
                    "sherpa (kokoro CPU) | cosyvoice (GPU, emotional)."
                ),
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
                "tts_sid",
                default_value="0",
                description=(
                    "kokoro voice id (0=af_heart default; Chinese voices: "
                    "zf_xiaobei/zf_xiaoni/zf_xiaoxiao/zf_zhishu, zm_* male). "
                    "vits-aishell3 has a single voice and ignores this."
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
                "tts_speaker",
                default_value="serena",
                description="qwentts 音色(qwentts.cpp CustomVoice speaker;2026-10-17 定稿 serena)。",
            ),
            DeclareLaunchArgument(
                "tts_instruct",
                default_value="",
                description="qwentts 默认语气档(instruct);空=节点默认「清晰自然、亲切温和」。",
            ),
            DeclareLaunchArgument(
                "qwentts_server",
                default_value=QWENTTS_SERVER,
                description="qwentts.cpp 的 tts-server 可执行文件;空=仓库内 tools/qwentts.cpp/build/tts-server。",
            ),
            DeclareLaunchArgument(
                "qwentts_model",
                default_value="",
                description="talker GGUF;空=自动挑 tools/qwentts.cpp/models/qwen-talker-*-customvoice-*.gguf(Q4_K_M 优先)。",
            ),
            DeclareLaunchArgument(
                "qwentts_codec",
                default_value="",
                description="codec GGUF;空=自动挑 tools/qwentts.cpp/models/qwen-tokenizer-*.gguf。",
            ),
            DeclareLaunchArgument(
                "qwentts_device",
                default_value="CPU",
                description="qwentts 计算设备:CPU | CUDA0(GPU 路径待上游修复 qwentts.cpp#40)。",
            ),
            DeclareLaunchArgument(
                "qwentts_max_seq",
                default_value="2048",
                description="qwentts KV cache 长度(=QT_MAX_SEQ);4096≈896MB,短句场景 1024~2048 足够。",
            ),
            DeclareLaunchArgument(
                "quick_spec",
                default_value="",
                description="即时反应层素材清单;空=仓库内 src/rak_speech/config/quick_phrases.toml。",
            ),
            DeclareLaunchArgument(
                "quick_wav_dir",
                default_value="",
                description="即时反应层 wav 目录;空=仓库内 acoustic/models/quick(缺素材即时层静默关闭)。",
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
                        "engine": LaunchConfiguration("engine"),
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
                        "backend": LaunchConfiguration("tts_backend"),
                        "sid": ParameterValue(
                            LaunchConfiguration("tts_sid"), value_type=int
                        ),
                        "speaker": LaunchConfiguration("tts_speaker"),
                        "instruct": LaunchConfiguration("tts_instruct"),
                        "qwentts_server": LaunchConfiguration("qwentts_server"),
                        "qwentts_model": LaunchConfiguration("qwentts_model"),
                        "qwentts_codec": LaunchConfiguration("qwentts_codec"),
                        "qwentts_device": LaunchConfiguration("qwentts_device"),
                        "qwentts_max_seq": ParameterValue(
                            LaunchConfiguration("qwentts_max_seq"), value_type=int
                        ),
                        "quick_spec": LaunchConfiguration("quick_spec"),
                        "quick_wav_dir": LaunchConfiguration("quick_wav_dir"),
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
