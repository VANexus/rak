"""Full-chain voice -> think -> act stack (P1, DESIGN DRAFT in
docs/brain_arm_tools_design.md until its ADR lands):

    mic (ALSA) -> audio_capture -> /audio -> asr_node -> /speech/transcript
    harness_node (agent loop subprocess, Claude Agent SDK)
      -> /tool_proposals -> tool_executor -> handlers (say / query / task / arm)
      -> /tool_results -> back into the loop
    loop final text -> /speech/say -> tts_node -> /audio_out -> speaker
    thinking / tool traces -> /harness/trace (String JSON, for the operator)

Requires the vendored SDK overlay (tools/harness-sdk) and, for the cloud path,
ANTHROPIC_API_KEY + ANTHROPIC_BASE_URL in the environment of this launch.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

MIC_DEFAULT = "plughw:0,0"          # Astra camera mic (array board not fitted)
SPEAKER_DEFAULT = "plughw:CARD=Device"
SDK_PATH = "/home/xrak/rak/tools/harness-sdk"


def generate_launch_description() -> LaunchDescription:
    def arg(name: str, default: str, description: str) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(name, default_value=default, description=description)

    mic = LaunchConfiguration("mic_device")
    speaker = LaunchConfiguration("speaker_device")
    capture = LaunchConfiguration("capture")
    wake = LaunchConfiguration("wake")
    allow_motion = LaunchConfiguration("allow_motion")
    motion_tools = LaunchConfiguration("motion_tools")
    voice_path = LaunchConfiguration("voice_path")
    sdk_path = LaunchConfiguration("agent_sdk_path")
    model = LaunchConfiguration("model")
    asr_engine = LaunchConfiguration("asr_engine")
    tts_backend = LaunchConfiguration("tts_backend")

    tool_executor = Node(
        package="rak_tool_executor",
        executable="tool_executor",
        name="tool_executor",
        output="screen",
        parameters=[{"allow_motion": allow_motion}],
    )
    harness = Node(
        package="rak_harness",
        executable="harness_node",
        name="harness",
        output="screen",
        parameters=[
            {
                "voice_path": voice_path,
                "agent_sdk_path": sdk_path,
                "agent_allow_motion_tools": motion_tools,
                "cloud_model": model,
            }
        ],
    )

    return LaunchDescription([
        arg("mic_device", MIC_DEFAULT, "ALSA capture device for ASR (array board id when fitted)."),
        arg("speaker_device", SPEAKER_DEFAULT, "ALSA playback device for TTS."),
        arg("capture", "true", "false: consume an already-published /audio topic."),
        arg("wake", "false", "true: vtn_node (needs the R818 array board)."),
        arg("allow_motion", "false",
            "tool_executor hardware gate: arm.home/task.goto execute only when true."),
        arg("motion_tools", "false",
            "expose motion-class tools to the agent loop (executor still gates each call)."),
        arg("asr_engine", "zipformer", "ASR engine: zipformer | vosk."),
        arg("tts_backend", "qwentts",
            "TTS backend: qwentts (qwentts.cpp + serena, 默认) | sherpa | cosyvoice."),
        arg("voice_path", "agent", "agent (Claude Agent SDK loop) | dialogue (ADR-0010 engine only)."),
        arg("agent_sdk_path", SDK_PATH, "vendored claude-agent-sdk overlay directory."),
        arg("model", "deepseek-chat", "model name passed to the loop (endpoint-dependent)."),
        # Speech stack (capture + ASR + TTS + playback), wake off without the board.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("rak_bringup") + "/launch/speech.launch.py"
            ),
            launch_arguments={
                "device": mic, "speaker_device": speaker,
                "capture": capture, "wake": wake,
                "engine": asr_engine, "tts_backend": tts_backend,
            }.items(),
        ),
        tool_executor,
        harness,
    ])
