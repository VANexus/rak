"""Full-chain voice -> think -> act stack (P1, DESIGN DRAFT in
docs/brain_arm_tools_design.md until its ADR lands):

    mic (ALSA) -> audio_capture -> /audio -> asr_node -> /speech/transcript
    harness_node (agent loop subprocess, Claude Agent SDK)
      -> /tool_proposals -> tool_executor -> handlers (say / query / task / arm)
      -> /tool_results -> back into the loop
    loop final text -> /speech/say -> tts_node -> /audio_out -> speaker
    thinking / tool traces -> /harness/trace (String JSON, for the operator)

SDK overlay 的位置**不再写死**（AGENTS §2 第 3 条）：由
`rak_harness.sdk` 按 `agent_sdk_path` 参数 → `RAK_HARNESS_SDK_DIR` 环境变量 →
ament 索引 → 源码树 `tools/harness-sdk` 依次解析；找不到就降级到 dialogue 语音路径
并打印每个候选的命中情况。云端路径另需 ANTHROPIC_API_KEY + ANTHROPIC_BASE_URL。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from rak_orchestration.launch_helpers import managed

MIC_DEFAULT = "plughw:0,0"          # Astra camera mic (array board not fitted)
SPEAKER_DEFAULT = "plughw:CARD=Device"


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

    # 两者都是 ManagedNode（ADR-0020）：资源在 configure、工作线程在 activate。
    # supervised:=true 时由 system_manager 驱动；单独跑本文件时自启到 active。
    tool_executor = managed(
        package="rak_tool_executor",
        executable="tool_executor",
        name="tool_executor",
        output="screen",
        parameters=[{"allow_motion": allow_motion}],
    )
    harness = managed(
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
            arg("agent_sdk_path", "",
            "vendored claude-agent-sdk overlay; 空 = 自动解析（RAK_HARNESS_SDK_DIR → ament → 源码树）."),
        arg("supervised", "false",
            "true = 由 system_manager 驱动 harness/tool_executor 的生命周期"),
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
        # managed() 返回 [节点, configure 事件, activate 事件]，必须展开
        *tool_executor,
        *harness,
    ])
