"""Contract test for ``speech.launch.py`` launch-argument declarations.

Regression this pins down: 92b5a8b (2026-10-30, a manual merge-conflict
resolution) dropped nine ``DeclareLaunchArgument`` blocks while keeping
tts_node's ``LaunchConfiguration(...)`` references.  ROS 2 only fails when the
Node action's parameters are actually expanded, so ``ros2 launch ... --show-args``
still passed.  The default ``tts:=true`` launch then died with
``launch configuration 'tts_speaker' does not exist`` and took the whole speech
chain (capture/ASR/wake/TTS/playback) down with it.

The check is static and needs no ROS runtime: every ``LaunchConfiguration("x")``
in the file must have a matching ``DeclareLaunchArgument("x")``.
"""

import ast
from pathlib import Path

LAUNCH = Path(__file__).parents[1] / "launch" / "speech.launch.py"

#: Defaults must mirror tts_node.declare() so the launch layer is a no-op unless
#: the operator overrides them.
TTS_DEFAULTS = {
    "tts_speaker": "serena",
    "tts_instruct": "",
    "qwentts_server": "",
    "qwentts_model": "",
    "qwentts_codec": "",
    "qwentts_device": "CPU",
    "qwentts_max_seq": "2048",
    "quick_spec": "",
    "quick_wav_dir": "",
}


def _tree() -> ast.Module:
    return ast.parse(LAUNCH.read_text(encoding="utf-8"))


def _first_string_arg(call: ast.Call) -> str | None:
    if call.args and isinstance(call.args[0], ast.Constant):
        value = call.args[0].value
        if isinstance(value, str):
            return value
    return None


def _names(tree: ast.Module, function: str) -> set[str]:
    return {
        name
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == function
        and (name := _first_string_arg(node)) is not None
    }


def _declared_defaults(tree: ast.Module) -> dict[str, str]:
    defaults: dict[str, str] = {}
    for node in ast.walk(tree):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
        ):
            continue
        name = _first_string_arg(node)
        if name is None:
            continue
        for keyword in node.keywords:
            if keyword.arg != "default_value":
                continue
            if isinstance(keyword.value, ast.Constant) and isinstance(
                keyword.value.value, str
            ):
                defaults[name] = keyword.value.value
    return defaults


def test_every_launch_configuration_is_declared():
    tree = _tree()
    declared = _names(tree, "DeclareLaunchArgument")
    used = _names(tree, "LaunchConfiguration")

    missing = sorted(used - declared)
    assert not missing, (
        "speech.launch.py reads LaunchConfiguration values it never declares: "
        f"{missing}. Default launch will abort with SubstitutionFailure."
    )


def test_tts_contract_defaults_match_tts_node():
    defaults = _declared_defaults(_tree())

    missing = sorted(set(TTS_DEFAULTS) - set(defaults))
    assert not missing, f"tts launch arguments lost their declarations: {missing}"

    mismatched = {
        name: (defaults[name], expected)
        for name, expected in TTS_DEFAULTS.items()
        if defaults[name] != expected
    }
    assert not mismatched, (
        "launch defaults drifted from tts_node.declare() (declared, expected): "
        f"{mismatched}"
    )
