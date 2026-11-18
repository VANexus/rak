"""Composition contract for ``robot.launch.py`` (ADR-0020 / ADR-0026).

Regression this pins down: ``stage:=full`` included ``speech.launch.py`` a second
time even though ``sensors.launch.py`` already includes it under ``mic:=true``.
That produced two audio_capture processes fighting for the same ALSA device and
duplicate /speech_asr, /speech_tts, /speech_vtn nodes and /speech/say services.

The shot is static: ``_launch_setup`` is executed with ``_include`` stubbed, so
no process is started and no package share needs to exist.
"""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

LAUNCH_DIR = Path(__file__).parents[1] / "launch"


def _load_robot_launch():
    path = LAUNCH_DIR / "robot.launch.py"
    spec = spec_from_file_location("robot_launch_for_test", path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(module, **overrides):
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations.update(overrides)
    return context


def _includes(module, **overrides):
    recorded = []

    def fake_include(name, arguments, package="rak_bringup"):
        recorded.append((name, dict(arguments)))
        return name

    context = _context(module, **overrides)
    with patch.object(module, "_include", fake_include):
        module._launch_setup(context)
    return recorded


def test_full_stage_has_exactly_one_speech_chain_owner():
    recorded = _includes(_load_robot_launch(), stage="full", mic="true")

    names = [name for name, _ in recorded]
    assert names.count("sensors.launch.py") == 1
    assert "speech.launch.py" not in names, (
        "sensors.launch.py already includes speech.launch.py under mic:=true; "
        "including it again starts a second full speech chain"
    )


def test_full_stage_keeps_expression_only_when_mic_chain_exists():
    module = _load_robot_launch()

    with_mic = dict(_includes(module, stage="full", mic="true"))
    domains = with_mic["supervisor.launch.py"]["domains"]
    assert "expression" in domains
    assert "mic" in domains

    without_mic = dict(_includes(module, stage="full", mic="false"))
    domains = without_mic["supervisor.launch.py"]["domains"]
    assert "expression" not in domains, (
        "expression (tts/audio_play) lives in the same speech.launch.py that "
        "sensors owns under mic; expecting it with mic:=false guarantees a "
        "spurious supervisor fault"
    )
    assert "mic" not in domains
