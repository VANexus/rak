from glob import glob
import os

from setuptools import find_packages, setup

package_name = "rak_speech"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # 即时反应层素材清单(ADR-0018):check_workspace 要求 config/ 必须随包安装。
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xrak",
    maintainer_email="xrak@todo.todo",
    description="Offline speech recognition node",
    license="Apache-2.0",
    # [AI] tests_require 在现代 setuptools 失效；用 extras_require["test"] 让 colcon 走 pytest。
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "asr_node = rak_speech.asr_node:main",
            "vtn_node = rak_speech.vtn_node:main",
            "tts_node = rak_speech.tts_node:main",
            # [AI] wake event DOA -> RViz arrow (sensors.launch.py single view)
            "wake_doa_viz = rak_speech.wake_doa_viz:main",
            # [AI] speaker id: /audio -> /speech/speaker. Online clustering,
            # no enrolment profiles -- labels are session-scoped (S1, S2, ...).
            "speaker_node = rak_speech.speaker_node:main",
        ]
    },
)
