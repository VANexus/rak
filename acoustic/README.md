# acoustic —— 声学(语音)工作区

机器人「听」和「说」相关的所有东西都放这里:麦克风阵列、唤醒、识别、方位、播报。

- **硬件**:讯飞/轮趣 R818 降噪板(6 麦环形阵列,实物装在雷达上)+ Jetson
- **对应需求**:②ASR　③唤醒　④TTS　⑤声音克隆　⑥声纹　⑦方位

> **正式链路在 ROS 包 `src/rak_speech` 里**(2026-10-11 起):
> `speech.launch.py` 组合 `audio_capture → /audio → asr_node`(识别)、
> `vtn_node`(板端唤醒+方位 → `/speech/wake`)、`tts_node → /audio_out →
> audio_play`(离线 TTS 播报)。协议与实测事实见 `docs/microphone_array.md`。
> 本目录的 `scripts/` 是**工程调试/commissioning 工具**,不进生产管线;
> `config/` 是留档,`models/` 是模型本体(不进 git)。

---

## 目录说明

### `scripts/` —— 调试工具脚本

| 脚本 | 干嘛的 | 怎么用 |
|---|---|---|
| `voice_setup.sh` | 开机后检查板子接好没、声音通路对不对 | `bash voice_setup.sh` |
| `vtn_probe.py` | 探测板子:握手、下发控制 JSON(协议真源) | `python3 vtn_probe.py` |
| `vtn_angle.py` | 读「声源方位」(谁在说话、在哪个方向) | `python3 vtn_angle.py` |
| `rec.py` | 把一段 wav 录音变成文字(离线识别) | `python3 rec.py 录音.wav` |
| `live.py` | 边听边出字(实时识别) | `arecord ... \| python3 live.py` |

> 生产链路请用 `ros2 launch rak_bringup speech.launch.py`;这些脚本只把结果
> 打印到屏幕,不写文件,用于绕开 ROS 单独验证板子/模型。
> `vtn_probe.py` 的帧协议已被 `src/rak_speech/rak_speech/vtn_protocol.py`
> 正式实现并有单元测试;脚本保留作协议对照和裸板探测。

### `models/` —— 声学模型本体(**不进 git**)

| 目录 | 用途 |
|---|---|
| `sherpa-onnx-streaming-zipformer-zh-14M-2023-02-23/` | ASR(Vosk 的备选,`live.py`/`rec.py` 用) |
| `kokoro-multi-lang-v1_1/` | **TTS 默认**:24 kHz、中英双语、103 声库(`tts_node` 自动选中) |
| `vits-zh-aishell3/` | TTS 低延迟备选:8 kHz、约 3× 实时(`-p model_dir:=` 切换) |

### `config/` —— 配置留档

| 文件 | 干嘛的 |
|---|---|
| `51-iflytek-adb.rules` | 让电脑认得降噪板的规则。**真正生效的那份在** `/etc/udev/rules.d/`,这份只是留档,换机器照着装 |
| `res.bin.bak` | 降噪板的唤醒词资源(历史版本,改之前) |
| `res_now.bin` | 降噪板的唤醒词资源(改之后) |

> `res*.bin` 内容互不相同,是不同版本。想回退旧唤醒词时用得上,别删。

---

## 当前状态(2026-10-11)

| 项 | 状态 |
|---|---|
| 唤醒词 | `hei4 rui4 ke4`,阈值 500(板上 res 资源) |
| 方位 | **只在唤醒时报一次**(板子固件决定);「持续方位」还在研究 |
| ASR | Vosk(topic 源)为生产路径;sherpa-onnx 可作备选后端 |
| TTS | `tts_node` + kokoro(24 kHz 中英),`/speech/say` 服务;合成约 0.75× 实时 |
| 喇叭 | USB C-Media 声卡,`plughw:CARD=Device`,`audio_play` 播 `/audio_out` |

---

## 千万别动

- **别删 `models/`** —— 删了识别/合成就废了
- **别删 `/etc/udev/rules.d/51-iflytek-adb.rules`** —— 删了电脑认不到降噪板
- 降噪板上 `/data/build/user_cfg.ini` 的 `debug` 必须是 `0`(= 不录音)
