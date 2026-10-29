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
| `sherpa-onnx-streaming-zipformer-zh-14M-2023-02-23/` | ASR 默认(流式 zipformer,`engine:=zipformer`) |
| `kokoro-multi-lang-v1_1/` | TTS 备选(`backend:=sherpa`):24 kHz、中英双语、103 声库 |
| `vits-zh-aishell3/` | TTS 低延迟备选:8 kHz、约 3× 实时(`-p model_dir:=` 切换) |
| `Fun-CosyVoice3-0.5B-2512/` + `cosyvoice_prompt_zh.wav` | TTS GPU 备选(`backend:=cosyvoice`,已证内存不可行) |
| `quick/` | **即时反应层素材**(ADR-0018):`<key>.wav`,由 `scripts/make_quick_phrases.py` 用 serena 预合成 |

> TTS 默认路线(`backend:=qwentts`)的模型不在这里:是 `tools/qwentts.cpp/models/` 下的
> `qwen-talker-0.6b-customvoice-Q4_K_M.gguf` + `qwen-tokenizer-12hz-Q8_0.gguf`(GGUF)。

### `config/` —— 配置留档

| 文件 | 干嘛的 |
|---|---|
| `51-iflytek-adb.rules` | 让电脑认得降噪板的规则。**真正生效的那份在** `/etc/udev/rules.d/`,这份只是留档,换机器照着装 |
| `res.bin.bak` | 降噪板的唤醒词资源(历史版本,改之前) |
| `res_now.bin` | 降噪板的唤醒词资源(改之后) |

> `res*.bin` 内容互不相同,是不同版本。想回退旧唤醒词时用得上,别删。

---

## 当前状态(2026-10-17)

| 项 | 状态 |
|---|---|
| 唤醒词 | `hei4 rui4 ke4`,阈值 500(板上 res 资源) |
| 方位 | **只在唤醒时报一次**(板子固件决定);「持续方位」还在研究 |
| ASR | 流式 zipformer 14M 为默认(`engine:=zipformer`);Vosk 为 `engine:=vosk` 回退 |
| TTS | **三层(ADR-0018)**:即时层播 `models/quick/*.wav`(<100ms,无推理);内容层 `backend:=qwentts`(qwentts.cpp + serena,首帧 ~1s,整句 RTF 3.5~5.6);超预算降级到即时素材 |
| 喇叭 | USB C-Media 声卡,`plughw:CARD=Device`,`audio_play` 播 `/audio_out` |

## 即时反应层素材(ADR-0018 层1)

```bash
python3 scripts/make_quick_phrases.py            # 全量重新生成(serena,逐条 1~20s)
python3 scripts/make_quick_phrases.py --only ack_wake,hold --force
```

清单是 `src/rak_speech/config/quick_phrases.toml`(key 对齐人格卡模板,情感档取自
`src/rak_harness/config/persona.toml`);改完文案/语气重跑一次即可,`tts_node` 启动时
按文案精确匹配。素材缺失时即时层静默关闭(内容层照常),所以换机器后没生成也能跑。

---

## 千万别动

- **别删 `models/`** —— 删了识别/合成就废了
- **别删 `/etc/udev/rules.d/51-iflytek-adb.rules`** —— 删了电脑认不到降噪板
- 降噪板上 `/data/build/user_cfg.ini` 的 `debug` 必须是 `0`(= 不录音)
