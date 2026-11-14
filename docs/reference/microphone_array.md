# 讯飞 XFM-DP 六麦克风阵列 —— 实测记录(2026-10-10)

> 状态：现行 · 最后更新：2026-10-30

`src/rak_speech`、`src/rak_bringup/launch/speech.launch.py` 依赖的事实都在这里。
换阵列或换板子时,先重跑本文命令再改代码。

## USB / ALSA 事实(实测)

```bash
lsusb                     # 2207:0001 Fuzhou Rockchip / XFM-DP-V0.0.18
cat /proc/asound/cards    # 2 [XFMDPV0018]: USB-Audio - XFM-DP-V0.0.18
cat /proc/asound/card2/stream0
```

- 声卡 **id 是 `XFMDPV0018`**,索引号(当前 2)插拔会变,所以设备一律写
  `hw:CARD=XFMDPV0018`(ALSA 按 id 解析),不要写 `hw:2,0`。
- UAC 流:capture only,Interface 2 Altset 1,**S16_LE / 1 通道 / 16000 Hz**
  ASYNC,数据包间隔 1000 µs。`/proc/asound/card2/` 下只有 `pcm0c`,**没有播放**,
  阵列不能当喇叭用。
- 设备共 3 个 USB 接口:`1.0` = vendor-specific(class `ff/42`,无驱动绑定)、
  `1.1` = audio control、`1.2` = audio streaming(`snd-usb-audio`)。
  **六路原始麦克风不走 UAC**:主机只能拿到模块侧处理完的单声道,波束成形/回声
  消除发生在模块内。要拿原始 6 路得逆向 vendor 接口的私有协议,不属于标准
  ROS 2 方案,本仓库不做。
- PipeWire 把它识别成 source `XFM-DP-V0.0.18 Mono`,但默认源是 Built-in Audio。
  采集节点用 `alsasrc` **直连 ALSA**,不经过 PipeWire,不受桌面路由影响。

> 旧文档(`rak_speech/audio.py` 早期注释)写过"vendor 接口未初始化时采集端点返回
> `EIO`,必须先初始化"。2026-10-10 实测**不复现**:`arecord` 与 GStreamer 均可
> 直接打开,无任何 vendor 初始化步骤。若再见 `EIO`,先查设备是否被别的进程占用。

## 标准 ROS 2 音频管线(2026-10-11 起:采集 + 唤醒 + 播报)

```
阵列 UAC(16k/1ch/S16) --ALSA--> audio_capture --/audio--> asr_node(Vosk)
                                                        -> /speech/transcript
                                                        -> /speech/intent
阵列板 vtn.sock --adb forward--> vtn_node --> /speech/wake(WakeEvent)
                                        --> /speech/vtn_ready(latched Bool)
/speech/say(Say 服务) --> tts_node(sherpa-onnx) --/audio_out--> audio_play --> USB 喇叭
```

- `audio_capture` 参数:`src=alsasrc dst=appsink format=wave device=hw:CARD=XFMDPV0018`
  `sample_rate=16000 channels=1 depth=16 sample_format=S16LE`。
  `format=wave` + `dst=appsink` 发布的是**裸 PCM**(`audio_common_msgs/AudioData`,
  `uint8[] data`,无 WAV 头);接文件 sink 时才由 `wavenc` 加头。
- `/audio_info` 用 **transient_local** QoS 发布,读取要带对应 QoS:
  `ros2 topic echo /audio_info --qos-durability transient_local --once`
- 一键启动:`ros2 launch rak_bringup speech.launch.py`
  (`capture:=false` 只起 ASR 消费别人发的 `/audio`;`wake/tts/playback:=false`
  分别关掉唤醒桥、合成、播报)。
- 备用直采路径(不需要 audio_common):`ros2 launch rak_speech asr.launch.py
  source:=arecord device:=hw:CARD=XFMDPV0018`。

## 实测性能:识别跟不上实时是常态

| 项目 | 实测值(2026-10-10,10 W 模式 + GNOME 桌面) |
| --- | --- |
| 采集 | 100 msg/s(10 ms/帧),非静音,peak≈0.07(安静房间) |
| Vosk `vosk-model-small-cn-0.22` 解码 | **0.84–1.01× 实时**(两段真实中文语音,空载) |
| 模型加载 | 0.56 s |

解码慢于实时意味着流必然跑在识别前面。`asr_node` 用 `AudioInbox`
(按**字节**设预算,默认 10 s = `audio_queue_seconds` 参数)承接:超预算才淘汰
**最旧**音频并打节流警告,短命令因此一个字节都不丢,只是转写晚不足 1 s。
若长期出现 `recogniser slower than the audio stream` 警告,按顺序考虑:
停掉桌面(`sudo scripts/rak-mode.sh headless`)、锁频(`jetson-clocks.service`)
换更快的识别后端。

## R818 降噪板:唤醒/方位/控制(vtn,2026-10-11 实测)

板子 = 讯飞 AIUI-RK3328-VTN(串号 `bc00143000108801ad0`),系统进程链
`/etc/init.d/vtn_init → watch → demo`。板上**没有 TTS**(字符串里只有时区数据
"St_Kitts" 的巧合命中),`manual_wakeup` 控制消息会被 demo 接收(`aiui.bp` 日志)
但**不产生** ivw 事件——想出唤醒事件只能真说话。

- 主机访问板子的唯一通道:`adb forward tcp:9100 localfilesystem:/tmp/vtn.sock`
  (udev 规则 `51-iflytek-adb.rules` 保证免密)。`vtn_node` 自己维护隧道与重连。
- 帧协议(《3_协议手册-R818降噪板.pdf》,与 demo 逐字节实测):
  `A5 | 01 | 类型 | 长度(LE u16) | 消息ID(LE u16) | 数据 | 校验(~sum+1)`;
  类型 `0x01` 握手(板→主机,须回 `0xFF` 确认,消息ID回显)、`0x04` 设备消息
  (AIUI 事件 JSON,唤醒在 `content.info` 内嵌 JSON 的 `ivw.angle/beam/score/keyword`)、
  `0x05` 主控消息(主机→板,UTF-8 JSON:`{"type":"verison"}`、
  `{"type":"wakeup_keywords","content":{"keyword":"...","threshold":"500"}}`)。
  实现:`src/rak_speech/rak_speech/vtn_protocol.py`(单测覆盖黄金帧 + 真实事件)。
- 接口:`/speech/wake`(`rak_interfaces/WakeEvent`:keyword/score/angle/beam/
  physical/power/start_ms/end_ms)、`/speech/vtn_ready`(latched Bool,握手成功=true)。
  **方位只在唤醒时报一次**(固件行为),没有持续方位流;angle 是环形阵自身坐标系,
  不是机器人航向。2026-10-11 全链路实测:真人唤醒 → 握手确认 → `/speech/wake`
  出事件(angle=90° beam=1 score=509)。
- 可视化:`wake_doa_viz`(sensors.launch `mic:=true` 自带,或单独
  `ros2 run rak_speech wake_doa_viz`)把最新 angle 以 10 Hz **连续重绘**成
  `/wake_doa` Marker——当前方位箭头不随事件消失(超过 `stale_sec` 只是变淡),
  最近 `trail_size` 次唤醒画成由暗到亮的轨迹箭头;绕着机器人边走边喊唤醒词,
  轨迹会画出你走过的圈。Marker 带 1.5 s lifetime:节点活着就一直刷新,节点
  死了箭头自行消失。独立小视图(不依赖 TF):`rviz2 -d
  $(ros2 pkg prefix rak_bringup)/share/rak_bringup/config/doa_rviz.rviz`
  (Fixed Frame=mic_array_link,俯视)。
- 唤醒词现状:`hei4 rui4 ke4`,阈值 500(板上 res 资源,留档在
  `acoustic/config/res*.bin`)。改词用板上打包工具,或 `vtn_node` 的
  `push_keywords` 参数下发 `wakeup_keywords` 控制 JSON(默认空 = 不碰板上配置)。
- 板端 `user_cfg.ini` 的 `debug` 必须是 `0`(= 不录音);板时钟不准(Jan 1 起步),
  `WakeEvent.header.stamp` 一律用主机时刻。

## 声纹识别(2026-10-17,在线聚类版)

```
/audio(16k 单声道) --silero VAD--> 3D-Speaker CAM++(sherpa-onnx,CPU 单线程)
    --> 在线聚类(余弦阈值,会话内质心) --> /speech/speaker(JSON:
        {"speaker":"S1","score":0.83,"new_cluster":false})
```

- **不要档案,实时识别**:不做预注册、不落盘。每段语音的嵌入与内存中的簇质心
  比对——相似(≥`threshold`)就投给该簇并更新质心,否则开新簇;标签 S1/S2/…
  在节点生命周期内稳定,重启归零。`max_speakers`(默认 8)封顶,超限时弱匹配
  折进最近簇,标签不膨胀。
- 模型(不进 git,`acoustic/models/speaker/`):
  `3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx`(~28MB)+
  `silero_vad.onnx`(~2MB)。搜索**优先 `speaker/` 子目录**,并排除 cosyvoice 等
  模型包内自带的同名 `campplus.onnx`(无 sherpa 元数据,会让加载报
  "Unknown model type")。
- 开销:模型 ~28MB、常驻 RSS ~78MB、单核 <5%、每窗 20–60ms;身份在开口 ~1s 后
  出现,持续说话中每 0.8s 刷新,停嘴即过期。参数:`threshold`(默认 0.5)、
  `max_speakers`、`eval_every_sec`、`window_sec`。
- 准确率边界:远场+阵列降噪后的单声道;2–5 人效果最好;同一个人换距离/嗓音
  可能分叉成两个簇(重启节点即重置)。这是"这几次说话是不是同一个人"的在线
  归纳,不是公安级声纹库比对。
- 一键启动:`speech.launch.py` 的 `speaker:=true`(默认),需要 `capture:=true`
  才有 `/audio`。rviz 里身份由 `wake_doa_viz` 合并进扇形状态文本
  (`S1 135 deg wake`)。

## TTS 与喇叭(2026-10-11 实测)

- **喇叭 = USB C-Media 声卡**(`0d8c:0012`,插在 Jetson 上),不是阵列的模拟
  喇叭端子;阵列 UAC 是 capture only(上文),板端 audiocodec 只服务板内 AIUI。
  播放设备写 `plughw:CARD=Device`(按 id,`plughw` 让 ALSA 负责重采样/声道转换)。
  mixer `PCM` 默认 38%(-23 dB)开,音量用 `amixer -c 0 cset` 调。
- `tts_node`(`/speech/say`,`rak_interfaces/Say`):sherpa-onnx 离线合成,回调只
  入队(合成在工作线程),`/audio_out` 发布裸 PCM(`AudioData`);`audio_play`
  (audio_common,`format=wave`)接 `alsasink`。实测:kokoro-multi-lang-v1_1
  24 kHz/103 声库/中英双语,加载 ~4 s,**合成约 0.75× 实时**(4 线程);
  vits-zh-aishell3 备选 8 kHz、约 3.2× 实时(`-p model_dir:=` 切换,
  同时把 launch 的 `tts_sample_rate` 改成 8000)。
- TTS 模型在 `acoustic/models/`(不进 git),来自
  `github.com/k2-fsa/sherpa-onnx/releases/tags/tts-models`;sherpa-onnx 为 pip
  安装(无 rosdep 键),缺失时 `tts_node` 降级(服务拒答,进程不崩)。
- 实测服务链:`ros2 service call /speech/say ... "{text: 你好}"` → accepted →
  `/audio_out` 收到 24 kHz 裸 PCM → C-Media `Status: Running`。


```bash
source /opt/ros/jazzy/setup.bash && source install/architecture/local_setup.bash
ros2 launch rak_bringup speech.launch.py          # 终端 1
ros2 topic hz /audio                              # ~100 msg/s
ros2 topic echo /audio_info --qos-durability transient_local --once
ros2 topic echo /speech/transcript                # 对着阵列说话
ros2 topic echo /speech/vtn_ready --qos-durability transient_local --once
ros2 topic echo /speech/wake                      # 说「hei rui ke」→ WakeEvent(带角度)
ros2 topic hz /wake_doa                           # 唤醒过一次后 ~10 Hz(连续重绘)
ros2 topic echo /speech/speaker                   # 声纹:{"speaker","score","new_cluster"}
ros2 service call /speech/say rak_interfaces/srv/Say "{text: 你好}"   # 喇叭出声
```

无声卡日志时的旁路检查:向 `/audio` 注入一段 16 kHz 单声道 S16 的 WAV 裸数据,
应能在 `/speech/transcript` 看到转写 —— 这只验证话题→识别段,不验证声学段;
声学段(阵列拾音)必须真人说话验收。
