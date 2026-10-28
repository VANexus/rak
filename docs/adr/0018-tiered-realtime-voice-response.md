# ADR-0018: 语音响应分层——即时反应层 + 分句流式 + 响应预算

- 状态: 提议中
- 日期: 2026-10-17
- 相关: `src/rak_speech/`（`tts_node.py`、`cosyvoice_backend.py`）、ADR-0010（呼吸感表达接口）、
  `tools/qwentts.cpp/`（板上 TTS runtime，`scripts/bench_*.py` 为选型依据）、
  memory `tts-selection-bench-2026-10-16`

## 背景

语音是"生命感"（ADR-0010 的目标）的主要出口，但 2026-10-16/17 的三模型选型 bench
（详见 `debug/cosyvoice_bench/` 与 memory）显示：在 3.4GB Orin Nano 上，
**离线 TTS 的整句 RTF 无法低于 ~3.5**：

| 路线 | 整句 RTF | TTFA（首帧出声） | 结论 |
|---|---|---|---|
| MOSS-TTS-Nano ONNX（CPU/TRT） | 3.5~4.1 | —（无流式） | TRT 已证无收益（每步 kernel 启动延迟主导） |
| CosyVoice3 TRT | 物理不可行 | — | Myelin 常量区超板载内存（memory 已存档） |
| **Qwen3-TTS 0.6B（qwentts.cpp，定稿路线）** | **3.5~5.6** | **0.55~1.7s** | GPU 路径待上游修复（qwentts.cpp#40），修复后预期 ~2 |

硬约束：Tegra 统一内存 3.4GB，torch 轮子不支持 sm87（memory 已存档），GPU 修复时间不可控。

关键洞察：**"生命感"的感知锚点是应答起始延迟（TTFA），不是整句合成完的时限**。
对 RTF <1 的执念会把架构往错误方向优化——真正缺的是对突发输入的瞬时反应能力。

## 决策

`rak_speech` 的语音输出采用**三层响应架构**，`tts_node` 按此重构（对接 ADR-0010 呼吸感提案接口）：

1. **即时反应层（<100ms）**：预合成一批短语气素材（"嗯？"、"来了！"、"哈哈"、"呜……"等，
   serena 音色、常用情感档位各一组），落盘缓存。收到 Harness 表情/反馈提案时直接播 wav，
   **不经过 TTS 推理**。素材集由配置文件声明，走 `/speech/quick` 之类的独立话题。
2. **内容层（TTFA ≤1.5s）**：真正要说的句子走 qwentts.cpp 流式合成（`--stream-by-line`）。
   **一次只合成一句**，句间自然停顿 ≥0.5s；长内容拆句排队，不空等整段。
3. **响应预算（写进节点参数）**：即时层 <100ms；首帧 ≤1.5s；单句合成超时丢弃并降级为即时层素材。
   预算不达标属于故障路径，不是调参空间。

TTS 定稿配置（2026-10-17 用户拍板）：`qwentts.cpp` + `--speaker serena` +
instruct 语气档（默认"清晰自然、亲切温和"；情感句按内容切换 instruct），
输出 `plughw:2,0`（USB 声卡），PCM 音量 14/37。

## 理由

- RTF 3.5~5.6 + 流式 TTFA 0.55s 的组合，在"短句 + 即时层兜底"下听感等效于实时：
  生命感 60% 来自即时层的语气反馈，内容层只需接得住（实测来源：本 ADR 背景表）。
- qwentts.cpp 是唯一同时满足离线、instruct 情感控制、流式、CPU 可跑的路线；
  CosyVoice/Ming 被 torch 无 sm87 + 内存墙堵死（来源：2026-10-16 bench）。
- MOSS TRT 引擎虽 8/8 构建成功但 RTF 无收益（batch-1 自回归每步开销主导），
  证明"给 AR-TTS 上 TRT"在此类负载上是反模式，故 TRT-Edge-LLM 只保留为
  DAC/CodePredictor 两段的 Plan B（见 ADR 待闭合项）。

## 后果

正面：
- 100ms 级的瞬时反应能力，不依赖任何模型推理，GPU 修复前后都成立。
- 长句断流被句间停顿掩盖，用户体验可预期。
- 响应预算显式化，回归测试有判据。

负面（代价）：
- 即时层素材是**离线预录的**，语义覆盖率有限——超出素材集的快速反应只能退化为内容层延迟。
  素材维护是持续成本。
- qwentts.cpp 绑定第三方 fork（ServeurpersoCom），上游 sm87 bug（issue #40）未修复期间
  GPU 加速不可用，RTF 受制于 CPU；需跟踪上游。
- 一次一句的节奏在"连续讲解"类内容上会显得机械，需要 Harness 在文案层配合（句长控制）。
- `tts_node` 重构期间，现 CosyVoice backend（已证不可行）与 qwentts.cpp 路线并存一个过渡期。

## 被否决的方案

- **等 GPU 修复后再设计**：本架构对 GPU 修复是透明的（修复只是让内容层 RTF 降，
  分层与预算不变），且即时层完全不依赖它——不构成等待理由。
- **TRT-Edge-LLM 全模型迁移**：Talker 的 AR 逐步解码在 TRT 下收益为负（MOSS 实测
  RTF ≈ 1.0 持平 CPU），且 3.4GB 上重新验证 Myelin 内存是已知深坑（CosyVoice 教训）。
- **云端 TTS 兜底**：语音是本体能力，依赖网络与"赋予灵魂"的定位冲突；且 Harness 云链路
  已有 12~30s 延迟（ADR-0010），叠加不可接受。
- **继续压 CPU RTF（6 线程/XNNPACK 等）**：实测无收益或更差（swap 干扰），每帧 ORT/GGML
  调用开销是地板，不值得再投入。

## 待闭合项

- [ ] `tts_node` 三层重构 + `/speech/quick` 接口（新会话实施）
- [ ] 即时层素材集：清单、情感档位、录制/合成流程
- [ ] ADR-0010 呼吸感提案 → 即时层/内容层的映射规则
- [ ] 上游 issue #40（sm87 GET_ROWS）跟踪；修复后验证 GPU 路径（预期 RTF ~2），
      并重新评估是否需要 DAC/CodePredictor 两段的 TRT-Edge-LLM Plan B
- [ ] qwentts.cpp fork 的长期维护策略（锁定 commit + 定期同步，或上游迁移）
