# 新队友上线：从邀请到第一个请求

> 状态：现行 · 最后更新：2026-11-16

这份文档给**第一次进项目的人**：照做一遍，就能自己拿到模型网关的 key、把仓库构建起来、
分清哪台机器干什么，并且知道那些"不在 GitHub 上"的东西是什么、去哪拿。

建议顺序：§1 配模型 → §3 起环境 → §2 接工具 → §4 认基础设施 → §5 出问题再查。

> 文中地址与状态均在 **2026-11-16 从主力机实测**，每节给了自检命令，别只信文字。
> **本文不含任何密钥、密码、token。** 凭据只写"找谁要、放哪"，见 §4.4。

---

## 1. 配模型：从邀请到第一个请求

### 1.1 拿到邀请并登录

网关：**<https://litellm.xrak.top/ui>**（同一主机的 `/v1` 就是 API 入口）。

流程：管理员发邀请链接 → 打开链接自己设密码 → 登录。团队是**邀请制**，没有公开注册，
收不到邀请就找管理员，不要试"注册"。

- 首次登录可能报 `password_reset_required`
  （提示 `This account's password must be changed before the session can be used`）。
  这不是密码错，是必须先改密码：打开 **<https://litellm.xrak.top/ui/change-password>**
  改完再登录。
- 普通成员的角色是 `internal_user`：**只能用自己的 key 调放行的模型，看不到管理员面板**。
  加模型、加人、提额度都找管理员。
- 当前实例是免费档，**用户上限 5 个**。第 6 个人登不进来不是你的问题，请管理员腾位置。

自检（不需要登录）：

```bash
curl -sS -o /dev/null -w '%{http_code}\n' https://litellm.xrak.top/health/liveliness   # 期望 200
```

### 1.2 建一个自己的 key

UI 左侧 **Virtual Keys → Create Key**。约定：

- **一台机器一个 key、一个工具一个 key**（比如 `shizurak-claude-code`、`orin-agent`），
  泄漏时只吊销一个，日志里也分得清是谁。
- key 完整内容一般**只显示一次**，当场复制走；丢了就删掉重建，不要找管理员"捞回来"。
- 不要借用别人的 key。

### 1.3 key 放哪

放**仓库外**、权限 600、从一个固定文件 export：

```bash
install -d -m 700 ~/.config/rak
cat > ~/.config/rak/litellm.env <<'EOF'
export LITELLM_API_KEY='sk-你自己新建的key'
export OPENAI_BASE_URL='https://litellm.xrak.top/v1'   # 认这个变量的工具直接可用；SDK 仍要显式传 base_url
EOF
chmod 600 ~/.config/rak/litellm.env

# 需要时加载（set -a 让变量自动 export）
set -a; . ~/.config/rak/litellm.env; set +a
```

`~/.config/rak/litellm.env` 在仓库目录之外，`git status` 永远看不到它；这也意味着
**换机器要重新建一次**，不要把它拷进任何仓。

### 1.4 第一个请求

```bash
set -a; . ~/.config/rak/litellm.env; set +a

# ① 这个 key 能调哪些模型：返回的 data[].id 就是 model 名，别猜
curl -sS https://litellm.xrak.top/v1/models \
  -H "Authorization: Bearer $LITELLM_API_KEY" | python3 -m json.tool | head -40

# ② 最小请求（model 换成上一步返回的某个 id）
curl -sS https://litellm.xrak.top/v1/chat/completions \
  -H "Authorization: Bearer $LITELLM_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{"model":"<model-id>","messages":[{"role":"user","content":"ping"}],"max_tokens":16}'
```

Python 等价写法：

```python
import os
from openai import OpenAI

client = OpenAI(base_url="https://litellm.xrak.top/v1",
                api_key=os.environ["LITELLM_API_KEY"])
resp = client.chat.completions.create(
    model="<model-id>",
    messages=[{"role": "user", "content": "ping"}],
    max_tokens=16,
)
print(resp.choices[0].message.content)
```

判定：

| 结果 | 含义 |
|---|---|
| 200 且响应里有 `choices` | 通了 |
| 401 `Authentication Error` | 头没带对，或 key 被删/禁用/额度用完 |
| 404 且 `model not found` | 模型名不在你这个 key 的放行清单里，回 ① 重抄 |
| 404 且提示路由 / 路径不存在 | base_url 多/少 `/v1`，见 §1.5 |

**模型名只从 `/v1/models` 取**，不要从别的项目配置里搬（例如
`src/rak_agent/config.toml` 里写的是 SiliconFlow 的模型名，在本网关不一定放行）。

### 1.5 base_url 到底填什么（最容易错的一处）

| 客户端协议 | base_url | 实际打到 |
|---|---|---|
| OpenAI 兼容（多数 IDE 插件、CLI、OpenAI SDK） | `https://litellm.xrak.top/v1` | `/v1/chat/completions` |
| Anthropic 兼容（Claude Code 等） | `https://litellm.xrak.top` | `/v1/messages` |
| Responses API | `https://litellm.xrak.top/v1` | `/v1/responses` |

规则一句话：**OpenAI 系带 `/v1`，Anthropic 系不带。** 多一个或少一个 `/v1` 就是 404。
网关同时开了 `/v1/messages`、`/v1/responses`、`/v1/embeddings`、`/v1/rerank`
（2026-11-16 探测：这些路径未带 key 均返回 401，说明路由存在）。

---

## 2. 工具侧配置

### 2.1 通用三件事

1. base_url 按 §1.5 填。
2. key 走环境变量或工具自己的 secret 存储，**不进仓库、不进截图、不进工单**。
3. 模型名从 `/v1/models` 抄，别猜。

### 2.2 常见工具对照

| 工具 | 在哪配 | 关键字段 |
|---|---|---|
| VS Code（Cline / Roo / Kilo / Continue 等） | 扩展设置里选 `OpenAI Compatible` | Base URL = `https://litellm.xrak.top/v1`；API Key = 你的 key；Model = `/v1/models` 里的 id |
| JetBrains 插件（Continue / 自定义 provider） | Settings → Tools → 对应插件 | 同上，选 OpenAI 兼容 + base URL + key |
| CherryStudio 等桌面客户端 | 设置 → 模型服务 → OpenAI | API 地址、key 同上 |
| Claude Code（命令行） | 环境变量（`~/.bashrc` 或启动前 export） | `ANTHROPIC_BASE_URL=https://litellm.xrak.top`、`ANTHROPIC_AUTH_TOKEN=$LITELLM_API_KEY`、`ANTHROPIC_MODEL=<放行的模型 id>` |
| Codex CLI | `~/.codex/config.toml` | 见下方片段 |
| 自己写代码 | OpenAI SDK | `base_url=.../v1`，`api_key=os.environ[...]` |

Codex CLI 片段：

```toml
model = "<model-id>"
model_provider = "rak"

[model_providers.rak]
name = "RAK LiteLLM"
base_url = "https://litellm.xrak.top/v1"
env_key = "LITELLM_API_KEY"
wire_api = "chat"
```

> 各客户端版本字段名会变，但核心永远只有两项：**base_url 和 key**。界面里找不到
> "OpenAI 兼容"选项的，就是那个工具没开这个能力，换工具而不是硬改。

### 2.3 仓库里已有的模型入口

`src/rak_agent/config.toml` 的 `[openai]` 段是 RAI 的 LLM 配置：

- `base_url` 可以改成 `https://litellm.xrak.top/v1`；
- 模型名改成 `/v1/models` 里的 id；
- **key 永远不写进这个文件**——按该文件注释，RAI 的 `ChatOpenAI` 不显式传 key，
  读的是环境变量 `OPENAI_API_KEY`：

```bash
export OPENAI_API_KEY="$LITELLM_API_KEY"
```

> 这条组合（RAI + 网关）尚未在本机验证（2026-11-16）。第一次改完先按 §1.4 验 key，
> 再用 §3.4 的 stub 确认环境本身是好的，不要把两件事混在一起排。

### 2.4 绝对不能提交的东西

- `sk-` 开头的任何 key；Harbor / GitLab / GitHub 的密码或 PAT；VNC、sudo 口令；私钥
  （`id_ed25519*`、`*.pem`、`*.key`）。
- 项目目录里的 `.env*`、`config.local.*`、`~/.config/rak/litellm.env` 的任何副本。
- 带 key 的截图、录屏、日志。
- `data/`、`tools/`、`debug/`、`build/`、`install/`、`log/`（已在 `.gitignore`，
  不要用 `git add -f` 绕过）。

提交前扫三条：

```bash
git status --short
git diff --cached | grep -nE 'sk-|password|secret|token|BEGIN .*PRIVATE KEY' || true
git grep -nE 'sk-[A-Za-z0-9_-]{16,}' || true
```

真泄漏了：UI 里删 key → 重建 → 告知管理员查用量日志。**只删文件不换 key 不算处置**。

---

## 3. 环境上手

### 3.1 两台机器，各干各的

| | 本机（x86 桌面，Shizurak） | orin（Jetson，`192.168.5.215`） |
|---|---|---|
| 跑什么 | 上层逻辑、编排、RViz、夹具、模型客户端 | 驱动、传感器、CAN 主站 |
| ROS | `lyrical`（`ls /opt/ros` 实测） | `jazzy` |
| 构建档 | `desktop` | `jetson` |
| 硬件 | **没有**雷达 / 深度相机 / `can0` | 全部真硬件 |

三条硬规矩：

- **桌面机不能直插传感器**：在本机 `ros2 launch rak_bringup lidar.launch.py` 不会出数据，
  这是设计不是故障。分工与实测证据见 [hardware_ownership.md](../hardware_ownership.md)。
- **源码只在本机改**。orin 上那份是**部署副本**：只 pull / 同步，不 commit；现场急修
  事后必须回合到本机，否则下次同步就被覆盖。
- 机器本身的基线（SSH 只允许密钥、默认无头、日志封顶）见
  [machine_baseline.md](../reference/machine_baseline.md)。

### 3.2 克隆

三个仓都在 GitHub 私有组织 `VANexus` 下，需要 org 邀请 + SSH key 或 PAT：

```bash
mkdir -p ~/work/code && cd ~/work/code
git clone git@github.com:VANexus/rak.git
git clone git@github.com:VANexus/vehical_rak_motion.git
git clone git@github.com:VANexus/vehical_rak_edge_ai.git
```

关于"含子仓"：

- 这里**没有 git submodule**。三个仓是并列的独立仓，"带子仓克隆"指的是**并列 checkout**，
  不是 `git clone --recursive`。
- 跨仓门禁（几何一致性、双板漂移、协议）要求三仓**并列在同一父目录**；
  `tools/ci_local.sh` 会检查每个目录下的 `.git`，缺一个直接 `exit 2`。
- 伞形仓（工作区根，放 `docs/`、`tools/`、`env/` 和跨仓门禁 workflow）本身也是 git 仓，
  但 2026-11-16 实测它**没有配置 `origin`**。要跑跨仓门禁就向维护者要一份完整目录，
  别指望 `git clone`。只做 `rak/` 的开发不受影响。

### 3.3 本机最小构建

```bash
cd <你的工作区>/rak          # 必须是仓库根；不要在家目录跑 colcon
ls /opt/ros                  # 本机是 lyrical；文档里出现 jazzy 是在讲 orin
source /opt/ros/lyrical/setup.bash

python3 scripts/build_workspace.py --profile desktop
source install/local_setup.bash
python3 scripts/check_workspace.py --installed
```

- 档位是唯一事实源：`src/rak_orchestration/config/build_profiles.yaml`（ADR-0026）。
  `desktop` 档会跳过 GPU / 厂商 SDK / 传感器包，所以本机**没有**
  `rak_astra_driver`、`lslidar_driver` 是正常的。
- 只想少建几个包时，把 colcon 参数放在 `--` 后面透传：

```bash
python3 scripts/build_workspace.py --profile desktop -- --packages-up-to rak_agent
```

- 依赖安装用 rosdep（系统包管理，不建 venv）：

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro "$ROS_DISTRO"
```

  两个只在 PyPI 的依赖（`vosk`、`urdf_parser_py`）在 `rosdep/rak-extra.yaml`，
  注册方式看该文件头注释。

### 3.4 启动最小闭环

```bash
source install/local_setup.bash
ros2 launch rak_bringup agent.launch.py decider:=stub sim:=true
```

`stub` 不需要模型也不需要 key——**它跑起来只证明环境是好的**，与模型网关无关。
这条通了再去接真模型。

传感器的正确姿势：在 orin 上起节点，本机只订阅。DDS 约定是
`ROS_DOMAIN_ID=47` + `rmw_cyclonedds_cpp`，网卡钉在局域网口
（本机 `wlp0s20f3`，orin `wlP1p1s0`），配置在
[cyclonedds.xml](../../src/rak_bringup/config/cyclonedds.xml)；
`source install/local_setup.bash` 时由 env-hook 自动设置 `CYCLONEDDS_URI`，不用手动 export。
测试一律用独立 domain（77 / 187），不要混进 47 的运行图。

### 3.5 门禁

```bash
python3 scripts/check_docs.py                # 文档：链接 / 索引 / 状态行
python3 scripts/check_workspace.py --installed
```

CI 的跨仓门禁在伞形仓 `.github/workflows/gates.yml`，需要一个仓库 secret
`ORG_REPO_TOKEN`（对三个子仓有 read 权限的 PAT）。

**缺这个 secret 时 job 仍然是绿的**：第一步会把"跨仓门禁未执行：缺少 ORG_REPO_TOKEN"
写进 job 运行摘要并打 warning annotation。看到这句话，就把那轮当作没跑，而不是通过。

本地等价物（不要 token，但要求三仓并列）：

```bash
cd <伞形仓根> && ./tools/ci_local.sh --no-build
```

---

## 4. 不在 GitHub 上的东西

这一节是本仓库之外的事实：内网地址、DNS 行为、集群、凭据渠道和团队约定。
**它们是新人最容易踩的部分，且看代码看不出来。**

### 4.1 网络分三层

| 层 | 网段 / 地址 | 承载 | 用来做什么 |
|---|---|---|---|
| 局域网 | `192.168.5.0/24`：本机 `.182`（`wlp0s20f3`）、orin `.215`（`wlP1p1s0`）、网关 `.1` | 直连 | DDS / 传感器 / 大流量、`ssh orin.xrak` |
| netmaker | `100.121.213.0/24`：本机 `.6`、集群 `.2`/`.4`/`.5` | WireGuard 覆盖网 | 管集群、访问服务、跨机运维 |
| 公网入口 | `nm.xrak.top` / `api.nm.xrak.top` → `8.129.26.180` | netmaker 服务器 | 不在局域网时接回内网 |

- **传感器 / DDS 不走 netmaker**：隧道 MTU 1420、多播也不自然，而两台机器本来就同网段。
- 旧文档里 orin 的 netmaker 地址写作 `100.121.213.20`；2026-11-16 实测**不可达**
  （无路由 / 丢包），当前可靠路径是 `192.168.5.215` / `orin.xrak`。地址会变，
  先 `ping`、`ip route get` 再下结论。
- `ssh xrak@orin.xrak` **只允许密钥**（`PasswordAuthentication no`）。口令登不进去不是故障。

### 4.2 DNS 与 fake-IP（最容易误判的一节）

- 路由器 DNS（`192.168.5.1`）对**所有**域名都返回 `198.18.0.0/15` 里的假地址，TTL=1：
  `dig +short harbor.xrak.top` → `198.18.1.4`，`dig +short github.com` → `198.18.1.34`。
  这是本机透明代理的 **fake-IP，不是真实服务器地址**。
- 主力机 `/etc/hosts` 把主要服务钉到了 `100.121.213.5`
  （`litellm` / `gitlab` / `grafana` / `minio` / `kas` / `registry` / `hydra.ng` / `auth-api.ng`），
  所以这些域名在本机是直连集群；`harbor`、`langfuse` 没钉，走代理也通。
- 因此：
  - **不要把 `198.18.x.x` 写进任何配置、`/etc/hosts` 或镜像构建**，换个网络就废。
  - 新机器 / 容器里"服务打不开"，顺序是：本机 `/etc/hosts` → DNS → 服务本身。
  - `ping 198.18.x.x` 有回应不代表服务在；`curl -sS -o /dev/null -w '%{http_code}'` 才算数。
- 自检：

```bash
getent hosts gitlab.xrak.top      # 100.121.213.5（被 hosts 钉死）
getent hosts harbor.xrak.top      # 198.18.x.x（fake-IP，走代理）
curl -sS -o /dev/null -w '%{http_code} %{remote_ip}\n' https://gitlab.xrak.top   # 302 100.121.213.5
```

### 4.3 集群里有什么

k3s 三节点（2026-11-16 `kubectl get nodes`）：

| 节点 | 内网地址 | 角色 |
|---|---|---|
| `rak-core` | `100.121.213.4` | control-plane + etcd |
| `rak-cloud` | `100.121.213.5` | 工作节点（对外服务都指向这个 IP） |
| `rak-socl` | `100.121.213.2` | 工作节点 |

对外服务（同一次实测）：

| 域名 | 是什么 | 什么时候用 | 当时状态 |
|---|---|---|---|
| `litellm.xrak.top` | 模型网关 | 所有人 | health 200 |
| `gitlab.xrak.top` | 自建 GitLab（仓库 / 议题 / CI / registry） | 看代码、建镜像 | 302（登录页） |
| `harbor.xrak.top` | 镜像仓库 | 构建 / 部署 | `/v2/` 401（未认证的正常响应） |
| `grafana.xrak.top` | 监控面板 | 看延迟 / 资源 | `/api/health` 200 |
| `minio.xrak.top` | S3 兼容对象存储 | 数据 / 模型 / 产物 | health 200 |
| `langfuse.xrak.top` | LLM 调用追踪 | 调 prompt、看 trace | health 200（当天部分组件在重启，功能可能不全） |
| `hydra.ng.xrak.top`、`auth-api.ng.xrak.top` | 自建认证（OAuth2 / OIDC） | 平台登录 | hydra 在跑；auth-api 根路径 500，遇到先找管理员 |

平台侧组件（不供日常开发直接使用，知道名字即可）：`cert-manager`（证书）、
`external-dns`（域名）、`traefik`（ingress）、`longhorn`（存储）、
`kyverno` / `trivy`（策略 / 扫描）、`velero`（备份）、`keda`、`kafka`、`milvus`、
`clickhouse`、`cnpg`（PostgreSQL）、`monitoring`、`security`。

集群访问要 kubeconfig（找集群维护者）；本机已装 `kubectl`，context 名是 `rak`。

### 4.4 凭据找谁要

| 要什么 | 找谁 | 存在哪 | 注意 |
|---|---|---|---|
| LiteLLM 邀请 / 提额 / 加用户 | 网关管理员 | UI 自助建 key，本机 `~/.config/rak/litellm.env` | 免费档 5 人；不共用 key |
| GitHub 组织 `VANexus` 权限 | 仓库管理员 | SSH key / PAT | 只申请需要的仓 |
| GitLab / Harbor 账号、Harbor 项目权限 | 集群维护者 | `~/.docker/config.json`、GitLab 个人设置 | Harbor 私有项目要显式加成员或 robot account |
| kubeconfig | 集群维护者 | `~/.kube/config` | 别把 kubeconfig 拷进仓库 |
| orin 的 SSH 公钥登记 / VNC / sudo | 机器负责人 | `~/.ssh/authorized_keys`、VNC 客户端 | 口令当面拿，不写进文档 / 群聊 |
| `ORG_REPO_TOKEN`（CI） | 伞形仓管理员 | GitHub repo Settings → Secrets | 缺了门禁会跳过，见 §3.5 |

凭据只走"发放渠道 → 本机私有配置"，**不进仓库、不进工单、不贴截图**。

### 4.5 本地约定（不知道就会踩）

1. **单一事实源**：协议在 MCU 仓（`CANOPEN-V5.md` + `docs/canopen/*.json`），
   构建档位在 `build_profiles.yaml`，agent 规则只在 `AGENTS.md`，文档索引只在
   [docs/README.md](../README.md)。同一事实不在多处各写一份。
2. **单主站纪律**：同一条 CAN 总线永远只能有一个 CANopen 主站（见 `AGENTS.md` 开头）。
   本机没有 `can0`，不要在桌面另起主站或第二个 `controller_manager`。
3. **开发机优先**：上层逻辑在本机写、本机跑；硬件节点在 orin 起；不要在 orin 上改源码。
4. **`colcon` 只在 `~/rak` 根跑**。在家目录跑会砸出 `~/build`、`~/install`、`~/log`，
   还会把别的仓卷进 discovery。
5. **测试域隔离**：运行图是 `ROS_DOMAIN_ID=47`；测试用 77 / 187。
6. **文档也是代码**：新文档要有状态行、进 `docs/README.md` 索引、相对链接可达，
   `python3 scripts/check_docs.py` 必须绿；改文档与改代码同一批提交。
7. **数据与模型资产不入库**：`data/`（episodes、`data/models/` 的 engine / tokenizer）、
   `tools/`、`debug/` 都在 `.gitignore`。fresh clone 不会有，找维护者要或自己导出。
8. **"构建通过" ≠ "能跑" ≠ "验收通过"**：硬件相关一律按
   [hardware_acceptance.md](../reference/hardware_acceptance.md) 的门槛读。

---

## 5. 排错对照

| 现象 | 先查 | 原因 / 处置 |
|---|---|---|
| 登录网关被拒 | 是否受邀、是否改过密码 | 没邀请 → 管理员；`password_reset_required` → `/ui/change-password` 改完重登；5 人上限 → 管理员腾位置 |
| `401 Authentication Error, No api key passed in` | 请求头 | 少了 `Authorization: Bearer <key>`，或 key 里带了空格 / 换行 |
| `401` 但头看着对 | key 本身 | 被删 / 禁用 / 额度用完；先用 `/v1/models` 单测，再换新 key |
| `404 model not found` | model 名 | 从 `/v1/models` 抄；可能不在你这个 key 的放行清单 |
| `404` 且路径可疑 | base_url | OpenAI 系带 `/v1`，Anthropic 系不带（§1.5） |
| `429` | 额度 / 限流 | 等一会儿或找管理员提额 |
| 镜像拉取 `401 Unauthorized` | `docker login` / imagePullSecret | Harbor 项目私有：用自己账号或 robot account 登录；k8s 里要配 `imagePullSecrets`；裸访问 `/v2/` 返回 401 是正常的 |
| 容器里解析不到 `*.xrak.top` | §4.2 | 容器没有本机的 hosts / fake-IP 环境；构建时显式 `--add-host`，或问集群内地址 |
| `gitlab.xrak.top` 打开是登录页 | — | 正常（302）；`git clone` 需要 PAT 或 SSH key |
| 跨机看不到话题 | domain / RMW / 网卡 | 两边 `ROS_DOMAIN_ID=47`、`rmw_cyclonedds_cpp` 一致；已 `source install/local_setup.bash`（env-hook 会设 `CYCLONEDDS_URI`）；`ping 192.168.5.215`；不要用 netmaker 网段跑 DDS |
| 本机 launch 雷达 / 相机没数据 | 机器分工 | 传感器只在 orin，见 §3.1 |
| `Failed to find a free participant index for domain 47` | 进程数 / 配置 | 配置里 `MaxAutoParticipantIndex=100` 已兜底；仍报说明 `CYCLONEDDS_URI` 没生效（没 source overlay 或被手动覆盖） |
| 家目录出现 `build/ install/ log/` | colcon 跑错位置 | 删掉，回到 `~/rak` 按 §3.3 跑 |
| CI 绿但跨仓门禁没跑 | job 运行摘要 | 缺 `ORG_REPO_TOKEN` 的 warning；补 secret 或本地跑 `ci_local.sh` |
| `check_docs.py` 失败 | 脚本输出 | 链接目标不存在 / 新文档没进索引 / 缺状态行 |
| orin 连不上 | 路径 | 先 `ping 192.168.5.215`；netmaker 地址可能已变；SSH 口令登录必然失败（只允许密钥） |

---

## 6. 一页自检

```bash
# 网关活着
curl -sS https://litellm.xrak.top/health/liveliness; echo

# 你的 key 能用
set -a; . ~/.config/rak/litellm.env; set +a
curl -sS -o /dev/null -w '%{http_code}\n' https://litellm.xrak.top/v1/models \
  -H "Authorization: Bearer $LITELLM_API_KEY"      # 期望 200

# 集群入口（302 / 401 都是"服务在"的正常响应）
for h in gitlab grafana minio langfuse harbor; do
  printf '%-9s ' "$h"
  curl -sS -o /dev/null -w '%{http_code} %{remote_ip}\n' --max-time 6 "https://$h.xrak.top" 2>&1 | tail -1
done

# 板子在
ping -c 1 -W 1 192.168.5.215

# 仓库是好的
cd ~/rak && python3 scripts/check_workspace.py --installed && python3 scripts/check_docs.py
```

全绿之后，按 §3.4 起 `agent.launch.py decider:=stub sim:=true`，再回头接 §1 的模型。
