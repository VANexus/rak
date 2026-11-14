"""CANopen v5 的 Jetson 侧契约层（ADR-0022）。

它**不是**协议栈，也不是驱动：生产链路的 NMT/SYNC/PDO/SDO 由 lely-core 承担（ADR-0021 已选）。
本模块只做一件事——把 `config/<node>.json`（下位机 SSOT）+ `canopen_contract.yaml`（通信参数
与单位）派生成**可执行的事实**：COB-ID、PDO 字节布局、速率、量化范围、活性判据、
以及 v5 内部 COB-ID 唯一性。

为什么值得单独一层：ADR-0021 记录 v4 的教训是"帧表在三处复制，出过量化因子错 10 倍"。
所以这里禁止手抄——布局只能从 JSON 算出来，单位必须带出处，名字对不上就抛异常。
它同时是明日上台架前的**唯一可离线验证部分**。
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping

import yaml

#: CiA 301 标准 COB-ID 基址（功能码 << 9），node_id 直接相加。[V5] §3 端口表
SYNC_COB_ID = 0x080
NMT_COB_ID = 0x000
EMCY_BASE = 0x080
TPDO_BASE = 0x180
RPDO_BASE = 0x200
#: SDO 的方向以**从站的 SDO 服务端**为参照（EDS `1200sub1` 原文：
#: "COB-ID client to server (rx) = 0x600+$NODEID"）。写反一个基址就是往空 ID 发请求，
#: 所以这里不用 TGA/RGA 这类简称，也不写"我们发/我们收"——只写服务端语义。
SDO_SRV_TX_BASE = 0x580  # 服务端→客户端：Jetson 在 0x580+node **收** SDO 应答
SDO_SRV_RX_BASE = 0x600  # 客户端→服务端：Jetson 往 0x600+node **发** SDO 请求
HEARTBEAT_BASE = 0x700
PDO_DISABLE_BIT = 0x80000000   # bit31 = 该 PDO 未启用（canopen_gate.py 第 4 项检查同一条）

#: JSON 里出现过的类型 → 位宽。出现表外类型直接失败，不猜。
TYPE_BITS = {
    "bool": 8, "int8": 8, "uint8": 8,
    "int16": 16, "uint16": 16,
    "int32": 32, "uint32": 32,
}

NMT_STATES = {
    0x00: "BootUp", 0x04: "Stopped", 0x05: "Operational",
    0x7F: "Pre-operational",
}
#: NMT 命令（CiA 301）：Enter Operational / Enter Pre-op / Reset node / Reset communication
NMT_COMMANDS = {"start": 0x01, "preop": 0x02, "reset": 0x04, "reset_comm": 0x05}


class ContractError(ValueError):
    """契约不自洽。消息里必须指出是哪个节点、哪个 PDO、哪个对象。"""


@dataclass(frozen=True)
class ObjectDef:
    name: str
    index: str
    type: str
    access: str
    bits: int
    offset_bits: int
    unit: str
    factor: float
    source: str
    todo: str = ""

    @property
    def raw_range(self) -> tuple[int, int]:
        return raw_limits(self.type)

    @property
    def physical_range(self) -> tuple[float, float]:
        low, high = raw_limits(self.type)
        return low * self.factor, high * self.factor

    @property
    def physical_resolution(self) -> float:
        return self.factor


def raw_limits(type_name: str) -> tuple[int, int]:
    bits = TYPE_BITS[type_name]
    if type_name.startswith("u") or type_name == "bool":
        return 0, (1 << bits) - 1
    half = 1 << (bits - 1)
    return -half, half - 1


@dataclass(frozen=True)
class PdoDef:
    node: str
    name: str                 # tpdo1 / rpdo1 …
    direction: str            # "tpdo"（板→Jetson）| "rpdo"（Jetson→板）
    cob_id: int
    transmission_type: int | None
    rate_hz: float | None
    objects: tuple[ObjectDef, ...]

    @property
    def byte_size(self) -> int:
        return sum(o.bits for o in self.objects) // 8

    @property
    def object_names(self) -> tuple[str, ...]:
        return tuple(o.name for o in self.objects)

    def object(self, name: str) -> ObjectDef:
        for item in self.objects:
            if item.name == name:
                return item
        raise ContractError(f"{self.node}.{self.name} 没有对象 {name!r}")


@dataclass(frozen=True)
class NodeContract:
    node: str
    node_id: int
    json_sha256: str
    sync_period_ms: float
    heartbeat_produce_ms: int
    consume_timeout_ms: int
    liveness: Mapping[str, Any]
    identity_sdo: Mapping[str, Any]
    pdos: tuple[PdoDef, ...]
    byte_order: str

    def pdo(self, name: str) -> PdoDef:
        for item in self.pdos:
            if item.name == name:
                return item
        raise ContractError(f"node {self.node} 没有 PDO {name!r}")

    def cob_ids(self) -> dict[int, str]:
        """该节点在总线上实际占用的 COB-ID → 语义标签。

        只列**清单里用到的** PDO：未使用的槽位 bit31=1（禁用），不占 ID —— 这正是
        canopen_gate.py 的第 4 项检查，两边必须同一个理解。
        """
        ids = {
            HEARTBEAT_BASE + self.node_id: f"{self.node}.heartbeat",
            EMCY_BASE + self.node_id: f"{self.node}.emcy",
            SDO_SRV_TX_BASE + self.node_id: f"{self.node}.sdo_srv_tx",
            SDO_SRV_RX_BASE + self.node_id: f"{self.node}.sdo_srv_rx",
        }
        for item in self.pdos:
            ids[item.cob_id] = f"{self.node}.{item.name}"
        return ids

    def find_cob(self, cob_id: int) -> PdoDef | None:
        for item in self.pdos:
            if item.cob_id == cob_id:
                return item
        return None


@dataclass
class Contract:
    nodes: dict[str, NodeContract]
    sync_cob_id: int = SYNC_COB_ID
    nmt_cob_id: int = NMT_COB_ID
    #: 装载时用的配置目录；EDS 副本就躺在同一目录里。
    #: 必须由 load() 传进来——模块自己用 `__file__` 推目录在安装态会指到
    #: `site-packages/config/`（本机实测踩过，见 default_config_dir 的注释）。
    root: Path | None = None

    def node(self, name: str) -> NodeContract:
        if name not in self.nodes:
            raise ContractError(f"未知节点 {name!r}，已知 {sorted(self.nodes)}")
        return self.nodes[name]

    # ---------------------------------------------------- 编解码（按契约，不靠记忆）
    def encode(self, node: str, pdo_name: str, values: Mapping[str, float]) -> bytes:
        """物理量 → PDO 载荷。越界**抛异常**而不是饱和：静默截断正是"看起来发了"的故障。"""
        pdo = self.node(node).pdo(pdo_name)
        buffer = bytearray(pdo.byte_size)
        for item in pdo.objects:
            if item.name not in values:
                raise ContractError(f"{node}.{pdo_name} 缺对象 {item.name!r}")
            raw = quantize(values[item.name], item)
            put_int(buffer, item.offset_bits, item.bits, raw, self.node(node).byte_order,
                    is_signed(item.type))
        return bytes(buffer)

    def decode(self, node: str, cob_id: int, payload: bytes) -> dict[str, float]:
        """PDO 载荷 → 物理量。长度不符即报错，不做"尽力解析"。"""
        contract = self.node(node)
        pdo = contract.find_cob(cob_id)
        if pdo is None:
            raise ContractError(f"node {node} 没有 COB-ID 0x{cob_id:03X} 的 PDO")
        if len(payload) < pdo.byte_size:
            raise ContractError(
                f"{node}.{pdo.name} 需要 {pdo.byte_size} 字节，实收 {len(payload)}"
            )
        out: dict[str, float] = {}
        for item in pdo.objects:
            raw = get_int(payload, item.offset_bits, item.bits, contract.byte_order,
                        is_signed(item.type))
            out[item.name] = raw * item.factor
        return out

    # ------------------------------------------------------- COB-ID 唯一性（v5 内部）
    def duplicate_cob_ids(self) -> list[tuple[int, str]]:
        """同一 COB-ID 被两个 PDO（或主站 SYNC/NMT）占用 = 总线自撞。

        2026-11-14：v4 已从本仓整体删除（无回滚分支、无帧常量），原来的
        "v4/v5 混跑碰撞"算术证明随之失效 —— 现在只剩**v5 内部**唯一性这一条。
        """
        seen: dict[int, str] = {}
        dupes: list[tuple[int, str]] = []
        for contract in self.nodes.values():
            for cob, owner in contract.cob_ids().items():
                if cob in seen:
                    dupes.append((cob, f"{seen[cob]} vs {owner}"))
                seen[cob] = owner
        for cob, owner in ((self.sync_cob_id, "master.sync"), (self.nmt_cob_id, "master.nmt")):
            if cob in seen:
                dupes.append((cob, f"{seen[cob]} vs {owner}"))
            seen[cob] = owner
        return sorted(dupes)


# ---------------------------------------------------------------- 纯函数（可单测）
def quantize(value: float, item: ObjectDef) -> int:
    low, high = item.raw_range
    raw = int(round(value / item.factor))
    if not low <= raw <= high:
        raise ContractError(
            f"{item.name}={value} 超出可表示范围 "
            f"[{item.physical_range[0]:g}, {item.physical_range[1]:g}] {item.unit}"
            f"（因子 {item.factor:g}，类型 {item.type}）"
        )
    return raw


def put_int(buffer: bytearray, offset_bits: int, bits: int, value: int, order: str,
            signed: bool) -> None:
    start, span = offset_bits // 8, bits // 8
    buffer[start:start + span] = value.to_bytes(span, order, signed=signed)


def get_int(payload: bytes, offset_bits: int, bits: int, order: str, signed: bool) -> int:
    start, span = offset_bits // 8, bits // 8
    return int.from_bytes(payload[start:start + span], order, signed=signed)


def is_signed(type_name: str) -> bool:
    """符号性来自类型本身，不靠"看起来像负数"猜 —— 猜错的代价是 32767 变 -1。"""
    return not (type_name.startswith("u") or type_name == "bool")


def pdo_rate_hz(transmission_type: int | None, sync_period_ms: float) -> float | None:
    """同步发送类型 → 速率。[SPEC]「SYNC 2 ms（500 Hz）；TPDO1 类型 5 = 每 5 个 SYNC」→100 Hz。

    0-240：每 N 个 SYNC；253：远程触发；254/255：事件驱动（无固定速率，返回 None）。
    """
    if transmission_type is None:
        return None
    if transmission_type in (254, 255):
        return None
    if transmission_type == 0:
        return None          # 同步计数 0 = 每个 SYNC 都发（主站节拍）
    if transmission_type > 254:
        raise ContractError(f"未知发送类型 {transmission_type}")
    return (1000.0 / sync_period_ms) / transmission_type


def heartbeat_state(data: bytes) -> str:
    if not data:
        raise ContractError("心跳帧为空")
    return NMT_STATES.get(data[0], f"unknown(0x{data[0]:02X})")


def command_liveness_ok(
    *, seq_changes: int, last_change_s: float, enable_bit_set: bool,
    operational: bool, window_ms: int, now_s: float | None = None,
) -> bool:
    """[V5] §5 deadman 的三条件与门：序号在窗内变过 + bit0 置位 + 节点 Operational。

    刻意写成纯函数：明日台架上它是"板子为什么不认我的命令"的第一嫌疑，
    必须能在没有硬件的情况下被测试覆盖。
    """
    del now_s  # 保留形参以便调用方显式传时钟
    if not operational:
        return False
    if not enable_bit_set:
        return False
    if seq_changes <= 0:
        return False
    return last_change_s * 1000.0 <= float(window_ms)


# ---------------------------------------------------------------- 装载
def default_config_dir() -> Path:
    """契约与 EDS 副本所在目录：安装态走 ament 索引，源码态走包目录。

    这里**不能**用 `Path(__file__).parents[1] / "config"`：源码树里它恰好对，
    装进 `share/`+`site-packages` 之后就指到 `site-packages/config/`（本机 lyrical
    实测：门禁在 `colcon build` 出来的前缀里直接报"缺少 EDS 副本"）。板子档同样是安装态。
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("rak_canopen")) / "config"
    except Exception:  # noqa: BLE001 - 没 source ROS 时按源码树跑门禁是支持的用法
        return Path(__file__).resolve().parents[1] / "config"


def load(config_dir: str | Path | None = None) -> Contract:
    root = Path(config_dir) if config_dir else default_config_dir()
    spec = yaml.safe_load((root / "canopen_contract.yaml").read_text(encoding="utf-8"))
    semantics = spec["semantics"]
    known_names = set(semantics)
    nodes: dict[str, NodeContract] = {}
    for name, node_spec in spec["nodes"].items():
        manifest = json.loads((root / node_spec["json"]).read_text(encoding="utf-8"))
        objects = {o["index"]: o for o in manifest["objects"]}
        pdos = []
        for group, members in manifest["pdo"].items():
            direction = "tpdo" if group.startswith("tpdo") else "rpdo"
            index = int(group.replace(direction, ""))
            base = TPDO_BASE if direction == "tpdo" else RPDO_BASE
            cob = base + (index - 1) * 0x100 + int(node_spec["node_id"])
            ttype = manifest.get("tpdo_types", {}).get(group) if direction == "tpdo" else None
            laid_out: list[ObjectDef] = []
            offset = 0
            for member in members:
                obj = objects.get(member)
                if obj is None:
                    raise ContractError(f"{name}.{group} 引用了清单外的对象 {member!r}")
                if obj["type"] not in TYPE_BITS:
                    raise ContractError(f"{name}.{group} 出现未知类型 {obj['type']!r}")
                sem = semantics.get(obj["name"])
                if sem is None:
                    raise ContractError(
                        f"{name}.{group} 的对象 {obj['name']!r} 没有语义条目（单位/因子/出处）"
                        " —— 禁止靠猜解释总线数值"
                    )
                bits = TYPE_BITS[obj["type"]]
                laid_out.append(ObjectDef(
                    name=obj["name"], index=obj["index"], type=obj["type"],
                    access=obj["access"], bits=bits, offset_bits=offset,
                    unit=sem["unit"], factor=float(sem["factor"]),
                    source=sem.get("source", ""), todo=sem.get("todo", ""),
                ))
                offset += bits
            total_bits = offset
            if total_bits > 64 or total_bits % 8:
                raise ContractError(
                    f"{name}.{group} 共 {total_bits} 位，超出/不齐 CANopen 的 1-8 字节 PDO"
                )
            pdos.append(PdoDef(
                node=name, name=group, direction=direction, cob_id=cob,
                transmission_type=ttype, rate_hz=pdo_rate_hz(ttype, float(node_spec["sync_period_ms"])),
                objects=tuple(laid_out),
            ))
        nodes[name] = NodeContract(
            node=name, node_id=int(node_spec["node_id"]),
            json_sha256=_sha(root / node_spec["json"]),
            sync_period_ms=float(node_spec["sync_period_ms"]),
            heartbeat_produce_ms=int(node_spec["heartbeat_produce_ms"]),
            consume_timeout_ms=int(node_spec["consume_timeout_ms"]),
            liveness=node_spec["liveness"], identity_sdo=node_spec["identity_sdo"],
            pdos=tuple(pdos), byte_order=spec.get("byte_order", "little"),
        )

    # 语义表里不许留僵尸条目：每个语义名都必须真的出现在某个 PDO 里
    mapped = {o.name for c in nodes.values() for p in c.pdos for o in p.objects}
    stale = sorted(known_names - mapped)
    if stale:
        raise ContractError(f"canopen_contract.yaml 有清单里不存在的对象语义: {stale}")

    return Contract(nodes=nodes, root=root)


def _sha(path: Path) -> str:
    import hashlib

    return hashlib.sha256(path.read_bytes()).hexdigest()


def decode_build_tag(raw: Iterable[int]) -> str:
    """`0x2F00 tag_packed` = build_tag 前 4 字符小端打包。[V5] §5 实测 52 37 33 = "R73"。"""
    data = bytes(raw)
    if not data:
        raise ContractError("tag_packed 为空")
    return data.rstrip(b"\x00").decode("ascii", errors="replace")
