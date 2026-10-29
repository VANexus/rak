"""系统编排清单（ADR-0020 的策略层）：故障域、组件、所有权与存活性判据的唯一声明来源。

这里只放**数据模型与不变量校验**，不碰 ROS —— 所以桌面 pytest 无需 DDS 就能跑，
`check_workspace.py` 也能直接 import 它来审现场清单。

校验的每条不变量都对应一个曾经只写在注释里的规矩：

- 一个节点恰好一个生命周期拥有者（否则出现两个权威源，静默空转或互相抢转移）；
- 同一运动接口只有一个写者（ADR-0011 D1，`arm_motion_writer:=jtc|wbc` 的互斥）；
- 机体 TF 只有一个发布者（ADR-0015，`publish_robot_tf:=false` 想表达却表达不了的东西）；
- 域依赖无环且可拓扑排序（否则编排顺序只能靠人记）；
- 每个 `age` 判据必须交代 `basis`（`measured`/`nominal`/`placeholder`），
  没实测依据的阈值不许伪装成事实（本文档库的事实分级规矩）。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

SUPPORTED_VERSION = 1
SYSTEM_MANAGER = "system_manager"

MODES = frozenset({"managed", "observed"})
#: 存在性判据：`node` = 发现图里要有这个节点名；`topic` = 不认节点名（第三方驱动的
#: 节点名未经现场核实），只认 `provides` 里的话题确实存在。
PRESENCE = frozenset({"node", "topic"})
PROBE_KINDS = frozenset({"age", "exists"})
AGE_BASES = frozenset({"measured", "nominal", "placeholder"})
ROLE_MOTION_WRITER = "motion_writer"
ROLE_BODY_TF = "body_tf_publisher"
ROLES = frozenset({ROLE_MOTION_WRITER, ROLE_BODY_TF})

#: `age` 判据里显式标 measured 的话题才允许用来降级故障域（其余仅告警）。
DEFAULT_DOMAIN_TIMEOUT_MS = 30_000.0


class ManifestError(ValueError):
    """清单不合法。消息必须能直接指出是哪个域/哪个组件/哪条不变量。"""


@dataclass(frozen=True)
class Probe:
    """一条话题级判据。

    `kind="exists"` 只查发现图里有没有发布者 —— 重负载话题（点云、图像）用 `age`
    就意味着为了测新鲜度去反序列化整帧，4 GB 单板上不划算，所以分开两档。
    """

    topic: str
    kind: str = "exists"
    max_age_ms: float | None = None
    basis: str = "placeholder"

    @property
    def escalation_allowed(self) -> bool:
        return self.kind == "age" and self.basis == "measured"


@dataclass(frozen=True)
class Component:
    node: str
    package: str
    executable: str
    mode: str
    owner: str
    launch: str
    presence: str = "node"
    role: str | None = None
    interface: str | None = None
    provides: tuple[Probe, ...] = ()
    needs: tuple[Probe, ...] = ()
    enabled: bool = True

    @property
    def system_managed(self) -> bool:
        return self.mode == "managed" and self.owner == SYSTEM_MANAGER

    @property
    def short_name(self) -> str:
        return self.node.lstrip("/").split("/")[-1]


@dataclass(frozen=True)
class Domain:
    name: str
    components: tuple[Component, ...]
    requires: tuple[str, ...] = ()
    timeout_ms: float = DEFAULT_DOMAIN_TIMEOUT_MS
    enabled: bool = True

    def managed(self) -> tuple[Component, ...]:
        return tuple(c for c in self.components if c.system_managed and c.enabled)

    def observed(self) -> tuple[Component, ...]:
        return tuple(c for c in self.components if c.enabled and not c.system_managed)


@dataclass(frozen=True)
class Manifest:
    version: int
    domains: tuple[Domain, ...]
    #: 确定性拓扑序（同名冲突时用名字打破对称，避免编排顺序随字典迭代顺序漂移）。
    order: tuple[str, ...]

    def domain(self, name: str) -> Domain:
        for domain in self.domains:
            if domain.name == name:
                return domain
        raise ManifestError(f"unknown domain: {name!r}")

    def enabled_domains(self) -> tuple[Domain, ...]:
        return tuple(d for d in self.domains if d.enabled and d.name in self.order)

    def dependents_of(self, name: str) -> frozenset[str]:
        """传递闭包：故障时该域之下要一并降级的下游域（不含自己）。"""
        self.domain(name)
        downstream: set[str] = set()
        frontier = [
            d.name for d in self.domains if name in d.requires and d.name != name
        ]
        while frontier:
            current = frontier.pop()
            if current in downstream:
                continue
            downstream.add(current)
            frontier.extend(
                d.name for d in self.domains if current in d.requires
            )
        return frozenset(downstream)

    def component(self, node: str) -> Component:
        for domain in self.domains:
            for component in domain.components:
                if component.node == node:
                    return component
        raise ManifestError(f"unknown component node: {node!r}")


def _probe(raw: Mapping[str, Any], where: str) -> Probe:
    topic = raw.get("topic")
    if not isinstance(topic, str) or not topic.startswith("/"):
        raise ManifestError(f"{where}: probe topic must be an absolute name, got {topic!r}")
    kind = raw.get("kind", "exists")
    if kind not in PROBE_KINDS:
        raise ManifestError(f"{where}: probe kind must be one of {sorted(PROBE_KINDS)}, got {kind!r}")
    max_age = raw.get("max_age_ms")
    basis = raw.get("basis", "placeholder")
    if basis not in AGE_BASES:
        raise ManifestError(f"{where}: basis must be one of {sorted(AGE_BASES)}, got {basis!r}")
    if kind == "age":
        if not isinstance(max_age, (int, float)) or max_age <= 0:
            raise ManifestError(f"{where}: {topic} uses kind=age but has no positive max_age_ms")
    return Probe(topic=topic, kind=kind, max_age_ms=float(max_age) if max_age else None, basis=basis)


def _component(raw: Mapping[str, Any], domain_name: str) -> Component:
    where = f"domain {domain_name!r} component {raw.get('node')!r}"
    node = raw.get("node")
    if not isinstance(node, str) or not node.startswith("/"):
        raise ManifestError(f"{where}: node must be an absolute node name")
    mode = raw.get("mode")
    if mode not in MODES:
        raise ManifestError(f"{where}: mode must be one of {sorted(MODES)}, got {mode!r}")
    owner = raw.get("owner")
    if not isinstance(owner, str) or not owner:
        raise ManifestError(f"{where}: owner is required (a component with no owner has nobody to activate it)")
    if mode == "managed" and owner == "external":
        raise ManifestError(f"{where}: managed components must name their manager, not just 'external'")
    if mode == "observed" and owner == SYSTEM_MANAGER:
        raise ManifestError(f"{where}: observed components cannot be owned by {SYSTEM_MANAGER}")
    for key in ("package", "executable", "launch"):
        if not isinstance(raw.get(key), str) or not raw[key]:
            raise ManifestError(f"{where}: {key} is required")
    role = raw.get("role")
    if role is not None and role not in ROLES:
        raise ManifestError(f"{where}: role must be one of {sorted(ROLES)}, got {role!r}")
    if role == ROLE_MOTION_WRITER and not raw.get("interface"):
        raise ManifestError(f"{where}: a motion_writer must declare which interface it writes")
    presence = raw.get("presence", "node")
    if presence not in PRESENCE:
        raise ManifestError(f"{where}: presence must be one of {sorted(PRESENCE)}, got {presence!r}")
    if presence == "topic" and mode == "managed":
        raise ManifestError(
            f"{where}: presence=topic 与 mode=managed 冲突 —— 不知道节点名就发不出 "
            "change_state，转移只能由它的拥有者做"
        )
    provides = tuple(_probe(p, f"{where} provides") for p in raw.get("provides", ()))
    if presence == "topic" and not provides:
        raise ManifestError(
            f"{where}: presence=topic 用话题代替节点名做存在性判据，"
            "必须至少声明一条 provides，否则这个组件永远无法判为就绪"
        )
    return Component(
        node=node,
        package=raw["package"],
        executable=raw["executable"],
        mode=mode,
        owner=owner,
        launch=raw["launch"],
        presence=presence,
        role=role,
        interface=raw.get("interface"),
        provides=provides,
        needs=tuple(_probe(p, f"{where} needs") for p in raw.get("needs", ())),
        enabled=bool(raw.get("enabled", True)),
    )


def _domain(raw: Mapping[str, Any]) -> Domain:
    name = raw.get("name")
    if not isinstance(name, str) or not name:
        raise ManifestError("every domain needs a name")
    components = tuple(_component(c, name) for c in raw.get("components", ()))
    if not components:
        raise ManifestError(f"domain {name!r} declares no components")
    requires = tuple(raw.get("requires", ()))
    if name in requires:
        raise ManifestError(f"domain {name!r} requires itself")
    timeout = raw.get("timeout_ms", DEFAULT_DOMAIN_TIMEOUT_MS)
    if not isinstance(timeout, (int, float)) or timeout <= 0:
        raise ManifestError(f"domain {name!r}: timeout_ms must be positive")
    return Domain(
        name=name,
        components=components,
        requires=requires,
        timeout_ms=float(timeout),
        enabled=bool(raw.get("enabled", True)),
    )


def _toposort(domains: Sequence[Domain]) -> tuple[str, ...]:
    """Kahn，同层按名字排序 —— 顺序必须是确定的，否则两次启动的编排时序不可比。"""
    by_name = {d.name: d for d in domains}
    missing = {
        dep for d in domains for dep in d.requires if dep not in by_name
    }
    if missing:
        raise ManifestError(f"domains require unknown domains: {sorted(missing)}")
    active = [d for d in domains if d.enabled]
    names = {d.name for d in active}
    remaining = {d.name: {dep for dep in d.requires if dep in names} for d in active}
    order: list[str] = []
    while remaining:
        ready = sorted(n for n, deps in remaining.items() if not deps)
        if not ready:
            raise ManifestError(f"dependency cycle among domains: {sorted(remaining)}")
        for name in ready:
            order.append(name)
            remaining.pop(name)
        for deps in remaining.values():
            deps.difference_update(ready)
    return tuple(order)


def from_dict(raw: Mapping[str, Any]) -> Manifest:
    version = raw.get("version")
    if version != SUPPORTED_VERSION:
        raise ManifestError(f"unsupported manifest version: {version!r} (expected {SUPPORTED_VERSION})")
    domains = tuple(_domain(d) for d in raw.get("domains", ()))
    if not domains:
        raise ManifestError("manifest declares no domains")

    names = [d.name for d in domains]
    duplicates = sorted({n for n in names if names.count(n) > 1})
    if duplicates:
        raise ManifestError(f"duplicate domain names: {duplicates}")

    nodes = [c.node for d in domains for c in d.components if c.enabled]
    doubled = sorted({n for n in nodes if nodes.count(n) > 1})
    if doubled:
        raise ManifestError(
            f"node owned by more than one component entry: {doubled} "
            "(ADR-0020 单一所有者规则：一个节点恰好一个生命周期拥有者)"
        )

    writers: dict[str, list[str]] = {}
    for d in domains:
        for c in d.components:
            if c.enabled and c.role == ROLE_MOTION_WRITER:
                writers.setdefault(c.interface, []).append(f"{d.name}:{c.node}")
    for interface, owners in sorted(writers.items()):
        if len(owners) > 1:
            raise ManifestError(
                f"interface {interface!r} has {len(owners)} motion writers: {owners} "
                "(ADR-0011 D1：同一批命令接口同一时刻只能有一个 active 写者)"
            )

    tf_publishers = [
        f"{d.name}:{c.node}"
        for d in domains
        for c in d.components
        if c.enabled and c.role == ROLE_BODY_TF
    ]
    if len(tf_publishers) > 1:
        raise ManifestError(
            f"{len(tf_publishers)} components claim the body TF publisher role: {tf_publishers} "
            "(ADR-0015：同一 TF 子坐标系只能有一个权威发布者)"
        )

    return Manifest(version=version, domains=domains, order=_toposort(domains))


def load(path: str | Path) -> Manifest:
    source = Path(path)
    if not source.is_file():
        raise ManifestError(f"manifest not found: {source}")
    raw = yaml.safe_load(source.read_text())
    if not isinstance(raw, Mapping):
        raise ManifestError(f"manifest is not a mapping: {source}")
    return from_dict(raw)
