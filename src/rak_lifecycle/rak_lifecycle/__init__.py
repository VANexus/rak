"""rak_lifecycle — 全栈受管生命周期的机制层（ADR-0020）。

包存在的理由：`robot.launch.py` 那套 `publish_robot_tf:=false` / `chassis:=false` 布尔参数
把"谁发 TF、谁有运动写权、谁依赖谁"写在注释里，没有任何可执行的东西钉住它。本包把这些
约束变成一份声明式 manifest + 一个按 manifest 编排的 manager + 一个自有节点统一基类。

分层位置：基础设施包，与 `rak_interfaces`/`rak_intent` 同级 —— 不依赖任何自有包。
`check_workspace.py` 校验这一条。
"""

__all__ = ["manifest", "managed_node", "system_manager"]
