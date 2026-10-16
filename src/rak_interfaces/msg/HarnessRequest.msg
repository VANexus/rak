# Harness ↔ 执行器规划请求(executor → harness)。ADR-0010 D2, schema_version=1。
# 时间用 epoch 秒浮点,避免给 rak_interfaces 加 builtin_interfaces 依赖。
uint8 SCHEMA_VERSION=1
std_msgs/Header header
string task_id
uint64 revision
string request_id
string text
string[] allowed_waypoints
float64 deadline_epoch_sec
string[] context_flags
