# Harness → 执行器结构化提案(harness → executor)。ADR-0010 D2, schema_version=1。
# 权限刻意收窄:只有 waypoint 别名,没有速度/关节字段。
uint8 SCHEMA_VERSION=1
uint8 KIND_NO_OP=0
uint8 KIND_SELECT_WAYPOINT=1
uint8 KIND_REVISE_PLAN=2
uint8 KIND_REQUEST_CLARIFICATION=3
std_msgs/Header header
string task_id
uint64 revision
string request_id
uint8 kind
string waypoint
string rationale
float64 expires_epoch_sec
