# Harness → 人可读事件(harness → UI/console,“呼吸感”软件侧)。ADR-0010 D4。
uint8 SCHEMA_VERSION=1
uint8 KIND_ACK=0
uint8 KIND_PROGRESS=1
uint8 KIND_RECOVERY=2
uint8 KIND_CLARIFICATION=3
std_msgs/Header header
string task_id
uint8 kind
string text
string reason
