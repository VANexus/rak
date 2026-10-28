# 每个 call_id 恰好一条终态结果(ok/rejected/unimplemented/timeout/error)。
string call_id
string tool
bool accepted          # 提案通过了权限与前置检查(不代表物理成功,见 reason/detail)
string status          # ok | rejected | unimplemented | timeout | error
string reason          # 人类可读的原因(拒绝时必须说明理由,给 LLM 下一轮用)
string detail_json     # 回灌 loop 上下文的负载(查询结果/确认状态/执行细节)
