# A decision plus the provenance needed to audit it. Mirrors
# brain/rak_ai/types.Decision; see Action.msg for the same caveat.

# stamp = wall time the decision was taken. `t` below is scene time and is
# what the episode corpus records, so the two are deliberately separate.
std_msgs/Header header

Action action

# Where it came from: s1 (Laya) | s2 (RAI) | guard (geometric veto) |
# fallback (low-confidence s1, not trusted to act).
string source

# Confidence fields. p is the top option's probability, margin is p(top) -
# p(second). NEITHER authorises anything on its own -- gate() turns them into
# an escalation tier and guard() can override everything. Untrusted by design.
float32 p
float32 margin

# How long System 1 took to answer, milliseconds.
float32 latency_ms

# Scene time in seconds (rak_ai.SceneState.t), not the header stamp.
float32 t

# gate() output: high | medium | low. This is escalation policy, not authority.
string tier
