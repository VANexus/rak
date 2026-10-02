# One typed output of the brain. This is a serialization edge for
# brain/rak_ai/types.Action -- keep it field-for-field with that pydantic model,
# because there must be exactly one schema for what the robot is allowed to do.

# ActionKind, literally one of:
#   stop | slow | continue | turn_left | turn_right | inspect | ask_operator
string kind

# ROS has no Optional. has_target=false means target_id is unused, and a
# consumer must not read it as a track id.
bool has_target
int64 target_id

# Why the brain chose this. Free text, for the operator and the audit log only
# -- never parsed, never authoritative.
string reason
