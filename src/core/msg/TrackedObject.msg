# A single tracked object in 3D space, output of the world perception layer.

# Unique, persistent track ID
int64 track_id

# Semantic class
string label
float32 score

# 3D bounding box centre in the camera frame (metres)
geometry_msgs/Point position

# Bounding box size (metres)
geometry_msgs/Vector3 size

# Linear velocity estimate (m/s), if available
geometry_msgs/Twist velocity

# Associated detection, if this frame produced one
vision_msgs/Detection3D detection
