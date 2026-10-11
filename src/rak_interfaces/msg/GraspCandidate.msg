# GraspNet frame convention: +X approach, +Y closing axis, +Z finger height.
# Pose is grasp center, not the fixture tool0 flange. Score is not force/probability.
std_msgs/Header header
uint32 id
geometry_msgs/Pose pose
float64 width_m
float64 depth_m
float64 score
