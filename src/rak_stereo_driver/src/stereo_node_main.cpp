// 独立可执行入口（与组件版共用同一份 CaptureNode）。
//
// 为什么不用 rclcpp_components_register_node 生成的那个 main：它只 spin，
// 而 CaptureNode 现在是 LifecycleNode（ADR-0020），没人发 configure/activate 时
// 进程起来却永远不开流 —— 正是"进程在但不干活"的静默故障。
// 「单独跑一个可执行」等价于 supervised:=false，应当自己走到 active。
#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "rak_stereo_driver/capture_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rak::stereo_driver::CaptureNode>();
  if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "stereo capture node failed to reach active");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
