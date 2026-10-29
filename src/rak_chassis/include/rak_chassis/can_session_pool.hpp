// Copyright 2026 RAK project
/* [AI] 2026-10-29 双下位机拆分：底盘板与机械臂板是两个物理节点、两个 CAN 节点，
 * 但在 Jetson 侧共处**同一条总线、同一个 SocketCAN 接口**。ros2_control 里它们必须
 * 是两个 <ros2_control> 段（故障域隔离：一边掉线不能把另一边的控制器一起 deactivate），
 * 可是**不能各开一个 socket** —— 100 Hz × 13 帧会被重复解码，且 mttcan 的 qlen 只有 10，
 * 双份读取更容易丢帧。
 *
 * 所以：两个 hardware 组件（同一进程）各 acquire 一次，共享同一个 CanSession；
 * 引用计数归零才 stop + 释放。参数必须一致，不一致直接失败（宁可起不来，
 * 也不要两个组件对同一条总线持有不同假设）。
 */
#ifndef RAK_CHASSIS__CAN_SESSION_POOL_HPP_
#define RAK_CHASSIS__CAN_SESSION_POOL_HPP_

#include "rak_chassis/can_session.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace rak_chassis
{

struct SessionConfig
{
  std::string channel{"can0"};
  uint32_t host_build_id{0U};
  bool allow_motion{false};

  bool operator==(const SessionConfig & other) const
  {
    return channel == other.channel &&
           host_build_id == other.host_build_id &&
           allow_motion == other.allow_motion;
  }
};

/* 取用/归还共享会话。acquire 失败时 error 里是可直接打日志的原因。 */
std::shared_ptr<CanSession> acquire_session(
  const SessionConfig & config, std::string & error);
void release_session(const std::string & channel);

/* 诊断/测试用：当前该接口上的取用计数（0 = 没有会话）。 */
std::size_t session_refcount(const std::string & channel);

}  // namespace rak_chassis

#endif  // RAK_CHASSIS__CAN_SESSION_POOL_HPP_
