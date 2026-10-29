// Copyright 2026 RAK project
/* [AI] 实现见 can_session_pool.hpp 的设计说明。
 * 进程级注册表：channel -> {session, config, refs}。
 * 只在 hardware 的 configure/cleanup 里进出，不在 100 Hz 控制环上调用。 */
#include "rak_chassis/can_session_pool.hpp"

#include <map>
#include <mutex>

namespace rak_chassis
{
namespace
{

struct Entry
{
  std::shared_ptr<CanSession> session;
  SessionConfig config;
  std::size_t refs{0U};
};

std::mutex & registry_mutex()
{
  static std::mutex mutex;
  return mutex;
}

std::map<std::string, Entry> & registry()
{
  static std::map<std::string, Entry> entries;
  return entries;
}

}  // namespace

std::shared_ptr<CanSession> acquire_session(
  const SessionConfig & config, std::string & error)
{
  std::lock_guard<std::mutex> lock(registry_mutex());
  auto & entries = registry();

  const auto it = entries.find(config.channel);
  if (it != entries.end()) {
    /* 同一条总线上两个组件的参数必须一致：不一致说明 URDF 两段写岔了，
     * 继续跑会让其中一个组件按错误的 allow_motion / build_id 行事。 */
    if (!(it->second.config == config)) {
      error = "session config mismatch on " + config.channel +
              " (channel/host_build_id/allow_motion must be identical across "
              "<ros2_control> sections)";
      return nullptr;
    }
    it->second.refs++;
    return it->second.session;
  }

  auto session = std::make_shared<CanSession>(config.channel, config.host_build_id);
  session->start();
  entries.emplace(config.channel, Entry{session, config, 1U});
  return session;
}

void release_session(const std::string & channel)
{
  std::lock_guard<std::mutex> lock(registry_mutex());
  auto & entries = registry();
  const auto it = entries.find(channel);
  if (it == entries.end()) {
    return;
  }
  if (it->second.refs > 1U) {
    it->second.refs--;
    return;
  }
  it->second.session->stop();
  entries.erase(it);
}

std::size_t session_refcount(const std::string & channel)
{
  std::lock_guard<std::mutex> lock(registry_mutex());
  const auto it = registry().find(channel);
  return (it == registry().end()) ? 0U : it->second.refs;
}

}  // namespace rak_chassis
