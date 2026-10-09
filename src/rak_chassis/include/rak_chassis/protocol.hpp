#ifndef RAK_CHASSIS__PROTOCOL_HPP_
#define RAK_CHASSIS__PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rak_chassis
{
constexpr uint8_t kProtocolVersion = 2U;
constexpr uint32_t kRequiredFeatures = 0x07U;
constexpr uint8_t kTypeHello = 0x01U;
constexpr uint8_t kTypeHelloAck = 0x02U;
constexpr uint8_t kTypeDriveCommand = 0x10U;
constexpr uint8_t kTypeArmCommand = 0x11U;    // [AI] 冻结版补齐：v2 备用链路臂命令（线格式自 R42 冻结未变）
constexpr uint8_t kTypeDriveState = 0x20U;
constexpr uint8_t kTypeArmState = 0x21U;
constexpr uint8_t kTypeDiagnostics = 0x22U;
constexpr uint8_t kTypeFaultEvent = 0x23U;
constexpr size_t kMaximumPayload = 128U;
constexpr size_t kMaximumWireFrame = 138U;

struct Frame
{
  uint8_t type{};
  uint16_t sequence{};
  std::vector<uint8_t> payload;
};

uint16_t crc16(const uint8_t *data, size_t size);
bool sequence_is_newer(uint16_t candidate, uint16_t previous);
bool encode_frame(const Frame &frame, std::vector<uint8_t> &wire);
bool decode_frame(const uint8_t *wire, size_t wire_size, Frame &frame);

uint16_t read_u16(const uint8_t *data);
uint32_t read_u32(const uint8_t *data);
uint64_t read_u64(const uint8_t *data);
int32_t read_i32(const uint8_t *data);
int64_t read_i64(const uint8_t *data);
void append_u16(std::vector<uint8_t> &data, uint16_t value);
void append_u32(std::vector<uint8_t> &data, uint32_t value);
void append_i32(std::vector<uint8_t> &data, int32_t value);
}  // [AI] 命名空间结束

#endif  // [AI] 头文件保护结束
