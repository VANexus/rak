#include "rak_chassis/protocol.hpp"

#include <algorithm>
#include <cstring>

namespace rak_chassis
{
namespace
{
bool known_type(uint8_t type)
{
  return (type >= 0x01U && type <= 0x04U) ||
         (type >= 0x10U && type <= 0x12U) ||
         (type >= 0x20U && type <= 0x23U);
}

bool cobs_encode(const std::vector<uint8_t> &input, std::vector<uint8_t> &output)
{
  output.clear();
  output.push_back(0U);
  size_t code_index = 0U;
  uint8_t code = 1U;
  for (const uint8_t byte : input) {
    if (byte == 0U) {
      output[code_index] = code;
      code_index = output.size();
      output.push_back(0U);
      code = 1U;
    } else {
      output.push_back(byte);
      ++code;
      if (code == 0xffU) {
        output[code_index] = code;
        code_index = output.size();
        output.push_back(0U);
        code = 1U;
      }
    }
  }
  output[code_index] = code;
  output.push_back(0U);
  return true;
}

bool cobs_decode(const uint8_t *wire, size_t wire_size, std::vector<uint8_t> &output)
{
  if (wire == nullptr || wire_size < 2U || wire[wire_size - 1U] != 0U) {
    return false;
  }
  output.clear();
  size_t index = 0U;
  const size_t encoded_size = wire_size - 1U;
  while (index < encoded_size) {
    const uint8_t code = wire[index++];
    const size_t run = static_cast<size_t>(code - 1U);
    if (code == 0U || run > encoded_size - index) {
      return false;
    }
    output.insert(output.end(), wire + index, wire + index + run);
    index += run;
    if (code != 0xffU && index < encoded_size) {
      output.push_back(0U);
    }
    if (output.size() > kMaximumPayload + 8U) {
      return false;
    }
  }
  return true;
}
}  // [AI] 私有实现命名空间结束

uint16_t crc16(const uint8_t *data, size_t size)
{
  uint16_t crc = 0xffffU;
  for (size_t i = 0U; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (unsigned bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<uint16_t>((crc << 1U) ^ 0x1021U) :
        static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

bool sequence_is_newer(uint16_t candidate, uint16_t previous)
{
  const uint16_t delta = static_cast<uint16_t>(candidate - previous);
  return delta != 0U && delta <= 0x7fffU;
}

bool encode_frame(const Frame &frame, std::vector<uint8_t> &wire)
{
  if (!known_type(frame.type) || frame.payload.size() > kMaximumPayload) {
    return false;
  }
  std::vector<uint8_t> raw;
  raw.reserve(frame.payload.size() + 8U);
  raw.push_back(kProtocolVersion);
  raw.push_back(frame.type);
  append_u16(raw, frame.sequence);
  append_u16(raw, static_cast<uint16_t>(frame.payload.size()));
  raw.insert(raw.end(), frame.payload.begin(), frame.payload.end());
  append_u16(raw, crc16(raw.data(), raw.size()));
  return cobs_encode(raw, wire);
}

bool decode_frame(const uint8_t *wire, size_t wire_size, Frame &frame)
{
  if (wire_size > kMaximumPayload + 10U) {
    return false;
  }
  std::vector<uint8_t> raw;
  if (!cobs_decode(wire, wire_size, raw) || raw.size() < 8U ||
    raw[0] != kProtocolVersion || !known_type(raw[1]))
  {
    return false;
  }
  const size_t payload_size = read_u16(&raw[4]);
  if (payload_size > kMaximumPayload || raw.size() != payload_size + 8U ||
    read_u16(&raw[raw.size() - 2U]) != crc16(raw.data(), raw.size() - 2U))
  {
    return false;
  }
  frame.type = raw[1];
  frame.sequence = read_u16(&raw[2]);
  frame.payload.assign(raw.begin() + 6, raw.end() - 2);
  return true;
}

uint16_t read_u16(const uint8_t *data)
{
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(data[1]) << 8U;
}

uint32_t read_u32(const uint8_t *data)
{
  return static_cast<uint32_t>(data[0]) |
         static_cast<uint32_t>(data[1]) << 8U |
         static_cast<uint32_t>(data[2]) << 16U |
         static_cast<uint32_t>(data[3]) << 24U;
}

uint64_t read_u64(const uint8_t *data)
{
  return static_cast<uint64_t>(read_u32(data)) |
         static_cast<uint64_t>(read_u32(data + 4U)) << 32U;
}

int32_t read_i32(const uint8_t *data)
{
  const uint32_t bits = read_u32(data);
  int32_t value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

int64_t read_i64(const uint8_t *data)
{
  const uint64_t bits = read_u64(data);
  int64_t value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void append_u16(std::vector<uint8_t> &data, uint16_t value)
{
  data.push_back(static_cast<uint8_t>(value));
  data.push_back(static_cast<uint8_t>(value >> 8U));
}

void append_u32(std::vector<uint8_t> &data, uint32_t value)
{
  for (unsigned shift = 0U; shift < 32U; shift += 8U) {
    data.push_back(static_cast<uint8_t>(value >> shift));
  }
}

void append_i32(std::vector<uint8_t> &data, int32_t value)
{
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u32(data, bits);
}
}  // [AI] 命名空间结束
