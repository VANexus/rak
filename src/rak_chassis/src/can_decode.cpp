#include "rak_chassis/can_decode.hpp"

namespace rak_chassis
{
NodeInfoFrame decode_node_info(const std::array<uint8_t, 8> &d)
{
  NodeInfoFrame frame;
  frame.proto = d[0];
  frame.hw_rev = d[1];
  frame.build_tag = read_u16(d, 2U);
  frame.fault = read_u16(d, 4U);
  frame.state = read_u16(d, 6U);
  frame.proto_ok = (d[0] == 4U);
  return frame;
}

DiagFrame decode_diag(const std::array<uint8_t, 8> &d)
{
  DiagFrame frame;
  frame.bat_mv = read_u16(d, 0U);
  frame.fault = read_u16(d, 2U);
  frame.state = read_u16(d, 4U);
  frame.mode = d[6];
  frame.busoff_cnt = d[7];
  return frame;
}

ArmDiagFrame decode_arm_diag(const std::array<uint8_t, 8> &d)
{
  ArmDiagFrame frame;
  frame.fault = read_u16(d, 0U);
  frame.state = read_u16(d, 2U);
  frame.mode = d[4];
  frame.servo_errcnt = d[5];
  frame.faulted = (frame.state & kStateFaultMask) != 0U ||
    frame.mode == 2U || frame.mode == 3U;
  return frame;
}
}  // namespace rak_chassis
