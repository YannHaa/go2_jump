#include "go2_jump_controller/unitree_crc.hpp"

#include <cstring>

namespace go2_jump_controller {

void FillLowCmdCrc(unitree_go::msg::LowCmd& msg) {
  RawLowCmd raw{};
  std::memcpy(&raw.head[0], &msg.head[0], 2);
  raw.level_flag = msg.level_flag;
  raw.frame_reserve = msg.frame_reserve;
  std::memcpy(&raw.sn[0], &msg.sn[0], 8);
  std::memcpy(&raw.version[0], &msg.version[0], 8);
  raw.bandwidth = msg.bandwidth;

  for (int i = 0; i < 20; ++i) {
    raw.motor_cmd[i].mode = msg.motor_cmd[i].mode;
    raw.motor_cmd[i].q = msg.motor_cmd[i].q;
    raw.motor_cmd[i].dq = msg.motor_cmd[i].dq;
    raw.motor_cmd[i].tau = msg.motor_cmd[i].tau;
    raw.motor_cmd[i].kp = msg.motor_cmd[i].kp;
    raw.motor_cmd[i].kd = msg.motor_cmd[i].kd;
    std::memcpy(&raw.motor_cmd[i].reserve[0], &msg.motor_cmd[i].reserve[0], 12);
  }

  raw.bms.off = msg.bms_cmd.off;
  std::memcpy(&raw.bms.reserve[0], &msg.bms_cmd.reserve[0], 3);
  std::memcpy(&raw.wireless_remote[0], &msg.wireless_remote[0], 40);
  std::memcpy(&raw.led[0], &msg.led[0], 12);
  std::memcpy(&raw.fan[0], &msg.fan[0], 2);
  raw.gpio = msg.gpio;
  raw.reserve = msg.reserve;

  raw.crc = Crc32Core(reinterpret_cast<uint32_t*>(&raw),
                      (sizeof(RawLowCmd) >> 2) - 1);
  msg.crc = raw.crc;
}

uint32_t Crc32Core(uint32_t* ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t crc32 = 0xFFFFFFFF;
  constexpr uint32_t polynomial = 0x04c11db7;

  for (uint32_t i = 0; i < len; ++i) {
    xbit = 1U << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; ++bits) {
      if (crc32 & 0x80000000U) {
        crc32 <<= 1;
        crc32 ^= polynomial;
      } else {
        crc32 <<= 1;
      }

      if (data & xbit) {
        crc32 ^= polynomial;
      }
      xbit >>= 1;
    }
  }

  return crc32;
}

}  // namespace go2_jump_controller
