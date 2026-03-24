#pragma once

#include <array>
#include <cstdint>

#include "unitree_go/msg/low_cmd.hpp"

namespace go2_jump_controller {

constexpr double kPosStop = 2.146E+9f;
constexpr double kVelStop = 16000.0f;

constexpr int FR_0 = 0;
constexpr int FR_1 = 1;
constexpr int FR_2 = 2;
constexpr int FL_0 = 3;
constexpr int FL_1 = 4;
constexpr int FL_2 = 5;
constexpr int RR_0 = 6;
constexpr int RR_1 = 7;
constexpr int RR_2 = 8;
constexpr int RL_0 = 9;
constexpr int RL_1 = 10;
constexpr int RL_2 = 11;

struct RawBmsCmd {
  uint8_t off;
  std::array<uint8_t, 3> reserve;
};

struct RawMotorCmd {
  uint8_t mode;
  float q;
  float dq;
  float tau;
  float kp;
  float kd;
  std::array<uint32_t, 3> reserve;
};

struct RawLowCmd {
  std::array<uint8_t, 2> head;
  uint8_t level_flag;
  uint8_t frame_reserve;
  std::array<uint32_t, 2> sn;
  std::array<uint32_t, 2> version;
  uint16_t bandwidth;
  std::array<RawMotorCmd, 20> motor_cmd;
  RawBmsCmd bms;
  std::array<uint8_t, 40> wireless_remote;
  std::array<uint8_t, 12> led;
  std::array<uint8_t, 2> fan;
  uint8_t gpio;
  uint32_t reserve;
  uint32_t crc;
};

uint32_t Crc32Core(uint32_t* ptr, uint32_t len);
void FillLowCmdCrc(unitree_go::msg::LowCmd& msg);

}  // namespace go2_jump_controller
