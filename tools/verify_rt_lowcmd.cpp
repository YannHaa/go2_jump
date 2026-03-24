#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::robot;

namespace {

constexpr const char* kTopicLowCmd = "rt/lowcmd";
constexpr const char* kTopicLowState = "rt/lowstate";
constexpr double kPosStopF = 2.146e9;
constexpr double kVelStopF = 16000.0;
constexpr double kDt = 0.002;

uint32_t crc32_core(uint32_t* ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t crc32 = 0xFFFFFFFF;
  const uint32_t polynomial = 0x04c11db7;

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

double L2NormDelta(const unitree_go::msg::dds_::LowState_& a,
                   const unitree_go::msg::dds_::LowState_& b) {
  double accum = 0.0;
  for (int i = 0; i < 12; ++i) {
    const double diff = a.motor_state()[i].q() - b.motor_state()[i].q();
    accum += diff * diff;
  }
  return std::sqrt(accum);
}

void InitializeLowCmd(unitree_go::msg::dds_::LowCmd_& low_cmd) {
  low_cmd.head()[0] = 0xFE;
  low_cmd.head()[1] = 0xEF;
  low_cmd.level_flag() = 0xFF;
  low_cmd.gpio() = 0;

  for (int i = 0; i < 20; ++i) {
    low_cmd.motor_cmd()[i].mode() = 0x01;
    low_cmd.motor_cmd()[i].q() = kPosStopF;
    low_cmd.motor_cmd()[i].kp() = 0.0;
    low_cmd.motor_cmd()[i].dq() = kVelStopF;
    low_cmd.motor_cmd()[i].kd() = 0.0;
    low_cmd.motor_cmd()[i].tau() = 0.0;
  }
}

void ApplyPose(unitree_go::msg::dds_::LowCmd_& low_cmd,
               const double* pose,
               double kp,
               double kd) {
  for (int i = 0; i < 12; ++i) {
    low_cmd.motor_cmd()[i].mode() = 0x01;
    low_cmd.motor_cmd()[i].q() = pose[i];
    low_cmd.motor_cmd()[i].dq() = 0.0;
    low_cmd.motor_cmd()[i].kp() = kp;
    low_cmd.motor_cmd()[i].kd() = kd;
    low_cmd.motor_cmd()[i].tau() = 0.0;
  }
  low_cmd.crc() =
      crc32_core(reinterpret_cast<uint32_t*>(&low_cmd),
                 (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
}

}  // namespace

int main(int argc, char** argv) {
  const std::string interface = argc > 1 ? argv[1] : "lo";
  ChannelFactory::Instance()->Init(1, interface);

  unitree_go::msg::dds_::LowState_ latest_low_state{};
  bool have_low_state = false;
  std::mutex low_state_mutex;

  auto publisher =
      std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(kTopicLowCmd);
  publisher->InitChannel();

  auto subscriber =
      std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(kTopicLowState);
  subscriber->InitChannel(
      [&](const void* message) {
        std::lock_guard<std::mutex> lock(low_state_mutex);
        latest_low_state = *static_cast<const unitree_go::msg::dds_::LowState_*>(message);
        have_low_state = true;
      },
      1);

  std::cout << "Waiting for rt/lowstate on interface " << interface << "..." << std::endl;
  const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!have_low_state && std::chrono::steady_clock::now() < wait_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (!have_low_state) {
    std::cerr << "No rt/lowstate received within timeout." << std::endl;
    return 2;
  }

  unitree_go::msg::dds_::LowState_ initial_low_state{};
  {
    std::lock_guard<std::mutex> lock(low_state_mutex);
    initial_low_state = latest_low_state;
  }

  std::cout << "Initial q[0..2]: "
            << initial_low_state.motor_state()[0].q() << ", "
            << initial_low_state.motor_state()[1].q() << ", "
            << initial_low_state.motor_state()[2].q() << std::endl;

  const double stand_down_pose[12] = {
      0.0473455,  1.22187,  -2.44375, -0.0473455, 1.22187,  -2.44375,
      0.0473455,  1.22187,  -2.44375, -0.0473455, 1.22187,  -2.44375,
  };
  const double stand_up_pose[12] = {
      0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
      0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
  };

  unitree_go::msg::dds_::LowCmd_ low_cmd{};
  InitializeLowCmd(low_cmd);

  for (int step = 0; step < 2200; ++step) {
    if (step < 800) {
      ApplyPose(low_cmd, stand_down_pose, 30.0, 3.5);
    } else {
      ApplyPose(low_cmd, stand_up_pose, 50.0, 3.5);
    }
    publisher->Write(low_cmd);

    if (step % 400 == 0) {
      std::lock_guard<std::mutex> lock(low_state_mutex);
      std::cout << "Tick " << latest_low_state.tick()
                << " q[0..2]="
                << latest_low_state.motor_state()[0].q() << ", "
                << latest_low_state.motor_state()[1].q() << ", "
                << latest_low_state.motor_state()[2].q() << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(kDt));
  }

  unitree_go::msg::dds_::LowState_ final_low_state{};
  {
    std::lock_guard<std::mutex> lock(low_state_mutex);
    final_low_state = latest_low_state;
  }

  const double delta_norm = L2NormDelta(final_low_state, initial_low_state);
  std::cout << "Final q[0..2]: "
            << final_low_state.motor_state()[0].q() << ", "
            << final_low_state.motor_state()[1].q() << ", "
            << final_low_state.motor_state()[2].q() << std::endl;
  std::cout << "Delta L2 norm over first 12 joints: " << delta_norm << std::endl;

  if (delta_norm < 0.25) {
    std::cerr << "LowCmd verification failed: joint response was too small." << std::endl;
    return 3;
  }

  std::cout << "LowCmd verification passed." << std::endl;
  return 0;
}
