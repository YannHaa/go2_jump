#include "go2_jump_planner/jump_plan.hpp"

#include <cstdlib>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
  go2_jump_planner::JumpPlannerConfig config;
  if (argc > 1) {
    config.target_distance_m = std::atof(argv[1]);
  }
  if (argc > 2) {
    config.takeoff_angle_deg = std::atof(argv[2]);
  }

  const auto plan = go2_jump_planner::MakeJumpPlan(config);

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "target_distance_m: " << plan.target_distance_m << '\n';
  std::cout << "takeoff_angle_deg: " << plan.takeoff_angle_deg << '\n';
  std::cout << "takeoff_speed_mps: " << plan.takeoff_speed_mps << '\n';
  std::cout << "flight_time_s: " << plan.estimated_flight_time_s << '\n';
  std::cout << "phase_durations_s: crouch=" << plan.crouch_duration_s
            << " push=" << plan.push_duration_s
            << " flight=" << plan.flight_duration_s
            << " landing=" << plan.landing_duration_s
            << " recovery=" << plan.recovery_duration_s << '\n';
  return 0;
}
