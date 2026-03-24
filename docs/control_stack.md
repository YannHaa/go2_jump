# Control Stack Overview

## Goal

This workspace targets a first-pass forward-jump workflow for Unitree Go2 using:

- `unitree_mujoco` as the primary simulator
- `unitree_ros2` for `LowCmd`, `LowState`, and `SportModeState`
- custom ROS 2 packages that stay below sport mode and publish low-level joint commands

The current focus is a minimal closed loop that is:

- reproducible on this remote host
- easy to tune
- suitable for later upgrades toward richer planning and control

## Workspace Layout

- `src/unitree_ros2`
  Official Unitree ROS 2 stack. This should be treated as upstream code.
- `src/unitree_mujoco`
  Official Unitree MuJoCo simulator. This should be treated as upstream code.
- `src/go2_jump_planner`
  Jump plan parameterization and target publisher.
- `src/go2_jump_controller`
  Low-level phase machine, `LowCmd` publishing, trial metrics, and reporting.
- `src/go2_jump_bringup`
  Launch files and shared runtime parameters.
- `scripts/`
  Docker build helpers, one-shot trial runner, and calibration sweep tools.
- `tools/`
  Standalone low-level DDS verification utilities.
- `docker/`
  Reproducible Humble image definition and third-party bootstrap helper.

## Runtime Chain

1. `unitree_mujoco` publishes `/lowstate`, `/sportmodestate`, and compatibility topics.
2. `go2_jump_planner` computes the current jump plan summary and publishes `/jump_target_distance`.
3. `go2_jump_controller` subscribes to:
   - `/lowstate`
   - `/sportmodestate`
   - `/jump_target_distance`
4. `go2_jump_controller` publishes `/lowcmd` at the configured control period.
5. At the end of each trial, the controller writes a structured report to:
   - `/workspace/reports/jump_metrics/latest_report.txt`

## Direct DDS Verification

The workspace also keeps a direct non-ROS verification path:

- `tools/verify_rt_lowcmd.cpp`
- `scripts/docker_verify_rt_lowcmd.sh`

That path talks straight to `rt/lowcmd` and `rt/lowstate` through the Unitree SDK2
DDS channel layer. It is useful when we want to separate “the simulator bridge is
healthy” from “the ROS 2 controller stack is healthy.”

## Controller Structure

The current controller is intentionally simple and parameter-driven:

- `stand -> crouch -> push -> flight -> landing -> recovery`
- PD tracking on all twelve controlled joints
- phase-dependent feedforward torques
- front/rear compactness bias for pitch shaping
- IMU pitch and pitch-rate feedback during push, flight, and landing

This is not yet a contact-rich optimal controller. It is a pragmatic foundation for
rapid simulation iteration.

## Current Metric Interpretation

The trial report now separates forward progress into:

- `takeoff_forward_displacement_m`
- `airborne_forward_progress_m`
- `landing_forward_displacement_m`
- `post_landing_forward_gain_m`
- `final_forward_displacement_m`

That split matters. On the current stack, final settled displacement can match the
target even when airborne forward progress is still small. For now, use both the
final and landing-related metrics when judging a tuning change.

## Parameter Modes

Takeoff speed calibration now supports two modes:

- manual mode
  `takeoff_speed_scale` is used directly
- curve mode
  `takeoff_speed_scale_distance_points_m` and `takeoff_speed_scale_values` are
  linearly interpolated against the target distance

The default workspace configuration uses curve mode for normal runs, while the sweep
script temporarily disables it to evaluate explicit manual scales.

## Known Limitations

- touchdown detection still uses height and vertical-velocity heuristics
- `foot_force_est` is currently zero in the observed MuJoCo bridge path on this host
- the current stack is calibrated more strongly for final settled displacement than
  for true airborne range
- recovery motion still contributes a large fraction of the final forward progress

## Recommended Next Technical Step

The next meaningful optimization loop should target airborne range directly, not just
final settled position. The highest-value knobs are:

- `push_front_tau_scale`
- `push_rear_tau_scale`
- `push_pitch_target_deg`
- `push_pitch_compactness_gain`
- `flight_pitch_target_deg`
- `landing_*` compactness shaping only after airborne progress improves
