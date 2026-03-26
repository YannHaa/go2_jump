# Go2 Jump Workspace

[中文文档](README.zh-CN.md)

## Project Overview

This repository is a teaching-oriented Go2 forward-jump project built around a
minimal low-level control loop in simulation.

The stack is organized around three principles:

- use `unitree_mujoco` as the primary simulator
- use `unitree_ros2` for `LowCmd` and `LowState`
- stay below sport mode and publish low-level joint commands directly

The current target is a short-distance, flat-ground, left-right symmetric forward
jump that is easy to run, inspect, and iterate on.

## What This Project Covers

- ROS 2 Humble workflow packaged in Docker
- Unitree MuJoCo simulation for Go2
- low-level `LowCmd` / `LowState` communication
- a parameterized jump controller with a phase machine
- repeatable one-shot trials with archived reports
- basic calibration workflows for distance tracking and airborne motion

## What This Project Does Not Cover Yet

- sport mode `frontJump`
- direct hardware deployment
- contact-rich optimal control
- full-state estimation beyond the simulator bridge

## Recommended Reading Order

1. Read this file for setup and the first trial.
2. Read [Control Stack](docs/control_stack.md) to understand the runtime chain.
3. Read [Algorithm Notes](algorithm.md) for the current planner/controller/WBC split.
4. Read [Calibration Workflow](docs/calibration_workflow.md) before tuning.
5. Read [Development Workflow](docs/development_workflow.md) before changing
   submodules, patches, or the build pipeline.

## Validated Baseline

The repository is currently validated with the following baseline:

- Ubuntu 22.04 host
- Docker-based ROS 2 Humble environment
- `unitree_mujoco` and `unitree_ros2` as pinned submodules
- low-level topics:
  `rt/lowcmd`, `rt/lowstate`, `/lowcmd`, `/lowstate`, `/sportmodestate`

Docker is used here to keep the ROS 2, MuJoCo, and Unitree SDK environment
reproducible across machines.

## Repository Layout

- `src/unitree_ros2`
  Official Unitree ROS 2 stack tracked as a submodule.
- `src/unitree_mujoco`
  Official Unitree MuJoCo simulator tracked as a submodule.
- `src/go2_jump_planner`
  Jump parameterization and target publishing.
- `src/go2_jump_controller`
  Low-level controller, phase machine, and trial reporting.
- `src/go2_jump_bringup`
  Launch files and shared parameter files.
- `scripts/`
  Build, launch, validation, and sweep helpers.
- `tools/`
  Direct DDS verification utilities.
- `patches/`
  Local patches applied on top of upstream submodules.
- `docs/`
  Architecture, calibration, and development documentation.
- `algorithm.md`
  Root-level notes on what the planner, controller, and experimental WBC layer compute.

## Quick Start

### 1. Clone the Repository

```bash
git clone --recurse-submodules https://github.com/matrix050628/go2_jump.git /home/hayan/go2_jump_ws
cd /home/hayan/go2_jump_ws
```

### 2. Bootstrap the Workspace

```bash
./scripts/bootstrap_workspace_repo.sh
./scripts/bootstrap_third_party.sh
```

`bootstrap_workspace_repo.sh` initializes submodules and applies the local
`unitree_mujoco` compatibility patch when needed.

### 3. Build the Docker Image

```bash
./scripts/docker_build_image.sh
```

If a local cache exists under `third_party/`, the image build reuses it. If not,
the helper downloads the required upstream sources.

### 4. Build the Workspace

```bash
./scripts/docker_build_workspace.sh
```

This builds:

- `unitree_ros2` support packages
- `go2_jump_planner`
- `go2_jump_controller`
- `go2_jump_bringup`
- `tools/verify_rt_lowcmd`
- `unitree_mujoco`

### 5. Start the Simulator

```bash
./scripts/docker_run_go2_mujoco.sh
```

On machines without a desktop session, the script starts `Xvfb` automatically so
MuJoCo can run headlessly while keeping the render loop alive.

### 6. Start the Jump Stack

Open a second terminal:

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_launch_jump_stack.sh 0.25
```

### 7. Run a Single End-to-End Trial

For most users, the fastest way to validate the project is the one-shot helper:

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_run_single_jump_trial.sh 0.25
```

This helper starts a fresh simulator, runs the planner and controller, waits for a
new report, archives the result and the launch context, and shuts the runtime
containers down cleanly.

## Standard Validation Commands

### Direct DDS Verification

```bash
./scripts/docker_verify_rt_lowcmd.sh
```

Use this when you want to verify that the simulator reacts to low-level DDS traffic
independently of the ROS 2 controller stack.

### Full Stack Validation

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

Use this when you want to validate the full simulation loop:

- `unitree_mujoco`
- `/lowstate`
- planner
- controller
- `/lowcmd`
- report generation

## Reading the Trial Report

The most useful report fields for jump quality are:

- `final_forward_displacement_m`
  Final settled position after recovery.
- `landing_forward_displacement_m`
  Forward position at landing detection.
- `airborne_forward_progress_m`
  Forward progress between takeoff and landing detection.
- `post_landing_forward_gain_m`
  Forward motion accumulated after landing detection.

For this project, `airborne_forward_progress_m` is the best first indicator of a
real forward jump. `final_forward_displacement_m` is still important, but it can be
inflated by post-landing recovery motion.

Two extra timing fields are especially useful after the recent controller refactor:

- `push_extension_after_plan_s`
  Extra push time required after the nominal push window before takeoff is
  actually detected.
- `flight_extension_after_plan_s`
  Extra flight time required beyond the ballistic estimate before landing is
  detected.

## Tuning Workflow

### Switch Jump Profiles

The workspace supports named parameter profiles through `GO2_JUMP_PROFILE`.

Use the current default-style profile explicitly:

```bash
GO2_JUMP_PROFILE=conservative_airborne \
./scripts/docker_run_single_jump_trial.sh 0.25
```

Probe the stronger airborne setting:

```bash
GO2_JUMP_PROFILE=aggressive_airborne \
./scripts/docker_run_single_jump_trial.sh 0.25
```

The supported profiles are currently:

- `config_default`
- `conservative_airborne`
- `aggressive_airborne`

### Calibrate Takeoff Speed

```bash
./scripts/sweep_takeoff_speed_scale.sh 0.20,0.25,0.30 1.00,1.03,1.06 1
```

Use this sweep to fit the `takeoff_speed_scale` curve for target-distance accuracy.

To recalibrate takeoff speed for a named profile, use:

```bash
./scripts/sweep_profile_takeoff_speed_scale.sh aggressive_airborne 0.25 0.94,0.97,1.00,1.03 1
```

This keeps the aggressive airborne posture shaping while searching for a lower
`takeoff_speed_scale` that brings the final distance back toward the target.

### Optimize Airborne Motion

```bash
./scripts/sweep_airborne_push_pitch.sh 0.25 0.88,0.92,0.96 1.08,1.12,1.16 -8.0,-5.0,-2.0 -2.0,0.0 1
```

Use this sweep to compare push/flight settings by airborne metrics. The script ranks
results by:

- `airborne_forward_progress_m`
- `airborne_completion_ratio`
- final-distance error

## Current Reference Settings

### Takeoff-Speed Curve

The calibrated distance-to-speed curve is:

- `0.20 m -> 1.09`
- `0.25 m -> 1.06`
- `0.30 m -> 1.06`

### YAML Baseline

The YAML default remains the quickest regression check for the stack. It combines:

- `takeoff_angle_deg = 35.0`
- `push_front_tau_scale = 0.96`
- `push_rear_tau_scale = 1.12`
- `push_pitch_target_deg = -5.0`
- `flight_pitch_target_deg = -2.0`
- `landing_support_blend = 0.40`
- `landing_touchdown_reference_blend = 0.80`

The March 25, 2026 baseline report at `0.25 m` (`reports/jump_metrics/trial_20260325_195717.txt`)
recorded approximately:

- `final_forward_displacement_m ~= 0.335`
- `airborne_forward_progress_m ~= 0.111`
- `post_landing_forward_gain_m ~= 0.223`
- `landing_pitch_deg ~= -58.1`
- `final_pitch_deg ~= -33.7`

This is still useful as a stable baseline, but it does not yet satisfy the project
goal of a mostly-airborne forward jump.

### Current Experimental Airborne Reference

The current best near-target airborne-first probe is now:

```bash
./scripts/run_airborne_priority_trial.sh 0.25
```

That helper currently sets:

- `takeoff_speed_scale = 1.04`
- `push_front_tau_scale = 1.04`
- `push_rear_tau_scale = 1.04`
- `push_pitch_target_deg = 8.0`
- `flight_pitch_target_deg = 6.0`
- `push_front_compact_delta_rad = -0.155`
- `push_rear_compact_delta_rad = 0.055`
- `flight_front_compact_delta_rad = 0.24`
- `flight_rear_compact_delta_rad = -0.14`
- `support_thigh_rad = 1.06`
- `support_calf_rad = -2.04`
- `support_front_compact_delta_rad = -0.04`
- `support_rear_compact_delta_rad = 0.12`
- `landing_support_blend = 0.30`
- `recovery_upright_blend = 0.70`

Repeated March 25, 2026 validations
(`reports/jump_metrics/trial_20260325_222842.txt` and
`reports/jump_metrics/trial_20260325_222913.txt`) recorded approximately:

- `final_forward_displacement_m ~= 0.253-0.255`
- `airborne_forward_progress_m ~= 0.163-0.165`
- `post_landing_forward_gain_m ~= 0.060-0.063`
- `landing_pitch_deg ~= -46.7 to -48.0`
- `final_pitch_deg ~= -39.4 to -39.6`

This is the current best engineering tradeoff in the repository: most of the
forward motion is now completed in flight, the post-landing catch-up is much
smaller than the YAML baseline, and the body attitude is noticeably more natural.

### Naturality-First Nearby Variant

A slightly more upright nearby variant used:

- `push_front_compact_delta_rad = -0.15`
- `push_rear_compact_delta_rad = 0.06`
- `push_pitch_target_deg = 8.0`
- `flight_pitch_target_deg = 6.0`

That variant reduced touchdown pitch further to roughly `-44.9 to -45.3 deg` and
kept `post_landing_forward_gain_m` around `0.047-0.056 m`, but it typically
finished slightly short at roughly `0.234-0.244 m`.

### Controller Snapshot on March 25, 2026

The latest controller iteration added:

- event-aware `push` and `flight` phases
- configurable landing/recovery damping
- forward-bias shaping in `crouch` and `push`
- a support-hold landing path with extra recovery diagnostics
- continuous touchdown-reference blending for landing support tuning
- launch and script overrides for `push/flight/landing` compactness surfaces

What this changed in practice:

- the controller now separates post-landing motion into
  `support_hold_forward_gain_m` and `release_to_complete_forward_gain_m`
- aggressive push/flight pitch targets plus explicit compactness-shape tuning can
  pull `airborne_forward_progress_m` into the `0.16 m` range at `0.25 m` target trials
- the current best experimental reference now keeps `post_landing_forward_gain_m`
  near `0.06 m` while staying near the requested target distance
- touchdown posture is still visibly nose-down, but materially cleaner than the
  older landing-support baseline

## Current Known Limitations

- `foot_force_est` is still zero in the present MuJoCo bridge path, so touchdown
  detection is heuristic.
- the current best experimental reference still settles about `0.06 m` beyond the
  measured airborne progress
- even the improved experimental reference still lands nose-down at roughly
  `-46 to -48 deg`
- the project is simulation-first and should not be treated as a hardware-ready jump
  controller

## Documentation Map

- [中文文档](README.zh-CN.md)
- [Control Stack](docs/control_stack.md)
- [控制链路说明](docs/control_stack.zh-CN.md)
- [Calibration Workflow](docs/calibration_workflow.md)
- [调参流程](docs/calibration_workflow.zh-CN.md)
- [Development Workflow](docs/development_workflow.md)
- [开发流程](docs/development_workflow.zh-CN.md)
