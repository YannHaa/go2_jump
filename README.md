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
3. Read [Calibration Workflow](docs/calibration_workflow.md) before tuning.
4. Read [Development Workflow](docs/development_workflow.md) before changing
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

### Conservative Default

The default control parameters are currently set to a conservative airborne
improvement mode:

- `push_front_tau_scale = 0.96`
- `push_rear_tau_scale = 1.12`
- `push_pitch_target_deg = -5.0`
- `flight_pitch_target_deg = -2.0`

This setting improves airborne motion while keeping the final distance close to the
`0.25 m` target.

### Aggressive Airborne Probe

The strongest airborne result in the latest focused sweep used:

- `push_front_tau_scale = 0.96`
- `push_rear_tau_scale = 1.12`
- `push_pitch_target_deg = -2.0`
- `flight_pitch_target_deg = 0.0`

This setting increased `airborne_forward_progress_m` to approximately `0.0655 m` in
the March 25, 2026 sweep, but it also pushed the final displacement to roughly
`0.306 m`. It is best treated as an exploration mode rather than the default.

### First Retuned Aggressive Reference

A single validation run on March 25, 2026 kept the aggressive airborne profile and
reduced `takeoff_speed_scale` to `1.00`:

- `final_forward_displacement_m ~= 0.2593`
- `airborne_forward_progress_m ~= 0.0527`

This is a useful next candidate when the goal is to keep a stronger airborne motion
without overshooting the `0.25 m` target as badly as the untuned aggressive setup.

## Current Known Limitations

- `foot_force_est` is still zero in the present MuJoCo bridge path, so touchdown
  detection is heuristic.
- airborne range is still smaller than final settled displacement
- recovery motion still contributes a large fraction of forward progress
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
