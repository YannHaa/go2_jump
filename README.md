# Go2 Jump MPC Workspace

[中文文档](README.zh-CN.md)

This repository has been reset onto a new mainline focused on one question:

how to build a high-quality, distance-conditioned forward jump controller for
Unitree Go2 using low-level control and a MuJoCo-native whole-body MPC stack.

The previous template-controller experiment has been intentionally removed from
the active project structure. The new repository is organized for research,
system identification, and paper-grade experiments.

## Mainline Direction

- simulator: `unitree_mujoco`
- communication: `unitree_ros2` + `LowCmd` / `LowState`
- task interface: distance-conditioned jump task specification
- controller goal: `MuJoCo-native whole-body MPC`
- landing strategy: contact-aware reactive landing
- evaluation target: airborne-dominant forward jump with clean touchdown

## Current Repository Layout

- `src/unitree_ros2`
  Upstream Unitree ROS 2 dependency.
- `src/unitree_mujoco`
  Upstream Unitree MuJoCo dependency.
- `src/go2_jump_msgs`
  ROS 2 interfaces for jump-task exchange.
- `src/go2_jump_core`
  Distance-conditioned jump task generation and reference sampling.
- `src/go2_jump_mpc`
  New whole-body MPC mainline package.
- `src/go2_jump_bringup`
  Launch files and shared parameter files for the MPC stack.
- `docs/`
  Architecture and research-program documents.
- `scripts/`
  Docker build and runtime helpers for the new stack.
- `patches/`
  Reproducible local patches applied on top of upstream dependencies.

## What Exists Today

The current reset already provides a verified minimum closed loop:

- Docker-based build for `unitree_ros2`, `go2_jump_*`, and `unitree_mujoco`
- a task-level jump specification
- a contact-aware preview controller scaffold with signal-validity guards
- `LowCmd` publication at controller rate
- `JumpControllerState` diagnostics for controller bring-up
- reproducibility hooks for upstream dependency patches

Verified on the current host:

- `/lowstate` publishes from `unitree_mujoco`
- `/lowcmd` publishes at about `200 Hz` when `enable_lowcmd_output=true`
- `/go2_jump/controller_state` is available for phase and contact debugging

The whole-body MPC optimizer itself is not finished yet. The current backend is
`reference_preview`, which keeps the external control chain stable while the
MuJoCo-native optimizer is rebuilt.

## Quick Start

```bash
cd /home/hayan/go2_jump_ws
./scripts/bootstrap_workspace_repo.sh
./scripts/bootstrap_third_party.sh
./scripts/docker_build_image.sh
./scripts/docker_build_workspace.sh
```

To start the simulator:

```bash
./scripts/docker_run_go2_mujoco.sh
```

To launch the new MPC-oriented stack:

```bash
./scripts/docker_launch_jump_mpc.sh 0.25
```

To run an end-to-end smoke test:

```bash
./scripts/docker_smoke_test_stack.sh 0.25
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true ./scripts/docker_smoke_test_stack.sh 0.20
```

## Recommended Reading Order

1. Read this file for the reset rationale and workspace entrypoints.
2. Read [Algorithm](algorithm.md) for the planner / controller / backend split.
3. Read [Architecture](docs/architecture.md) for the new package split.
4. Read [Research Program](docs/research_program.md) for the MPC roadmap and
   experiment plan.

## Current Assumptions

- The repository still relies on `unitree_mujoco` and `unitree_ros2`.
- A local compatibility patch is kept for the MuJoCo bridge under `patches/`.
- Docker remains the primary environment because it keeps ROS 2, MuJoCo, and
  Unitree dependencies reproducible across machines.
