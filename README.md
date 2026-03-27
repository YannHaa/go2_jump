# Go2 Jump MPC Workspace

[中文文档](README.zh-CN.md)

This repository is focused on one engineering target:

build a distance-conditioned forward jump controller for Unitree Go2 on the
`LowCmd / LowState` interface, using `unitree_mujoco` as the primary simulator
and a MuJoCo-native whole-body MPC line as the main control direction.

The old template-only jump experiment is no longer the project baseline. The
active tree is organized around simulation fidelity, contact-aware control, and
repeatable backend iteration.

## Mainline

- simulator: `unitree_mujoco`
- transport: `unitree_ros2`
- task interface: `JumpTask`
- low-level command path: `LowCmd`
- core control package: `go2_jump_mpc`
- long-term target: `contact-aware planner -> MuJoCo-native MPC -> low-level execution`

## Repository Layout

- `src/unitree_ros2`
  Upstream Unitree ROS 2 dependency.
- `src/unitree_mujoco`
  Upstream Unitree MuJoCo dependency, with local compatibility fixes.
- `src/go2_jump_msgs`
  ROS 2 messages for jump tasks and controller diagnostics.
- `src/go2_jump_core`
  Distance-conditioned task construction and reference sampling.
- `src/go2_jump_mpc`
  Contact estimation, phase management, preview control, and MuJoCo-native MPC backend.
- `src/go2_jump_bringup`
  Launch files and shared parameter sets.
- `scripts/`
  Docker build, launch, and smoke-test helpers.
- `docs/`
  Architecture and research notes.

## Current Status

The current workspace has a working minimum closed loop and one experimental
native backend.

Verified:

- headless `unitree_mujoco` now steps correctly and publishes non-zero `/lowstate`
- `/lowstate` carries non-zero joint state, IMU state, and tick progression
- `foot_force` / `foot_force_est` are no longer transport placeholders; during active low-level control they now reflect MuJoCo contact-derived support loads
- `go2_jump_mpc` publishes `/lowcmd` through the real low-level command path; with the current native backend configuration the observed publication rate is typically in the `80-100 Hz` range
- `reference_preview` remains the stable baseline backend
- `mujoco_native_mpc` is integrated, builds cleanly, launches, and drives the same low-level command path
- repeatable single-trial reports can be generated directly from the Docker workflow and are written to `reports/trials/`

Current limitation:

- `mujoco_native_mpc` is not yet a high-quality forward jump controller
- the backend can now produce cleaner airborne-dominant motion than the earlier template baseline, but distance tracking is still below target and run-to-run consistency still needs work

In other words, the project is past bring-up, but still in controller-development
mode rather than benchmark-ready mode.

## Quick Start

Build the workspace:

```bash
cd /home/hayan/go2_jump_ws
./scripts/bootstrap_workspace_repo.sh
./scripts/bootstrap_third_party.sh
./scripts/docker_build_image.sh
./scripts/docker_build_workspace.sh
```

Run the simulator:

```bash
./scripts/docker_run_go2_mujoco.sh
```

Launch the jump stack with the stable preview backend:

```bash
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true \
./scripts/docker_launch_jump_mpc.sh 0.25
```

Launch the MuJoCo-native backend:

```bash
GO2_JUMP_SOLVER_BACKEND=mujoco_native_mpc \
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true \
./scripts/docker_launch_jump_mpc.sh 0.25
```

Run the smoke test:

```bash
./scripts/docker_smoke_test_stack.sh 0.25
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true \
./scripts/docker_smoke_test_stack.sh 0.25
GO2_JUMP_SOLVER_BACKEND=mujoco_native_mpc \
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true \
./scripts/docker_smoke_test_stack.sh 0.25
```

Run one instrumented trial and save a report:

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

The script writes `summary.json`, `stack.log`, `sim.log`, and `recorder.log`
under `reports/trials/<timestamp>_d<distance>_mujoco_native_mpc/`.

Run a small repeated batch and aggregate the result:

```bash
GO2_JUMP_BATCH_REPEATS=2 \
GO2_JUMP_BATCH_DISTANCES="0.20 0.25 0.30 0.35" \
./scripts/docker_run_jump_batch.sh
```

The batch helper writes a manifest and aggregated statistics under
`reports/batches/<timestamp>/`.

## Recommended Reading Order

1. Read this file for the repository entrypoints and current status.
2. Read [Algorithm](algorithm.md) for the planner / controller / backend split.
3. Read [Architecture](docs/architecture.md) for the package structure.
4. Read [Research Program](docs/research_program.md) for the roadmap beyond the current controller.

## Practical Notes

- Docker is the reference environment.
- `reference_preview` is still the safest backend when the goal is transport and interface validation.
- `mujoco_native_mpc` is the active development path.
- The most useful debug topics today are `/lowstate`, `/lowcmd`, and `/go2_jump/controller_state`.
- For controller tuning, prefer `./scripts/docker_run_single_jump_trial.sh <distance>` over ad-hoc `ros2 topic echo`, because it records phase order, airborne displacement, and lowcmd rate in one place.
