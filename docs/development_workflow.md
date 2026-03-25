# Development Workflow

## Purpose

This document describes how the repository is structured and how to update it
without losing reproducibility.

The main design rule is simple:

- keep project-specific code in the root repository
- keep upstream vendor code in pinned submodules
- store local upstream modifications as explicit patches

## Repository Policy

### Root Repository

The root repository tracks:

- custom ROS 2 packages
- build and runtime scripts
- documentation
- local patch files

### Upstream Submodules

The following directories are tracked as submodules:

- `src/unitree_ros2`
- `src/unitree_mujoco`

This keeps upstream history intact and makes it easier to update dependencies in a
controlled way.

### Local Patch Policy

The local `unitree_mujoco` compatibility changes are stored in:

- [`patches/unitree_mujoco/0001-go2-sim-compat.patch`](../patches/unitree_mujoco/0001-go2-sim-compat.patch)

Do not leave important changes only inside the submodule working tree. If a local
submodule modification is required for the project, it should also be captured under
`patches/`.

## Fresh Clone Procedure

After cloning the repository:

```bash
cd /home/hayan/go2_jump_ws
git submodule update --init --recursive
./scripts/bootstrap_workspace_repo.sh
./scripts/bootstrap_third_party.sh
./scripts/docker_build_image.sh
./scripts/docker_build_workspace.sh
```

## Daily Development Loop

### Rebuild After Code Changes

```bash
./scripts/docker_build_workspace.sh
```

### Run a Focused Validation

Choose the narrowest useful validation:

- DDS bridge only

```bash
./scripts/docker_verify_rt_lowcmd.sh
```

- full jump stack

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

### Review the Trial Report

Check:

- `airborne_forward_progress_m`
- `final_forward_displacement_m`
- `max_abs_pitch_deg`
- `max_joint_tracking_error_rad`

## Updating a Submodule

When intentionally updating `unitree_ros2` or `unitree_mujoco`:

1. move the submodule to the target upstream commit
2. re-apply the local patch if required
3. rebuild the workspace
4. rerun DDS verification
5. rerun at least one full jump trial

Do not update a submodule and skip validation. The bridge, message definitions, and
runtime libraries are tightly coupled.

## Build and Runtime Notes

### Third-Party Cache

Optional cached dependencies may be stored under `third_party/`. The build scripts
reuse them when available and fall back to official upstream sources otherwise.

### MuJoCo Runtime

On systems without a graphical desktop, `docker_run_go2_mujoco.sh` starts `Xvfb`
automatically. This keeps the MuJoCo render loop alive while still allowing the
simulator to run in a headless workflow.

### CycloneDDS Runtime

When running Unitree SDK2 binaries manually, keep `/opt/unitree_robotics/lib` ahead
of the workspace CycloneDDS libraries in `LD_LIBRARY_PATH`. This prevents mixed DDS
runtime combinations that can destabilize the MuJoCo bridge.

## Suggested Commit Discipline

- commit infrastructure changes separately from controller tuning when practical
- record the reason for any default-parameter change in the commit message
- mention the validation command used after a behavior change
- push only after the workspace is buildable and at least one targeted validation has
  passed
