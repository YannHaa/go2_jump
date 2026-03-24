# Development Workflow

## Repository Strategy

This workspace keeps the custom jump work in the root repository and keeps the two
large vendor dependencies pinned as submodules:

- `src/unitree_ros2`
- `src/unitree_mujoco`

That split keeps our own code reviewable while still following the official upstream
repositories closely.

## Local Compatibility Patch

`unitree_mujoco` currently needs one local compatibility patch on top of the pinned
upstream commit:

- [0001-go2-sim-compat.patch](/home/hayan/go2_jump_ws/patches/unitree_mujoco/0001-go2-sim-compat.patch)

That patch carries the DDS bridge compatibility aliases and the headless-stability
fixes that were required on this host.

Apply it manually with:

```bash
cd /home/hayan/go2_jump_ws
./scripts/apply_local_patches.sh
```

or let the build helper do it automatically.

## Fresh Clone Flow

After cloning the root repository:

```bash
cd /home/hayan/go2_jump_ws
git submodule update --init --recursive
./scripts/bootstrap_workspace_repo.sh
./scripts/bootstrap_third_party.sh
./scripts/docker_build_image.sh
./scripts/docker_build_workspace.sh
```

`bootstrap_workspace_repo.sh` does two things:

- syncs and initializes the submodules
- applies the local `unitree_mujoco` patch if needed

## Updating Upstream Dependencies

When we intentionally move either upstream repository:

1. update the submodule checkout to the new upstream commit
2. re-apply or refresh the local patch if needed
3. rebuild the workspace
4. rerun the low-level verification and a fresh jump trial

Do not commit ad hoc edits directly inside the submodule without also capturing the
change in `patches/`.

## Verification Paths

There are now two quick validation layers:

- direct DDS validation

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_verify_rt_lowcmd.sh
```

This runs the standalone `tools/verify_rt_lowcmd` utility against `rt/lowcmd` and
`rt/lowstate` to confirm that the simulator reacts to low-level commands.

- full ROS 2 jump-stack validation

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_run_single_jump_trial.sh 0.25
```

This exercises the planner, controller, `/lowcmd`, `/lowstate`, and report
generation end to end.
