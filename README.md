# Go2 Jump Workspace

This workspace is set up for a first-pass Go2 forward-jump workflow based on:

- `unitree_ros2` for `LowCmd` and `LowState`
- `unitree_mujoco` as the primary simulator
- a custom low-level ROS 2 controller that stays below sport mode

## Why Docker is used here

The current host is `Ubuntu 22.04`, which matches the recommended `ROS 2 Humble` route, but the current user does not have passwordless `sudo`. Because of that, the main environment is reproduced with Docker using the same `Humble` direction Unitree documents in its devcontainer and CI.

## Workspace layout

- `src/unitree_ros2`: official Unitree ROS 2 repository
- `src/unitree_mujoco`: official Unitree MuJoCo repository
- `src/go2_jump_planner`: jump parameterization library and target publisher
- `src/go2_jump_controller`: low-level jump controller node
- `src/go2_jump_bringup`: launch and config package
- `docker/Dockerfile.humble`: reproducible Humble image
- `scripts/`: image build, workspace build, sim launch, and stack launch helpers

## Documentation

- [Control Stack Overview](/home/hayan/go2_jump_ws/docs/control_stack.md)
- [Calibration Workflow](/home/hayan/go2_jump_ws/docs/calibration_workflow.md)
- [Development Workflow](/home/hayan/go2_jump_ws/docs/development_workflow.md)

## Repository strategy

This repository keeps the custom jump code in the root workspace and pins the two
official upstream dependencies as submodules:

- `src/unitree_ros2`
- `src/unitree_mujoco`

`unitree_mujoco` also carries one local compatibility patch under
[patches/unitree_mujoco](/home/hayan/go2_jump_ws/patches/unitree_mujoco/0001-go2-sim-compat.patch).
The workspace bootstrap and build helpers apply that patch automatically when needed.

## First build

```bash
cd /home/hayan/go2_jump_ws
./scripts/bootstrap_workspace_repo.sh
./scripts/bootstrap_third_party.sh
./scripts/docker_build_image.sh
./scripts/docker_build_workspace.sh
```

If you skip `./scripts/bootstrap_third_party.sh`, the Docker image build now falls
back to the official upstream sources automatically.

For a fresh clone from GitHub, prefer:

```bash
git clone --recurse-submodules https://github.com/matrix050628/go2_jump.git /home/hayan/go2_jump_ws
cd /home/hayan/go2_jump_ws
./scripts/bootstrap_workspace_repo.sh
```

## Source the environment inside the container

```bash
source /workspace/scripts/container_source_env.sh
```

That script sources:

- `/opt/ros/humble/setup.bash`
- `unitree_ros2/cyclonedds_ws/install/setup.bash`
- `unitree_ros2/example/install/setup.bash`
- `/workspace/install/setup.bash`

and also:

- forces CycloneDDS onto loopback for simulation
- sets `ROS_DOMAIN_ID=1` to match `unitree_mujoco/simulate/config.yaml`

## Launch flow

Terminal 1, start MuJoCo:

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_run_go2_mujoco.sh
```

On this remote host there is no `DISPLAY`, so the helper script:

- starts a private `Xvfb` instance manually instead of relying on `xvfb-run`
- forces `libddsc.so` and `libddscxx.so` to both load from `/opt/unitree_robotics/lib`

It intentionally does **not** set `UNITREE_MUJOCO_HEADLESS=1`. In the current
`unitree_mujoco` codepath that flag skips the GLFW render loop and leaves the main thread
sleeping, which keeps DDS alive but can leave physics stepping stalled. Running through
`Xvfb` gives a real virtual display while still keeping the simulator headless from the host's
point of view.

That last point is important: the crash was caused by a mixed CycloneDDS runtime where
`libddsc.so` came from `unitree_ros2/cyclonedds_ws/install/cyclonedds/lib` while
`libddscxx.so` came from `/opt/unitree_robotics/lib`. With that mismatch,
`unitree_mujoco` aborted with `free(): invalid pointer` during DDS topic construction for
`LowState`, `SportModeState`, and `WirelessController`.

Terminal 2, start planner + controller:

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_launch_jump_stack.sh 0.25
```

The default jump target is `0.25 m`.

For a repeatable one-shot trial that starts a fresh simulator, runs the jump stack,
and archives the resulting report:

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_run_single_jump_trial.sh 0.25
```

The one-shot helper now starts both MuJoCo and the ROS 2 jump stack in detached
containers, tails their logs, waits for a genuinely new report file, archives the
report under `reports/jump_metrics/`, and then stops both runtime containers. This
avoids the earlier failure mode where `timeout` killed the host-side `docker run`
client but left old ROS 2 publishers or MuJoCo processes alive in the background.

For a direct low-level DDS check below the ROS 2 controller path:

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_verify_rt_lowcmd.sh
```

That helper starts a fresh MuJoCo instance and runs the standalone
`tools/verify_rt_lowcmd` utility against `rt/lowcmd` and `rt/lowstate`.

## Third-party dependency handling

The workspace can use optional local cache entries under `third_party/`:

- `third_party/unitree_sdk2`
- `third_party/mujoco-3.3.6-linux-x86_64.tar.gz`

If they are present, the Docker build reuses them. If they are absent, the image
build falls back to the official upstream sources through
`docker/bootstrap_third_party.sh`.

## Current controller behavior

The first controller version is intentionally simple:

- target distance -> ballistic takeoff-speed estimate
- optional `takeoff_speed_scale` calibration to compensate for low-level execution losses
- optional distance-indexed `takeoff_speed_scale` curve for automatic interpolation
- stand -> crouch -> push -> flight -> landing -> recovery phase machine
- front/rear-biased joint trajectories for pitch shaping
- PD control on all actuated leg joints
- parameterized feedforward torque hooks for push and landing phases
- IMU pitch and pitch-rate feedback that adjusts leg compactness during push, flight, and landing

It is designed for flat-ground short hops in simulation first, not for direct use on hardware.

## Rebuild after code changes

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_build_workspace.sh
```

For parameter-only changes in [jump_params.yaml](/home/hayan/go2_jump_ws/src/go2_jump_bringup/config/jump_params.yaml), a rebuild is not required because the launch helper bind-mounts the workspace into the runtime container.

You can also override the runtime launch parameters without editing YAML:

```bash
cd /home/hayan/go2_jump_ws
GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE=false \
GO2_JUMP_TAKEOFF_SPEED_SCALE=1.09 \
./scripts/docker_run_single_jump_trial.sh 0.20
```

## Calibration sweep

For multi-distance takeoff-speed calibration, use:

```bash
cd /home/hayan/go2_jump_ws
./scripts/sweep_takeoff_speed_scale.sh 0.20,0.25,0.30 1.00,1.03,1.06 1
```

The sweep helper:

- reuses the fresh one-shot trial helper for every run
- writes a raw CSV under `reports/calibration/`
- writes a human-readable summary with recommendations by final displacement and by landing displacement

On this host, the latest focused sweeps produced:

- final-displacement recommendations:
  `0.20 m -> takeoff_speed_scale ~= 1.09`
  `0.25 m -> takeoff_speed_scale ~= 1.06`
  `0.30 m -> takeoff_speed_scale ~= 1.06`
- the default runtime config now uses a distance-indexed scale curve with those values
- a strong warning that landing displacement is still far below target even when final displacement is accurate, so the current stack is better calibrated for final settled position than for true airborne range

## Notes

- The current shell on the host is polluted by Conda, so system Python is avoided for ROS builds.
- The current remote shell has no `DISPLAY`, so the MuJoCo helper launches `Xvfb` directly on `:199` by default.
- The controller does not call any sport-mode front-jump behavior. It publishes `LowCmd` directly.
- `go2_jump_planner`, `go2_jump_controller`, `go2_jump_bringup`, `stand_go2`, `unitree_ros2_example`, and `unitree_mujoco` all build successfully inside the Docker image.
- `tools/verify_rt_lowcmd` is also built inside the Docker image and is wrapped by `./scripts/docker_verify_rt_lowcmd.sh`.
- Current stable path: `unitree_mujoco` now runs with the full bridge enabled, including both `rt/*` topics and the compatibility aliases `/lowcmd`, `/lowstate`, `/sportmodestate`, and `/wirelesscontroller`.
- Verified on this host: `ros2 topic list` sees those topics, `ros2 topic echo /lowstate --once` receives real state, and `go2_jump_controller` completes its `crouch -> push -> flight -> landing -> recovery` sequence against the MuJoCo simulator.
- Verified low-level effect: in a fresh simulator run, the steady baseline was `q0=-0.1403`, `q1=1.2210`, `q2=-2.7202`, `pitch=-0.1040`, and during the jump-controller run `/lowstate` moved through `q1=[0.5937, 1.2800]`, `q2=[-2.7229, -1.1867]`, `pitch=[-0.8697, 1.1360]`, confirming that published `LowCmd` messages are actually affecting the simulated robot.
- The controller now emits a `jump_trial_report` at the end of a run and writes it to `/workspace/reports/jump_metrics/latest_report.txt`. The one-shot trial helper also archives timestamped copies under `/home/hayan/go2_jump_ws/reports/jump_metrics/`.
- The trial report now separates `takeoff_forward_displacement_m`, `airborne_forward_progress_m`, `landing_forward_displacement_m`, and `post_landing_forward_gain_m`, which makes it much easier to see whether a tuning change improved the true jump or only the post-landing recovery motion.
- The controller now logs startup-readiness diagnostics once per second while waiting to auto-start, which makes it much easier to distinguish “DDS is broken” from “the robot never reached a stable pre-jump pose”.
- Current tuned baseline on this host uses `takeoff_angle_deg=45.0` and a distance-indexed scale curve configured in [jump_params.yaml](/home/hayan/go2_jump_ws/src/go2_jump_bringup/config/jump_params.yaml).
- The current curve reproduces the recent empirical recommendations:
  `0.20 m -> 1.09`
  `0.25 m -> 1.06`
  `0.30 m -> 1.06`
- The earlier `takeoff_angle_deg=35.0` experiment regressed to roughly `0.186 m` to `0.190 m`, so the current improvement came from explicit takeoff-speed calibration rather than lowering the takeoff angle.
- See [Calibration Workflow](/home/hayan/go2_jump_ws/docs/calibration_workflow.md) for the latest sweep findings and interpretation guidance.
- Current limitation: in this MuJoCo bridge path `foot_force_est` is still reported as zero during the trials above, so touchdown detection still relies on height and vertical-velocity heuristics instead of a real contact-force event.
- Remaining caution: if you run `unitree_sdk2` binaries manually in an ad hoc shell, keep `/opt/unitree_robotics/lib` ahead of the workspace CycloneDDS library path, otherwise the mixed `ddsc` / `ddscxx` runtime can reintroduce the allocator crash.
