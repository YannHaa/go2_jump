# Calibration Workflow

[中文](calibration_workflow.zh-CN.md)

## Purpose

This document explains how to tune the jump stack in a repeatable way.

The calibration process is split into two stages:

1. fit the takeoff-speed scale so the final distance is roughly correct
2. improve true airborne forward motion without losing control of the final result

## Before You Start

Make sure the workspace builds and the simulator responds to low-level commands.

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_build_workspace.sh
./scripts/docker_verify_rt_lowcmd.sh
```

## Recommended Tuning Order

### Step 1: Run a Baseline Trial

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

This gives you a reference report for the current default configuration.

### Step 2: Select a Working Profile

If you want to compare controller behavior systematically, select a named profile
before starting a sweep.

```bash
GO2_JUMP_PROFILE=conservative_airborne \
./scripts/docker_run_single_jump_trial.sh 0.25
```

```bash
GO2_JUMP_PROFILE=aggressive_airborne \
./scripts/docker_run_single_jump_trial.sh 0.25
```

### Step 3: Fit the Takeoff-Speed Curve

```bash
./scripts/sweep_takeoff_speed_scale.sh 0.20,0.25,0.30 1.00,1.03,1.06 1
```

Use this sweep when the main problem is distance tracking.

The sweep writes:

- a raw CSV under `reports/calibration/`
- a summary file under `reports/calibration/`

### Step 4: Improve Airborne Motion

```bash
./scripts/sweep_airborne_push_pitch.sh 0.25 0.88,0.92,0.96 1.08,1.12,1.16 -8.0,-5.0,-2.0 -2.0,0.0 1
```

Use this sweep after the takeoff-speed curve is already reasonable.

This script compares:

- front/rear push torque scaling
- push-phase pitch target
- flight-phase pitch target

and ranks the results by airborne performance and final-distance error.

### Step 5: Re-Fit Takeoff Speed for a Stronger Profile

When a profile improves airborne motion but overshoots the final distance, keep the
profile and retune only `takeoff_speed_scale`.

```bash
./scripts/sweep_profile_takeoff_speed_scale.sh aggressive_airborne 0.25 0.94,0.97,1.00,1.03 1
```

This is the recommended follow-up after the current aggressive airborne probe.

## Fast Command Reference

### Build

```bash
./scripts/docker_build_workspace.sh
```

### Single Trial

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

### Manual Takeoff-Speed Override

```bash
GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE=false \
GO2_JUMP_TAKEOFF_SPEED_SCALE=1.09 \
./scripts/docker_run_single_jump_trial.sh 0.20
```

### Run with a Named Profile

```bash
GO2_JUMP_PROFILE=aggressive_airborne \
./scripts/docker_run_single_jump_trial.sh 0.25
```

### Manual Push / Pitch Override

```bash
GO2_JUMP_PUSH_FRONT_TAU_SCALE=0.96 \
GO2_JUMP_PUSH_REAR_TAU_SCALE=1.12 \
GO2_JUMP_PUSH_PITCH_TARGET_DEG=-2.0 \
GO2_JUMP_FLIGHT_PITCH_TARGET_DEG=0.0 \
./scripts/docker_run_single_jump_trial.sh 0.25
```

### Landing Support Override

```bash
GO2_JUMP_LANDING_SUPPORT_BLEND=0.40 \
GO2_JUMP_LANDING_TOUCHDOWN_REFERENCE_BLEND=0.80 \
./scripts/docker_run_single_jump_trial.sh 0.25
```

## How to Read a Trial Report

### Distance Metrics

- `final_forward_displacement_m`
  Final settled displacement after recovery.
- `landing_forward_displacement_m`
  Forward displacement at landing detection.
- `airborne_forward_progress_m`
  Forward motion from takeoff to landing detection.
- `post_landing_forward_gain_m`
  Forward motion accumulated after landing detection.
- `support_hold_forward_gain_m`
  Forward motion accumulated while the controller is still holding the landing
  support pose.
- `release_to_complete_forward_gain_m`
  Forward motion accumulated after the controller starts releasing toward the
  recovery target.

### Ratio Metrics

- `airborne_completion_ratio`
  `airborne_forward_progress_m / target_distance_m`
- `post_landing_completion_ratio`
  `post_landing_forward_gain_m / target_distance_m`

These ratios are useful when comparing runs with the same target distance.

### Practical Interpretation

- high `final_forward_displacement_m` with low `airborne_forward_progress_m`
  usually means recovery motion is compensating for a weak jump
- rising `airborne_forward_progress_m` with stable or lower
  `max_abs_pitch_deg` is usually a good sign
- large gains in airborne progress paired with large final overshoot often mean the
  configuration should be treated as an exploration mode, not a new default
- large positive `push_extension_after_plan_s` means the nominal push window is too
  short to reach true takeoff
- large positive `flight_extension_after_plan_s` means the controller still lands
  later than the ballistic estimate expects

## Reference Results

### Takeoff-Speed Curve

The focused sweeps on March 24, 2026 produced the following working curve:

- `0.20 m -> 1.09`
- `0.25 m -> 1.06`
- `0.30 m -> 1.06`

The corresponding summary files are generated locally under
`reports/calibration/`:

- `speed_scale_sweep_20260324_214558_summary.txt`
- `speed_scale_sweep_20260324_215112_summary.txt`

### Airborne Sweep on March 25, 2026

The focused airborne sweep summary is:

- `reports/calibration/airborne_sweep_20260325_005214_summary.txt`

Two settings are especially useful as references at `0.25 m`:

- conservative improvement
  `push_front_tau_scale=0.96`, `push_rear_tau_scale=1.12`,
  `push_pitch_target_deg=-5.0`, `flight_pitch_target_deg=-2.0`
  Result:
  `avg_final_m ~= 0.2512`, `avg_airborne_m ~= 0.0462`
- aggressive airborne exploration
  `push_front_tau_scale=0.96`, `push_rear_tau_scale=1.12`,
  `push_pitch_target_deg=-2.0`, `flight_pitch_target_deg=0.0`
  Result:
  `avg_final_m ~= 0.3060`, `avg_airborne_m ~= 0.0655`

The conservative setting is the current default because it improves airborne
progress while keeping the final displacement near the target.

### First Retuned Aggressive Check on March 25, 2026

The first single-point re-fit of the aggressive airborne profile used:

- `GO2_JUMP_PROFILE=aggressive_airborne`
- `GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE=false`
- `GO2_JUMP_TAKEOFF_SPEED_SCALE=1.00`

Result from the one-shot validation:

- `final_forward_displacement_m ~= 0.2593`
- `airborne_forward_progress_m ~= 0.0527`

This is not yet a promoted default, but it confirms that the aggressive posture
shape can be retained while pulling final distance back toward the target.

### Controller Refactor Sweep on March 25, 2026

After the event-aware phase refactor and forward-bias landing experiments, the
focused re-fit summary is:

- `reports/calibration/speed_scale_sweep_20260325_163702_summary.txt`

Key takeaways from that sweep:

- the stable controller path can now reach roughly `0.08-0.10 m` airborne progress
  at a `0.25 m` target
- lowering `takeoff_speed_scale` alone is not yet enough to bring the final
  displacement back near target, because post-landing forward gain still dominates
- an experimental touchdown-hold landing branch can suppress post-landing gain, but
  it is currently too unstable to become the default

### Airborne-First Reference on March 25, 2026

The landing-support iteration is still important, but the current best direction
is no longer a landing-only tweak. The stronger result came from combining:

- the same touchdown-aware landing support path
- more upright `push` and `flight` pitch targets
- explicit `push` and `flight` compactness-shape tuning

The fastest way to reproduce the current experimental reference is:

```bash
./scripts/run_airborne_priority_trial.sh 0.25
```

The script currently applies:

- `takeoff_speed_scale = 1.04`
- `push_front_tau_scale = 1.04`
- `push_rear_tau_scale = 1.04`
- `push_pitch_target_deg = 8.0`
- `flight_pitch_target_deg = 6.0`
- `push_front_compact_delta_rad = -0.155`
- `push_rear_compact_delta_rad = 0.055`
- `flight_front_compact_delta_rad = 0.24`
- `flight_rear_compact_delta_rad = -0.14`
- `landing_support_blend = 0.30`
- `landing_touchdown_reference_blend = 0.80`

Repeated `0.25 m` validations on March 25, 2026
(`reports/jump_metrics/trial_20260325_222842.txt` and
`reports/jump_metrics/trial_20260325_222913.txt`) reported approximately:

- `final_forward_displacement_m ~= 0.253-0.255`
- `airborne_forward_progress_m ~= 0.163-0.165`
- `post_landing_forward_gain_m ~= 0.060-0.063`
- `landing_pitch_deg ~= -46.7 to -48.0`
- `final_pitch_deg ~= -39.4 to -39.6`

This is the current best near-target compromise in the repository. It does not
fully solve touchdown posture, but it is much closer to the intended behavior:
most of the motion is now completed in flight and the post-landing catch-up is
kept comparatively small.

One nearby variant is worth keeping in mind when the priority is a cleaner body
shape over exact distance:

- `push_front_compact_delta_rad = -0.15`
- `push_rear_compact_delta_rad = 0.06`
- `push_pitch_target_deg = 8.0`
- `flight_pitch_target_deg = 6.0`

That variant typically landed around:

- `final_forward_displacement_m ~= 0.234-0.244`
- `airborne_forward_progress_m ~= 0.158-0.160`
- `post_landing_forward_gain_m ~= 0.047-0.056`
- `landing_pitch_deg ~= -44.9 to -45.3`

## Current Limitations

- `foot_force_est` is still zero in the present MuJoCo bridge path
- touchdown detection is therefore heuristic rather than contact-driven
- the measured landing position should be interpreted as an event estimate, not as a
  direct contact-force measurement

## Recommended Next Experiment

The next useful experiment is now to keep the current airborne-first reference and
search for a cleaner touchdown so that:

- `airborne_forward_progress_m` stays high
- `post_landing_forward_gain_m` drops further
- the robot lands with a more natural pitch attitude

The shortest path is:

```bash
./scripts/run_airborne_priority_trial.sh 0.25
```

The next technical step is not another large takeoff-speed sweep. It is either:

- finer `push_front_compact_delta_rad` / `push_rear_compact_delta_rad` refinement
  around `-0.155 / 0.055`
- or a dedicated descent-attitude shaping path that improves touchdown posture
  without reintroducing early landing-pose blending
