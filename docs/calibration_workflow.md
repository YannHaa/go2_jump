# Calibration Workflow

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

### Step 2: Fit the Takeoff-Speed Curve

```bash
./scripts/sweep_takeoff_speed_scale.sh 0.20,0.25,0.30 1.00,1.03,1.06 1
```

Use this sweep when the main problem is distance tracking.

The sweep writes:

- a raw CSV under `reports/calibration/`
- a summary file under `reports/calibration/`

### Step 3: Improve Airborne Motion

```bash
./scripts/sweep_airborne_push_pitch.sh 0.25 0.88,0.92,0.96 1.08,1.12,1.16 -8.0,-5.0,-2.0 -2.0,0.0 1
```

Use this sweep after the takeoff-speed curve is already reasonable.

This script compares:

- front/rear push torque scaling
- push-phase pitch target
- flight-phase pitch target

and ranks the results by airborne performance and final-distance error.

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

### Manual Push / Pitch Override

```bash
GO2_JUMP_PUSH_FRONT_TAU_SCALE=0.96 \
GO2_JUMP_PUSH_REAR_TAU_SCALE=1.12 \
GO2_JUMP_PUSH_PITCH_TARGET_DEG=-2.0 \
GO2_JUMP_FLIGHT_PITCH_TARGET_DEG=0.0 \
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

## Current Limitations

- `foot_force_est` is still zero in the present MuJoCo bridge path
- touchdown detection is therefore heuristic rather than contact-driven
- the measured landing position should be interpreted as an event estimate, not as a
  direct contact-force measurement

## Recommended Next Experiment

The next useful experiment is to keep the aggressive airborne mode and retune
`takeoff_speed_scale` downward so that:

- `airborne_forward_progress_m` stays high
- final displacement returns closer to the `0.25 m` target
