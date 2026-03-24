# Calibration Workflow

## Purpose

This document records the current takeoff-speed calibration workflow for the Go2 jump
stack and explains how to interpret the reported metrics.

## Fast Commands

Build the workspace after code changes:

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_build_workspace.sh
```

Run a single fresh trial:

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_run_single_jump_trial.sh 0.25
```

Override the takeoff-speed scale manually:

```bash
cd /home/hayan/go2_jump_ws
GO2_JUMP_USE_TAKEOFF_SPEED_SCALE_CURVE=false \
GO2_JUMP_TAKEOFF_SPEED_SCALE=1.09 \
./scripts/docker_run_single_jump_trial.sh 0.20
```

Run a multi-distance sweep:

```bash
cd /home/hayan/go2_jump_ws
./scripts/sweep_takeoff_speed_scale.sh 0.20,0.25,0.30 1.00,1.03,1.06 1
```

## Current Default Curve

The current workspace default is:

- distance points: `[0.20, 0.25, 0.30]`
- speed scales: `[1.09, 1.06, 1.06]`

This is configured in:

- [jump_params.yaml](/home/hayan/go2_jump_ws/src/go2_jump_bringup/config/jump_params.yaml)

## Current Empirical Recommendations

Based on the latest sweeps on March 24, 2026:

- `0.20 m -> takeoff_speed_scale ~= 1.09`
- `0.25 m -> takeoff_speed_scale ~= 1.06`
- `0.30 m -> takeoff_speed_scale ~= 1.06`

Summary files:

- [speed_scale_sweep_20260324_214558_summary.txt](/home/hayan/go2_jump_ws/reports/calibration/speed_scale_sweep_20260324_214558_summary.txt)
- [speed_scale_sweep_20260324_215112_summary.txt](/home/hayan/go2_jump_ws/reports/calibration/speed_scale_sweep_20260324_215112_summary.txt)

## How To Read The Trial Report

Use these fields together:

- `final_forward_displacement_m`
  Final settled forward displacement after recovery.
- `landing_forward_displacement_m`
  Forward displacement when landing is detected.
- `airborne_forward_progress_m`
  Forward progress from takeoff detection to landing detection.
- `post_landing_forward_gain_m`
  Forward displacement accumulated after landing detection.

Interpretation on the current stack:

- high `final_forward_displacement_m` with very low `landing_forward_displacement_m`
  usually means the robot did not truly jump forward much in the air
- increasing `post_landing_forward_gain_m`
  often means recovery motion is compensating for weak airborne range
- if `max_abs_pitch_deg` falls while `airborne_forward_progress_m` rises
  the tuning direction is usually promising

## Current Limitation

The current MuJoCo bridge path on this host reports `foot_force_est` as zero during
the tested runs. Because of that:

- touchdown is still detected with height and vertical-velocity heuristics
- landing displacement should be treated as a heuristic event metric, not a true
  contact-force event

## Recommended Next Sweep

The current takeoff-speed curve is already useful for final settled position. The next
best sweep should hold the scale curve fixed and vary pitch / push distribution:

1. sweep `push_front_tau_scale` and `push_rear_tau_scale`
2. sweep `push_pitch_target_deg`
3. compare `airborne_forward_progress_m` first, then `final_forward_displacement_m`

That next loop should move the system from “lands near the target eventually” toward
“actually covers the target distance in the jump itself.”
