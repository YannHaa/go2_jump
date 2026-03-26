# Algorithm

[中文版本](algorithm.zh-CN.md)

This repository is being rebuilt around a three-layer jump-control stack:

`planner -> controller -> WBC/MPC backend -> LowCmd`

The purpose of this document is to clarify what each layer computes, what is
already implemented in the repository, and what still needs to be replaced by
the final MuJoCo-native optimizer.

## 1. Planner

The planner answers one question:

given a target forward distance, what kind of takeoff and landing event should
the robot try to realize?

The current planner lives in `go2_jump_core` and computes:

- target takeoff speed from a ballistic approximation
- decomposition of takeoff velocity into forward and vertical components
- nominal phase timing for crouch, push, flight, landing, and settle
- body-level reference quantities used by the controller preview

The current planner does not solve a full centroidal optimization problem yet.
It provides a clean task interface that can later feed:

- a centroidal jump planner
- a MuJoCo rollout optimizer
- a published baseline controller for comparison

In practical terms, the planner should remain the stable layer that maps:

`distance request -> jump task specification`

## 2. Controller

The controller answers the next question:

given the current robot observation and the active jump task, what should the
robot try to do in the next few control steps?

The current controller lives in `go2_jump_mpc` and already performs four useful
jobs:

- receives `JumpTask` and builds a short horizon preview
- converts phase-level references into joint-space reference poses
- generates a `LowCmd` command stream at control rate
- uses contact observation to override phase timing when the measured contact
  sequence disagrees with the nominal schedule

That last point matters. A jump controller cannot rely on time alone. The
actual contact sequence may deviate because of:

- earlier-than-expected liftoff
- earlier-than-expected touchdown
- asymmetric or partial landing contact

For that reason the current controller already contains a minimal contact-aware
phase manager:

- if support contact disappears early during push, the controller can switch to
  flight early
- if contact reappears during nominal flight, the controller can switch to
  landing early
- once touchdown is stable and vertical motion is small, the controller can
  switch to settle

To avoid false overrides, the controller only enables those contact-based phase
changes after it has seen a non-trivial contact signal from the observation
stream.

This is still a preview controller, not the final optimizer, but it gives the
project an observation-driven closed loop instead of a pure open-loop script.

## 3. WBC / MPC Backend

The backend answers the final question:

how should the low-level command be computed so the robot respects dynamics,
contact, torque limits, and task priorities at the same time?

In the final target architecture, this layer will be a MuJoCo-native whole-body
optimization backend. It should solve for:

- joint torques or torque-consistent low-level commands
- contact-consistent whole-body motion
- takeoff impulse quality
- body pitch and landing posture regulation
- post-touchdown stabilization

The intended backend structure is:

1. Roll out dynamics over a short horizon.
2. Penalize deviation from jump-task objectives.
3. Penalize excessive torque, joint velocity, and pose excursions.
4. Use contact-aware constraints or soft penalties through takeoff and landing.
5. Apply the first command of the horizon and repeat.

The current repository does not contain that final solver yet. Today, the
backend is a `reference_preview` scaffold that keeps the outer ROS 2 interface
stable while the solver is rebuilt.

## Current Implementation Status

What is implemented now:

- task-level jump specification
- reference preview generation
- contact-aware phase override logic
- `LowCmd` publication path
- `JumpControllerState` diagnostic topic
- reproducible simulator + ROS 2 smoke-test path

What is not implemented yet:

- centroidal jump optimizer
- torque-level whole-body QP
- MuJoCo-native shooting or SQP backend
- reactive landing redistribution across feet
- identification-backed model calibration

## Recommended Next Solver Upgrade

The next useful upgrade is not “more joint-space tuning”. It is replacing the
preview backend with a backend that explicitly reasons about:

- centroidal momentum
- contact schedule hypotheses
- joint and torque constraints
- touchdown cost and post-touchdown recovery

That will let the repository move from:

`task-conditioned motion script`

to:

`task-conditioned optimization controller`
