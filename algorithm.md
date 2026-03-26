# Algorithm

[中文版本](algorithm.zh-CN.md)

This project is organized as a three-layer control stack:

`planner -> controller -> MuJoCo-native WBC/MPC backend -> LowCmd`

The purpose of this document is to state, in engineering terms, what each layer
computes today and what still needs to improve before the stack becomes a
high-quality forward jump controller.

## 1. Planner

The planner converts a distance request into a jump task.

Current implementation: `go2_jump_core`

Inputs:

- target jump distance
- takeoff angle
- optional takeoff-speed scaling
- desired takeoff and landing body pitch

Outputs:

- takeoff speed from a ballistic approximation
- forward and vertical takeoff velocity components
- estimated crouch / push / flight / landing / settle durations
- phase-level reference quantities:
  - desired forward velocity
  - desired vertical velocity
  - desired body pitch
  - desired body height offset
  - leg retraction ratio
  - landing brace factor

What the planner is not doing yet:

- no centroidal optimization
- no explicit contact schedule search
- no terrain-aware timing
- no optimization over landing footprint

Why this layer still matters:

The planner keeps the task interface stable. As the backend changes from preview
control to native MPC, the upper-level contract remains:

`distance request -> jump task specification`

## 2. Controller

The controller translates the active jump task and the current observation into
the next low-level command.

Current implementation: `go2_jump_mpc`

### 2.1 Observation processing

The controller reads:

- joint position and velocity from `/lowstate`
- IMU orientation and angular velocity from `/lowstate`
- body position and linear velocity from `/sportmodestate`
- foot contact proxy from `/lowstate.foot_force_est`

The contact path now has three stages:

1. `unitree_mujoco` reconstructs support loads from MuJoCo contact forces.
2. `go2_jump_mpc_node` filters those loads and applies hysteresis / debounce.
3. The controller derives boolean foot contact and contact-count signals from the filtered loads.

This matters because jump phase transitions are extremely sensitive to noisy
contact measurements. A raw threshold on a weak or delayed proxy is not enough.

### 2.2 Phase management

The controller does not rely on time alone.

It starts from the nominal phase given by `SampleJumpReference`, then applies
contact-driven overrides such as:

- early takeoff detection
- delayed push extension when nominal flight starts but stance contact is still present
- delayed landing when the nominal landing window starts but the robot is still airborne
- early touchdown detection
- settle transition after touchdown is stable and vertical motion is small

The practical goal is simple:

make the executed phase track the real contact sequence, not only the nominal clock.

### 2.3 Command synthesis

For the current control tick, the controller builds:

- joint-space reference pose `q_ref`
- zero or near-zero `dq_ref`
- phase-dependent PD gains
- phase-dependent feedforward torques

The feedforward term is still simple. It is not a full rigid-body inverse
dynamics solve. It is a structured low-level bias that gives the push and
landing phases more authority than a pure pose tracker.

## 3. MuJoCo-native WBC / MPC Backend

The backend chooses the action that should be applied now, subject to dynamics
and contact behavior over a short horizon.

Current implementation: `mujoco_native_mpc`

This backend is not a textbook QP-WBC yet. It is a native MuJoCo rollout MPC
backbone:

1. Reconstruct a MuJoCo state from the current observation.
2. Sample a small set of candidate action modifiers.
3. Roll each candidate forward inside MuJoCo for a short horizon.
4. Score the rollout against phase objectives, contact behavior, pitch, and terminal velocity.
5. Take the best candidate and apply its first-step action on the real control path.

### 3.1 What is optimized today

The rollout cost currently reasons about:

- desired forward velocity
- desired vertical velocity
- body pitch and roll
- contact count consistency with crouch / push / flight / landing / settle
- forward displacement over the short horizon
- control effort

The candidate action lattice perturbs:

- push extension strength
- body pitch bias
- flight tuck amount
- landing brace amount

### 3.2 What this backend already solves

Compared with the preview-only baseline, the native backend already does three
important things:

- it evaluates actions in the same MuJoCo model used for simulation
- it reasons about short-horizon consequences instead of only the current phase pose
- it shares the same low-level `LowCmd` execution path as the rest of the stack

### 3.3 What is still missing

This backend is not finished. The main remaining gaps are:

- takeoff timing is still not clean enough
- predicted phase and real contact can still drift apart
- no explicit centroidal momentum state
- no contact-force optimization variable
- no full-body QP with torque-consistent task priorities
- no reactive landing redistribution across individual feet

So the backend is already real, but still early.

## 4. How the Three Layers Work Together

At each control tick:

1. The planner provides the nominal jump task and phase reference.
2. The controller aligns that reference with measured contact and current robot state.
3. The native backend searches over short-horizon actions in MuJoCo.
4. The selected first action is converted into `LowCmd`.
5. The simulator publishes the next `LowState`, and the loop repeats.

That is the current minimum closed loop for this repository.

## 5. Near-term Engineering Priority

The most important next step is not more pose tuning.

It is tightening the coupling between:

- real contact estimation
- executed phase logic
- native rollout prediction

Until those three are aligned, the backend can still produce commands that are
locally plausible but globally mistimed for a clean forward jump.
