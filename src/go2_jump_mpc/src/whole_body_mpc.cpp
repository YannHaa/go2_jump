#include "go2_jump_mpc/whole_body_mpc.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

namespace go2_jump_mpc {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr std::size_t kFootCount = 4;

using JointArray = std::array<double, kControlledJointCount>;

constexpr std::array<const char*, kControlledJointCount> kJointNames{{
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
}};

constexpr std::array<const char*, kControlledJointCount> kActuatorNames{{
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
}};

constexpr std::array<const char*, kFootCount> kFootGeomNames{{
    "FR", "FL", "RR", "RL",
}};

constexpr std::array<const char*, kFootCount> kFootBodyNames{{
    "FR_foot", "FL_foot", "RR_foot", "RL_foot",
}};

struct CandidateAction {
  double push_extension_scale{0.0};
  double pitch_adjust_scale{0.0};
  double flight_tuck_scale{0.0};
  double landing_brace_scale{0.0};
};

struct RolloutResult {
  double score{std::numeric_limits<double>::infinity()};
  CandidateAction candidate{};
  go2_jump_core::JumpReferenceSample first_sample{};
  JointArray first_q_ref{};
  JointArray first_tau_ff{};
  double first_kp{0.0};
  double first_kd{0.0};
};

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double Square(double value) { return value * value; }

double Lerp(double a, double b, double alpha) {
  return a + (b - a) * Clamp(alpha, 0.0, 1.0);
}

double Smooth01(double value) {
  const double x = Clamp(value, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

JointArray BlendPose(const JointArray& a, const JointArray& b, double alpha) {
  JointArray out{};
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = Lerp(a[i], b[i], alpha);
  }
  return out;
}

int CountContacts(const std::array<bool, kFootCount>& foot_contact) {
  return static_cast<int>(std::count(foot_contact.begin(), foot_contact.end(), true));
}

std::array<double, 4> QuaternionFromRpy(const std::array<double, 3>& rpy) {
  const double half_roll = 0.5 * rpy[0];
  const double half_pitch = 0.5 * rpy[1];
  const double half_yaw = 0.5 * rpy[2];
  const double cr = std::cos(half_roll);
  const double sr = std::sin(half_roll);
  const double cp = std::cos(half_pitch);
  const double sp = std::sin(half_pitch);
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);
  return {
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
  };
}

std::array<double, 3> RpyFromQuaternion(const mjtNum* quat_wxyz) {
  const double w = quat_wxyz[0];
  const double x = quat_wxyz[1];
  const double y = quat_wxyz[2];
  const double z = quat_wxyz[3];

  std::array<double, 3> rpy{};
  rpy[0] = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  rpy[1] = std::asin(Clamp(2.0 * (w * y - z * x), -1.0, 1.0));
  rpy[2] = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return rpy;
}

std::filesystem::path ResolveMujocoModelPath(const std::string& configured_path) {
  std::vector<std::filesystem::path> candidates;
  if (!configured_path.empty()) {
    candidates.emplace_back(configured_path);
  }
  candidates.emplace_back("/workspace/src/unitree_mujoco/unitree_robots/go2/scene.xml");
  candidates.emplace_back("/home/hayan/go2_jump_ws/src/unitree_mujoco/unitree_robots/go2/scene.xml");
  candidates.emplace_back(std::filesystem::current_path() /
                          "src/unitree_mujoco/unitree_robots/go2/scene.xml");

  for (const auto& candidate : candidates) {
    if (!candidate.empty() && std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return {};
}

bool IsMujocoNativeBackend(const std::string& backend_name) {
  return backend_name == "mujoco_sampling" ||
         backend_name == "mujoco_native_mpc";
}

std::vector<CandidateAction> CandidateActionsForPhase(
    go2_jump_core::JumpPhase phase) {
  std::vector<CandidateAction> candidates{
      {},
      {1.0, 0.0, 0.0, 0.0},
      {1.6, 0.0, 0.0, 0.0},
      {-1.0, 0.0, 0.0, 0.0},
      {0.0, 1.0, 0.0, 0.0},
      {0.0, -1.0, 0.0, 0.0},
      {1.0, 1.0, 0.0, 0.0},
      {1.5, 0.8, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, -1.0, 0.0},
      {0.0, 0.0, 0.0, 1.0},
      {0.0, 0.0, 0.0, 1.4},
      {0.8, 1.6, 0.0, 0.0},
      {0.8, -1.6, 0.0, 0.0},
      {1.6, 1.4, 0.0, 0.0},
      {1.6, -1.4, 0.0, 0.0},
  };

  if (phase == go2_jump_core::JumpPhase::kFlight) {
    candidates.push_back({0.0, 0.5, 1.2, 0.0});
    candidates.push_back({0.0, 0.9, 1.6, 0.0});
    candidates.push_back({0.0, -0.5, -1.0, 0.5});
  } else if (phase == go2_jump_core::JumpPhase::kLanding ||
             phase == go2_jump_core::JumpPhase::kSettle) {
    candidates.push_back({0.0, 0.3, 0.3, 1.4});
    candidates.push_back({0.0, 0.6, 0.0, 1.8});
    candidates.push_back({0.0, -0.3, -0.2, 0.8});
  } else {
    candidates.push_back({1.2, 0.6, 0.0, 0.0});
    candidates.push_back({2.0, 0.9, 0.0, 0.0});
    candidates.push_back({-0.8, -0.6, 0.0, 0.0});
  }

  return candidates;
}

go2_jump_core::JumpReferenceSample ApplyCandidateToSample(
    go2_jump_core::JumpReferenceSample sample, const CandidateAction& action) {
  if (sample.phase == go2_jump_core::JumpPhase::kCrouch) {
    sample.desired_body_pitch_deg += 2.0 * action.pitch_adjust_scale;
  } else if (sample.phase == go2_jump_core::JumpPhase::kPush) {
    sample.desired_body_pitch_deg += 6.5 * action.pitch_adjust_scale;
    sample.desired_forward_velocity_mps *=
        1.0 + 0.10 * action.push_extension_scale +
        0.10 * action.pitch_adjust_scale;
    sample.desired_vertical_velocity_mps *=
        1.0 + 0.12 * action.push_extension_scale -
        0.03 * std::abs(action.pitch_adjust_scale);
  } else if (sample.phase == go2_jump_core::JumpPhase::kFlight) {
    sample.desired_body_pitch_deg += 3.0 * action.pitch_adjust_scale;
    sample.leg_retraction_ratio = Clamp(
        sample.leg_retraction_ratio + 0.22 * action.flight_tuck_scale, 0.0, 1.0);
    sample.landing_brace_factor = Clamp(
        sample.landing_brace_factor + 0.08 * action.landing_brace_scale, 0.0, 1.0);
  } else if (sample.phase == go2_jump_core::JumpPhase::kLanding ||
             sample.phase == go2_jump_core::JumpPhase::kSettle) {
    sample.desired_body_pitch_deg += 2.0 * action.pitch_adjust_scale;
    sample.landing_brace_factor = Clamp(
        sample.landing_brace_factor + 0.22 * action.landing_brace_scale, 0.0, 1.5);
  }
  return sample;
}

JointArray ApplyCandidateToPose(const JointArray& nominal_pose,
                                const go2_jump_core::JumpReferenceSample& sample,
                                const CandidateAction& action) {
  JointArray pose = nominal_pose;
  const double forward_scale = Clamp(
      std::abs(sample.desired_forward_velocity_mps) / 1.2, 0.5, 1.8);

  auto apply_leg_delta = [&pose](std::size_t thigh_idx, std::size_t calf_idx,
                                 double thigh_delta, double calf_delta) {
    pose[thigh_idx] += thigh_delta;
    pose[calf_idx] += calf_delta;
  };

  if (sample.phase == go2_jump_core::JumpPhase::kCrouch) {
    const double front_thigh_delta = -0.04 * action.pitch_adjust_scale * forward_scale;
    const double front_calf_delta = 0.08 * action.pitch_adjust_scale * forward_scale;
    const double rear_thigh_delta = 0.07 * action.pitch_adjust_scale * forward_scale;
    const double rear_calf_delta = -0.12 * action.pitch_adjust_scale * forward_scale;
    for (std::size_t thigh_idx : {1u, 4u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, front_thigh_delta, front_calf_delta);
    }
    for (std::size_t thigh_idx : {7u, 10u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, rear_thigh_delta, rear_calf_delta);
    }
  } else if (sample.phase == go2_jump_core::JumpPhase::kPush) {
    const double thigh_delta = -0.12 * action.push_extension_scale;
    const double calf_delta = 0.24 * action.push_extension_scale;
    for (std::size_t thigh_idx : {1u, 4u, 7u, 10u}) {
      const std::size_t calf_idx = thigh_idx + 1;
      apply_leg_delta(thigh_idx, calf_idx, thigh_delta, calf_delta);
    }
    const double front_thigh_bias =
        0.08 * action.pitch_adjust_scale * forward_scale;
    const double front_calf_bias =
        -0.12 * action.pitch_adjust_scale * forward_scale;
    const double rear_thigh_bias =
        -0.12 * action.pitch_adjust_scale * forward_scale;
    const double rear_calf_bias =
        0.20 * action.pitch_adjust_scale * forward_scale;
    for (std::size_t thigh_idx : {1u, 4u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, front_thigh_bias, front_calf_bias);
    }
    for (std::size_t thigh_idx : {7u, 10u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, rear_thigh_bias, rear_calf_bias);
    }
  } else if (sample.phase == go2_jump_core::JumpPhase::kFlight) {
    const double thigh_delta = 0.14 * action.flight_tuck_scale;
    const double calf_delta = -0.22 * action.flight_tuck_scale;
    for (std::size_t thigh_idx : {1u, 4u, 7u, 10u}) {
      const std::size_t calf_idx = thigh_idx + 1;
      apply_leg_delta(thigh_idx, calf_idx, thigh_delta, calf_delta);
    }
    const double front_thigh_bias =
        -0.04 * action.pitch_adjust_scale * forward_scale;
    const double front_calf_bias =
        0.06 * action.pitch_adjust_scale * forward_scale;
    const double rear_thigh_bias =
        0.04 * action.pitch_adjust_scale * forward_scale;
    const double rear_calf_bias =
        -0.06 * action.pitch_adjust_scale * forward_scale;
    for (std::size_t thigh_idx : {1u, 4u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, front_thigh_bias, front_calf_bias);
    }
    for (std::size_t thigh_idx : {7u, 10u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, rear_thigh_bias, rear_calf_bias);
    }
  } else if (sample.phase == go2_jump_core::JumpPhase::kLanding ||
             sample.phase == go2_jump_core::JumpPhase::kSettle) {
    const double thigh_delta = 0.10 * action.landing_brace_scale;
    const double calf_delta = -0.18 * action.landing_brace_scale;
    for (std::size_t thigh_idx : {1u, 4u, 7u, 10u}) {
      const std::size_t calf_idx = thigh_idx + 1;
      apply_leg_delta(thigh_idx, calf_idx, thigh_delta, calf_delta);
    }
  }

  return pose;
}

JointArray ApplyCandidateToFeedforward(const JointArray& nominal_tau,
                                       const go2_jump_core::JumpReferenceSample& sample,
                                       const CandidateAction& action,
                                       double max_feedforward_torque_nm) {
  JointArray tau = nominal_tau;
  const double forward_scale = Clamp(
      std::abs(sample.desired_forward_velocity_mps) / 1.2, 0.5, 1.8);

  if (sample.phase == go2_jump_core::JumpPhase::kPush) {
    const double thigh_delta =
        Clamp(2.4 * action.push_extension_scale, -max_feedforward_torque_nm,
              max_feedforward_torque_nm);
    const double calf_delta =
        Clamp(3.6 * action.push_extension_scale, -max_feedforward_torque_nm,
              max_feedforward_torque_nm);
    for (std::size_t idx : {1u, 4u, 7u, 10u}) {
      tau[idx] = Clamp(tau[idx] + thigh_delta, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
    for (std::size_t idx : {2u, 5u, 8u, 11u}) {
      tau[idx] = Clamp(tau[idx] + calf_delta, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
    const double front_thigh_bias =
        Clamp(-1.2 * action.pitch_adjust_scale * forward_scale,
              -max_feedforward_torque_nm, max_feedforward_torque_nm);
    const double front_calf_bias =
        Clamp(-2.0 * action.pitch_adjust_scale * forward_scale,
              -max_feedforward_torque_nm, max_feedforward_torque_nm);
    const double rear_thigh_bias =
        Clamp(2.2 * action.pitch_adjust_scale * forward_scale,
              -max_feedforward_torque_nm, max_feedforward_torque_nm);
    const double rear_calf_bias =
        Clamp(3.4 * action.pitch_adjust_scale * forward_scale,
              -max_feedforward_torque_nm, max_feedforward_torque_nm);
    for (std::size_t idx : {1u, 4u}) {
      tau[idx] = Clamp(tau[idx] + front_thigh_bias, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
    for (std::size_t idx : {2u, 5u}) {
      tau[idx] = Clamp(tau[idx] + front_calf_bias, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
    for (std::size_t idx : {7u, 10u}) {
      tau[idx] = Clamp(tau[idx] + rear_thigh_bias, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
    for (std::size_t idx : {8u, 11u}) {
      tau[idx] = Clamp(tau[idx] + rear_calf_bias, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
  } else if (sample.phase == go2_jump_core::JumpPhase::kLanding) {
    const double brace_delta =
        Clamp(-2.0 * action.landing_brace_scale, -max_feedforward_torque_nm,
              max_feedforward_torque_nm);
    for (std::size_t idx : {1u, 4u, 7u, 10u, 2u, 5u, 8u, 11u}) {
      tau[idx] = Clamp(tau[idx] + brace_delta, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
  }

  return tau;
}

}  // namespace

struct WholeBodyMpc::MujocoBackend {
  explicit MujocoBackend(const WholeBodyMpcConfig& config) {
    const auto model_path_candidate = ResolveMujocoModelPath(config.mujoco_model_path);
    if (model_path_candidate.empty()) {
      error_message =
          "Unable to locate Go2 MuJoCo scene.xml for the MuJoCo-native backend.";
      return;
    }

    model_path = model_path_candidate.string();
    char error_buffer[1024] = {};
    model = mj_loadXML(model_path.c_str(), nullptr, error_buffer,
                       sizeof(error_buffer));
    if (!model) {
      error_message = error_buffer;
      return;
    }

    data = mj_makeData(model);
    if (!data) {
      error_message = "mj_makeData failed for the MuJoCo-native backend.";
      return;
    }

    home_key_id = mj_name2id(model, mjOBJ_KEY, "home");
    base_body_id = mj_name2id(model, mjOBJ_BODY, "base_link");
    imu_site_id = mj_name2id(model, mjOBJ_SITE, "imu");

    for (std::size_t i = 0; i < kJointNames.size(); ++i) {
      const int joint_id = mj_name2id(model, mjOBJ_JOINT, kJointNames[i]);
      if (joint_id < 0) {
        error_message = std::string("Missing joint in MuJoCo model: ") + kJointNames[i];
        return;
      }
      joint_qposadr[i] = model->jnt_qposadr[joint_id];
      joint_dofadr[i] = model->jnt_dofadr[joint_id];

      const int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, kActuatorNames[i]);
      if (actuator_id < 0) {
        error_message =
            std::string("Missing actuator in MuJoCo model: ") + kActuatorNames[i];
        return;
      }
      actuator_ids[i] = actuator_id;
      actuator_limits[i] = model->actuator_ctrllimited[actuator_id]
                               ? model->actuator_ctrlrange[2 * actuator_id + 1]
                               : 45.0;
    }

    for (std::size_t i = 0; i < kFootGeomNames.size(); ++i) {
      foot_geom_ids[i] = mj_name2id(model, mjOBJ_GEOM, kFootGeomNames[i]);
      foot_body_ids[i] = mj_name2id(model, mjOBJ_BODY, kFootBodyNames[i]);
      if (foot_geom_ids[i] >= 0) {
        foot_geom_radius[i] = model->geom_size[3 * foot_geom_ids[i]];
      }
    }

    if (home_key_id >= 0) {
      mj_resetDataKeyframe(model, data, home_key_id);
    } else {
      mj_resetData(model, data);
    }
    mj_forward(model, data);

    ready = true;
  }

  ~MujocoBackend() {
    if (data) {
      mj_deleteData(data);
      data = nullptr;
    }
    if (model) {
      mj_deleteModel(model);
      model = nullptr;
    }
  }

  void SyncObservation(const RobotObservation& observation) {
    if (!ready || !model || !data) {
      return;
    }

    if (home_key_id >= 0) {
      mj_resetDataKeyframe(model, data, home_key_id);
    } else {
      mj_resetData(model, data);
    }

    std::fill(data->ctrl, data->ctrl + model->nu, 0.0);
    if (model->nq >= 7) {
      data->qpos[0] = 0.0;
      data->qpos[1] = 0.0;
      data->qpos[2] = 0.0;
      const auto quat = QuaternionFromRpy(observation.body_rpy);
      data->qpos[3] = quat[0];
      data->qpos[4] = quat[1];
      data->qpos[5] = quat[2];
      data->qpos[6] = quat[3];
    }

    for (std::size_t i = 0; i < joint_qposadr.size(); ++i) {
      data->qpos[joint_qposadr[i]] = observation.q[i];
    }

    mj_fwdPosition(model, data);

    if (observation.sportstate_received && imu_site_id >= 0) {
      data->qpos[0] = observation.position[0] - data->site_xpos[3 * imu_site_id + 0];
      data->qpos[1] = observation.position[1] - data->site_xpos[3 * imu_site_id + 1];
      data->qpos[2] = observation.position[2] - data->site_xpos[3 * imu_site_id + 2];
    } else {
      double required_z_shift = 0.0;
      for (std::size_t i = 0; i < foot_geom_ids.size(); ++i) {
        const int geom_id = foot_geom_ids[i];
        if (geom_id < 0) {
          continue;
        }
        const double geom_z = data->geom_xpos[3 * geom_id + 2];
        required_z_shift =
            std::max(required_z_shift, foot_geom_radius[i] - geom_z);
      }
      data->qpos[2] += required_z_shift;
    }

    std::fill(data->qvel, data->qvel + model->nv, 0.0);
    if (model->nv >= 6) {
      data->qvel[0] = observation.body_velocity[0];
      data->qvel[1] = observation.body_velocity[1];
      data->qvel[2] = observation.body_velocity[2];
      data->qvel[3] = observation.body_angular_velocity[0];
      data->qvel[4] = observation.body_angular_velocity[1];
      data->qvel[5] = observation.body_angular_velocity[2];
    }
    for (std::size_t i = 0; i < joint_dofadr.size(); ++i) {
      data->qvel[joint_dofadr[i]] = observation.dq[i];
    }

    mj_forward(model, data);
  }

  std::array<double, kFootCount> FootContactForces() const {
    std::array<double, kFootCount> forces{};
    if (!ready || !model || !data || data->ncon <= 0) {
      return forces;
    }

    std::array<double, 3> world_up{0.0, 0.0, 1.0};
    const double gravity_norm = std::sqrt(
        Square(model->opt.gravity[0]) + Square(model->opt.gravity[1]) +
        Square(model->opt.gravity[2]));
    if (gravity_norm > 1e-6) {
      world_up = {
          -model->opt.gravity[0] / gravity_norm,
          -model->opt.gravity[1] / gravity_norm,
          -model->opt.gravity[2] / gravity_norm,
      };
    }

    for (int contact_index = 0; contact_index < data->ncon; ++contact_index) {
      const auto& contact = data->contact[contact_index];
      double contact_wrench[6]{};
      mj_contactForce(model, data, contact_index, contact_wrench);
      const double contact_force_world[3]{
          contact.frame[0] * contact_wrench[0] +
              contact.frame[3] * contact_wrench[1] +
              contact.frame[6] * contact_wrench[2],
          contact.frame[1] * contact_wrench[0] +
              contact.frame[4] * contact_wrench[1] +
              contact.frame[7] * contact_wrench[2],
          contact.frame[2] * contact_wrench[0] +
              contact.frame[5] * contact_wrench[1] +
              contact.frame[8] * contact_wrench[2],
      };
      const double support_load_n = std::max(
          std::abs(contact_wrench[0]),
          std::abs(contact_force_world[0] * world_up[0] +
                   contact_force_world[1] * world_up[1] +
                   contact_force_world[2] * world_up[2]));
      if (support_load_n <= 0.0) {
        continue;
      }

      for (std::size_t foot_index = 0; foot_index < foot_geom_ids.size();
           ++foot_index) {
        const int foot_geom_id = foot_geom_ids[foot_index];
        if (foot_geom_id < 0) {
          continue;
        }
        if (contact.geom1 == foot_geom_id || contact.geom2 == foot_geom_id) {
          forces[foot_index] += support_load_n;
        }
      }
    }

    return forces;
  }

  std::array<bool, kFootCount> FootContacts(double threshold) const {
    std::array<bool, kFootCount> contacts{};
    const auto forces = FootContactForces();
    for (std::size_t i = 0; i < contacts.size(); ++i) {
      contacts[i] = forces[i] >= threshold;
    }
    return contacts;
  }

  std::array<double, 3> BaseRpy() const {
    if (!ready || !model || !data || base_body_id < 0) {
      return {};
    }
    return RpyFromQuaternion(&data->xquat[4 * base_body_id]);
  }

  mjModel* model{nullptr};
  mjData* data{nullptr};
  bool ready{false};
  std::string error_message;
  std::string model_path;

  int home_key_id{-1};
  int base_body_id{-1};
  int imu_site_id{-1};
  std::array<int, kControlledJointCount> joint_qposadr{};
  std::array<int, kControlledJointCount> joint_dofadr{};
  std::array<int, kControlledJointCount> actuator_ids{};
  std::array<double, kControlledJointCount> actuator_limits{};
  std::array<int, kFootCount> foot_geom_ids{};
  std::array<int, kFootCount> foot_body_ids{};
  std::array<double, kFootCount> foot_geom_radius{};
};

WholeBodyMpc::WholeBodyMpc(WholeBodyMpcConfig config)
    : config_(std::move(config)) {
  if (IsMujocoNativeBackend(config_.solver_backend)) {
    mujoco_backend_ = std::make_unique<MujocoBackend>(config_);
  }
}

WholeBodyMpc::~WholeBodyMpc() = default;

void WholeBodyMpc::SetTask(const go2_jump_core::JumpTaskSpec& task) {
  task_ = task;
  have_task_ = true;
  execution_phase_state_ = {};
}

bool WholeBodyMpc::HasTask() const { return have_task_; }

PhaseEventState WholeBodyMpc::BuildExecutionPhaseEventState(
    const RobotObservation& observation, double task_elapsed_s) {
  const int contact_count = CountContacts(observation.foot_contact);
  UpdatePhaseEventState(execution_phase_state_, contact_count,
                        observation.contact_signal_valid,
                        observation.body_velocity[2], task_elapsed_s);
  return execution_phase_state_;
}

void WholeBodyMpc::UpdatePhaseEventState(PhaseEventState& state, int contact_count,
                                         bool contact_signal_valid,
                                         double body_vertical_velocity_mps,
                                         double task_elapsed_s) const {
  state.contact_signal_valid = state.contact_signal_valid || contact_signal_valid;
  if (!state.contact_signal_valid) {
    return;
  }

  const double takeoff_gate_time_s =
      task_.crouch_duration_s + 0.30 * task_.push_duration_s;
  const bool takeoff_candidate =
      task_elapsed_s >= takeoff_gate_time_s &&
      contact_count <= config_.flight_contact_count_max;
  if (!state.takeoff_latched) {
    if (takeoff_candidate) {
      if (state.takeoff_candidate_start_s < 0.0) {
        state.takeoff_candidate_start_s = task_elapsed_s;
      }
      if (task_elapsed_s - state.takeoff_candidate_start_s >=
          config_.takeoff_latch_dwell_s) {
        state.takeoff_latched = true;
        state.takeoff_time_s = state.takeoff_candidate_start_s;
      }
    } else {
      state.takeoff_candidate_start_s = -1.0;
    }
  }
  if (state.takeoff_latched && state.takeoff_time_s >= 0.0) {
    state.takeoff_candidate_start_s = state.takeoff_time_s;
  }

  if (state.takeoff_latched && !state.touchdown_latched) {
    const double flight_elapsed_s = task_elapsed_s - state.takeoff_time_s;
    const bool touchdown_candidate =
        flight_elapsed_s >= config_.min_flight_time_before_touchdown_s &&
        contact_count >= config_.touchdown_contact_count_threshold &&
        (body_vertical_velocity_mps <= 0.35 || contact_count == 4);
    if (touchdown_candidate) {
      if (state.touchdown_candidate_start_s < 0.0) {
        state.touchdown_candidate_start_s = task_elapsed_s;
      }
      if (task_elapsed_s - state.touchdown_candidate_start_s >=
          config_.touchdown_latch_dwell_s) {
        state.touchdown_latched = true;
        state.touchdown_time_s = state.touchdown_candidate_start_s;
      }
    } else {
      state.touchdown_candidate_start_s = -1.0;
    }
  }
  if (state.touchdown_latched && state.touchdown_time_s >= 0.0) {
    state.touchdown_candidate_start_s = state.touchdown_time_s;
  }

  if (state.touchdown_latched && !state.settle_latched) {
    const double touchdown_elapsed_s = task_elapsed_s - state.touchdown_time_s;
    const bool settle_candidate =
        contact_count == 4 &&
        std::abs(body_vertical_velocity_mps) <=
            config_.settle_vertical_velocity_threshold_mps &&
        touchdown_elapsed_s >= 0.5 * task_.landing_duration_s;
    if (settle_candidate) {
      if (state.settle_candidate_start_s < 0.0) {
        state.settle_candidate_start_s = task_elapsed_s;
      }
      if (task_elapsed_s - state.settle_candidate_start_s >=
          config_.settle_latch_dwell_s) {
        state.settle_latched = true;
        state.settle_time_s = state.settle_candidate_start_s;
      }
    } else {
      state.settle_candidate_start_s = -1.0;
    }
  }
  if (state.settle_latched && state.settle_time_s >= 0.0) {
    state.settle_candidate_start_s = state.settle_time_s;
  }
}

WholeBodyMpcCommand WholeBodyMpc::Solve(const RobotObservation& observation,
                                        double task_elapsed_s) {
  if (config_.solver_backend == "reference_preview") {
    return SolveReferencePreview(observation, task_elapsed_s);
  }
  if (IsMujocoNativeBackend(config_.solver_backend)) {
    return SolveMujocoSampling(observation, task_elapsed_s);
  }

  auto fallback = SolveReferencePreview(observation, task_elapsed_s);
  fallback.backend_name = config_.solver_backend;
  fallback.backend_ready = false;
  return fallback;
}

WholeBodyMpcCommand WholeBodyMpc::SolveReferencePreview(
    const RobotObservation& observation, double task_elapsed_s) {
  WholeBodyMpcCommand command{};
  if (!have_task_ || !observation.lowstate_received) {
    return command;
  }

  command.valid = true;
  command.backend_name = "reference_preview";
  command.backend_ready = true;
  command.lowcmd_enabled = config_.enable_lowcmd_output;
  command.contact_signal_valid = observation.contact_signal_valid;
  command.foot_contact = observation.foot_contact;
  command.contact_count = CountContacts(observation.foot_contact);

  const auto phase_state =
      BuildExecutionPhaseEventState(observation, task_elapsed_s);
  const auto planned_sample = go2_jump_core::SampleJumpReference(
      task_, config_.reference_config, task_elapsed_s);
  const auto current_sample =
      ApplyContactOverrides(planned_sample, phase_state, command.contact_count,
                            observation.body_velocity[2], task_elapsed_s);
  command.contact_override = (current_sample.phase != planned_sample.phase);
  command.phase = current_sample.phase;
  command.desired_forward_velocity_mps =
      current_sample.desired_forward_velocity_mps;
  command.desired_vertical_velocity_mps =
      current_sample.desired_vertical_velocity_mps;
  command.desired_body_pitch_deg = current_sample.desired_body_pitch_deg;
  command.desired_body_height_offset_m =
      current_sample.desired_body_height_offset_m;
  command.q_ref = BuildPoseForSample(current_sample);
  command.dq_ref.fill(0.0);
  command.tau_ff = BuildFeedforwardForSample(task_, current_sample);
  command.uniform_kp = GainsForPhase(current_sample.phase, false);
  command.uniform_kd = GainsForPhase(current_sample.phase, true);

  command.preview.reserve(std::max(config_.horizon_steps, 1));
  for (int step = 0; step < std::max(config_.horizon_steps, 1); ++step) {
    const double time_from_now = step * config_.control_dt_s;
    const auto sample = go2_jump_core::SampleJumpReference(
        task_, config_.reference_config, task_elapsed_s + time_from_now);
    command.preview.push_back(PreviewStep{
        time_from_now,
        sample.phase,
        sample.desired_forward_velocity_mps,
        sample.desired_vertical_velocity_mps,
        sample.desired_body_pitch_deg,
        sample.desired_body_height_offset_m,
        sample.landing_brace_factor,
    });
  }

  return command;
}

WholeBodyMpcCommand WholeBodyMpc::SolveMujocoSampling(
    const RobotObservation& observation, double task_elapsed_s) {
  auto command = SolveReferencePreview(observation, task_elapsed_s);
  if (!command.valid) {
    return command;
  }

  command.backend_name = "mujoco_native_mpc";
  command.backend_ready = mujoco_backend_ && mujoco_backend_->ready;
  if (!command.backend_ready) {
    return command;
  }

  auto& backend = *mujoco_backend_;
  backend.SyncObservation(observation);

  std::vector<mjtNum> qpos_start(backend.model->nq, 0.0);
  std::vector<mjtNum> qvel_start(backend.model->nv, 0.0);
  std::memcpy(qpos_start.data(), backend.data->qpos,
              sizeof(mjtNum) * backend.model->nq);
  std::memcpy(qvel_start.data(), backend.data->qvel,
              sizeof(mjtNum) * backend.model->nv);

  const int rollout_steps =
      std::max(1, std::min(config_.mujoco_rollout_steps, config_.horizon_steps));
  const int substeps = std::max(1, config_.mujoco_rollout_substeps);
  const double rollout_dt = substeps * backend.model->opt.timestep;
  const double gravity_mps2 = std::max(
      std::abs(backend.model->opt.gravity[2]), config_.reference_config.gravity_mps2);
  const double push_end = task_.crouch_duration_s + task_.push_duration_s;
  const double x_start = backend.data->qpos[0];

  auto rollout_phase_state = execution_phase_state_;
  const auto seed_contact = backend.FootContacts(config_.contact_release_threshold_n);
  UpdatePhaseEventState(rollout_phase_state, CountContacts(seed_contact),
                        rollout_phase_state.contact_signal_valid,
                        backend.data->qvel[2], task_elapsed_s);

  RolloutResult best_result;
  const auto seed_sample = ApplyContactOverrides(
      go2_jump_core::SampleJumpReference(task_, config_.reference_config, task_elapsed_s),
      rollout_phase_state, CountContacts(seed_contact), backend.data->qvel[2],
      task_elapsed_s);

  for (const auto& candidate : CandidateActionsForPhase(seed_sample.phase)) {
    std::memcpy(backend.data->qpos, qpos_start.data(),
                sizeof(mjtNum) * backend.model->nq);
    std::memcpy(backend.data->qvel, qvel_start.data(),
                sizeof(mjtNum) * backend.model->nv);
    std::fill(backend.data->ctrl, backend.data->ctrl + backend.model->nu, 0.0);
    mj_forward(backend.model, backend.data);

    RolloutResult rollout{};
    double score = 0.0;
    double target_dx = 0.0;
    double accumulated_control_energy = 0.0;
    auto candidate_phase_state = rollout_phase_state;

    for (int step = 0; step < rollout_steps; ++step) {
      const double current_time = task_elapsed_s + step * rollout_dt;
      const auto planned_sample = go2_jump_core::SampleJumpReference(
          task_, config_.reference_config, current_time);
      const auto predicted_contact =
          backend.FootContacts(config_.contact_release_threshold_n);
      const int predicted_contact_count = CountContacts(predicted_contact);
      const auto contact_adjusted_sample = ApplyContactOverrides(
          planned_sample, candidate_phase_state, predicted_contact_count,
          backend.data->qvel[2], current_time);
      const auto sampled_reference =
          ApplyCandidateToSample(contact_adjusted_sample, candidate);
      const auto nominal_q_ref = BuildPoseForSample(sampled_reference);
      const auto q_ref =
          ApplyCandidateToPose(nominal_q_ref, sampled_reference, candidate);
      const auto nominal_tau_ff =
          BuildFeedforwardForSample(task_, sampled_reference);
      const auto tau_ff = ApplyCandidateToFeedforward(
          nominal_tau_ff, sampled_reference, candidate,
          config_.max_feedforward_torque_nm);
      const double kp = GainsForPhase(sampled_reference.phase, false);
      const double kd = GainsForPhase(sampled_reference.phase, true);

      if (step == 0) {
        rollout.first_sample = sampled_reference;
        rollout.first_q_ref = q_ref;
        rollout.first_tau_ff = tau_ff;
        rollout.first_kp = kp;
        rollout.first_kd = kd;
      }

      for (std::size_t joint_index = 0; joint_index < kControlledJointCount;
           ++joint_index) {
        const double q = backend.data->qpos[backend.joint_qposadr[joint_index]];
        const double dq = backend.data->qvel[backend.joint_dofadr[joint_index]];
        double torque =
            tau_ff[joint_index] + kp * (q_ref[joint_index] - q) - kd * dq;
        torque = Clamp(torque, -backend.actuator_limits[joint_index],
                       backend.actuator_limits[joint_index]);
        backend.data->ctrl[backend.actuator_ids[joint_index]] = torque;
        accumulated_control_energy += std::abs(torque);
      }

      for (int substep = 0; substep < substeps; ++substep) {
        mj_step(backend.model, backend.data);
      }

      const auto base_rpy = backend.BaseRpy();
      const auto next_contact =
          backend.FootContacts(config_.contact_release_threshold_n);
      const int next_contact_count = CountContacts(next_contact);
      const double vx = backend.data->qvel[0];
      const double vz = backend.data->qvel[2];
      const double pitch_deg = base_rpy[1] * kRadToDeg;
      const double roll_deg = base_rpy[0] * kRadToDeg;
      const double next_time = current_time + rollout_dt;
      const double ballistic_time_s = std::max(0.0, next_time - push_end);
      const double ballistic_vz =
          task_.target_takeoff_velocity_z_mps - gravity_mps2 * ballistic_time_s;

      UpdatePhaseEventState(candidate_phase_state, next_contact_count,
                            candidate_phase_state.contact_signal_valid, vz,
                            next_time);

      target_dx += sampled_reference.desired_forward_velocity_mps * rollout_dt;

      switch (sampled_reference.phase) {
        case go2_jump_core::JumpPhase::kCrouch:
          score += 0.04 * Square(vx);
          score += 0.03 * Square(vz);
          score += 0.03 * Square(pitch_deg - sampled_reference.desired_body_pitch_deg);
          if (next_contact_count < 2) {
            score += 8.0;
          }
          break;
        case go2_jump_core::JumpPhase::kPush: {
          score += 0.28 * Square(vx - sampled_reference.desired_forward_velocity_mps);
          score += 0.16 * Square(vz - sampled_reference.desired_vertical_velocity_mps);
          score += 0.05 * Square(pitch_deg - sampled_reference.desired_body_pitch_deg);
          const double push_progress = Clamp(
              sampled_reference.time_in_phase_s / std::max(task_.push_duration_s, 1e-6),
              0.0, 1.0);
          score += 0.18 * push_progress *
                   Square(vx - task_.target_takeoff_velocity_x_mps);
          score += 0.14 * push_progress *
                   Square(vz - task_.target_takeoff_velocity_z_mps);
          if (push_progress > 0.60 &&
              next_contact_count > config_.flight_contact_count_max) {
            score += 5.0;
          }
          if (push_progress < 0.25 &&
              next_contact_count <= config_.flight_contact_count_max) {
            score += 7.0;
          }
          break;
        }
        case go2_jump_core::JumpPhase::kFlight:
          score += 14.0 * next_contact_count;
          score += 0.24 * Square(vx - task_.target_takeoff_velocity_x_mps);
          score += 0.12 * Square(vz - ballistic_vz);
          score += 0.05 * Square(pitch_deg - sampled_reference.desired_body_pitch_deg);
          score += 0.02 * Square(roll_deg);
          break;
        case go2_jump_core::JumpPhase::kLanding:
          if (next_contact_count < config_.touchdown_contact_count_threshold) {
            score += 18.0;
          }
          score += 0.08 * Square(vx - sampled_reference.desired_forward_velocity_mps);
          score += 0.14 * Square(vz - sampled_reference.desired_vertical_velocity_mps);
          score += 0.08 * Square(pitch_deg - sampled_reference.desired_body_pitch_deg);
          score += 0.03 * Square(roll_deg);
          break;
        case go2_jump_core::JumpPhase::kSettle:
          if (next_contact_count < 3) {
            score += 10.0;
          }
          score += 0.06 * Square(vx);
          score += 0.10 * Square(vz);
          score += 0.06 * Square(pitch_deg);
          score += 0.03 * Square(roll_deg);
          break;
      }
    }

    const double end_dx = backend.data->qpos[0] - x_start;
    const double end_vx = backend.data->qvel[0];
    const double end_vz = backend.data->qvel[2];
    const auto end_contact =
        backend.FootContacts(config_.contact_release_threshold_n);
    const int end_contact_count = CountContacts(end_contact);
    const double horizon_end_time = task_elapsed_s + rollout_steps * rollout_dt;
    const auto end_reference = ApplyContactOverrides(
        go2_jump_core::SampleJumpReference(
            task_, config_.reference_config, horizon_end_time),
        candidate_phase_state, end_contact_count, end_vz, horizon_end_time);
    const double target_end_vx = (end_reference.phase == go2_jump_core::JumpPhase::kFlight)
                                     ? task_.target_takeoff_velocity_x_mps
                                     : end_reference.desired_forward_velocity_mps;
    const double target_end_vz = (end_reference.phase == go2_jump_core::JumpPhase::kFlight)
                                     ? (task_.target_takeoff_velocity_z_mps -
                                        gravity_mps2 *
                                            std::max(0.0, horizon_end_time - push_end))
                                     : end_reference.desired_vertical_velocity_mps;
    score += 2.20 * Square(end_dx - target_dx);
    score += 0.55 * Square(end_vx - target_end_vx);
    score += 0.28 * Square(end_vz - target_end_vz);
    score += 0.0008 * accumulated_control_energy;

    rollout.score = score;
    rollout.candidate = candidate;
    if (rollout.score < best_result.score) {
      best_result = rollout;
    }
  }

  if (!std::isfinite(best_result.score)) {
    command.backend_ready = false;
    return command;
  }

  const auto planned_now = go2_jump_core::SampleJumpReference(
      task_, config_.reference_config, task_elapsed_s);
  const auto actual_sample =
      ApplyContactOverrides(planned_now, execution_phase_state_,
                            command.contact_count, observation.body_velocity[2],
                            task_elapsed_s);
  const auto command_sample =
      ApplyCandidateToSample(actual_sample, best_result.candidate);
  const auto nominal_q_ref = BuildPoseForSample(command_sample);
  const auto nominal_tau_ff =
      BuildFeedforwardForSample(task_, command_sample);

  command.phase = command_sample.phase;
  command.contact_override = (command_sample.phase != planned_now.phase);
  command.desired_forward_velocity_mps =
      command_sample.desired_forward_velocity_mps;
  command.desired_vertical_velocity_mps =
      command_sample.desired_vertical_velocity_mps;
  command.desired_body_pitch_deg = command_sample.desired_body_pitch_deg;
  command.desired_body_height_offset_m =
      command_sample.desired_body_height_offset_m;
  command.q_ref = ApplyCandidateToPose(nominal_q_ref, command_sample,
                                       best_result.candidate);
  command.dq_ref.fill(0.0);
  command.tau_ff = ApplyCandidateToFeedforward(
      nominal_tau_ff, command_sample, best_result.candidate,
      config_.max_feedforward_torque_nm);
  command.uniform_kp = GainsForPhase(command_sample.phase, false);
  command.uniform_kd = GainsForPhase(command_sample.phase, true);
  return command;
}

std::array<double, kControlledJointCount> WholeBodyMpc::BuildPoseForSample(
    const go2_jump_core::JumpReferenceSample& sample) const {
  std::array<double, kControlledJointCount> pose = config_.stand_pose;
  switch (sample.phase) {
    case go2_jump_core::JumpPhase::kCrouch:
      pose = BlendPose(config_.stand_pose, config_.crouch_pose, 0.9);
      break;
    case go2_jump_core::JumpPhase::kPush:
      pose = config_.push_pose;
      break;
    case go2_jump_core::JumpPhase::kFlight:
      pose = BlendPose(config_.flight_pose, config_.landing_pose,
                       sample.landing_brace_factor);
      break;
    case go2_jump_core::JumpPhase::kLanding:
      pose = config_.landing_pose;
      break;
    case go2_jump_core::JumpPhase::kSettle:
      pose = BlendPose(config_.landing_pose, config_.settle_pose, 0.8);
      break;
  }

  const double pitch_rad = sample.desired_body_pitch_deg * kPi / 180.0;
  double phase_pitch_scale = 1.0;
  switch (sample.phase) {
    case go2_jump_core::JumpPhase::kCrouch:
      phase_pitch_scale = 1.2;
      break;
    case go2_jump_core::JumpPhase::kPush:
      phase_pitch_scale = 2.4;
      break;
    case go2_jump_core::JumpPhase::kFlight:
      phase_pitch_scale = 1.8;
      break;
    case go2_jump_core::JumpPhase::kLanding:
      phase_pitch_scale = 1.3;
      break;
    case go2_jump_core::JumpPhase::kSettle:
      phase_pitch_scale = 1.0;
      break;
  }
  const double hip_delta =
      Clamp(0.18 * phase_pitch_scale * pitch_rad, -0.14, 0.14);
  const double thigh_delta =
      Clamp(0.28 * phase_pitch_scale * pitch_rad, -0.22, 0.22);
  const double calf_delta =
      Clamp(-0.40 * phase_pitch_scale * pitch_rad, -0.28, 0.28);

  pose[0] -= hip_delta;
  pose[3] += hip_delta;
  pose[6] -= 0.5 * hip_delta;
  pose[9] += 0.5 * hip_delta;

  pose[1] += thigh_delta;
  pose[4] += thigh_delta;
  pose[7] -= thigh_delta;
  pose[10] -= thigh_delta;

  pose[2] += calf_delta;
  pose[5] += calf_delta;
  pose[8] -= 0.6 * calf_delta;
  pose[11] -= 0.6 * calf_delta;

  return pose;
}

std::array<double, kControlledJointCount> WholeBodyMpc::BuildFeedforwardForSample(
    const go2_jump_core::JumpTaskSpec& task,
    const go2_jump_core::JumpReferenceSample& sample) const {
  std::array<double, kControlledJointCount> tau{};
  const double push_scale = Clamp(task.target_takeoff_speed_mps / 2.2, 0.0, 1.0);
  const double forward_bias_scale =
      Clamp(task.target_takeoff_velocity_x_mps / 1.4, 0.0, 1.4);
  const double landing_scale = Clamp(sample.landing_brace_factor, 0.0, 1.0);

  if (sample.phase == go2_jump_core::JumpPhase::kPush) {
    const double thigh_tau =
        Clamp(5.5 + 6.0 * push_scale, 0.0, config_.max_feedforward_torque_nm);
    const double calf_tau =
        Clamp(9.0 + 8.5 * push_scale, 0.0, config_.max_feedforward_torque_nm);
    const double front_thigh_tau = Clamp(
        thigh_tau - 0.8 * forward_bias_scale, 0.0,
        config_.max_feedforward_torque_nm);
    const double front_calf_tau = Clamp(
        calf_tau - 1.2 * forward_bias_scale, 0.0,
        config_.max_feedforward_torque_nm);
    const double rear_thigh_tau = Clamp(
        thigh_tau + 1.4 * forward_bias_scale, 0.0,
        config_.max_feedforward_torque_nm);
    const double rear_calf_tau = Clamp(
        calf_tau + 2.2 * forward_bias_scale, 0.0,
        config_.max_feedforward_torque_nm);
    for (std::size_t idx : {1u, 4u}) {
      tau[idx] = front_thigh_tau;
    }
    for (std::size_t idx : {2u, 5u}) {
      tau[idx] = front_calf_tau;
    }
    for (std::size_t idx : {7u, 10u}) {
      tau[idx] = rear_thigh_tau;
    }
    for (std::size_t idx : {8u, 11u}) {
      tau[idx] = rear_calf_tau;
    }
  } else if (sample.phase == go2_jump_core::JumpPhase::kLanding) {
    const double thigh_tau = Clamp(-3.0 - 5.0 * landing_scale,
                                   -config_.max_feedforward_torque_nm, 0.0);
    const double calf_tau = Clamp(-4.5 - 7.5 * landing_scale,
                                  -config_.max_feedforward_torque_nm, 0.0);
    for (std::size_t idx : {1u, 4u, 7u, 10u}) {
      tau[idx] = thigh_tau;
    }
    for (std::size_t idx : {2u, 5u, 8u, 11u}) {
      tau[idx] = calf_tau;
    }
  }

  return tau;
}

double WholeBodyMpc::GainsForPhase(go2_jump_core::JumpPhase phase,
                                   bool derivative) const {
  switch (phase) {
    case go2_jump_core::JumpPhase::kCrouch:
      return derivative ? config_.default_kd : config_.default_kp;
    case go2_jump_core::JumpPhase::kPush:
      return derivative ? config_.push_kd : config_.push_kp;
    case go2_jump_core::JumpPhase::kFlight:
      return derivative ? config_.flight_kd : config_.flight_kp;
    case go2_jump_core::JumpPhase::kLanding:
      return derivative ? config_.landing_kd : config_.landing_kp;
    case go2_jump_core::JumpPhase::kSettle:
      return derivative ? config_.settle_kd : config_.settle_kp;
  }
  return derivative ? config_.default_kd : config_.default_kp;
}

go2_jump_core::JumpReferenceSample WholeBodyMpc::ApplyContactOverrides(
    const go2_jump_core::JumpReferenceSample& planned_sample,
    const RobotObservation& observation, double task_elapsed_s) {
  const auto phase_state =
      BuildExecutionPhaseEventState(observation, task_elapsed_s);
  return ApplyContactOverrides(planned_sample, phase_state,
                               CountContacts(observation.foot_contact),
                               observation.body_velocity[2], task_elapsed_s);
}

go2_jump_core::JumpReferenceSample WholeBodyMpc::ApplyContactOverrides(
    const go2_jump_core::JumpReferenceSample& planned_sample,
    const PhaseEventState& state, int contact_count,
    double body_vertical_velocity_mps, double task_elapsed_s) const {
  auto sample = planned_sample;
  if (!state.contact_signal_valid) {
    return sample;
  }

  const double push_end = task_.crouch_duration_s + task_.push_duration_s;
  const double max_push_extension_s = std::max(
      config_.late_takeoff_window_s,
      std::min(0.34, std::max(0.18, task_.estimated_flight_time_s + 0.10)));
  const double max_flight_hold_s = task_.estimated_flight_time_s + 0.12;
  const double flight_duration_s = std::max(task_.estimated_flight_time_s, 1e-6);
  const double landing_duration_s = std::max(task_.landing_duration_s, 1e-6);
  const double settle_duration_s = std::max(task_.settle_duration_s, 1e-6);
  const double takeoff_time_s =
      state.takeoff_latched ? state.takeoff_time_s : state.takeoff_candidate_start_s;
  const double touchdown_time_s = state.touchdown_latched
                                      ? state.touchdown_time_s
                                      : state.touchdown_candidate_start_s;

  const auto build_push_hold_sample = [&]() {
    auto hold_sample = go2_jump_core::SampleJumpReference(
        task_, config_.reference_config, std::max(0.0, push_end - 1e-4));
    hold_sample.phase = go2_jump_core::JumpPhase::kPush;
    hold_sample.time_in_phase_s = task_.push_duration_s;
    hold_sample.desired_forward_velocity_mps = task_.target_takeoff_velocity_x_mps;
    hold_sample.desired_vertical_velocity_mps = task_.target_takeoff_velocity_z_mps;
    hold_sample.desired_body_pitch_deg = task_.objective.target_takeoff_pitch_deg;
    hold_sample.desired_body_height_offset_m =
        config_.reference_config.push_height_offset_m;
    hold_sample.leg_retraction_ratio = 0.0;
    hold_sample.landing_brace_factor = 0.0;
    return hold_sample;
  };

  const auto build_flight_sample = [&](double flight_elapsed_s) {
    go2_jump_core::JumpReferenceSample flight_sample{};
    const double alpha = Clamp(flight_elapsed_s / flight_duration_s, 0.0, 1.0);
    const double descent_alpha = Smooth01(std::max(0.0, (alpha - 0.5) / 0.5));
    flight_sample.phase = go2_jump_core::JumpPhase::kFlight;
    flight_sample.time_in_phase_s = flight_elapsed_s;
    flight_sample.desired_forward_velocity_mps = task_.target_takeoff_velocity_x_mps;
    flight_sample.desired_vertical_velocity_mps =
        task_.target_takeoff_velocity_z_mps -
        config_.reference_config.gravity_mps2 * flight_elapsed_s;
    flight_sample.desired_body_pitch_deg =
        Lerp(task_.objective.target_takeoff_pitch_deg,
             task_.objective.target_landing_pitch_deg, descent_alpha);
    flight_sample.desired_body_height_offset_m = Lerp(
        config_.reference_config.flight_height_offset_m,
        config_.reference_config.landing_height_offset_m, descent_alpha);
    flight_sample.leg_retraction_ratio = Smooth01(alpha);
    flight_sample.landing_brace_factor = descent_alpha;
    return flight_sample;
  };

  const auto build_landing_sample = [&](double landing_elapsed_s) {
    go2_jump_core::JumpReferenceSample landing_sample{};
    const double alpha = Smooth01(landing_elapsed_s / landing_duration_s);
    landing_sample.phase = go2_jump_core::JumpPhase::kLanding;
    landing_sample.time_in_phase_s = landing_elapsed_s;
    landing_sample.desired_forward_velocity_mps = 0.0;
    landing_sample.desired_vertical_velocity_mps = 0.0;
    landing_sample.desired_body_pitch_deg =
        Lerp(task_.objective.target_landing_pitch_deg, 0.0, alpha);
    landing_sample.desired_body_height_offset_m =
        Lerp(config_.reference_config.landing_height_offset_m, 0.0, alpha);
    landing_sample.leg_retraction_ratio = 0.0;
    landing_sample.landing_brace_factor = Lerp(1.0, 0.3, alpha);
    return landing_sample;
  };

  const auto build_settle_sample = [&](double settle_elapsed_s) {
    go2_jump_core::JumpReferenceSample settle_sample{};
    const double alpha = Smooth01(settle_elapsed_s / settle_duration_s);
    settle_sample.phase = go2_jump_core::JumpPhase::kSettle;
    settle_sample.time_in_phase_s = settle_elapsed_s;
    settle_sample.desired_forward_velocity_mps = 0.0;
    settle_sample.desired_vertical_velocity_mps = 0.0;
    settle_sample.desired_body_pitch_deg = 0.0;
    settle_sample.desired_body_height_offset_m = 0.0;
    settle_sample.leg_retraction_ratio = 0.0;
    settle_sample.landing_brace_factor = Lerp(0.3, 0.0, alpha);
    return settle_sample;
  };

  if (!state.takeoff_latched &&
      contact_count > config_.flight_contact_count_max &&
      task_elapsed_s <= push_end + max_push_extension_s &&
      (planned_sample.phase == go2_jump_core::JumpPhase::kFlight ||
       planned_sample.phase == go2_jump_core::JumpPhase::kLanding ||
       planned_sample.phase == go2_jump_core::JumpPhase::kSettle) &&
      body_vertical_velocity_mps >= -0.20) {
    return build_push_hold_sample();
  }

  if (takeoff_time_s >= 0.0 && !state.touchdown_latched) {
    const double flight_elapsed_s = std::max(0.0, task_elapsed_s - takeoff_time_s);
    if (planned_sample.phase == go2_jump_core::JumpPhase::kPush ||
        planned_sample.phase == go2_jump_core::JumpPhase::kFlight ||
        ((planned_sample.phase == go2_jump_core::JumpPhase::kLanding ||
          planned_sample.phase == go2_jump_core::JumpPhase::kSettle) &&
         flight_elapsed_s <= max_flight_hold_s)) {
      return build_flight_sample(flight_elapsed_s);
    }
  }

  if (touchdown_time_s >= 0.0 && !state.settle_latched) {
    const double landing_elapsed_s =
        std::max(0.0, task_elapsed_s - touchdown_time_s);
    return build_landing_sample(landing_elapsed_s);
  }

  if (state.settle_latched) {
    const double settle_start_time_s =
        state.settle_time_s >= 0.0 ? state.settle_time_s : task_elapsed_s;
    const double settle_elapsed_s =
        std::max(0.0, task_elapsed_s - settle_start_time_s);
    return build_settle_sample(settle_elapsed_s);
  }

  return sample;
}

}  // namespace go2_jump_mpc
