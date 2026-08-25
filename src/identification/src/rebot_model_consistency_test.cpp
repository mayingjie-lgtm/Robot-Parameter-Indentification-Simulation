/**
 * @file rebot_model_consistency_test.cpp
 * @brief Cross-check canonical reBot-DM MuJoCo and Pinocchio dynamics.
 */

#include "rebot_pinocchio_dynamics.hpp"

#include <mujoco/mujoco.h>

#include <Eigen/Core>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::size_t kDof = 6;
constexpr std::size_t kRandomStateCount = 100;
constexpr double kMappingTolerance = 1e-12;
constexpr double kGravityToleranceNm = 1e-12;
constexpr double kMassTolerance = 1e-12;
constexpr double kInverseToleranceNm = 1e-12;
constexpr double kRegressorToleranceNm = 1e-12;
constexpr double kGripperLockPosition = 0.05;

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

struct ExpectedJoint {
  const char *name;
  std::array<double, 3> axis;
  double lower;
  double upper;
  double effort;
};

constexpr std::array<ExpectedJoint, kDof> kExpectedJoints{{
    {"joint1", {0.0, 0.0, 1.0}, -2.8, 2.8, 27.0},
    {"joint2", {0.0, 0.0, -1.0}, -3.14, 0.0, 27.0},
    {"joint3", {0.0, 0.0, 1.0}, -3.14, 0.0, 27.0},
    {"joint4", {0.0, 0.0, 1.0}, -1.87, 1.57, 7.0},
    {"joint5", {0.0, 0.0, 1.0}, -1.57, 1.57, 7.0},
    {"joint6", {0.0, 0.0, 1.0}, -3.14, 3.14, 7.0},
}};

struct MujocoMapping {
  std::array<int, kDof> joint_ids{};
  std::array<int, kDof> qpos_indices{};
  std::array<int, kDof> dof_indices{};
  std::array<int, kDof> actuator_indices{};
  std::array<int, 2> gripper_qpos_indices{};
  std::array<int, 2> gripper_dof_indices{};
};

struct State {
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd qdd;
};

struct TorqueMetrics {
  std::array<double, kDof> sum_squared{};
  std::array<double, kDof> max_abs{};
  double global_max_abs = 0.0;
  double global_sum_squared = 0.0;
  std::size_t scalar_count = 0;
  Eigen::VectorXd worst_q = Eigen::VectorXd::Zero(kDof);
  Eigen::VectorXd worst_qd = Eigen::VectorXd::Zero(kDof);
  Eigen::VectorXd worst_qdd = Eigen::VectorXd::Zero(kDof);

  /** Accumulate one six-joint torque error and retain the worst state. */
  void add(const Eigen::VectorXd &error, const State &state) {
    const double state_max = error.cwiseAbs().maxCoeff();
    if (state_max > global_max_abs) {
      global_max_abs = state_max;
      worst_q = state.q;
      worst_qd = state.qd;
      worst_qdd = state.qdd;
    }
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      const double value = error(static_cast<Eigen::Index>(joint));
      const double absolute = std::abs(value);
      sum_squared[joint] += value * value;
      max_abs[joint] = std::max(max_abs[joint], absolute);
      global_sum_squared += value * value;
      ++scalar_count;
    }
  }

  /** Return aggregate RMSE across every accumulated joint observation. */
  double rmse() const {
    return std::sqrt(global_sum_squared / static_cast<double>(scalar_count));
  }
};

/** Load the canonical MuJoCo model and surface parser errors. */
ModelPtr loadMujocoModel(const std::filesystem::path &path) {
  char error[1024]{};
  ModelPtr model(mj_loadXML(path.string().c_str(), nullptr, error, sizeof(error)),
                 mj_deleteModel);
  if (!model) {
    throw std::runtime_error("Failed to load MuJoCo model: " + std::string(error));
  }
  // Dynamics gates intentionally exclude joint-limit/contact/equality forces.
  model->opt.disableflags |= mjDSBL_CONSTRAINT;
  return model;
}

/** Resolve every arm/gripper coordinate by name rather than XML position. */
MujocoMapping resolveMujocoMapping(const mjModel *model) {
  MujocoMapping mapping;
  for (std::size_t index = 0; index < kDof; ++index) {
    const std::string joint_name = "joint" + std::to_string(index + 1);
    const std::string actuator_name = "actuator" + std::to_string(index + 1);
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name.c_str());
    const int actuator_id =
        mj_name2id(model, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (joint_id < 0 || actuator_id < 0) {
      throw std::runtime_error("Incomplete reBot MuJoCo arm mapping");
    }
    mapping.joint_ids[index] = joint_id;
    mapping.qpos_indices[index] = model->jnt_qposadr[joint_id];
    mapping.dof_indices[index] = model->jnt_dofadr[joint_id];
    mapping.actuator_indices[index] = actuator_id;
  }
  for (std::size_t index = 0; index < 2; ++index) {
    const std::string name = "gripper_joint" + std::to_string(index + 1);
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, name.c_str());
    if (joint_id < 0) {
      throw std::runtime_error("Incomplete reBot MuJoCo gripper mapping");
    }
    mapping.gripper_qpos_indices[index] = model->jnt_qposadr[joint_id];
    mapping.gripper_dof_indices[index] = model->jnt_dofadr[joint_id];
  }
  return mapping;
}

/** Assign one arm state while keeping both gripper joints at the fixed Phase 4A opening. */
void setMujocoState(const mjModel *model, mjData *data,
                    const MujocoMapping &mapping, const State &state) {
  mj_resetData(model, data);
  mju_zero(data->qvel, model->nv);
  mju_zero(data->qacc, model->nv);
  for (std::size_t index = 0; index < kDof; ++index) {
    data->qpos[mapping.qpos_indices[index]] =
        state.q(static_cast<Eigen::Index>(index));
    data->qvel[mapping.dof_indices[index]] =
        state.qd(static_cast<Eigen::Index>(index));
    data->qacc[mapping.dof_indices[index]] =
        state.qdd(static_cast<Eigen::Index>(index));
  }
  for (std::size_t index = 0; index < 2; ++index) {
    data->qpos[mapping.gripper_qpos_indices[index]] = kGripperLockPosition;
    data->qvel[mapping.gripper_dof_indices[index]] = 0.0;
    data->qacc[mapping.gripper_dof_indices[index]] = 0.0;
  }
}

/** Extract the six arm entries from one MuJoCo generalized-force vector. */
Eigen::VectorXd extractArmVector(const mjtNum *values,
                                 const MujocoMapping &mapping) {
  Eigen::VectorXd result(kDof);
  for (std::size_t index = 0; index < kDof; ++index) {
    result(static_cast<Eigen::Index>(index)) = values[mapping.dof_indices[index]];
  }
  return result;
}

/** Compute MuJoCo gravity/bias torque at the supplied state. */
Eigen::VectorXd mujocoBias(const mjModel *model, mjData *data,
                           const MujocoMapping &mapping, const State &state) {
  setMujocoState(model, data, mapping, state);
  mj_forward(model, data);
  return extractArmVector(data->qfrc_bias, mapping);
}

/** Compute the arm block of MuJoCo's full joint-space mass matrix. */
Eigen::MatrixXd mujocoMassMatrix(const mjModel *model, mjData *data,
                                 const MujocoMapping &mapping,
                                 const State &state) {
  setMujocoState(model, data, mapping, state);
  mj_forward(model, data);
  std::vector<mjtNum> dense(static_cast<std::size_t>(model->nv * model->nv), 0.0);
  mj_fullM(model, data, dense.data());
  Eigen::MatrixXd mass(kDof, kDof);
  for (std::size_t row = 0; row < kDof; ++row) {
    for (std::size_t column = 0; column < kDof; ++column) {
      mass(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
          dense[static_cast<std::size_t>(mapping.dof_indices[row] * model->nv +
                                         mapping.dof_indices[column])];
    }
  }
  return mass;
}

/** Compute unconstrained MuJoCo inverse dynamics for the six arm joints. */
Eigen::VectorXd mujocoInverseDynamics(const mjModel *model, mjData *data,
                                      const MujocoMapping &mapping,
                                      const State &state) {
  setMujocoState(model, data, mapping, state);
  mj_inverse(model, data);
  return extractArmVector(data->qfrc_inverse, mapping);
}

/** Build one full Pinocchio state with both gripper coordinates at the Phase 4A lock. */
void setFullPinocchioState(const pinocchio::Model &model, const State &state,
                           Eigen::VectorXd &q, Eigen::VectorXd &qd,
                           Eigen::VectorXd &qdd) {
  q = pinocchio::neutral(model);
  qd = Eigen::VectorXd::Zero(model.nv);
  qdd = Eigen::VectorXd::Zero(model.nv);
  for (std::size_t index = 0; index < kDof; ++index) {
    const pinocchio::JointIndex joint_id =
        model.getJointId("joint" + std::to_string(index + 1));
    q(model.idx_qs[joint_id]) = state.q(static_cast<Eigen::Index>(index));
    qd(model.idx_vs[joint_id]) = state.qd(static_cast<Eigen::Index>(index));
    qdd(model.idx_vs[joint_id]) = state.qdd(static_cast<Eigen::Index>(index));
  }
  for (std::size_t index = 0; index < 2; ++index) {
    const pinocchio::JointIndex joint_id =
        model.getJointId("gripper_joint" + std::to_string(index + 1));
    q(model.idx_qs[joint_id]) = kGripperLockPosition;
  }
}

/** Extract the six arm block of the unreduced Pinocchio mass matrix. */
Eigen::MatrixXd fullPinocchioArmMassMatrix(
    const rebot_dynamics::ReBotPinocchioDynamics &dynamics, const State &state) {
  const pinocchio::Model &model = dynamics.fullModel();
  pinocchio::Data data(model);
  Eigen::VectorXd q, qd, qdd;
  setFullPinocchioState(model, state, q, qd, qdd);
  pinocchio::crba(model, data, q);
  data.M.template triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().template triangularView<Eigen::StrictlyLower>();
  Eigen::MatrixXd arm_mass(kDof, kDof);
  for (std::size_t row = 0; row < kDof; ++row) {
    const pinocchio::JointIndex row_id =
        model.getJointId("joint" + std::to_string(row + 1));
    for (std::size_t column = 0; column < kDof; ++column) {
      const pinocchio::JointIndex column_id =
          model.getJointId("joint" + std::to_string(column + 1));
      arm_mass(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
          data.M(model.idx_vs[row_id], model.idx_vs[column_id]);
    }
  }
  return arm_mass;
}

/** Extract six arm torques from unreduced Pinocchio RNEA with locked gripper state. */
Eigen::VectorXd fullPinocchioArmInverseDynamics(
    const rebot_dynamics::ReBotPinocchioDynamics &dynamics, const State &state) {
  const pinocchio::Model &model = dynamics.fullModel();
  pinocchio::Data data(model);
  Eigen::VectorXd q, qd, qdd;
  setFullPinocchioState(model, state, q, qd, qdd);
  const Eigen::VectorXd tau = pinocchio::rnea(model, data, q, qd, qdd);
  Eigen::VectorXd arm_tau(kDof);
  for (std::size_t index = 0; index < kDof; ++index) {
    const pinocchio::JointIndex joint_id =
        model.getJointId("joint" + std::to_string(index + 1));
    arm_tau(static_cast<Eigen::Index>(index)) = tau(model.idx_vs[joint_id]);
  }
  return arm_tau;
}

/** Produce deterministic states inside a 10% margin of every arm joint limit. */
std::vector<State> makeRandomStates(std::size_t count, unsigned int seed) {
  std::mt19937 generator(seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> velocity(-0.8, 0.8);
  std::uniform_real_distribution<double> acceleration(-1.5, 1.5);
  std::vector<State> states;
  states.reserve(count);
  for (std::size_t sample = 0; sample < count; ++sample) {
    State state{Eigen::VectorXd(kDof), Eigen::VectorXd(kDof),
                Eigen::VectorXd(kDof)};
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      const double range = kExpectedJoints[joint].upper - kExpectedJoints[joint].lower;
      const double lower = kExpectedJoints[joint].lower + 0.1 * range;
      const double upper = kExpectedJoints[joint].upper - 0.1 * range;
      state.q(static_cast<Eigen::Index>(joint)) = lower + unit(generator) * (upper - lower);
      state.qd(static_cast<Eigen::Index>(joint)) = velocity(generator);
      state.qdd(static_cast<Eigen::Index>(joint)) = acceleration(generator);
    }
    states.push_back(std::move(state));
  }
  return states;
}

/** Print a six-element state vector at full diagnostic precision. */
void printVector(const char *label, const Eigen::VectorXd &values) {
  std::cout << label << "=[";
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    if (index > 0) {
      std::cout << ", ";
    }
    std::cout << std::scientific << std::setprecision(12) << values(index);
  }
  std::cout << "]\n";
}

/** Print per-joint and aggregate torque-error metrics. */
void printTorqueMetrics(const char *title, const TorqueMetrics &metrics,
                        std::size_t state_count) {
  std::cout << "\n" << title << " over " << state_count << " states\n";
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    const double joint_rmse =
        std::sqrt(metrics.sum_squared[joint] / static_cast<double>(state_count));
    std::cout << "  J" << joint + 1 << " RMSE=" << std::scientific
              << std::setprecision(12) << joint_rmse
              << " max_abs=" << metrics.max_abs[joint] << " Nm\n";
  }
  std::cout << "  aggregate RMSE=" << metrics.rmse()
            << " Nm global max_abs=" << metrics.global_max_abs << " Nm\n";
  printVector("  worst q", metrics.worst_q);
  printVector("  worst qd", metrics.worst_qd);
  printVector("  worst qdd", metrics.worst_qdd);
}

/** Gate 1: verify MuJoCo and reduced Pinocchio represent the same six joints. */
bool validateJointMapping(
    const mjModel *mujoco_model, const MujocoMapping &mapping,
    rebot_dynamics::ReBotPinocchioDynamics &pinocchio_model) {
  const pinocchio::Model &pin = pinocchio_model.model();
  bool passed = pin.nq == static_cast<int>(kDof) && pin.nv == static_cast<int>(kDof);
  std::cout << "\nGate 1 - joint mapping\n";
  std::cout << "joint mj_id qpos dof pin_id pin_q pin_v axis                lower upper effort\n";
  for (std::size_t index = 0; index < kDof; ++index) {
    const int mj_joint_id = mapping.joint_ids[index];
    const pinocchio::JointIndex pin_joint_id = pinocchio_model.armJointIds()[index];
    const Eigen::Vector3d pin_axis = pinocchio_model.localJointAxis(index);
    const mjtNum *mj_axis = &mujoco_model->jnt_axis[3 * mj_joint_id];
    const double mj_lower = mujoco_model->jnt_range[2 * mj_joint_id];
    const double mj_upper = mujoco_model->jnt_range[2 * mj_joint_id + 1];
    const double pin_lower = pin.lowerPositionLimit(pin.idx_qs[pin_joint_id]);
    const double pin_upper = pin.upperPositionLimit(pin.idx_qs[pin_joint_id]);
    const double pin_effort = pin.upperEffortLimit(pin.idx_vs[pin_joint_id]);
    const double mj_effort =
        mujoco_model->actuator_ctrlrange[2 * mapping.actuator_indices[index] + 1];

    std::cout << kExpectedJoints[index].name << " " << mj_joint_id << " "
              << mapping.qpos_indices[index] << " " << mapping.dof_indices[index]
              << " " << pin_joint_id << " " << pin.idx_qs[pin_joint_id] << " "
              << pin.idx_vs[pin_joint_id] << " [" << pin_axis.transpose() << "] "
              << pin_lower << " " << pin_upper << " " << pin_effort << "\n";

    const bool axis_ok =
        std::abs(pin_axis.x() - mj_axis[0]) <= kMappingTolerance &&
        std::abs(pin_axis.y() - mj_axis[1]) <= kMappingTolerance &&
        std::abs(pin_axis.z() - mj_axis[2]) <= kMappingTolerance;
    const bool limits_ok = std::abs(pin_lower - mj_lower) <= kMappingTolerance &&
                           std::abs(pin_upper - mj_upper) <= kMappingTolerance;
    const bool effort_ok = std::abs(pin_effort - mj_effort) <= kMappingTolerance;
    const bool dimensions_ok = pin.joints[pin_joint_id].nq() == 1 &&
                               pin.joints[pin_joint_id].nv() == 1;
    passed = axis_ok && limits_ok && effort_ok && dimensions_ok && passed;
  }
  std::cout << "  full Pinocchio nq=" << pinocchio_model.fullModel().nq
            << " nv=" << pinocchio_model.fullModel().nv
            << ", reduced nq=" << pin.nq << " nv=" << pin.nv << "\n";
  std::cout << "  gripper_lock_position=["
            << pinocchio_model.gripperLockPosition()[0] << ", "
            << pinocchio_model.gripperLockPosition()[1] << "] m\n";
  return passed;
}

/** Diagnose compiled MuJoCo inertia tensors against Pinocchio for links 1-5. */
void printCompiledInertiaDiagnostics(
    const mjModel *model,
    const rebot_dynamics::ReBotPinocchioDynamics &pinocchio_model) {
  std::cout << "\nCompiled inertia diagnostics (link1..link5)\n";
  for (std::size_t index = 0; index < 5; ++index) {
    const std::string body_name = "link" + std::to_string(index + 1);
    const int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    const pinocchio::JointIndex joint_id =
        pinocchio_model.model().getJointId("joint" + std::to_string(index + 1));
    if (body_id < 0) {
      throw std::runtime_error("Missing MuJoCo body " + body_name);
    }
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotation;
    mju_quat2Mat(rotation.data(), &model->body_iquat[4 * body_id]);
    Eigen::Vector3d principal;
    principal << model->body_inertia[3 * body_id],
        model->body_inertia[3 * body_id + 1],
        model->body_inertia[3 * body_id + 2];
    const Eigen::Matrix3d mujoco_inertia =
        rotation * principal.asDiagonal() * rotation.transpose();
    const Eigen::Matrix3d pin_inertia =
        pinocchio_model.model().inertias[joint_id].inertia();
    const Eigen::Vector3d mujoco_com =
        Eigen::Map<const Eigen::Vector3d>(&model->body_ipos[3 * body_id]);
    const Eigen::Vector3d pin_com =
        pinocchio_model.model().inertias[joint_id].lever();
    const double mass_error =
        std::abs(model->body_mass[body_id] -
                 pinocchio_model.model().inertias[joint_id].mass());
    std::cout << "  " << body_name
              << " inertia_max_abs="
              << (mujoco_inertia - pin_inertia).cwiseAbs().maxCoeff()
              << " com_max_abs=" << (mujoco_com - pin_com).cwiseAbs().maxCoeff()
              << " mass_abs=" << mass_error << "\n";
  }
}

/** Gate 2: compare one static gravity state. */
bool validateStaticGravity(
    const mjModel *model, mjData *data, const MujocoMapping &mapping,
    rebot_dynamics::ReBotPinocchioDynamics &pinocchio_model) {
  State state{Eigen::VectorXd(kDof), Eigen::VectorXd::Zero(kDof),
              Eigen::VectorXd::Zero(kDof)};
  state.q << 0.0, -1.0, -1.0, 0.0, 0.0, 0.0;
  const Eigen::VectorXd tau_mujoco = mujocoBias(model, data, mapping, state);
  const Eigen::VectorXd tau_pinocchio =
      pinocchio_model.inverseDynamics(state.q, state.qd, state.qdd);
  const Eigen::VectorXd error = tau_mujoco - tau_pinocchio;
  std::cout << "\nGate 2 - static gravity\n";
  printVector("  tau_mujoco", tau_mujoco);
  printVector("  tau_pinocchio", tau_pinocchio);
  printVector("  error", error);
  std::cout << "  max_abs=" << error.cwiseAbs().maxCoeff() << " Nm\n";
  return error.cwiseAbs().maxCoeff() <= kGravityToleranceNm;
}

/** Gate 3: compare gravity torque over deterministic random poses. */
bool validateRandomGravity(
    const mjModel *model, mjData *data, const MujocoMapping &mapping,
    rebot_dynamics::ReBotPinocchioDynamics &pinocchio_model,
    const std::vector<State> &states) {
  TorqueMetrics metrics;
  for (const State &raw_state : states) {
    State state = raw_state;
    state.qd.setZero();
    state.qdd.setZero();
    const Eigen::VectorXd error =
        mujocoBias(model, data, mapping, state) -
        pinocchio_model.inverseDynamics(state.q, state.qd, state.qdd);
    metrics.add(error, state);
  }
  printTorqueMetrics("Gate 3 - random-pose gravity", metrics, states.size());
  return metrics.global_max_abs <= kGravityToleranceNm;
}

/** Gate 4: compare the complete six-by-six rigid-body mass matrix. */
bool validateMassMatrix(
    const mjModel *model, mjData *data, const MujocoMapping &mapping,
    rebot_dynamics::ReBotPinocchioDynamics &pinocchio_model,
    const std::vector<State> &states) {
  double max_abs_error = 0.0;
  double max_relative_frobenius = 0.0;
  double max_mujoco_vs_full_pin = 0.0;
  double max_full_vs_reduced_pin = 0.0;
  Eigen::VectorXd worst_q = Eigen::VectorXd::Zero(kDof);
  Eigen::MatrixXd worst_error = Eigen::MatrixXd::Zero(kDof, kDof);
  for (const State &state : states) {
    const Eigen::MatrixXd mj_mass = mujocoMassMatrix(model, data, mapping, state);
    const Eigen::MatrixXd pin_mass = pinocchio_model.massMatrix(state.q);
    const Eigen::MatrixXd full_pin_mass =
        fullPinocchioArmMassMatrix(pinocchio_model, state);
    const Eigen::MatrixXd error = mj_mass - pin_mass;
    const double state_max = error.cwiseAbs().maxCoeff();
    const double relative = error.norm() / std::max(pin_mass.norm(), 1e-15);
    max_mujoco_vs_full_pin =
        std::max(max_mujoco_vs_full_pin,
                 (mj_mass - full_pin_mass).cwiseAbs().maxCoeff());
    max_full_vs_reduced_pin =
        std::max(max_full_vs_reduced_pin,
                 (full_pin_mass - pin_mass).cwiseAbs().maxCoeff());
    if (state_max > max_abs_error) {
      max_abs_error = state_max;
      worst_q = state.q;
      worst_error = error;
    }
    max_relative_frobenius = std::max(max_relative_frobenius, relative);
  }
  std::cout << "\nGate 4 - M(q) over " << states.size() << " states\n";
  std::cout << "  MuJoCo vs reduced Pin max_abs=" << std::scientific
            << std::setprecision(12) << max_abs_error << " kg*m^2\n";
  std::cout << "  MuJoCo vs full Pin max_abs=" << max_mujoco_vs_full_pin
            << " kg*m^2\n";
  std::cout << "  full Pin vs reduced Pin max_abs=" << max_full_vs_reduced_pin
            << " kg*m^2\n";
  std::cout << "  max_relative_frobenius=" << max_relative_frobenius << "\n";
  printVector("  worst q", worst_q);
  std::cout << "  worst error matrix:\n" << worst_error << "\n";
  return max_abs_error <= kMassTolerance && max_relative_frobenius <= 1e-12;
}

/** Gate 5: compare unconstrained rigid-body inverse dynamics over random states. */
bool validateInverseDynamics(
    const mjModel *model, mjData *data, const MujocoMapping &mapping,
    rebot_dynamics::ReBotPinocchioDynamics &pinocchio_model,
    const std::vector<State> &states) {
  TorqueMetrics metrics;
  double max_mujoco_vs_full_pin = 0.0;
  double max_full_vs_reduced_pin = 0.0;
  for (const State &state : states) {
    const Eigen::VectorXd tau_mujoco =
        mujocoInverseDynamics(model, data, mapping, state);
    const Eigen::VectorXd tau_reduced_pin =
        pinocchio_model.inverseDynamics(state.q, state.qd, state.qdd);
    const Eigen::VectorXd tau_full_pin =
        fullPinocchioArmInverseDynamics(pinocchio_model, state);
    metrics.add(tau_mujoco - tau_reduced_pin, state);
    max_mujoco_vs_full_pin =
        std::max(max_mujoco_vs_full_pin,
                 (tau_mujoco - tau_full_pin).cwiseAbs().maxCoeff());
    max_full_vs_reduced_pin =
        std::max(max_full_vs_reduced_pin,
                 (tau_full_pin - tau_reduced_pin).cwiseAbs().maxCoeff());
  }
  printTorqueMetrics("Gate 5 - inverse dynamics", metrics, states.size());
  std::cout << "  MuJoCo vs full Pin max_abs=" << max_mujoco_vs_full_pin
            << " Nm\n";
  std::cout << "  full Pin vs reduced Pin max_abs=" << max_full_vs_reduced_pin
            << " Nm\n";
  std::cout << "  explicit contributions: rigid-body=compared, armature=0, damping=0, friction=0\n";
  return metrics.global_max_abs <= kInverseToleranceNm;
}

/** Gate 6: verify Pinocchio's rigid-body Y*theta identity independently of MuJoCo. */
bool validatePinocchioRegressorIdentity(
    rebot_dynamics::ReBotPinocchioDynamics &pinocchio_model,
    const std::vector<State> &states) {
  const Eigen::VectorXd theta = pinocchio_model.rigidBodyParameterVector();
  TorqueMetrics metrics;
  for (const State &state : states) {
    const Eigen::MatrixXd regressor =
        pinocchio_model.torqueRegressor(state.q, state.qd, state.qdd);
    if (regressor.cols() != theta.size()) {
      throw std::runtime_error("Pinocchio torque regressor/parameter dimensions disagree");
    }
    const Eigen::VectorXd tau_y = regressor * theta;
    const Eigen::VectorXd tau_rnea =
        pinocchio_model.inverseDynamics(state.q, state.qd, state.qdd);
    metrics.add(tau_y - tau_rnea, state);
  }
  printTorqueMetrics("Gate 6 - Pinocchio Y*theta identity", metrics, states.size());
  std::cout << "  theta_size=" << theta.size() << " (6 rigid-body blocks x 10)\n";
  return metrics.global_max_abs <= kRegressorToleranceNm;
}

} // namespace

int main() {
  try {
    const std::filesystem::path root(PROJECT_ROOT_DIR);
    const std::filesystem::path mjcf_path = root / "rebot_dm" / "rebot_dm.xml";
    const std::filesystem::path urdf_path = root / "rebot_dm" / "rebot_dm.urdf";

    ModelPtr mujoco_model = loadMujocoModel(mjcf_path);
    DataPtr mujoco_data(mj_makeData(mujoco_model.get()), mj_deleteData);
    if (!mujoco_data) {
      throw std::runtime_error("Failed to allocate MuJoCo data");
    }
    const MujocoMapping mapping = resolveMujocoMapping(mujoco_model.get());
    rebot_dynamics::ReBotPinocchioDynamics pinocchio_model(
        urdf_path, {kGripperLockPosition, kGripperLockPosition});

    std::cout << std::string(88, '=') << "\n";
    std::cout << "reBot-DM MuJoCo / Pinocchio Phase 4A consistency\n";
    std::cout << std::string(88, '=') << "\n";
    std::cout << "simulation truth: armature=0 damping=0 frictionloss=0\n";

    bool passed = validateJointMapping(mujoco_model.get(), mapping, pinocchio_model);
    printCompiledInertiaDiagnostics(mujoco_model.get(), pinocchio_model);
    passed = validateStaticGravity(mujoco_model.get(), mujoco_data.get(), mapping,
                                   pinocchio_model) && passed;

    const std::vector<State> gravity_states =
        makeRandomStates(kRandomStateCount, 20260825U);
    passed = validateRandomGravity(mujoco_model.get(), mujoco_data.get(), mapping,
                                   pinocchio_model, gravity_states) && passed;

    const std::vector<State> dynamic_states =
        makeRandomStates(kRandomStateCount, 20260826U);
    passed = validateMassMatrix(mujoco_model.get(), mujoco_data.get(), mapping,
                                pinocchio_model, dynamic_states) && passed;
    passed = validateInverseDynamics(mujoco_model.get(), mujoco_data.get(), mapping,
                                     pinocchio_model, dynamic_states) && passed;
    passed = validatePinocchioRegressorIdentity(pinocchio_model, dynamic_states) &&
             passed;

    std::cout << "\n" << std::string(88, '=') << "\n";
    std::cout << (passed ? "[PASS] reBot-DM Phase 4A dynamics consistency\n"
                        : "[FAIL] reBot-DM Phase 4A dynamics consistency\n");
    std::cout << std::string(88, '=') << "\n";
    return passed ? 0 : 1;
  } catch (const std::exception &exception) {
    std::cerr << "rebot_model_consistency_test failed: " << exception.what() << "\n";
    return 1;
  }
}
