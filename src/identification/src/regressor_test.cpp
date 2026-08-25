/**
 * @file regressor_test.cpp
 * @brief Validate the Piper torque regressor without external experiment data.
 */

#include "mujoco_piper_regressor.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::size_t kDof = mujoco_dynamics::MuJoCoPiperRegressor::N_DOF;
constexpr double kTorqueTolerance = 1e-6;
constexpr double kAccelerationTolerance = 1e-8;
constexpr double kRankRelativeTolerance = 1e-6;

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

/** Describes one deterministic regressor evaluation state. */
struct TestState {
  std::string name;
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd qdd;
};

/** Stores MuJoCo indices for the six Piper arm joints. */
struct PiperMapping {
  std::array<int, kDof> qpos_indices{};
  std::array<int, kDof> dof_indices{};
  std::array<int, kDof> actuator_indices{};
};

/** Stores inverse torque and its forward-simulation closure result. */
struct MujocoReference {
  Eigen::VectorXd tau_inverse;
  Eigen::VectorXd tau_simulation;
  Eigen::VectorXd qdd_simulation;
};

/**
 * Verify the dedicated scene holds the complete gripper at its open pose.
 *
 * @param scene_path Identification scene containing the gripper lock.
 * @return True when finger positions, arm constraint forces, and contacts pass.
 */
bool validateFixedGripperScene(const std::filesystem::path &scene_path) {
  char error[1024]{};
  ModelPtr model(
      mj_loadXML(scene_path.string().c_str(), nullptr, error, sizeof(error)),
      mj_deleteModel);
  if (!model) {
    throw std::runtime_error("Failed to load identification scene: " +
                             std::string(error));
  }
  DataPtr data(mj_makeData(model.get()), mj_deleteData);
  if (!data) {
    throw std::runtime_error("Failed to allocate identification scene data");
  }
  const int home_id = mj_name2id(model.get(), mjOBJ_KEY, "home");
  if (home_id < 0) {
    throw std::runtime_error("Identification scene is missing the home key");
  }
  mj_resetDataKeyframe(model.get(), data.get(), home_id);

  const int joint7 = mj_name2id(model.get(), mjOBJ_JOINT, "joint7");
  const int joint8 = mj_name2id(model.get(), mjOBJ_JOINT, "joint8");
  if (joint7 < 0 || joint8 < 0) {
    throw std::runtime_error("Identification scene is missing gripper joints");
  }
  const std::array<double, kDof> arm_test_position{0.2, 0.8, -0.9,
                                                   0.3, -0.25, 0.4};
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    const std::string name = "joint" + std::to_string(joint + 1);
    const int joint_id = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
    data->qpos[model->jnt_qposadr[joint_id]] = arm_test_position[joint];
  }
  mj_forward(model.get(), data.get());
  double max_finger_error = 0.0;
  double max_arm_constraint = 0.0;
  int max_contacts = 0;
  // A short free-flight window isolates gripper equalities before the
  // uncontrolled arm can reach its own joint limits.
  for (int step = 0; step < 10; ++step) {
    mj_step(model.get(), data.get());
    max_finger_error = std::max(
        max_finger_error,
        std::abs(data->qpos[model->jnt_qposadr[joint7]] - 0.035));
    max_finger_error = std::max(
        max_finger_error,
        std::abs(data->qpos[model->jnt_qposadr[joint8]] + 0.035));
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      const std::string name = "joint" + std::to_string(joint + 1);
      const int joint_id = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
      const int dof_id = model->jnt_dofadr[joint_id];
      max_arm_constraint =
          std::max(max_arm_constraint, std::abs(data->qfrc_constraint[dof_id]));
    }
    max_contacts = std::max(max_contacts, data->ncon);
  }
  std::cout << "Fixed gripper max position error: " << max_finger_error
            << " m\n";
  std::cout << "Arm max constraint effort: " << max_arm_constraint
            << " Nm\n";
  std::cout << "Maximum contact count: " << max_contacts << "\n";
  // Equality constraints are intentionally compliant; sub-micrometre motion
  // is the stable fixed-gripper criterion at the 1 ms simulation step.
  return max_finger_error <= 1e-6 && max_arm_constraint <= 1e-10 &&
         max_contacts == 0;
}

/**
 * Print a named vector with enough precision for numerical comparisons.
 *
 * @param name Display label.
 * @param values Vector values.
 */
void printVector(const std::string &name, const Eigen::VectorXd &values) {
  std::cout << "  " << std::setw(30) << std::left << name << ": [";
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    std::cout << std::setw(14) << std::scientific << std::setprecision(6)
              << values(i);
    if (i + 1 < values.size()) {
      std::cout << ", ";
    }
  }
  std::cout << "]\n";
}

/**
 * Resolve the Piper arm mapping by object name instead of assuming XML order.
 *
 * @param model Loaded MuJoCo Piper model.
 * @return Joint-position, DOF, and actuator indices for joints 1 through 6.
 */
PiperMapping resolvePiperMapping(const mjModel *model) {
  PiperMapping mapping;
  for (std::size_t i = 0; i < kDof; ++i) {
    const std::string joint_name = "joint" + std::to_string(i + 1);
    const std::string actuator_name = "actuator" + std::to_string(i + 1);
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name.c_str());
    const int actuator_id =
        mj_name2id(model, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (joint_id < 0 || actuator_id < 0) {
      throw std::runtime_error("Piper joint/actuator mapping is incomplete");
    }
    mapping.qpos_indices[i] = model->jnt_qposadr[joint_id];
    mapping.dof_indices[i] = model->jnt_dofadr[joint_id];
    mapping.actuator_indices[i] = actuator_id;
  }
  return mapping;
}

/**
 * Assign the six arm states while preserving the gripper home configuration.
 *
 * @param model Loaded MuJoCo model.
 * @param data Mutable MuJoCo data.
 * @param mapping Piper arm index mapping.
 * @param q Joint positions [rad].
 * @param qd Joint velocities [rad/s].
 * @param qdd Joint accelerations [rad/s^2].
 */
void setArmState(const mjModel *model, mjData *data,
                 const PiperMapping &mapping, const Eigen::VectorXd &q,
                 const Eigen::VectorXd &qd, const Eigen::VectorXd &qdd) {
  mju_zero(data->qvel, model->nv);
  mju_zero(data->qacc, model->nv);
  mju_zero(data->ctrl, model->nu);
  mju_zero(data->qfrc_applied, model->nv);
  mju_zero(data->xfrc_applied, 6 * model->nbody);
  for (std::size_t i = 0; i < kDof; ++i) {
    data->qpos[mapping.qpos_indices[i]] = q(static_cast<Eigen::Index>(i));
    data->qvel[mapping.dof_indices[i]] = qd(static_cast<Eigen::Index>(i));
    data->qacc[mapping.dof_indices[i]] = qdd(static_cast<Eigen::Index>(i));
  }
}

/**
 * Compute MuJoCo inverse torque and verify it through forward dynamics.
 *
 * Constraints are disabled so joint-limit/contact forces do not contaminate
 * the unconstrained rigid-body identity. Non-arm generalized forces are
 * applied directly so the prescribed gripper acceleration remains consistent
 * while arm forces still pass through the actual motor actuators.
 *
 * @param model Loaded MuJoCo Piper model.
 * @param data Reusable MuJoCo data.
 * @param mapping Piper arm index mapping.
 * @param state Desired arm state and acceleration.
 * @return Inverse torque, actuator effort, and simulated acceleration.
 */
MujocoReference computeMujocoReference(const mjModel *model, mjData *data,
                                       const PiperMapping &mapping,
                                       const TestState &state) {
  const int home_id = mj_name2id(model, mjOBJ_KEY, "home");
  if (home_id >= 0) {
    mj_resetDataKeyframe(model, data, home_id);
  } else {
    mj_resetData(model, data);
  }
  setArmState(model, data, mapping, state.q, state.qd, state.qdd);
  mj_inverse(model, data);

  Eigen::VectorXd inverse_all(model->nv);
  for (int i = 0; i < model->nv; ++i) {
    inverse_all(i) = data->qfrc_inverse[i];
  }

  MujocoReference reference;
  reference.tau_inverse.resize(static_cast<Eigen::Index>(kDof));
  for (std::size_t i = 0; i < kDof; ++i) {
    reference.tau_inverse(static_cast<Eigen::Index>(i)) =
        inverse_all(mapping.dof_indices[i]);
  }

  if (home_id >= 0) {
    mj_resetDataKeyframe(model, data, home_id);
  } else {
    mj_resetData(model, data);
  }
  setArmState(model, data, mapping, state.q, state.qd,
              Eigen::VectorXd::Zero(static_cast<Eigen::Index>(kDof)));

  std::vector<bool> arm_dof(static_cast<std::size_t>(model->nv), false);
  for (std::size_t i = 0; i < kDof; ++i) {
    const int actuator_id = mapping.actuator_indices[i];
    const double torque = reference.tau_inverse(static_cast<Eigen::Index>(i));
    if (model->actuator_ctrllimited[actuator_id]) {
      const double lower = model->actuator_ctrlrange[2 * actuator_id];
      const double upper = model->actuator_ctrlrange[2 * actuator_id + 1];
      if (torque < lower || torque > upper) {
        throw std::runtime_error("Inverse torque exceeds Piper actuator range");
      }
    }
    data->ctrl[actuator_id] = torque;
    arm_dof[static_cast<std::size_t>(mapping.dof_indices[i])] = true;
  }
  for (int i = 0; i < model->nv; ++i) {
    if (!arm_dof[static_cast<std::size_t>(i)]) {
      data->qfrc_applied[i] = inverse_all(i);
    }
  }
  mj_forward(model, data);

  reference.tau_simulation.resize(static_cast<Eigen::Index>(kDof));
  reference.qdd_simulation.resize(static_cast<Eigen::Index>(kDof));
  for (std::size_t i = 0; i < kDof; ++i) {
    const int dof_index = mapping.dof_indices[i];
    reference.tau_simulation(static_cast<Eigen::Index>(i)) =
        data->qfrc_actuator[dof_index];
    reference.qdd_simulation(static_cast<Eigen::Index>(i)) =
        data->qacc[dof_index];
  }
  return reference;
}

/**
 * Evaluate one state against the MuJoCo engine.
 *
 * @param state Test state.
 * @param regressor Piper regressor under test.
 * @param theta Model parameter vector in regressor ordering.
 * @param model Loaded MuJoCo model.
 * @param data Reusable MuJoCo data.
 * @param mapping Piper arm mapping.
 * @return True only if every requested numerical identity meets tolerance.
 */
bool evaluateState(const TestState &state,
                   const mujoco_dynamics::MuJoCoPiperRegressor &regressor,
                   const Eigen::VectorXd &theta, const mjModel *model,
                   mjData *data, const PiperMapping &mapping,
                   double torque_tolerance, bool verbose) {
  const Eigen::MatrixXd y = regressor.computeRegressorMatrix(
      state.q, state.qd, state.qdd, mujoco_dynamics::MuJoCoParamFlags::ALL);
  const Eigen::VectorXd tau_regressor = y * theta;
  const MujocoReference mujoco =
      computeMujocoReference(model, data, mapping, state);

  const double mujoco_error =
      (tau_regressor - mujoco.tau_inverse).cwiseAbs().maxCoeff();
  const double simulation_torque_error =
      (mujoco.tau_simulation - mujoco.tau_inverse).cwiseAbs().maxCoeff();
  const double simulation_acceleration_error =
      (mujoco.qdd_simulation - state.qdd).cwiseAbs().maxCoeff();

  const bool passed = mujoco_error <= torque_tolerance &&
                      simulation_torque_error <= kTorqueTolerance &&
                      simulation_acceleration_error <= kAccelerationTolerance;
  if (verbose || !passed) {
    std::cout << "\n" << std::string(80, '-') << "\n";
    std::cout << "  " << state.name << "\n";
    std::cout << std::string(80, '-') << "\n";
    printVector("q", state.q);
    printVector("qd", state.qd);
    printVector("qdd", state.qdd);
    printVector("tau_mujoco_inverse", mujoco.tau_inverse);
    printVector("tau_regressor", tau_regressor);
    printVector("tau_simulation", mujoco.tau_simulation);
    std::cout << "  max|regressor - MuJoCo|      = " << mujoco_error << " Nm\n";
    std::cout << "  max|simulation - MuJoCo|     = "
              << simulation_torque_error << " Nm\n";
    std::cout << "  max|qdd_simulation - target| = "
              << simulation_acceleration_error << " rad/s^2\n";
  }
  return passed;
}

/**
 * Generate reproducible legal Piper states away from joint limits.
 *
 * @param count Number of states.
 * @param seed Fixed random seed.
 * @return Random positions, velocities, and accelerations.
 */
std::vector<TestState> makeRandomStates(std::size_t count, unsigned int seed) {
  const std::array<double, kDof> lower{-2.618, 0.0,   -2.967,
                                       -1.745, -1.22, -2.0944};
  const std::array<double, kDof> upper{2.618, 3.14, 0.0, 1.745, 1.22, 2.0944};
  std::mt19937 generator(seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> velocity(-0.8, 0.8);
  std::uniform_real_distribution<double> acceleration(-1.5, 1.5);

  std::vector<TestState> states;
  states.reserve(count);
  for (std::size_t sample = 0; sample < count; ++sample) {
    TestState state{"random_" + std::to_string(sample + 1),
                    Eigen::VectorXd(kDof), Eigen::VectorXd(kDof),
                    Eigen::VectorXd(kDof)};
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      const double margin = 0.1 * (upper[joint] - lower[joint]);
      const double q_lower = lower[joint] + margin;
      const double q_upper = upper[joint] - margin;
      state.q(static_cast<Eigen::Index>(joint)) =
          q_lower + unit(generator) * (q_upper - q_lower);
      state.qd(static_cast<Eigen::Index>(joint)) = velocity(generator);
      state.qdd(static_cast<Eigen::Index>(joint)) = acceleration(generator);
    }
    states.push_back(std::move(state));
  }
  return states;
}

/**
 * Print and verify the 60 inertial, 6 armature, and 6 damping columns.
 *
 * @param regressor Piper regressor providing parameter names.
 * @return True when the expected 72-column layout is present.
 */
bool printParameterLayout(
    const mujoco_dynamics::MuJoCoPiperRegressor &regressor) {
  const auto names =
      regressor.getParameterNames(mujoco_dynamics::MuJoCoParamFlags::ALL);
  if (names.size() != 72) {
    std::cout << "Unexpected parameter count: " << names.size() << "\n";
    return false;
  }
  std::cout << "\nParameter layout (" << names.size() << " columns):\n";
  std::cout << "  [0, 59]  six body-local inertial blocks\n";
  std::cout
      << "           each [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]\n";
  std::cout << "  [60, 65] " << names[60] << " ... " << names[65] << "\n";
  std::cout << "  [66, 71] " << names[66] << " ... " << names[71] << "\n";
  return names[60] == "armature_1" && names[65] == "armature_6" &&
         names[66] == "damping_1" && names[71] == "damping_6";
}

/**
 * Stack state regressors and report rank across relative thresholds.
 *
 * @param states States used as columns of Q, Qd, and Qdd.
 * @param regressor Piper regressor under test.
 */
Eigen::Index analyzeObservationMatrix(
    const std::vector<TestState> &states,
    const mujoco_dynamics::MuJoCoPiperRegressor &regressor) {
  Eigen::MatrixXd q(kDof, static_cast<Eigen::Index>(states.size()));
  Eigen::MatrixXd qd(kDof, static_cast<Eigen::Index>(states.size()));
  Eigen::MatrixXd qdd(kDof, static_cast<Eigen::Index>(states.size()));
  for (std::size_t i = 0; i < states.size(); ++i) {
    q.col(static_cast<Eigen::Index>(i)) = states[i].q;
    qd.col(static_cast<Eigen::Index>(i)) = states[i].qd;
    qdd.col(static_cast<Eigen::Index>(i)) = states[i].qdd;
  }

  const Eigen::MatrixXd w = regressor.computeObservationMatrix(
      q, qd, qdd, mujoco_dynamics::MuJoCoParamFlags::ALL);
  Eigen::MatrixXd scaled_w = w;
  Eigen::Index zero_columns = 0;
  for (Eigen::Index column = 0; column < scaled_w.cols(); ++column) {
    const double norm = scaled_w.col(column).norm();
    if (norm <= 0.0) {
      ++zero_columns;
      continue;
    }
    scaled_w.col(column) /= norm;
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(scaled_w);
  const Eigen::VectorXd singular_values = svd.singularValues();
  const double sigma_max = singular_values(0);
  const double threshold = kRankRelativeTolerance * sigma_max;
  const Eigen::Index rank = (singular_values.array() > threshold).count();

  std::cout << "\n" << std::string(80, '-') << "\n";
  std::cout << "  Observation matrix diagnostics\n";
  std::cout << std::string(80, '-') << "\n";
  std::cout << "  W shape: " << w.rows() << " x " << w.cols() << "\n";
  std::cout << "  structurally zero columns: " << zero_columns << "\n";
  for (const double relative_tolerance : {1e-12, 1e-10, 1e-8, 1e-6}) {
    const double relative_threshold = relative_tolerance * sigma_max;
    const Eigen::Index threshold_rank =
        (singular_values.array() > relative_threshold).count();
    std::cout << "  rank at relative tolerance " << relative_tolerance << ": "
              << threshold_rank << " / " << w.cols() << "\n";
  }
  std::cout << "  effective condition number: "
            << sigma_max / singular_values(rank - 1) << "\n";
  printVector("singular values", singular_values);
  return rank;
}

/** Validate explicit MuJoCo frictionloss against the six sign(qdot) columns. */
bool validateFrictionloss(
    const std::filesystem::path &model_path, mjModel *model, mjData *data,
    const PiperMapping &mapping, std::vector<TestState> states) {
  const std::vector<double> friction{0.20, 0.20, 0.15, 0.10, 0.08, 0.05};
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    model->dof_frictionloss[mapping.dof_indices[joint]] = friction[joint];
  }
  model->opt.disableflags &= ~mjDSBL_CONSTRAINT;
  mujoco_dynamics::MuJoCoPiperRegressor regressor(
      model_path, {0.035, -0.035}, friction);
  const auto flags =
      mujoco_dynamics::MuJoCoParamFlags::ALL_WITH_FRICTION;
  const Eigen::VectorXd theta = regressor.computeParameterVector(flags);
  double max_torque_error = 0.0;
  double max_acceleration_error = 0.0;
  for (auto &state : states) {
    state.q << 0.2, 0.8, -0.9, 0.3, -0.25, 0.4;
    state.qdd *= 0.2;
    for (Eigen::Index joint = 0; joint < state.qd.size(); ++joint) {
      if (std::abs(state.qd(joint)) < 0.05) {
        state.qd(joint) = state.qd(joint) < 0.0 ? -0.1 : 0.1;
      }
    }
    const Eigen::VectorXd tau_regressor =
        regressor.computeRegressorMatrix(state.q, state.qd, state.qdd, flags) *
        theta;
    const MujocoReference reference =
        computeMujocoReference(model, data, mapping, state);
    max_torque_error = std::max(
        max_torque_error,
        (tau_regressor - reference.tau_inverse).cwiseAbs().maxCoeff());
    max_acceleration_error = std::max(
        max_acceleration_error,
        (reference.qdd_simulation - state.qdd).cwiseAbs().maxCoeff());
  }
  std::cout << "Frictionloss max torque error: " << max_torque_error
            << " Nm\n";
  std::cout << "Frictionloss forward closure error: "
            << max_acceleration_error << " rad/s^2\n";
  return max_torque_error <= 1e-5 &&
         max_acceleration_error <= kAccelerationTolerance;
}

} // namespace

int main() {
  try {
    char error[1024]{};
    const std::filesystem::path model_path =
        std::filesystem::path(PROJECT_ROOT_DIR) / "piper" / "piper.xml";
    mujoco_dynamics::MuJoCoPiperRegressor regressor(model_path);
    const Eigen::VectorXd theta = regressor.computeParameterVector(
        mujoco_dynamics::MuJoCoParamFlags::ALL);
    ModelPtr model(
        mj_loadXML(model_path.string().c_str(), nullptr, error, sizeof(error)),
        mj_deleteModel);
    if (!model) {
      throw std::runtime_error("Failed to load Piper model: " +
                               std::string(error));
    }
    DataPtr data(mj_makeData(model.get()), mj_deleteData);
    if (!data) {
      throw std::runtime_error("Failed to allocate MuJoCo data");
    }
    model->opt.disableflags |= mjDSBL_CONSTRAINT;
    const PiperMapping mapping = resolvePiperMapping(model.get());

    std::cout << std::string(80, '=') << "\n";
    std::cout << "  Piper regressor consistency test\n";
    std::cout << std::string(80, '=') << "\n";
    bool passed = printParameterLayout(regressor);
    passed = validateFixedGripperScene(
                 std::filesystem::path(PROJECT_ROOT_DIR) / "piper" /
                 "scene_identification.xml") &&
             passed;

    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(kDof);
    Eigen::VectorXd q_motion(kDof);
    q_motion << 0.2, 0.8, -0.9, 0.3, -0.25, 0.4;
    Eigen::VectorXd qd_motion(kDof);
    qd_motion << 0.4, -0.3, 0.25, -0.2, 0.15, -0.1;
    Eigen::VectorXd qdd_motion(kDof);
    qdd_motion << 0.5, -0.4, 0.3, -0.2, 0.15, -0.1;

    const std::vector<TestState> focused_states{
        {"gravity_only", zero, zero, zero},
        {"inertia_and_armature", q_motion, zero, qdd_motion},
        {"coriolis_and_damping", q_motion, qd_motion, zero},
    };
    for (std::size_t index = 0; index < focused_states.size(); ++index) {
      const double tolerance = index < 2 ? 1e-9 : kTorqueTolerance;
      passed = evaluateState(focused_states[index], regressor, theta,
                             model.get(), data.get(), mapping, tolerance,
                             true) && passed;
    }

    const auto random_validation_states = makeRandomStates(100, 20260824U);
    for (const auto &state : random_validation_states) {
      passed = evaluateState(state, regressor, theta, model.get(), data.get(),
                             mapping, kTorqueTolerance, false) && passed;
    }
    std::cout << "\nValidated " << random_validation_states.size()
              << " deterministic random states.\n";

    Eigen::Index stable_rank = -1;
    for (const auto sample_count : {256U, 512U, 1024U}) {
      const auto rank_states = makeRandomStates(sample_count,
                                                20260825U + sample_count);
      const Eigen::Index rank = analyzeObservationMatrix(rank_states, regressor);
      if (stable_rank < 0) {
        stable_rank = rank;
      } else if (rank != stable_rank) {
        std::cout << "Rank changed from " << stable_rank << " to " << rank
                  << " when increasing the state count.\n";
        passed = false;
      }
    }
    passed = validateFrictionloss(model_path, model.get(), data.get(), mapping,
                                  makeRandomStates(100, 20260826U)) &&
             passed;

    std::cout << "\n" << std::string(80, '=') << "\n";
    std::cout
        << (passed ? "  [PASS] All Piper torque identities agree.\n"
                   : "  [FAIL] At least one Piper torque identity differs.\n");
    std::cout << std::string(80, '=') << "\n";
    return passed ? 0 : 1;
  } catch (const std::exception &exception) {
    std::cerr << "regressor_test failed: " << exception.what() << "\n";
    return 1;
  }
}
