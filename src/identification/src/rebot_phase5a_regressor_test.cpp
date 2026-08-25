/**
 * @file rebot_phase5a_regressor_test.cpp
 * @brief Validate the reBot-DM augmented Pinocchio regressor against MuJoCo.
 */

#include "identification/identification.hpp"
#include "rebot_pinocchio_regressor.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <mujoco/mujoco.h>

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
constexpr std::size_t kNumericalSamples = 128;
constexpr std::size_t kRankSamples = 256;
constexpr unsigned int kRandomSeed = 20260825U;
constexpr double kTorqueTolerance = 1e-10;
constexpr double kConstraintTolerance = 1e-10;
constexpr double kFrictionSpeedThreshold = 0.05;
constexpr double kRankRelativeTolerance = 1e-6;

const std::vector<double> kArmatureTruth{0.020, 0.025, 0.018,
                                         0.010, 0.008, 0.006};
const std::vector<double> kDampingTruth{0.040, 0.035, 0.030,
                                        0.020, 0.015, 0.010};
const std::vector<double> kFrictionTruth{0.080, 0.070, 0.060,
                                         0.040, 0.030, 0.020};

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

/** One legal deterministic six-joint dynamics state. */
struct TestState {
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd qdd;
};

/** MuJoCo indices for joint1 through joint6. */
struct ArmMapping {
  std::array<int, kDof> qpos_indices{};
  std::array<int, kDof> dof_indices{};
};

/** Accumulated numerical errors for one augmented-dynamics gate. */
struct GateMetrics {
  double torque_max_abs{0.0};
  double constraint_max_abs{0.0};
};

/** Column-scaled SVD facts for one raw regressor layout. */
struct RankMetrics {
  Eigen::Index raw_columns{0};
  Eigen::Index rank{0};
  double relative_threshold{0.0};
  double effective_condition{0.0};
  std::vector<Eigen::Index> structural_zero_columns;
  std::vector<Eigen::Index> near_zero_columns;
  Eigen::VectorXd singular_values;
};

/** Resolve the six arm joints by name rather than XML ordering assumptions. */
ArmMapping resolveArmMapping(const mjModel *model) {
  ArmMapping mapping;
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    const std::string name = "joint" + std::to_string(joint + 1);
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, name.c_str());
    if (joint_id < 0) {
      throw std::runtime_error("Missing reBot joint in MuJoCo model: " + name);
    }
    mapping.qpos_indices[joint] = model->jnt_qposadr[joint_id];
    mapping.dof_indices[joint] = model->jnt_dofadr[joint_id];
  }
  return mapping;
}

/** Resolve and set both gripper joints to the frozen 0.05 m configuration. */
void setGripperLock(const mjModel *model, mjData *data) {
  for (int index = 0; index < 2; ++index) {
    const std::string name = "gripper_joint" + std::to_string(index + 1);
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, name.c_str());
    if (joint_id < 0) {
      throw std::runtime_error("Missing reBot gripper joint: " + name);
    }
    data->qpos[model->jnt_qposadr[joint_id]] = 0.05;
    data->qvel[model->jnt_dofadr[joint_id]] = 0.0;
    data->qacc[model->jnt_dofadr[joint_id]] = 0.0;
  }
}

/** Write and read back all three explicit actuator simulation-truth vectors. */
void applySimulationTruth(mjModel *model, const ArmMapping &mapping,
                          const std::vector<double> &armature,
                          const std::vector<double> &damping,
                          const std::vector<double> &frictionloss) {
  const auto value_or_zero = [](const std::vector<double> &values,
                                std::size_t index) {
    return values.empty() ? 0.0 : values[index];
  };
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    const int dof = mapping.dof_indices[joint];
    const double armature_value = value_or_zero(armature, joint);
    const double damping_value = value_or_zero(damping, joint);
    const double friction_value = value_or_zero(frictionloss, joint);
    model->dof_armature[dof] = armature_value;
    model->dof_damping[dof] = damping_value;
    model->dof_frictionloss[dof] = friction_value;
    if (model->dof_armature[dof] != armature_value ||
        model->dof_damping[dof] != damping_value ||
        model->dof_frictionloss[dof] != friction_value) {
      throw std::runtime_error("Failed to read back injected MuJoCo truth");
    }
  }
}

/** Generate legal states away from limits and the dry-friction zero-speed zone. */
std::vector<TestState> makeRandomStates(std::size_t count, unsigned int seed) {
  const std::array<double, kDof> lower{-2.8, -3.14, -3.14,
                                       -1.87, -1.57, -3.14};
  const std::array<double, kDof> upper{2.8, 0.0, 0.0, 1.57, 1.57, 3.14};
  std::mt19937 generator(seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> speed_magnitude(0.10, 0.70);
  std::uniform_int_distribution<int> sign_choice(0, 1);
  std::uniform_real_distribution<double> acceleration(-1.0, 1.0);

  std::vector<TestState> states;
  states.reserve(count);
  for (std::size_t sample = 0; sample < count; ++sample) {
    TestState state{Eigen::VectorXd(kDof), Eigen::VectorXd(kDof),
                    Eigen::VectorXd(kDof)};
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      const double margin = 0.10 * (upper[joint] - lower[joint]);
      const double q_min = lower[joint] + margin;
      const double q_max = upper[joint] - margin;
      state.q(static_cast<Eigen::Index>(joint)) =
          q_min + unit(generator) * (q_max - q_min);
      const double magnitude = speed_magnitude(generator);
      state.qd(static_cast<Eigen::Index>(joint)) =
          sign_choice(generator) == 0 ? -magnitude : magnitude;
      state.qdd(static_cast<Eigen::Index>(joint)) = acceleration(generator);
    }
    states.push_back(std::move(state));
  }
  return states;
}

/** Evaluate MuJoCo inverse dynamics for one prescribed arm state. */
Eigen::VectorXd mujocoInverseTorque(const mjModel *model, mjData *data,
                                    const ArmMapping &mapping,
                                    const TestState &state) {
  mj_resetData(model, data);
  setGripperLock(model, data);
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    data->qpos[mapping.qpos_indices[joint]] =
        state.q(static_cast<Eigen::Index>(joint));
    data->qvel[mapping.dof_indices[joint]] =
        state.qd(static_cast<Eigen::Index>(joint));
    data->qacc[mapping.dof_indices[joint]] =
        state.qdd(static_cast<Eigen::Index>(joint));
  }
  mj_inverse(model, data);

  Eigen::VectorXd torque(kDof);
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    torque(static_cast<Eigen::Index>(joint)) =
        data->qfrc_inverse[mapping.dof_indices[joint]];
  }
  return torque;
}

/** Compare one augmented regressor level against the same MuJoCo truth. */
GateMetrics runNumericalGate(
    const char *label, const std::filesystem::path &urdf_path, mjModel *model,
    mjData *data, const ArmMapping &mapping,
    const std::vector<TestState> &states,
    mujoco_dynamics::MuJoCoParamFlags flags,
    const std::vector<double> &armature,
    const std::vector<double> &damping,
    const std::vector<double> &frictionloss) {
  applySimulationTruth(model, mapping, armature, damping, frictionloss);
  rebot_dynamics::ReBotPinocchioRegressor regressor(
      urdf_path, armature, damping, frictionloss);
  const Eigen::VectorXd theta = regressor.computeParameterVector(flags);

  GateMetrics metrics;
  for (const TestState &state : states) {
    const Eigen::VectorXd tau_regressor =
        regressor.computeRegressorMatrix(state.q, state.qd, state.qdd, flags) *
        theta;
    const Eigen::VectorXd tau_mujoco =
        mujocoInverseTorque(model, data, mapping, state);
    metrics.torque_max_abs =
        std::max(metrics.torque_max_abs,
                 (tau_regressor - tau_mujoco).cwiseAbs().maxCoeff());

    for (std::size_t joint = 0; joint < kDof; ++joint) {
      const double expected_constraint =
          frictionloss.empty()
              ? 0.0
              : -frictionloss[joint] *
                    (state.qd(static_cast<Eigen::Index>(joint)) > 0.0 ? 1.0
                                                                      : -1.0);
      const double actual_constraint =
          data->qfrc_constraint[mapping.dof_indices[joint]];
      metrics.constraint_max_abs =
          std::max(metrics.constraint_max_abs,
                   std::abs(actual_constraint - expected_constraint));
    }
  }

  std::cout << label << " torque_max_abs=" << std::scientific
            << std::setprecision(12) << metrics.torque_max_abs
            << " constraint_semantics_max_abs=" << metrics.constraint_max_abs
            << " samples=" << states.size() << "\n";
  return metrics;
}

/** Compute column-scaled SVD facts without asserting full raw-column rank. */
RankMetrics analyzeRank(rebot_dynamics::ReBotPinocchioRegressor &regressor,
                        const std::vector<TestState> &states,
                        mujoco_dynamics::MuJoCoParamFlags flags,
                        const char *label) {
  Eigen::MatrixXd q(kDof, static_cast<Eigen::Index>(states.size()));
  Eigen::MatrixXd qd(kDof, static_cast<Eigen::Index>(states.size()));
  Eigen::MatrixXd qdd(kDof, static_cast<Eigen::Index>(states.size()));
  for (std::size_t sample = 0; sample < states.size(); ++sample) {
    q.col(static_cast<Eigen::Index>(sample)) = states[sample].q;
    qd.col(static_cast<Eigen::Index>(sample)) = states[sample].qd;
    qdd.col(static_cast<Eigen::Index>(sample)) = states[sample].qdd;
  }

  const Eigen::MatrixXd observation =
      regressor.computeObservationMatrix(q, qd, qdd, flags);
  Eigen::MatrixXd scaled = observation;
  Eigen::VectorXd column_norms(observation.cols());
  double max_column_norm = 0.0;
  for (Eigen::Index column = 0; column < observation.cols(); ++column) {
    const double norm = observation.col(column).norm();
    column_norms(column) = norm;
    max_column_norm = std::max(max_column_norm, norm);
  }

  RankMetrics metrics;
  metrics.raw_columns = observation.cols();
  const double structural_zero_tolerance = 1e-14;
  const double near_zero_tolerance = 1e-10 * max_column_norm;
  for (Eigen::Index column = 0; column < observation.cols(); ++column) {
    if (column_norms(column) <= structural_zero_tolerance) {
      metrics.structural_zero_columns.push_back(column);
      continue;
    }
    if (column_norms(column) <= near_zero_tolerance) {
      metrics.near_zero_columns.push_back(column);
    }
    scaled.col(column) /= column_norms(column);
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(scaled);
  metrics.singular_values = svd.singularValues();
  const double sigma_max = metrics.singular_values(0);
  metrics.relative_threshold = kRankRelativeTolerance * sigma_max;
  metrics.rank =
      (metrics.singular_values.array() > metrics.relative_threshold).count();
  metrics.effective_condition =
      metrics.rank > 0
          ? sigma_max /
                metrics.singular_values(static_cast<Eigen::Index>(metrics.rank - 1))
          : std::numeric_limits<double>::infinity();

  const auto names = regressor.getParameterNames(flags);
  std::cout << label << " raw_columns=" << metrics.raw_columns
            << " numerical_rank=" << metrics.rank
            << " relative_rank_threshold=" << kRankRelativeTolerance
            << " effective_condition=" << metrics.effective_condition << "\n";
  std::cout << label << " structural_zero_columns=";
  if (metrics.structural_zero_columns.empty()) {
    std::cout << "none";
  } else {
    for (Eigen::Index column : metrics.structural_zero_columns) {
      std::cout << column << "(" << names[static_cast<std::size_t>(column)]
                << ") ";
    }
  }
  std::cout << "\n";
  std::cout << label << " near_zero_columns=";
  if (metrics.near_zero_columns.empty()) {
    std::cout << "none";
  } else {
    for (Eigen::Index column : metrics.near_zero_columns) {
      std::cout << column << "(" << names[static_cast<std::size_t>(column)]
                << ") ";
    }
  }
  std::cout << "\n";
  std::cout << label << " near_zero_singular_directions="
            << (metrics.raw_columns - metrics.rank) << "\n";
  std::cout << label << " singular_values=[";
  for (Eigen::Index index = 0; index < metrics.singular_values.size(); ++index) {
    std::cout << std::scientific << std::setprecision(8)
              << metrics.singular_values(index);
    if (index + 1 < metrics.singular_values.size()) {
      std::cout << ", ";
    }
  }
  std::cout << "]\n";
  return metrics;
}

/** Verify the fixed 60/72/78 parameter ordering contract. */
bool validateParameterLayout(
    rebot_dynamics::ReBotPinocchioRegressor &regressor) {
  const auto rigid = regressor.getParameterNames(
      mujoco_dynamics::MuJoCoParamFlags::NONE);
  const auto augmented = regressor.getParameterNames(
      mujoco_dynamics::MuJoCoParamFlags::ALL);
  const auto friction = regressor.getParameterNames(
      mujoco_dynamics::MuJoCoParamFlags::ALL_WITH_FRICTION);
  const bool passed =
      rigid.size() == 60 && augmented.size() == 72 && friction.size() == 78 &&
      augmented[60] == "armature_1" && augmented[65] == "armature_6" &&
      augmented[66] == "damping_1" && augmented[71] == "damping_6" &&
      friction[72] == "frictionloss_1" && friction[77] == "frictionloss_6";
  std::cout << "parameter_layout rigid=" << rigid.size()
            << " augmented=" << augmented.size()
            << " with_friction=" << friction.size()
            << " passed=" << std::boolalpha << passed << "\n";
  return passed;
}

/** Verify Identification dispatches reBot calls to the Pinocchio regressor. */
bool validateIdentificationBranch(const std::filesystem::path &urdf_path,
                                  const std::vector<TestState> &states) {
  Identification identifier("rebot_dm", urdf_path, kFrictionTruth,
                            kArmatureTruth, kDampingTruth);
  rebot_dynamics::ReBotPinocchioRegressor direct(
      urdf_path, kArmatureTruth, kDampingTruth, kFrictionTruth);
  const auto flags =
      mujoco_dynamics::MuJoCoParamFlags::ALL_WITH_FRICTION;
  if (identifier.numParameters(mujoco_dynamics::MuJoCoParamFlags::NONE) != 60 ||
      identifier.numParameters(mujoco_dynamics::MuJoCoParamFlags::ALL) != 72 ||
      identifier.numParameters(flags) != 78) {
    std::cout << "identification_dispatch parameter_count_mismatch\n";
    return false;
  }

  const Eigen::Index samples =
      std::min<Eigen::Index>(8, static_cast<Eigen::Index>(states.size()));
  Eigen::MatrixXd q(kDof, samples);
  Eigen::MatrixXd qd(kDof, samples);
  Eigen::MatrixXd qdd(kDof, samples);
  for (Eigen::Index sample = 0; sample < samples; ++sample) {
    q.col(sample) = states[static_cast<std::size_t>(sample)].q;
    qd.col(sample) = states[static_cast<std::size_t>(sample)].qd;
    qdd.col(sample) = states[static_cast<std::size_t>(sample)].qdd;
  }
  const double theta_error =
      (identifier.getGroundTruthParameters(flags) -
       direct.computeParameterVector(flags))
          .cwiseAbs()
          .maxCoeff();
  const double observation_error =
      (identifier.computeObservationMatrix(q, qd, qdd, flags) -
       direct.computeObservationMatrix(q, qd, qdd, flags))
          .cwiseAbs()
          .maxCoeff();
  const bool passed = theta_error <= 1e-15 && observation_error <= 1e-15;
  std::cout << "identification_dispatch theta_max_abs=" << std::scientific
            << theta_error << " observation_max_abs=" << observation_error
            << " passed=" << std::boolalpha << passed << "\n";
  return passed;
}

} // namespace

int main() {
  try {
    const std::filesystem::path root(PROJECT_ROOT_DIR);
    const std::filesystem::path urdf_path = root / "rebot_dm" / "rebot_dm.urdf";
    const std::filesystem::path mjcf_path = root / "rebot_dm" / "rebot_dm.xml";
    char error[1024]{};
    ModelPtr model(
        mj_loadXML(mjcf_path.string().c_str(), nullptr, error, sizeof(error)),
        mj_deleteModel);
    if (!model) {
      throw std::runtime_error("Failed to load reBot MJCF: " +
                               std::string(error));
    }
    DataPtr data(mj_makeData(model.get()), mj_deleteData);
    if (!data) {
      throw std::runtime_error("Failed to allocate reBot MuJoCo data");
    }
    const ArmMapping mapping = resolveArmMapping(model.get());

    const std::vector<TestState> numerical_states =
        makeRandomStates(kNumericalSamples, kRandomSeed);
    const std::vector<TestState> rank_states =
        makeRandomStates(kRankSamples, kRandomSeed + 1U);

    std::cout << std::string(88, '=') << "\n";
    std::cout << "reBot-DM Phase 5A augmented regressor / simulation truth gate\n";
    std::cout << "seed=" << kRandomSeed
              << " friction_speed_threshold=" << kFrictionSpeedThreshold
              << "\n";
    std::cout << "simulation truth armature=[0.020,0.025,0.018,0.010,0.008,0.006]\n";
    std::cout << "simulation truth damping=[0.040,0.035,0.030,0.020,0.015,0.010]\n";
    std::cout << "simulation truth frictionloss=[0.080,0.070,0.060,0.040,0.030,0.020]\n";

    rebot_dynamics::ReBotPinocchioRegressor layout_regressor(
        urdf_path, kArmatureTruth, kDampingTruth, kFrictionTruth);
    bool passed = validateParameterLayout(layout_regressor);
    passed = validateIdentificationBranch(urdf_path, numerical_states) && passed;

    const GateMetrics gate_a = runNumericalGate(
        "Gate A rigid-only", urdf_path, model.get(), data.get(), mapping,
        numerical_states, mujoco_dynamics::MuJoCoParamFlags::NONE, {}, {}, {});
    const GateMetrics gate_b = runNumericalGate(
        "Gate B rigid+armature", urdf_path, model.get(), data.get(), mapping,
        numerical_states, mujoco_dynamics::MuJoCoParamFlags::ARMATURE,
        kArmatureTruth, {}, {});
    const GateMetrics gate_c = runNumericalGate(
        "Gate C rigid+armature+damping", urdf_path, model.get(), data.get(),
        mapping, numerical_states, mujoco_dynamics::MuJoCoParamFlags::ALL,
        kArmatureTruth, kDampingTruth, {});
    const GateMetrics gate_d = runNumericalGate(
        "Gate D +frictionloss", urdf_path, model.get(), data.get(), mapping,
        numerical_states,
        mujoco_dynamics::MuJoCoParamFlags::ALL_WITH_FRICTION, kArmatureTruth,
        kDampingTruth, kFrictionTruth);

    for (const GateMetrics &metrics : {gate_a, gate_b, gate_c, gate_d}) {
      passed = metrics.torque_max_abs <= kTorqueTolerance &&
               metrics.constraint_max_abs <= kConstraintTolerance && passed;
    }

    rebot_dynamics::ReBotPinocchioRegressor rank_regressor(
        urdf_path, kArmatureTruth, kDampingTruth, kFrictionTruth);
    const RankMetrics rank_60 =
        analyzeRank(rank_regressor, rank_states,
                    mujoco_dynamics::MuJoCoParamFlags::NONE, "Rank 60");
    const RankMetrics rank_72 =
        analyzeRank(rank_regressor, rank_states,
                    mujoco_dynamics::MuJoCoParamFlags::ALL, "Rank 72");
    const RankMetrics rank_78 = analyzeRank(
        rank_regressor, rank_states,
        mujoco_dynamics::MuJoCoParamFlags::ALL_WITH_FRICTION, "Rank 78");
    passed = rank_60.rank > 0 && rank_72.rank > 0 && rank_78.rank > 0 && passed;

    std::cout << std::string(88, '=') << "\n";
    std::cout << (passed ? "[PASS]" : "[FAIL]")
              << " reBot-DM Phase 5A augmented regressor gate\n";
    std::cout << std::string(88, '=') << "\n";
    return passed ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << "reBot Phase 5A test failed: " << error.what() << "\n";
    return 1;
  }
}
