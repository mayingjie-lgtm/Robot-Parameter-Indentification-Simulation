/**
 * @file rebot_pinocchio_regressor.cpp
 * @brief Pinocchio rigid-body regressor with explicit reBot actuator terms.
 */

#include "rebot_pinocchio_regressor.hpp"

#include <cmath>
#include <stdexcept>

namespace rebot_dynamics {

std::array<double, ReBotPinocchioRegressor::kArmDof>
ReBotPinocchioRegressor::parseTruthVector(const std::vector<double> &values,
                                          const char *name) {
  std::array<double, kArmDof> result{};
  if (values.empty()) {
    return result;
  }
  if (values.size() != kArmDof) {
    throw std::invalid_argument(std::string(name) + " must contain six values");
  }
  for (std::size_t index = 0; index < kArmDof; ++index) {
    if (!std::isfinite(values[index]) || values[index] < 0.0) {
      throw std::invalid_argument(std::string(name) +
                                  " must contain finite non-negative values");
    }
    result[index] = values[index];
  }
  return result;
}

ReBotPinocchioRegressor::ReBotPinocchioRegressor(
    const std::filesystem::path &urdf_path,
    const std::vector<double> &armature,
    const std::vector<double> &damping,
    const std::vector<double> &frictionloss)
    : dynamics_(urdf_path, {0.05, 0.05}),
      armature_(parseTruthVector(armature, "armature")),
      damping_(parseTruthVector(damping, "damping")),
      frictionloss_(parseTruthVector(frictionloss, "frictionloss")) {}

std::size_t ReBotPinocchioRegressor::numParameters(
    mujoco_dynamics::MuJoCoParamFlags flags) const {
  std::size_t count = kRigidParameters;
  if (mujoco_dynamics::hasFlag(flags,
                               mujoco_dynamics::MuJoCoParamFlags::ARMATURE)) {
    count += kArmDof;
  }
  if (mujoco_dynamics::hasFlag(flags,
                               mujoco_dynamics::MuJoCoParamFlags::DAMPING)) {
    count += kArmDof;
  }
  if (mujoco_dynamics::hasFlag(
          flags, mujoco_dynamics::MuJoCoParamFlags::FRICTION_LOSS)) {
    count += kArmDof;
  }
  return count;
}

Eigen::VectorXd ReBotPinocchioRegressor::computeParameterVector(
    mujoco_dynamics::MuJoCoParamFlags flags) const {
  Eigen::VectorXd theta = Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(numParameters(flags)));
  theta.head(static_cast<Eigen::Index>(kRigidParameters)) =
      dynamics_.rigidBodyParameterVector();

  Eigen::Index offset = static_cast<Eigen::Index>(kRigidParameters);
  const auto append = [&theta, &offset](const auto &values) {
    for (double value : values) {
      theta(offset++) = value;
    }
  };
  if (mujoco_dynamics::hasFlag(flags,
                               mujoco_dynamics::MuJoCoParamFlags::ARMATURE)) {
    append(armature_);
  }
  if (mujoco_dynamics::hasFlag(flags,
                               mujoco_dynamics::MuJoCoParamFlags::DAMPING)) {
    append(damping_);
  }
  if (mujoco_dynamics::hasFlag(
          flags, mujoco_dynamics::MuJoCoParamFlags::FRICTION_LOSS)) {
    append(frictionloss_);
  }
  return theta;
}

Eigen::MatrixXd ReBotPinocchioRegressor::computeRegressorMatrix(
    const Eigen::VectorXd &q, const Eigen::VectorXd &qd,
    const Eigen::VectorXd &qdd, mujoco_dynamics::MuJoCoParamFlags flags) {
  Eigen::MatrixXd regressor = Eigen::MatrixXd::Zero(
      static_cast<Eigen::Index>(kArmDof),
      static_cast<Eigen::Index>(numParameters(flags)));
  regressor.leftCols(static_cast<Eigen::Index>(kRigidParameters)) =
      dynamics_.torqueRegressor(q, qd, qdd);

  Eigen::Index offset = static_cast<Eigen::Index>(kRigidParameters);
  if (mujoco_dynamics::hasFlag(flags,
                               mujoco_dynamics::MuJoCoParamFlags::ARMATURE)) {
    for (Eigen::Index joint = 0; joint < static_cast<Eigen::Index>(kArmDof);
         ++joint) {
      regressor(joint, offset + joint) = qdd(joint);
    }
    offset += static_cast<Eigen::Index>(kArmDof);
  }
  if (mujoco_dynamics::hasFlag(flags,
                               mujoco_dynamics::MuJoCoParamFlags::DAMPING)) {
    // MuJoCo passive damping is -d*qdot, so actuator compensation is +d*qdot.
    for (Eigen::Index joint = 0; joint < static_cast<Eigen::Index>(kArmDof);
         ++joint) {
      regressor(joint, offset + joint) = qd(joint);
    }
    offset += static_cast<Eigen::Index>(kArmDof);
  }
  if (mujoco_dynamics::hasFlag(
          flags, mujoco_dynamics::MuJoCoParamFlags::FRICTION_LOSS)) {
    // Away from zero speed, dry-friction actuator compensation follows sign(qd).
    for (Eigen::Index joint = 0; joint < static_cast<Eigen::Index>(kArmDof);
         ++joint) {
      regressor(joint, offset + joint) =
          qd(joint) > 0.0 ? 1.0 : (qd(joint) < 0.0 ? -1.0 : 0.0);
    }
  }
  return regressor;
}

Eigen::MatrixXd ReBotPinocchioRegressor::computeObservationMatrix(
    const Eigen::MatrixXd &Q, const Eigen::MatrixXd &Qd,
    const Eigen::MatrixXd &Qdd, mujoco_dynamics::MuJoCoParamFlags flags) {
  if (Q.rows() != static_cast<Eigen::Index>(kArmDof) ||
      Qd.rows() != Q.rows() || Qdd.rows() != Q.rows() ||
      Q.cols() != Qd.cols() || Q.cols() != Qdd.cols()) {
    throw std::invalid_argument(
        "reBot observation matrices must be 6xK with matching sample counts");
  }

  Eigen::MatrixXd observation = Eigen::MatrixXd::Zero(
      static_cast<Eigen::Index>(kArmDof) * Q.cols(),
      static_cast<Eigen::Index>(numParameters(flags)));
  for (Eigen::Index sample = 0; sample < Q.cols(); ++sample) {
    observation.block(sample * static_cast<Eigen::Index>(kArmDof), 0,
                      static_cast<Eigen::Index>(kArmDof), observation.cols()) =
        computeRegressorMatrix(Q.col(sample), Qd.col(sample), Qdd.col(sample),
                               flags);
  }
  return observation;
}

std::vector<std::string> ReBotPinocchioRegressor::getParameterNames(
    mujoco_dynamics::MuJoCoParamFlags flags) const {
  std::vector<std::string> names;
  names.reserve(numParameters(flags));
  const char *parameter_names[] = {"m",   "mx",  "my",  "mz",  "Ixx",
                                   "Ixy", "Ixz", "Iyy", "Iyz", "Izz"};
  for (std::size_t body = 0; body < kArmDof; ++body) {
    for (const char *parameter_name : parameter_names) {
      names.push_back("joint" + std::to_string(body + 1) + "_" +
                      parameter_name);
    }
  }
  const auto append_joint_names = [&names](const char *prefix) {
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      names.push_back(std::string(prefix) + "_" + std::to_string(joint + 1));
    }
  };
  if (mujoco_dynamics::hasFlag(flags,
                               mujoco_dynamics::MuJoCoParamFlags::ARMATURE)) {
    append_joint_names("armature");
  }
  if (mujoco_dynamics::hasFlag(flags,
                               mujoco_dynamics::MuJoCoParamFlags::DAMPING)) {
    append_joint_names("damping");
  }
  if (mujoco_dynamics::hasFlag(
          flags, mujoco_dynamics::MuJoCoParamFlags::FRICTION_LOSS)) {
    append_joint_names("frictionloss");
  }
  return names;
}

} // namespace rebot_dynamics
