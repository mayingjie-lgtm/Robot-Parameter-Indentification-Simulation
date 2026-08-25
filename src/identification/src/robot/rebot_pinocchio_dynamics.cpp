/**
 * @file rebot_pinocchio_dynamics.cpp
 * @brief Minimal Pinocchio rigid-body dynamics implementation for reBot-DM.
 */

#include "rebot_pinocchio_dynamics.hpp"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/regressor.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace rebot_dynamics {

namespace {

constexpr double kGravity = 9.81;

/** Resolve one required joint and fail with a useful model-contract error. */
pinocchio::JointIndex requireJoint(const pinocchio::Model &model,
                                   const std::string &name) {
  if (!model.existJointName(name)) {
    throw std::runtime_error("Pinocchio model is missing required joint: " + name);
  }
  return model.getJointId(name);
}

} // namespace

ReBotPinocchioDynamics::ReBotPinocchioDynamics(
    const std::filesystem::path &urdf_path,
    std::array<double, kGripperDof> gripper_lock_position)
    : gripper_lock_position_(gripper_lock_position) {
  pinocchio::urdf::buildModel(urdf_path.string(), full_model_);
  full_model_.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -kGravity);
  full_model_.gravity.angular().setZero();

  if (full_model_.nq != 8 || full_model_.nv != 8) {
    throw std::runtime_error("Expected full reBot-DM Pinocchio model nq=8 nv=8, got nq=" +
                             std::to_string(full_model_.nq) + " nv=" +
                             std::to_string(full_model_.nv));
  }

  Eigen::VectorXd reference_configuration = pinocchio::neutral(full_model_);
  std::vector<pinocchio::JointIndex> joints_to_lock;
  joints_to_lock.reserve(kGripperDof);
  for (std::size_t index = 0; index < kGripperDof; ++index) {
    const std::string name = "gripper_joint" + std::to_string(index + 1);
    const pinocchio::JointIndex joint_id = requireJoint(full_model_, name);
    if (full_model_.joints[joint_id].nq() != 1 ||
        full_model_.joints[joint_id].nv() != 1) {
      throw std::runtime_error(name + " must be a one-DoF prismatic joint");
    }
    reference_configuration(full_model_.idx_qs[joint_id]) =
        gripper_lock_position_[index];
    joints_to_lock.push_back(joint_id);
  }

  model_ = pinocchio::buildReducedModel(full_model_, joints_to_lock,
                                        reference_configuration);
  model_.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -kGravity);
  model_.gravity.angular().setZero();
  if (model_.nq != static_cast<int>(kArmDof) ||
      model_.nv != static_cast<int>(kArmDof)) {
    throw std::runtime_error("Reduced reBot-DM Pinocchio model must have nq=6 nv=6");
  }

  for (std::size_t index = 0; index < kArmDof; ++index) {
    const std::string name = "joint" + std::to_string(index + 1);
    arm_joint_ids_[index] = requireJoint(model_, name);
    const pinocchio::JointIndex joint_id = arm_joint_ids_[index];
    if (model_.joints[joint_id].nq() != 1 || model_.joints[joint_id].nv() != 1) {
      throw std::runtime_error(name + " must remain one-DoF after reduction");
    }
  }

  data_ = std::make_unique<pinocchio::Data>(model_);
}

void ReBotPinocchioDynamics::validateStateVector(
    const Eigen::VectorXd &vector, const char *name) const {
  if (vector.size() != model_.nv) {
    throw std::invalid_argument(std::string(name) + " must have six elements");
  }
}

Eigen::VectorXd ReBotPinocchioDynamics::inverseDynamics(
    const Eigen::VectorXd &q, const Eigen::VectorXd &qd,
    const Eigen::VectorXd &qdd) {
  validateStateVector(q, "q");
  validateStateVector(qd, "qd");
  validateStateVector(qdd, "qdd");
  return pinocchio::rnea(model_, *data_, q, qd, qdd);
}

Eigen::MatrixXd ReBotPinocchioDynamics::massMatrix(const Eigen::VectorXd &q) {
  validateStateVector(q, "q");
  pinocchio::crba(model_, *data_, q);
  // CRBA computes one triangle. Mirror it explicitly before cross-engine tests.
  data_->M.template triangularView<Eigen::StrictlyLower>() =
      data_->M.transpose().template triangularView<Eigen::StrictlyLower>();
  return data_->M;
}

Eigen::MatrixXd ReBotPinocchioDynamics::torqueRegressor(
    const Eigen::VectorXd &q, const Eigen::VectorXd &qd,
    const Eigen::VectorXd &qdd) {
  validateStateVector(q, "q");
  validateStateVector(qd, "qd");
  validateStateVector(qdd, "qdd");
  return pinocchio::computeJointTorqueRegressor(model_, *data_, q, qd, qdd);
}

Eigen::VectorXd ReBotPinocchioDynamics::rigidBodyParameterVector() const {
  if (model_.njoints != kArmDof + 1) {
    throw std::runtime_error(
        "Reduced reBot-DM model must contain universe plus six actuated joints");
  }
  Eigen::VectorXd parameters(10 * static_cast<Eigen::Index>(kArmDof));
  for (std::size_t index = 0; index < kArmDof; ++index) {
    const pinocchio::JointIndex joint_id = arm_joint_ids_[index];
    parameters.segment<10>(10 * static_cast<Eigen::Index>(index)) =
        model_.inertias[joint_id].toDynamicParameters();
  }
  return parameters;
}

Eigen::Vector3d
ReBotPinocchioDynamics::localJointAxis(std::size_t arm_joint_index) {
  if (arm_joint_index >= kArmDof) {
    throw std::out_of_range("reBot arm joint index must be in [0, 5]");
  }
  const Eigen::VectorXd q = pinocchio::neutral(model_);
  pinocchio::computeJointJacobians(model_, *data_, q);
  const pinocchio::JointIndex joint_id = arm_joint_ids_[arm_joint_index];
  const Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian =
      pinocchio::getJointJacobian(model_, *data_, joint_id, pinocchio::LOCAL);
  const Eigen::Index velocity_index = model_.idx_vs[joint_id];
  return jacobian.block<3, 1>(3, velocity_index);
}

} // namespace rebot_dynamics
