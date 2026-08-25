#pragma once

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <Eigen/Core>

#include <array>
#include <filesystem>
#include <memory>

namespace rebot_dynamics {

/**
 * Minimal Pinocchio dynamics wrapper for the six-DoF reBot-DM arm.
 *
 * The full URDF is loaded first, then gripper_joint1 and gripper_joint2 are
 * locked at an explicit configuration so their mass and inertia remain in the
 * reduced six-DoF model. This class intentionally exposes only the operations
 * needed by Phase 4A and later reBot identification.
 */
class ReBotPinocchioDynamics {
public:
  static constexpr std::size_t kArmDof = 6;
  static constexpr std::size_t kGripperDof = 2;

  /**
   * Load the canonical URDF and build the reduced arm model.
   *
   * @param urdf_path Canonical repository-local reBot-DM URDF.
   * @param gripper_lock_position Fixed gripper positions [m].
   */
  ReBotPinocchioDynamics(
      const std::filesystem::path &urdf_path,
      std::array<double, kGripperDof> gripper_lock_position);

  /** Return the full Pinocchio model loaded from the canonical URDF. */
  const pinocchio::Model &fullModel() const { return full_model_; }

  /** Return the six-DoF model after locking both gripper joints. */
  const pinocchio::Model &model() const { return model_; }

  /** Return the reduced-model Pinocchio IDs for joint1 through joint6. */
  const std::array<pinocchio::JointIndex, kArmDof> &armJointIds() const {
    return arm_joint_ids_;
  }

  /** Return the explicit two-joint gripper lock position [m]. */
  const std::array<double, kGripperDof> &gripperLockPosition() const {
    return gripper_lock_position_;
  }

  /** Compute rigid-body inverse dynamics torque for the six arm joints. */
  Eigen::VectorXd inverseDynamics(const Eigen::VectorXd &q,
                                  const Eigen::VectorXd &qd,
                                  const Eigen::VectorXd &qdd);

  /** Compute the symmetric rigid-body joint-space mass matrix. */
  Eigen::MatrixXd massMatrix(const Eigen::VectorXd &q);

  /** Compute Pinocchio's rigid-body joint torque regressor. */
  Eigen::MatrixXd torqueRegressor(const Eigen::VectorXd &q,
                                  const Eigen::VectorXd &qd,
                                  const Eigen::VectorXd &qdd);

  /** Stack each reduced body inertia in Pinocchio dynamic-parameter order. */
  Eigen::VectorXd rigidBodyParameterVector() const;

  /** Return the joint axis expressed in the reduced joint LOCAL frame. */
  Eigen::Vector3d localJointAxis(std::size_t arm_joint_index);

private:
  /** Reject vectors that do not match the reduced six-DoF model. */
  void validateStateVector(const Eigen::VectorXd &vector,
                           const char *name) const;

  pinocchio::Model full_model_;
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::array<pinocchio::JointIndex, kArmDof> arm_joint_ids_{};
  std::array<double, kGripperDof> gripper_lock_position_{};
};

} // namespace rebot_dynamics
