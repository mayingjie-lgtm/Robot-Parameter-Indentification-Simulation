#pragma once

#include "mujoco_regressor.hpp"
#include "rebot_pinocchio_dynamics.hpp"

#include <Eigen/Core>

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace rebot_dynamics {

/**
 * Augmented six-DoF reBot-DM identification regressor.
 *
 * The first 60 columns are Pinocchio's native rigid-body joint torque
 * regressor. Optional actuator-compensation columns are appended without
 * changing Pinocchio's rigid-body parameter ordering.
 */
class ReBotPinocchioRegressor {
public:
  static constexpr std::size_t kArmDof = 6;
  static constexpr std::size_t kRigidParameters = 60;

  /**
   * Build the reduced reBot model and store explicit simulation-truth terms.
   *
   * @param urdf_path Repository-local reBot-DM URDF.
   * @param armature Six reflected-inertia values [kg*m^2].
   * @param damping Six viscous damping values [N*m*s/rad].
   * @param frictionloss Six MuJoCo dry-friction values [N*m].
   */
  ReBotPinocchioRegressor(
      const std::filesystem::path &urdf_path,
      const std::vector<double> &armature = {},
      const std::vector<double> &damping = {},
      const std::vector<double> &frictionloss = {});

  /** Return the parameter count for the requested augmented model. */
  std::size_t numParameters(
      mujoco_dynamics::MuJoCoParamFlags flags =
          mujoco_dynamics::MuJoCoParamFlags::ALL) const;

  /** Return rigid parameters followed by enabled actuator truth parameters. */
  Eigen::VectorXd computeParameterVector(
      mujoco_dynamics::MuJoCoParamFlags flags =
          mujoco_dynamics::MuJoCoParamFlags::ALL) const;

  /** Compute one 6 x N augmented torque regressor. */
  Eigen::MatrixXd computeRegressorMatrix(
      const Eigen::VectorXd &q, const Eigen::VectorXd &qd,
      const Eigen::VectorXd &qdd,
      mujoco_dynamics::MuJoCoParamFlags flags =
          mujoco_dynamics::MuJoCoParamFlags::ALL);

  /** Stack sample regressors in sample-major torque-row order. */
  Eigen::MatrixXd computeObservationMatrix(
      const Eigen::MatrixXd &Q, const Eigen::MatrixXd &Qd,
      const Eigen::MatrixXd &Qdd,
      mujoco_dynamics::MuJoCoParamFlags flags =
          mujoco_dynamics::MuJoCoParamFlags::ALL);

  /** Return stable human-readable names matching the exact column ordering. */
  std::vector<std::string> getParameterNames(
      mujoco_dynamics::MuJoCoParamFlags flags =
          mujoco_dynamics::MuJoCoParamFlags::ALL) const;

private:
  /** Parse an optional six-element truth vector, using zeros when omitted. */
  static std::array<double, kArmDof>
  parseTruthVector(const std::vector<double> &values, const char *name);

  ReBotPinocchioDynamics dynamics_;
  std::array<double, kArmDof> armature_{};
  std::array<double, kArmDof> damping_{};
  std::array<double, kArmDof> frictionloss_{};
};

} // namespace rebot_dynamics
