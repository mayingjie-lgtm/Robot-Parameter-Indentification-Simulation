#pragma once

#include "mujoco_regressor.hpp"

#include <filesystem>

namespace mujoco_dynamics {

class MuJoCoPiperRegressor {
public:
  static constexpr std::size_t N_DOF = 6;
  static constexpr std::size_t N_BODIES = 6; // link1-6

  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;
  using Matrix3d = Eigen::Matrix3d;
  using Vector3d = Eigen::Vector3d;
  using Matrix4d = Eigen::Matrix4d;
  using Quaterniond = Eigen::Quaterniond;

  /** Load the repository Piper model with the fixed-open gripper convention. */
  MuJoCoPiperRegressor();

  /**
   * Load Piper kinematics and inertial truth from a compiled MuJoCo model.
   *
   * @param model_path Full Piper MJCF path.
   * @param gripper_position Fixed joint7/joint8 positions used for link6
   * composition.
   * @param frictionloss Optional runtime plant values overriding model zeros.
   */
  explicit MuJoCoPiperRegressor(
      const std::filesystem::path &model_path,
      const std::array<double, 2> &gripper_position = {0.035, -0.035},
      const std::vector<double> &frictionloss = {});

  VectorXd
  computeParameterVector(MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  std::size_t
  numParameters(MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  MatrixXd
  computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                         const VectorXd &qdd,
                         MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  MatrixXd computeObservationMatrix(
      const MatrixXd &Q, const MatrixXd &Qd, const MatrixXd &Qdd,
      MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

  std::vector<std::string>
  getParameterNames(MuJoCoParamFlags flags = MuJoCoParamFlags::ALL) const;

private:
  std::array<MuJoCoBody, N_BODIES + 1> bodies_; // base_link + link1-6
  std::array<double, N_DOF> frictionloss_{};
  Vector3d gravity_{0, 0, -9.81};

  void initBodies(const std::filesystem::path &model_path,
                  const std::array<double, 2> &gripper_position,
                  const std::vector<double> &frictionloss);

  std::vector<Matrix4d> computeBodyTransforms(const VectorXd &q) const;
  MatrixXd computeBodyOriginJacobian(std::size_t body_idx,
                                     const VectorXd &q) const;
  MatrixXd computeBodyOriginJacobianDerivative(std::size_t body_idx,
                                               const VectorXd &q,
                                               const VectorXd &qd) const;
  static Matrix3d skew(const Vector3d &v);
  static Matrix4d poseToTransform(const Vector3d &pos, const Quaterniond &quat);

  MatrixXd computeBodyRegressorBlock(std::size_t body_idx, const VectorXd &q,
                                     const VectorXd &qd,
                                     const VectorXd &qdd) const;
};

} // namespace mujoco_dynamics
