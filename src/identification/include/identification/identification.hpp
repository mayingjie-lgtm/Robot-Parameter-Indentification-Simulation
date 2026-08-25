#ifndef IDENTIFICATION_IDENTIFICATION_HPP_
#define IDENTIFICATION_IDENTIFICATION_HPP_

#include "identification/algorithms.hpp"
#include "identification/data_loader.hpp"
#include "mujoco_regressor.hpp"
#include "mujoco_piper_regressor.hpp"
#include "rebot_pinocchio_regressor.hpp"
#include "robot/robot_model.hpp"
#include <filesystem>
#include <memory>
#include <string>

/**
 * @brief 参数辨识类（按机械臂切换已验证的动力学回归器）
 *
 * Panda/Piper 使用既有 MuJoCo 回归器；reBot-DM 使用 Pinocchio 刚体
 * regressor 加显式 actuator compensation 列，保持各自已验证的参数顺序。
 */
class Identification {
public:
  /**
   * @brief 构造函数
   *
   * @param robot_type 机械臂类型，支持 "panda" / "piper" / "rebot_dm"
   * @param model_path Robot model source; Piper expects MJCF, reBot expects URDF.
   * @param frictionloss Optional six-joint dry-friction simulation truth.
   * @param armature Optional six-joint reflected-inertia simulation truth.
   * @param damping Optional six-joint viscous-damping simulation truth.
   * @param model 保留用于兼容性的机器人模型指针
   */
  explicit Identification(const std::string &robot_type = "panda",
                          const std::filesystem::path &model_path = {},
                          const std::vector<double> &frictionloss = {},
                          const std::vector<double> &armature = {},
                          const std::vector<double> &damping = {},
                          std::unique_ptr<robot::RobotModel> model = nullptr);

  /**
   * @brief Process data: calculate qdd and filter
   */
  void preprocess(ExperimentData &data);

  /**
   * @brief Solve identification problem using specified algorithm
   *
   * @param data Experiment data
   * @param algorithm_type "OLS", "WLS", "IRLS", "TLS", "EKF", "ML", "CLOE"
   * @param flags 动力学参数标志 (ARMATURE, DAMPING, FRICTION_LOSS)
   * @return Identified base-parameter vector for the selected regressor.
   */
  Eigen::VectorXd solve(const ExperimentData &data,
                        const std::string &algorithm_type = "OLS",
                        mujoco_dynamics::MuJoCoParamFlags flags =
                            mujoco_dynamics::MuJoCoParamFlags::ALL);

  /**
   * @brief 获取参数数量
   */
  std::size_t numParameters(mujoco_dynamics::MuJoCoParamFlags flags =
                                mujoco_dynamics::MuJoCoParamFlags::ALL) const;

  /**
   * @brief 获取 Ground Truth 参数向量 (用于验证)
   */
  Eigen::VectorXd
  getGroundTruthParameters(mujoco_dynamics::MuJoCoParamFlags flags =
                               mujoco_dynamics::MuJoCoParamFlags::ALL) const;

  /**
   * @brief 计算观测矩阵 W
   */
  Eigen::MatrixXd
  computeObservationMatrix(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &Qd,
                           const Eigen::MatrixXd &Qdd,
                           mujoco_dynamics::MuJoCoParamFlags flags =
                               mujoco_dynamics::MuJoCoParamFlags::ALL) const;

  /**
   * @brief 使用指定算法和参数预测整段实验的堆叠力矩向量
   */
  Eigen::VectorXd
  predictTorques(const ExperimentData &data, const Eigen::VectorXd &params,
                 const std::string &algorithm_type = "OLS",
                 mujoco_dynamics::MuJoCoParamFlags flags =
                     mujoco_dynamics::MuJoCoParamFlags::ALL) const;

private:
  std::unique_ptr<robot::RobotModel> model_; // 保留用于兼容性
  std::string robot_type_;
  mujoco_dynamics::MuJoCoRegressor panda_regressor_;
  mujoco_dynamics::MuJoCoPiperRegressor piper_regressor_;
  std::unique_ptr<rebot_dynamics::ReBotPinocchioRegressor> rebot_regressor_;
};

#endif // IDENTIFICATION_IDENTIFICATION_HPP_
