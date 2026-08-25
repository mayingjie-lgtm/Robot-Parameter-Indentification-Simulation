/**
 * @file mujoco_regressor.cpp
 * @brief 基于 MuJoCo 运动学的回归矩阵计算实现
 */

#include "mujoco_piper_regressor.hpp"

#include <mujoco/mujoco.h>

#include <iostream>
#include <memory>
#include <stdexcept>

namespace mujoco_dynamics {

// ============================================================================
// MuJoCoPiperRegressor 构造函数
// ============================================================================

MuJoCoPiperRegressor::MuJoCoPiperRegressor()
    : MuJoCoPiperRegressor(std::filesystem::path(PROJECT_ROOT_DIR) / "piper" /
                           "piper.xml") {}

MuJoCoPiperRegressor::MuJoCoPiperRegressor(
    const std::filesystem::path &model_path,
    const std::array<double, 2> &gripper_position,
    const std::vector<double> &frictionloss) {
  initBodies(model_path, gripper_position, frictionloss);
}

void MuJoCoPiperRegressor::initBodies(
    const std::filesystem::path &model_path,
    const std::array<double, 2> &gripper_position,
    const std::vector<double> &frictionloss) {
  char error[1024]{};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
      mj_loadXML(model_path.string().c_str(), nullptr, error, sizeof(error)),
      mj_deleteModel);
  if (!model) {
    throw std::runtime_error("Failed to load Piper model parameters: " +
                             std::string(error));
  }
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
      mj_makeData(model.get()), mj_deleteData);
  if (!data) {
    throw std::runtime_error("Failed to allocate Piper model data");
  }
  if (!frictionloss.empty() && frictionloss.size() != N_DOF) {
    throw std::runtime_error("Piper frictionloss must contain six values");
  }

  const auto body_matrix = [](const mjtNum *values) -> Matrix3d {
    return Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
               values)
        .template cast<double>()
        .eval();
  };
  const auto body_inertia = [&body_matrix](const mjModel *loaded,
                                           int body_id) -> Matrix3d {
    mjtNum rotation_values[9];
    mju_quat2Mat(rotation_values, loaded->body_iquat + 4 * body_id);
    const Matrix3d rotation = body_matrix(rotation_values);
    const Vector3d principal(
        loaded->body_inertia[3 * body_id],
        loaded->body_inertia[3 * body_id + 1],
        loaded->body_inertia[3 * body_id + 2]);
    return (rotation * principal.asDiagonal() * rotation.transpose()).eval();
  };
  const auto assign_inertia = [](MuJoCoBody &body, const Matrix3d &inertia) {
    body.Ixx = inertia(0, 0);
    body.Ixy = inertia(0, 1);
    body.Ixz = inertia(0, 2);
    body.Iyy = inertia(1, 1);
    body.Iyz = inertia(1, 2);
    body.Izz = inertia(2, 2);
  };
  const auto require_id = [model_ptr = model.get()](mjtObj type,
                                                     const std::string &name) {
    const int id = mj_name2id(model_ptr, type, name.c_str());
    if (id < 0) {
      throw std::runtime_error("Piper model is missing object: " + name);
    }
    return id;
  };

  const int base_id = require_id(mjOBJ_BODY, "base_link");
  bodies_[0].name = "base_link";
  bodies_[0].pos = Vector3d(model->body_pos[3 * base_id],
                            model->body_pos[3 * base_id + 1],
                            model->body_pos[3 * base_id + 2]);
  bodies_[0].quat = Quaterniond(model->body_quat[4 * base_id],
                                model->body_quat[4 * base_id + 1],
                                model->body_quat[4 * base_id + 2],
                                model->body_quat[4 * base_id + 3])
                          .normalized();
  bodies_[0].has_joint = false;

  for (std::size_t index = 1; index <= N_BODIES; ++index) {
    const std::string body_name = "link" + std::to_string(index);
    const std::string joint_name = "joint" + std::to_string(index);
    const int body_id = require_id(mjOBJ_BODY, body_name);
    const int joint_id = require_id(mjOBJ_JOINT, joint_name);
    const int dof_id = model->jnt_dofadr[joint_id];
    auto &body = bodies_[index];
    body.name = body_name;
    body.pos = Vector3d(model->body_pos[3 * body_id],
                        model->body_pos[3 * body_id + 1],
                        model->body_pos[3 * body_id + 2]);
    body.quat = Quaterniond(model->body_quat[4 * body_id],
                            model->body_quat[4 * body_id + 1],
                            model->body_quat[4 * body_id + 2],
                            model->body_quat[4 * body_id + 3])
                    .normalized();
    body.mass = model->body_mass[body_id];
    body.com = Vector3d(model->body_ipos[3 * body_id],
                        model->body_ipos[3 * body_id + 1],
                        model->body_ipos[3 * body_id + 2]);
    assign_inertia(body, body_inertia(model.get(), body_id));
    body.joint_axis = Vector3d(model->jnt_axis[3 * joint_id],
                               model->jnt_axis[3 * joint_id + 1],
                               model->jnt_axis[3 * joint_id + 2]);
    body.armature = model->dof_armature[dof_id];
    body.damping = model->dof_damping[dof_id];
    frictionloss_[index - 1] = frictionloss.empty()
                                   ? model->dof_frictionloss[dof_id]
                                   : frictionloss[index - 1];
    body.has_joint = true;
  }

  const int home_id = mj_name2id(model.get(), mjOBJ_KEY, "home");
  if (home_id >= 0) {
    mj_resetDataKeyframe(model.get(), data.get(), home_id);
  } else {
    mj_resetData(model.get(), data.get());
  }
  for (std::size_t finger = 0; finger < gripper_position.size(); ++finger) {
    const int joint_id =
        require_id(mjOBJ_JOINT, "joint" + std::to_string(7 + finger));
    data->qpos[model->jnt_qposadr[joint_id]] = gripper_position[finger];
  }
  mj_forward(model.get(), data.get());

  const int link6_id = require_id(mjOBJ_BODY, "link6");
  const Matrix3d world_from_link6 = body_matrix(data->xmat + 9 * link6_id);
  const Vector3d link6_origin_world(data->xpos[3 * link6_id],
                                    data->xpos[3 * link6_id + 1],
                                    data->xpos[3 * link6_id + 2]);
  auto &link6 = bodies_[6];
  double total_mass = link6.mass;
  Vector3d total_first_moment = link6.mass * link6.com;
  Matrix3d link6_com_inertia;
  link6_com_inertia << link6.Ixx, link6.Ixy, link6.Ixz, link6.Ixy,
      link6.Iyy, link6.Iyz, link6.Ixz, link6.Iyz, link6.Izz;
  Matrix3d total_origin_inertia =
      link6_com_inertia +
      link6.mass * (link6.com.squaredNorm() * Matrix3d::Identity() -
                    link6.com * link6.com.transpose());

  for (const char *finger_name : {"left_finger", "right_finger"}) {
    const int finger_id = require_id(mjOBJ_BODY, finger_name);
    const double mass = model->body_mass[finger_id];
    const Vector3d com_world(data->xipos[3 * finger_id],
                             data->xipos[3 * finger_id + 1],
                             data->xipos[3 * finger_id + 2]);
    const Vector3d com_link6 =
        world_from_link6.transpose() * (com_world - link6_origin_world);
    const Matrix3d world_from_inertia =
        body_matrix(data->ximat + 9 * finger_id);
    const Vector3d principal(model->body_inertia[3 * finger_id],
                             model->body_inertia[3 * finger_id + 1],
                             model->body_inertia[3 * finger_id + 2]);
    const Matrix3d inertia_world =
        world_from_inertia * principal.asDiagonal() *
        world_from_inertia.transpose();
    const Matrix3d inertia_link6 =
        world_from_link6.transpose() * inertia_world * world_from_link6;
    total_mass += mass;
    total_first_moment += mass * com_link6;
    total_origin_inertia +=
        inertia_link6 +
        mass * (com_link6.squaredNorm() * Matrix3d::Identity() -
                com_link6 * com_link6.transpose());
  }

  link6.mass = total_mass;
  link6.com = total_first_moment / total_mass;
  const Matrix3d composite_com_inertia =
      total_origin_inertia -
      total_mass * (link6.com.squaredNorm() * Matrix3d::Identity() -
                    link6.com * link6.com.transpose());
  assign_inertia(link6, composite_com_inertia);
  gravity_ = Vector3d(model->opt.gravity[0], model->opt.gravity[1],
                      model->opt.gravity[2]);
}

// ============================================================================
// 辅助函数
// ============================================================================

MuJoCoPiperRegressor::Matrix3d MuJoCoPiperRegressor::skew(const Vector3d &v) {
  Matrix3d S;
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return S;
}

MuJoCoPiperRegressor::Matrix4d
MuJoCoPiperRegressor::poseToTransform(const Vector3d &pos, const Quaterniond &quat) {
  Matrix4d T = Matrix4d::Identity();
  T.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = pos;
  return T;
}

// ============================================================================
// 运动学
// ============================================================================

std::vector<MuJoCoPiperRegressor::Matrix4d>
MuJoCoPiperRegressor::computeBodyTransforms(const VectorXd &q) const {
  std::vector<Matrix4d> transforms(N_BODIES + 1);

  // link0 在世界坐标系
  transforms[0] = poseToTransform(bodies_[0].pos, bodies_[0].quat);

  std::size_t joint_idx = 0;

  for (std::size_t i = 1; i <= N_BODIES; ++i) {
    const auto &body = bodies_[i];

    // 相对于父 body 的基础变换
    Matrix4d T_parent_body = poseToTransform(body.pos, body.quat);

    // 如果有关节，添加关节旋转
    if (body.has_joint && joint_idx < N_DOF) {
      double angle = q(joint_idx);
      Quaterniond joint_rot =
          Quaterniond(Eigen::AngleAxisd(angle, body.joint_axis));

      Matrix4d T_joint = Matrix4d::Identity();
      T_joint.block<3, 3>(0, 0) = joint_rot.toRotationMatrix();

      transforms[i] = transforms[i - 1] * T_parent_body * T_joint;
      ++joint_idx;
    } else {
      transforms[i] = transforms[i - 1] * T_parent_body;
    }
  }

  return transforms;
}

/**
 * @brief 计算 Body 原点的雅可比矩阵 (用于回归矩阵)
 * 回归矩阵使用标准惯性参数 (m, mc, I_origin)，惯量定义在 Body 原点
 */
MuJoCoPiperRegressor::MatrixXd
MuJoCoPiperRegressor::computeBodyOriginJacobian(std::size_t body_idx,
                                           const VectorXd &q) const {
  MatrixXd J = MatrixXd::Zero(6, N_DOF);

  auto transforms = computeBodyTransforms(q);

  // Body 原点位置 (不是 COM！)
  Vector3d p_origin = transforms[body_idx].block<3, 1>(0, 3);

  // 对每个关节
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i <= N_BODIES; ++i) {
    if (bodies_[i].has_joint) {
      // 关节轴在世界坐标系中的方向
      Vector3d z_axis = transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;

      // 关节原点位置
      Vector3d p_joint = transforms[i].block<3, 1>(0, 3);

      // 线速度雅可比: z × (p_origin - p_joint)
      J.block<3, 1>(0, joint_count) = z_axis.cross(p_origin - p_joint);

      // 角速度雅可比: z
      J.block<3, 1>(3, joint_count) = z_axis;

      ++joint_count;
    }
  }

  return J;
}

MuJoCoPiperRegressor::MatrixXd
MuJoCoPiperRegressor::computeBodyOriginJacobianDerivative(
    std::size_t body_idx, const VectorXd &q, const VectorXd &qd) const {
  MatrixXd derivative = MatrixXd::Zero(6, N_DOF);
  const auto transforms = computeBodyTransforms(q);
  const MatrixXd jacobian = computeBodyOriginJacobian(body_idx, q);
  const Vector3d body_origin =
      transforms[body_idx].block<3, 1>(0, 3);
  const Vector3d body_velocity = jacobian.topRows(3) * qd;

  Vector3d preceding_angular_velocity = Vector3d::Zero();
  for (std::size_t joint = 1; joint <= body_idx; ++joint) {
    const Eigen::Index column = static_cast<Eigen::Index>(joint - 1);
    const Vector3d axis = jacobian.block<3, 1>(3, column);
    const Vector3d joint_origin =
        transforms[joint].block<3, 1>(0, 3);
    Vector3d joint_velocity = Vector3d::Zero();
    for (std::size_t ancestor = 1; ancestor < joint; ++ancestor) {
      const Eigen::Index ancestor_column =
          static_cast<Eigen::Index>(ancestor - 1);
      const Vector3d ancestor_axis =
          jacobian.block<3, 1>(3, ancestor_column);
      const Vector3d ancestor_origin =
          transforms[ancestor].block<3, 1>(0, 3);
      joint_velocity += ancestor_axis.cross(joint_origin - ancestor_origin) *
                        qd(ancestor_column);
    }
    const Vector3d axis_derivative = preceding_angular_velocity.cross(axis);
    derivative.block<3, 1>(0, column) =
        axis_derivative.cross(body_origin - joint_origin) +
        axis.cross(body_velocity - joint_velocity);
    derivative.block<3, 1>(3, column) = axis_derivative;
    preceding_angular_velocity += axis * qd(column);
  }
  return derivative;
}

// ============================================================================
// 参数向量
// ============================================================================

std::size_t MuJoCoPiperRegressor::numParameters(MuJoCoParamFlags flags) const {
  std::size_t params = N_BODIES * MuJoCoInertialParams::PARAMS_PER_BODY;

  if (hasFlag(flags, MuJoCoParamFlags::ARMATURE)) {
    params += N_DOF;
  }

  if (hasFlag(flags, MuJoCoParamFlags::DAMPING)) {
    params += N_DOF;
  }

  if (hasFlag(flags, MuJoCoParamFlags::FRICTION_LOSS)) {
    params += N_DOF;
  }

  return params;
}

MuJoCoPiperRegressor::VectorXd
MuJoCoPiperRegressor::computeParameterVector(MuJoCoParamFlags flags) const {
  const std::size_t num_params = numParameters(flags);
  VectorXd theta = VectorXd::Zero(num_params);

  // 填充惯性参数 (link1-6)
  for (std::size_t i = 0; i < N_BODIES; ++i) {
    std::size_t body_idx = i + 1;
    auto sip = MuJoCoInertialParams::fromMuJoCoBody(bodies_[body_idx]);
    auto sip_vec = sip.toVector();

    std::size_t offset = i * MuJoCoInertialParams::PARAMS_PER_BODY;
    theta.segment(offset, MuJoCoInertialParams::PARAMS_PER_BODY) = sip_vec;
  }

  std::size_t current_offset = N_BODIES * MuJoCoInertialParams::PARAMS_PER_BODY;

  // Armature 参数
  if (hasFlag(flags, MuJoCoParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      theta(current_offset + i) = bodies_[i + 1].armature;
    }
    current_offset += N_DOF;
  }

  // Damping 参数
  if (hasFlag(flags, MuJoCoParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      theta(current_offset + i) = bodies_[i + 1].damping;
    }
    current_offset += N_DOF;
  }

  if (hasFlag(flags, MuJoCoParamFlags::FRICTION_LOSS)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      theta(current_offset + i) = frictionloss_[i];
    }
  }

  return theta;
}

std::vector<std::string>
MuJoCoPiperRegressor::getParameterNames(MuJoCoParamFlags flags) const {
  std::vector<std::string> names;

  const char *body_names[] = {"link1", "link2", "link3",
                              "link4", "link5", "link6"};
  const char *param_names[] = {"m",   "mx",  "my",  "mz",  "Ixx",
                               "Ixy", "Ixz", "Iyy", "Iyz", "Izz"};

  for (std::size_t i = 0; i < N_BODIES; ++i) {
    for (int j = 0; j < 10; ++j) {
      names.push_back(std::string(body_names[i]) + "_" + param_names[j]);
    }
  }

  if (hasFlag(flags, MuJoCoParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      names.push_back("armature_" + std::to_string(i + 1));
    }
  }

  if (hasFlag(flags, MuJoCoParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      names.push_back("damping_" + std::to_string(i + 1));
    }
  }

  if (hasFlag(flags, MuJoCoParamFlags::FRICTION_LOSS)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      names.push_back("frictionloss_" + std::to_string(i + 1));
    }
  }

  return names;
}

// ============================================================================
// 回归矩阵
// ============================================================================

MuJoCoPiperRegressor::MatrixXd MuJoCoPiperRegressor::computeBodyRegressorBlock(
    std::size_t body_idx, const VectorXd &q, const VectorXd &qd,
    const VectorXd &qdd) const {

  /**
   * 标准惯性参数回归矩阵推导（Body 局部坐标系）
   *
   * 参数向量: θ = [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
   * - (mx, my, mz) = m * (cx, cy, cz) 是一阶矩（局部坐标系）
   * - I_origin 是在 Body 原点的惯量张量（局部坐标系）
   *
   * 动力学方程（在 Body 局部坐标系中）：
   *   f_local = m * (a_local - g_local) + K * mc_local
   *   n_local = [mc_local]× * (a_local - g_local) + I_origin * α_local +
   * [ω_local]× * I_origin * ω_local
   *
   * 其中 K = [α_local]× + [ω_local]× * [ω_local]×
   *
   * 关节扭矩：τ = J_v_local^T * f_local + J_w_local^T * n_local
   */

  auto transforms = computeBodyTransforms(q);
  Matrix3d R = transforms[body_idx].block<3, 3>(0, 0); // Body 到 World 旋转

  // The analytic derivative avoids turning structural null directions into
  // small non-zero singular values during rank analysis.
  const MatrixXd J_world = computeBodyOriginJacobian(body_idx, q);
  const MatrixXd J_world_dot =
      computeBodyOriginJacobianDerivative(body_idx, q, qd);

  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_world = J_world.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_world = J_world.bottomRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_dot_world =
      J_world_dot.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_dot_world =
      J_world_dot.bottomRows(3);

  // ========== 世界坐标系中的运动学量 ==========
  Vector3d a_origin_world = Jv_world * qdd + Jv_dot_world * qd;
  Vector3d omega_world = Jw_world * qd;
  Vector3d alpha_world = Jw_world * qdd + Jw_dot_world * qd;

  // ========== 转换到 Body 局部坐标系 ==========
  Matrix3d Rt = R.transpose();
  Vector3d a_local = Rt * a_origin_world;
  Vector3d omega_local = Rt * omega_world;
  Vector3d alpha_local = Rt * alpha_world;
  Vector3d g_local = Rt * gravity_;
  Vector3d b_local = a_local - g_local;

  // 局部坐标系中的雅可比
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_local = Rt * Jv_world;
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_local = Rt * Jw_world;

  // ========== 构建回归矩阵块 ==========
  MatrixXd Y_block = MatrixXd::Zero(N_DOF, 10);

  // K 矩阵: K = [α]× + [ω]× * [ω]×
  Matrix3d alpha_skew = skew(alpha_local);
  Matrix3d omega_skew = skew(omega_local);
  Matrix3d K = alpha_skew + omega_skew * omega_skew;

  double ox = omega_local(0), oy = omega_local(1), oz = omega_local(2);
  double ax = alpha_local(0), ay = alpha_local(1), az = alpha_local(2);

  // 1. 质量 m: f = m * b, n = 0
  Y_block.col(0) = Jv_local.transpose() * b_local;

  // 2. 一阶矩 (mx, my, mz): f = K * e_i, n = e_i × b
  for (int i = 0; i < 3; ++i) {
    Vector3d e_i = Vector3d::Zero();
    e_i(i) = 1.0;
    Y_block.col(1 + i) = Jv_local.transpose() * K.col(i) +
                         Jw_local.transpose() * e_i.cross(b_local);
  }

  // 3. 惯量张量: n = E * α + [ω]× * E * ω
  Y_block.col(4) = Jw_local.transpose() * Vector3d(ax, ox * oz, -ox * oy);
  Y_block.col(5) = Jw_local.transpose() *
                   Vector3d(ay - ox * oz, ax + oy * oz, ox * ox - oy * oy);
  Y_block.col(6) = Jw_local.transpose() *
                   Vector3d(az + ox * oy, oz * oz - ox * ox, ax - oy * oz);
  Y_block.col(7) = Jw_local.transpose() * Vector3d(-oy * oz, ay, ox * oy);
  Y_block.col(8) = Jw_local.transpose() *
                   Vector3d(oy * oy - oz * oz, az - ox * oy, ay + ox * oz);
  Y_block.col(9) = Jw_local.transpose() * Vector3d(oy * oz, -ox * oz, az);

  return Y_block;
}

MuJoCoPiperRegressor::MatrixXd
MuJoCoPiperRegressor::computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                                        const VectorXd &qdd,
                                        MuJoCoParamFlags flags) const {

  const std::size_t num_params = numParameters(flags);
  MatrixXd Y = MatrixXd::Zero(N_DOF, num_params);

  // 计算每个 Body 的回归矩阵块
  for (std::size_t i = 0; i < N_BODIES; ++i) {
    std::size_t body_idx = i + 1;
    MatrixXd Y_body = computeBodyRegressorBlock(body_idx, q, qd, qdd);

    std::size_t offset = i * MuJoCoInertialParams::PARAMS_PER_BODY;
    Y.block(0, offset, N_DOF, MuJoCoInertialParams::PARAMS_PER_BODY) = Y_body;
  }

  std::size_t current_offset = N_BODIES * MuJoCoInertialParams::PARAMS_PER_BODY;

  // Armature 回归: τ_armature = armature * q̈
  if (hasFlag(flags, MuJoCoParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      Y(i, current_offset + i) = qdd(i);
    }
    current_offset += N_DOF;
  }

  // MuJoCo applies passive damping as -d*qdot, so the actuator must provide
  // +d*qdot in the inverse-dynamics torque balance.
  if (hasFlag(flags, MuJoCoParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      Y(i, current_offset + i) = qd(i);
    }
    current_offset += N_DOF;
  }

  // Away from zero velocity, MuJoCo frictionloss requires actuator
  // compensation with the same sign as qdot.
  if (hasFlag(flags, MuJoCoParamFlags::FRICTION_LOSS)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      Y(i, current_offset + i) =
          qd(i) > 0.0 ? 1.0 : (qd(i) < 0.0 ? -1.0 : 0.0);
    }
  }

  return Y;
}

MuJoCoPiperRegressor::MatrixXd
MuJoCoPiperRegressor::computeObservationMatrix(const MatrixXd &Q, const MatrixXd &Qd,
                                          const MatrixXd &Qdd,
                                          MuJoCoParamFlags flags) const {

  const std::size_t K = Q.cols(); // 样本数
  const std::size_t num_params = numParameters(flags);

  MatrixXd W = MatrixXd::Zero(N_DOF * K, num_params);

#ifdef IDENTIFICATION_USE_OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(K); ++k) {
    VectorXd q = Q.col(k);
    VectorXd qd = Qd.col(k);
    VectorXd qdd = Qdd.col(k);

    MatrixXd Y_k = computeRegressorMatrix(q, qd, qdd, flags);
    W.block(k * static_cast<Eigen::Index>(N_DOF), 0,
            static_cast<Eigen::Index>(N_DOF),
            static_cast<Eigen::Index>(num_params)) = Y_k;
  }

  return W;
}

} // namespace mujoco_dynamics
