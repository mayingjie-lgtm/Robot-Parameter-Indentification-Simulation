/**
 * @file rebot_identification_model_closure.cpp
 * @brief Diagnose the clean-identification oracle floor against full gripper dynamics.
 */

#include "force_node/force_controller.hpp"
#include "rebot_pinocchio_dynamics.hpp"
#include "sim_com_node/panda_simulator.hpp"

#include <pinocchio/algorithm/rnea.hpp>

#include <mujoco/mujoco.h>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::size_t kArmDof = 6;
constexpr std::size_t kGripperDof = 2;
constexpr double kSlidingThreshold = 0.05;
constexpr double kConstraintTolerance = 1e-10;
constexpr double kCrossEngineClosureTolerance = 1e-10;

struct ErrorAccumulator {
  double sum_squared{0.0};
  double max_error{0.0};
  std::size_t count{0};

  /** Add one scalar prediction error to the aggregate statistics. */
  void add(double error) {
    sum_squared += error * error;
    max_error = std::max(max_error, std::abs(error));
    ++count;
  }

  /** Return root-mean-square error, rejecting an empty accumulator. */
  double rmse() const {
    if (count == 0) {
      throw std::runtime_error("cannot compute RMSE without observations");
    }
    return std::sqrt(sum_squared / static_cast<double>(count));
  }
};

struct WorstSample {
  double error{0.0};
  std::size_t sample{0};
  std::size_t joint{0};
  double time{0.0};
  double tau_effort{0.0};
  double reduced_prediction{0.0};
  double full_prediction{0.0};
  std::array<double, kGripperDof> gripper_q{};
  std::array<double, kGripperDof> gripper_qd{};
  std::array<double, kGripperDof> gripper_qdd{};
};

/** Record one candidate when its absolute error exceeds the current worst. */
void updateWorst(WorstSample &worst, double error, std::size_t sample,
                 std::size_t joint, double time, double tau_effort,
                 double reduced_prediction, double full_prediction,
                 const std::array<double, kGripperDof> &gripper_q,
                 const std::array<double, kGripperDof> &gripper_qd,
                 const std::array<double, kGripperDof> &gripper_qdd) {
  if (std::abs(error) <= std::abs(worst.error)) {
    return;
  }
  worst.error = error;
  worst.sample = sample;
  worst.joint = joint;
  worst.time = time;
  worst.tau_effort = tau_effort;
  worst.reduced_prediction = reduced_prediction;
  worst.full_prediction = full_prediction;
  worst.gripper_q = gripper_q;
  worst.gripper_qd = gripper_qd;
  worst.gripper_qdd = gripper_qdd;
}

/** Print one captured worst-error state with its gripper kinematics. */
void printWorst(const char *label, const WorstSample &worst) {
  std::cout << label << " sample=" << worst.sample << " joint=J"
            << (worst.joint + 1) << " time=" << worst.time
            << " error=" << worst.error << " tau_effort=" << worst.tau_effort
            << " reduced_prediction=" << worst.reduced_prediction
            << " full_prediction=" << worst.full_prediction << "\n";
  std::cout << label << " gripper_q=[" << worst.gripper_q[0] << ","
            << worst.gripper_q[1] << "] qd=[" << worst.gripper_qd[0] << ","
            << worst.gripper_qd[1] << "] qdd=[" << worst.gripper_qdd[0] << ","
            << worst.gripper_qdd[1] << "]\n";
}

/** Resolve a required MuJoCo scalar joint by name. */
int requireMjJoint(const mjModel *model, const std::string &name) {
  const int id = mj_name2id(model, mjOBJ_JOINT, name.c_str());
  if (id < 0) {
    throw std::runtime_error("MuJoCo model is missing joint: " + name);
  }
  if (model->jnt_type[id] == mjJNT_FREE || model->jnt_type[id] == mjJNT_BALL) {
    throw std::runtime_error(name + " must be a scalar joint");
  }
  return id;
}

/** Apply one six-joint simulation-truth vector to MuJoCo DOFs. */
void applyJointTruth(const mjModel *model, mjtNum *target,
                     const std::array<int, kArmDof> &joint_ids,
                     const std::vector<double> &values,
                     const std::string &name) {
  if (values.size() != kArmDof) {
    throw std::runtime_error(name + " must contain exactly six values");
  }
  for (std::size_t joint = 0; joint < kArmDof; ++joint) {
    target[model->jnt_dofadr[joint_ids[joint]]] = values[joint];
  }
}

/** Return whether one arm row is valid for the hard-Coulomb clean closure. */
bool saturatedSliding(double velocity, double constraint, double frictionloss) {
  if (std::abs(velocity) < kSlidingThreshold) {
    return false;
  }
  const double motion_sign = velocity > 0.0 ? 1.0 : -1.0;
  return std::abs(std::abs(constraint) - frictionloss) <=
             kConstraintTolerance &&
         constraint * motion_sign <= kConstraintTolerance;
}

} // namespace

/**
 * Reproduce a frozen excitation and compare reduced vs full-gripper Pinocchio
 * actuator requirements against the same MuJoCo forward state semantics.
 */
int main(int argc, char **argv) {
  try {
    std::uint32_t seed = 20260826U;
    if (argc > 2) {
      std::cerr << "Usage: rebot_identification_model_closure [trajectory_seed]\n";
      return 2;
    }
    if (argc == 2) {
      seed = static_cast<std::uint32_t>(std::stoul(argv[1]));
    }

    const std::filesystem::path root(PROJECT_ROOT_DIR);
    auto controller_config = force_node::ForceController::loadConfig(
        root / "config" / "rebot_dm_excitation_controller.yaml");
    controller_config.trajectory_seed = seed;
    force_node::ForceController controller(
        controller_config, root / "rebot_dm" / "rebot_dm_runtime.xml");
    const auto sim_config = sim_com_node::PandaSimulator::loadConfig(
        root / "config" / "rebot_dm_excitation_sim_node.yaml");

    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML((root / "rebot_dm" / "scene_runtime.xml").string().c_str(),
                   nullptr, error, sizeof(error)),
        mj_deleteModel);
    if (!model) {
      throw std::runtime_error(std::string("failed to load runtime scene: ") +
                               error);
    }
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()),
                                                           mj_deleteData);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> inverse_data(
        mj_makeData(model.get()), mj_deleteData);
    if (!data || !inverse_data) {
      throw std::runtime_error("failed to allocate MuJoCo data");
    }
    model->opt.timestep = 1.0 / sim_config.simulation_rate_hz;
    const int keyframe = mj_name2id(model.get(), mjOBJ_KEY,
                                    sim_config.initial_keyframe.c_str());
    if (keyframe < 0) {
      throw std::runtime_error("runtime_home keyframe is missing");
    }
    mj_resetDataKeyframe(model.get(), data.get(), keyframe);

    std::array<int, kArmDof> arm_joint_ids{};
    std::array<int, kArmDof> arm_qpos{};
    std::array<int, kArmDof> arm_dof{};
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      arm_joint_ids[joint] =
          requireMjJoint(model.get(), "joint" + std::to_string(joint + 1));
      arm_qpos[joint] = model->jnt_qposadr[arm_joint_ids[joint]];
      arm_dof[joint] = model->jnt_dofadr[arm_joint_ids[joint]];
    }
    std::array<int, kGripperDof> gripper_joint_ids{};
    std::array<int, kGripperDof> gripper_qpos{};
    std::array<int, kGripperDof> gripper_dof{};
    for (std::size_t joint = 0; joint < kGripperDof; ++joint) {
      gripper_joint_ids[joint] = requireMjJoint(
          model.get(), "gripper_joint" + std::to_string(joint + 1));
      gripper_qpos[joint] = model->jnt_qposadr[gripper_joint_ids[joint]];
      gripper_dof[joint] = model->jnt_dofadr[gripper_joint_ids[joint]];
    }

    applyJointTruth(model.get(), model->dof_armature, arm_joint_ids,
                    sim_config.joint_armature, "joint_armature");
    applyJointTruth(model.get(), model->dof_damping, arm_joint_ids,
                    sim_config.joint_damping, "joint_damping");
    applyJointTruth(model.get(), model->dof_frictionloss, arm_joint_ids,
                    sim_config.joint_frictionloss, "joint_frictionloss");

    rebot_dynamics::ReBotPinocchioDynamics dynamics(
        root / "rebot_dm" / "rebot_dm.urdf", {0.05, 0.05});
    const pinocchio::Model &full_model = dynamics.fullModel();
    pinocchio::Data full_data(full_model);

    std::array<pinocchio::JointIndex, kArmDof> full_arm_joint_ids{};
    std::array<pinocchio::JointIndex, kGripperDof> full_gripper_joint_ids{};
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      const std::string name = "joint" + std::to_string(joint + 1);
      if (!full_model.existJointName(name)) {
        throw std::runtime_error("Pinocchio full model is missing joint: " + name);
      }
      full_arm_joint_ids[joint] = full_model.getJointId(name);
    }
    for (std::size_t joint = 0; joint < kGripperDof; ++joint) {
      const std::string name = "gripper_joint" + std::to_string(joint + 1);
      if (!full_model.existJointName(name)) {
        throw std::runtime_error("Pinocchio full model is missing joint: " + name);
      }
      full_gripper_joint_ids[joint] = full_model.getJointId(name);
    }

    ErrorAccumulator reduced_error;
    ErrorAccumulator full_error;
    ErrorAccumulator full_minus_reduced;
    ErrorAccumulator inverse_vs_actuator;
    ErrorAccumulator full_vs_inverse;
    std::array<std::size_t, kArmDof> selected_counts{};
    std::array<double, kGripperDof> max_gripper_position_deviation{};
    std::array<double, kGripperDof> max_gripper_speed{};
    std::array<double, kGripperDof> max_gripper_acceleration{};
    WorstSample worst_reduced;
    WorstSample worst_full;

    double simulation_time = 0.0;
    std::size_t sample_index = 0;
    while (simulation_time < controller.trajectoryDuration()) {
      force_node::JointSample sample;
      sample.position.resize(kArmDof);
      sample.velocity.resize(kArmDof);
      sample.effort.resize(kArmDof);
      for (std::size_t joint = 0; joint < kArmDof; ++joint) {
        sample.position[joint] = data->qpos[arm_qpos[joint]];
        sample.velocity[joint] = data->qvel[arm_dof[joint]];
        sample.effort[joint] = data->qfrc_actuator[arm_dof[joint]];
      }
      const auto command = controller.computeCommand(sample, simulation_time);
      for (std::size_t joint = 0; joint < kArmDof; ++joint) {
        data->ctrl[joint] = command.torque[joint];
      }

      Eigen::VectorXd q_arm(kArmDof);
      Eigen::VectorXd qd_arm(kArmDof);
      Eigen::VectorXd q_full = Eigen::VectorXd::Zero(full_model.nq);
      Eigen::VectorXd qd_full = Eigen::VectorXd::Zero(full_model.nv);
      for (std::size_t joint = 0; joint < kArmDof; ++joint) {
        q_arm(static_cast<Eigen::Index>(joint)) = sample.position[joint];
        qd_arm(static_cast<Eigen::Index>(joint)) = sample.velocity[joint];
        const auto pin_joint = full_arm_joint_ids[joint];
        q_full(full_model.idx_qs[pin_joint]) = sample.position[joint];
        qd_full(full_model.idx_vs[pin_joint]) = sample.velocity[joint];
      }
      std::array<double, kGripperDof> gripper_q{};
      std::array<double, kGripperDof> gripper_qd{};
      for (std::size_t joint = 0; joint < kGripperDof; ++joint) {
        gripper_q[joint] = data->qpos[gripper_qpos[joint]];
        gripper_qd[joint] = data->qvel[gripper_dof[joint]];
        const auto pin_joint = full_gripper_joint_ids[joint];
        q_full(full_model.idx_qs[pin_joint]) = gripper_q[joint];
        qd_full(full_model.idx_vs[pin_joint]) = gripper_qd[joint];
        max_gripper_position_deviation[joint] =
            std::max(max_gripper_position_deviation[joint],
                     std::abs(gripper_q[joint] - 0.05));
        max_gripper_speed[joint] =
            std::max(max_gripper_speed[joint], std::abs(gripper_qd[joint]));
      }

      mju_copy(inverse_data->qpos, data->qpos, model->nq);
      mju_copy(inverse_data->qvel, data->qvel, model->nv);
      mj_step(model.get(), data.get());
      mju_copy(inverse_data->qacc, data->qacc, model->nv);
      mj_inverse(model.get(), inverse_data.get());

      Eigen::VectorXd qdd_arm(kArmDof);
      Eigen::VectorXd qdd_full = Eigen::VectorXd::Zero(full_model.nv);
      for (std::size_t joint = 0; joint < kArmDof; ++joint) {
        qdd_arm(static_cast<Eigen::Index>(joint)) = data->qacc[arm_dof[joint]];
        qdd_full(full_model.idx_vs[full_arm_joint_ids[joint]]) =
            data->qacc[arm_dof[joint]];
      }
      std::array<double, kGripperDof> gripper_qdd{};
      for (std::size_t joint = 0; joint < kGripperDof; ++joint) {
        gripper_qdd[joint] = data->qacc[gripper_dof[joint]];
        qdd_full(full_model.idx_vs[full_gripper_joint_ids[joint]]) =
            gripper_qdd[joint];
        max_gripper_acceleration[joint] =
            std::max(max_gripper_acceleration[joint],
                     std::abs(gripper_qdd[joint]));
      }

      const Eigen::VectorXd reduced_rigid =
          dynamics.inverseDynamics(q_arm, qd_arm, qdd_arm);
      const Eigen::VectorXd full_rigid =
          pinocchio::rnea(full_model, full_data, q_full, qd_full, qdd_full);

      for (std::size_t joint = 0; joint < kArmDof; ++joint) {
        const double constraint = data->qfrc_constraint[arm_dof[joint]];
        if (!saturatedSliding(sample.velocity[joint], constraint,
                              sim_config.joint_frictionloss[joint])) {
          continue;
        }
        ++selected_counts[joint];
        const double actuator_terms =
            sim_config.joint_armature[joint] *
                qdd_arm(static_cast<Eigen::Index>(joint)) +
            sim_config.joint_damping[joint] * sample.velocity[joint] -
            constraint;
        const double reduced_prediction =
            reduced_rigid(static_cast<Eigen::Index>(joint)) + actuator_terms;
        const double full_prediction =
            full_rigid(full_model.idx_vs[full_arm_joint_ids[joint]]) +
            actuator_terms;
        const double tau_effort = data->qfrc_actuator[arm_dof[joint]];
        const double inverse_torque =
            inverse_data->qfrc_inverse[arm_dof[joint]];
        const double reduced_delta = reduced_prediction - tau_effort;
        const double full_delta = full_prediction - tau_effort;
        reduced_error.add(reduced_delta);
        full_error.add(full_delta);
        full_minus_reduced.add(full_prediction - reduced_prediction);
        inverse_vs_actuator.add(inverse_torque - tau_effort);
        full_vs_inverse.add(full_prediction - inverse_torque);
        updateWorst(worst_reduced, reduced_delta, sample_index, joint,
                    simulation_time, tau_effort, reduced_prediction,
                    full_prediction, gripper_q, gripper_qd, gripper_qdd);
        updateWorst(worst_full, full_delta, sample_index, joint,
                    simulation_time, tau_effort, reduced_prediction,
                    full_prediction, gripper_q, gripper_qd, gripper_qdd);
      }

      simulation_time += model->opt.timestep;
      ++sample_index;
    }

    std::cout << std::scientific << std::setprecision(12);
    std::cout << "seed=" << seed << " samples=" << sample_index
              << " accepted_scale=" << controller.acceptedTrajectoryScale()
              << " accepted_attempt=" << controller.acceptedTrajectoryAttempt()
              << "\n";
    std::cout << "saturated_sliding_counts=[";
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      if (joint) std::cout << ",";
      std::cout << selected_counts[joint];
    }
    std::cout << "]\n";
    std::cout << "reduced_fixed_gripper_rmse=" << reduced_error.rmse()
              << " max=" << reduced_error.max_error << "\n";
    std::cout << "full_actual_gripper_rmse=" << full_error.rmse()
              << " max=" << full_error.max_error << "\n";
    std::cout << "full_minus_reduced_rmse=" << full_minus_reduced.rmse()
              << " max=" << full_minus_reduced.max_error << "\n";
    std::cout << "mujoco_inverse_vs_forward_actuator_rmse="
              << inverse_vs_actuator.rmse()
              << " max=" << inverse_vs_actuator.max_error << "\n";
    std::cout << "full_pinocchio_vs_mujoco_inverse_rmse="
              << full_vs_inverse.rmse() << " max=" << full_vs_inverse.max_error
              << "\n";
    std::cout << "max_gripper_position_deviation=["
              << max_gripper_position_deviation[0] << ","
              << max_gripper_position_deviation[1] << "]\n";
    std::cout << "max_gripper_speed=[" << max_gripper_speed[0] << ","
              << max_gripper_speed[1] << "]\n";
    std::cout << "max_gripper_acceleration=[" << max_gripper_acceleration[0]
              << "," << max_gripper_acceleration[1] << "]\n";
    printWorst("worst_reduced", worst_reduced);
    printWorst("worst_full", worst_full);

    bool passed = full_vs_inverse.max_error <= kCrossEngineClosureTolerance;
    for (const std::size_t count : selected_counts) {
      passed = passed && count > 0;
    }
    if (!passed) {
      std::cerr << "[FAIL] full Pinocchio model does not match MuJoCo inverse dynamics\n";
      return 1;
    }
    std::cout << "[PASS] oracle floor decomposed into reduced fixed-gripper "
                 "coupling and MuJoCo forward/inverse solver residual\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "reBot identification model-closure diagnostic failed: "
              << error.what() << "\n";
    return 1;
  }
}
