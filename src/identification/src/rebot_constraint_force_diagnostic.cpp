#include "force_node/force_controller.hpp"
#include "sim_com_node/panda_simulator.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
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
constexpr std::size_t kCategoryCount = 5;
constexpr double kReconstructionTolerance = 1e-10;

enum class ConstraintCategory : std::size_t {
  FRICTION = 0,
  EQUALITY = 1,
  LIMIT = 2,
  CONTACT = 3,
  OTHER = 4,
};

struct RowRecord {
  int type{0};
  int id{0};
  int state{0};
  double force{0.0};
  double joint6_contribution{0.0};
};

struct RepresentativeSample {
  double target_speed{0.0};
  double distance{std::numeric_limits<double>::infinity()};
  double time{0.0};
  double q6{0.0};
  double qd6{0.0};
  double qdd6{0.0};
  double frictionloss6{0.0};
  double total6{0.0};
  std::array<double, kCategoryCount> category6{};
  double expected_hard_coulomb{0.0};
  std::vector<RowRecord> rows;
};

/** Map a MuJoCo solver row type into the diagnostic's physical categories. */
ConstraintCategory categoryForType(int type) {
  switch (type) {
  case mjCNSTR_FRICTION_DOF:
  case mjCNSTR_FRICTION_TENDON:
    return ConstraintCategory::FRICTION;
  case mjCNSTR_EQUALITY:
    return ConstraintCategory::EQUALITY;
  case mjCNSTR_LIMIT_JOINT:
  case mjCNSTR_LIMIT_TENDON:
    return ConstraintCategory::LIMIT;
  case mjCNSTR_CONTACT_FRICTIONLESS:
  case mjCNSTR_CONTACT_PYRAMIDAL:
  case mjCNSTR_CONTACT_ELLIPTIC:
    return ConstraintCategory::CONTACT;
  default:
    return ConstraintCategory::OTHER;
  }
}

/** Return a stable textual name for a MuJoCo constraint type. */
const char *constraintTypeName(int type) {
  switch (type) {
  case mjCNSTR_EQUALITY: return "equality";
  case mjCNSTR_FRICTION_DOF: return "friction_dof";
  case mjCNSTR_FRICTION_TENDON: return "friction_tendon";
  case mjCNSTR_LIMIT_JOINT: return "joint_limit";
  case mjCNSTR_LIMIT_TENDON: return "tendon_limit";
  case mjCNSTR_CONTACT_FRICTIONLESS: return "contact_frictionless";
  case mjCNSTR_CONTACT_PYRAMIDAL: return "contact_pyramidal";
  case mjCNSTR_CONTACT_ELLIPTIC: return "contact_elliptic";
  default: return "other";
  }
}

/** Return a stable textual name for the MuJoCo constraint-state regime. */
const char *constraintStateName(int state) {
  switch (state) {
  case mjCNSTRSTATE_SATISFIED: return "satisfied";
  case mjCNSTRSTATE_QUADRATIC: return "quadratic";
  case mjCNSTRSTATE_LINEARNEG: return "linear_negative";
  case mjCNSTRSTATE_LINEARPOS: return "linear_positive";
  case mjCNSTRSTATE_CONE: return "cone";
  default: return "unknown";
  }
}

/** Apply one configured six-joint simulation-truth vector to MuJoCo DOFs. */
void applyJointTruth(const mjModel *model, mjtNum *target,
                     const std::vector<int> &joint_indices,
                     const std::vector<double> &values,
                     const std::string &name) {
  if (values.size() != kArmDof) {
    throw std::runtime_error(name + " must contain exactly six values");
  }
  for (std::size_t joint = 0; joint < kArmDof; ++joint) {
    const int dof = model->jnt_dofadr[joint_indices[joint]];
    target[dof] = values[joint];
  }
}

/** Project selected constraint-space forces into generalized coordinates. */
std::array<std::vector<mjtNum>, kCategoryCount>
decomposeConstraintForce(const mjModel *model, const mjData *data) {
  std::array<std::vector<mjtNum>, kCategoryCount> generalized;
  std::array<std::vector<mjtNum>, kCategoryCount> row_force;
  for (std::size_t category = 0; category < kCategoryCount; ++category) {
    generalized[category].assign(static_cast<std::size_t>(model->nv), 0.0);
    row_force[category].assign(static_cast<std::size_t>(data->nefc), 0.0);
  }

  for (int row = 0; row < data->nefc; ++row) {
    const std::size_t category = static_cast<std::size_t>(categoryForType(data->efc_type[row]));
    row_force[category][static_cast<std::size_t>(row)] = data->efc_force[row];
  }
  for (std::size_t category = 0; category < kCategoryCount; ++category) {
    mj_mulJacTVec(model, data, generalized[category].data(), row_force[category].data());
  }
  return generalized;
}

/** Compute one solver row's generalized-force contribution on a chosen DOF. */
double rowContribution(const mjModel *model, const mjData *data, int row, int dof) {
  std::vector<mjtNum> row_force(static_cast<std::size_t>(data->nefc), 0.0);
  std::vector<mjtNum> generalized(static_cast<std::size_t>(model->nv), 0.0);
  row_force[static_cast<std::size_t>(row)] = data->efc_force[row];
  mj_mulJacTVec(model, data, generalized.data(), row_force.data());
  return generalized[static_cast<std::size_t>(dof)];
}

/** Capture the active solver rows and force split for one J6 representative state. */
void updateRepresentative(RepresentativeSample &sample, double time,
                          double q6, double qd6, double qdd6,
                          double frictionloss6, double total6,
                          const std::array<std::vector<mjtNum>, kCategoryCount> &split,
                          const mjModel *model, const mjData *data, int joint6_dof) {
  const double distance = std::abs(std::abs(qd6) - sample.target_speed);
  if (distance >= sample.distance) {
    return;
  }
  sample.distance = distance;
  sample.time = time;
  sample.q6 = q6;
  sample.qd6 = qd6;
  sample.qdd6 = qdd6;
  sample.frictionloss6 = frictionloss6;
  sample.total6 = total6;
  for (std::size_t category = 0; category < kCategoryCount; ++category) {
    sample.category6[category] = split[category][static_cast<std::size_t>(joint6_dof)];
  }
  sample.expected_hard_coulomb =
      qd6 >= 0.0 ? -frictionloss6 : frictionloss6;
  sample.rows.clear();
  for (int row = 0; row < data->nefc; ++row) {
    sample.rows.push_back(RowRecord{data->efc_type[row], data->efc_id[row],
                                    data->efc_state[row], data->efc_force[row],
                                    rowContribution(model, data, row, joint6_dof)});
  }
}

} // namespace

/**
 * Reproduce one deterministic reBot excitation rollout and decompose every
 * MuJoCo constraint force by solver-row type.
 */
int main(int argc, char **argv) {
  try {
    std::uint32_t seed = 20260826U;
    if (argc > 2) {
      std::cerr << "Usage: rebot_constraint_force_diagnostic [trajectory_seed]\n";
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
                   nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model) {
      throw std::runtime_error(std::string("failed to load runtime scene: ") + error);
    }
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()),
                                                           mj_deleteData);
    if (!data) {
      throw std::runtime_error("failed to allocate MuJoCo data");
    }
    model->opt.timestep = 1.0 / sim_config.simulation_rate_hz;

    const int keyframe = mj_name2id(model.get(), mjOBJ_KEY,
                                    sim_config.initial_keyframe.c_str());
    if (keyframe < 0) {
      throw std::runtime_error("runtime_home keyframe is missing");
    }
    mj_resetDataKeyframe(model.get(), data.get(), keyframe);

    std::vector<int> joint_indices;
    for (int joint = 0; joint < model->njnt; ++joint) {
      if (model->jnt_type[joint] != mjJNT_FREE && model->jnt_type[joint] != mjJNT_BALL) {
        joint_indices.push_back(joint);
      }
    }
    if (joint_indices.size() < kArmDof || model->nu != static_cast<int>(kArmDof)) {
      throw std::runtime_error("runtime scene does not expose six controlled arm joints");
    }

    applyJointTruth(model.get(), model->dof_armature, joint_indices,
                    sim_config.joint_armature, "joint_armature");
    applyJointTruth(model.get(), model->dof_damping, joint_indices,
                    sim_config.joint_damping, "joint_damping");
    applyJointTruth(model.get(), model->dof_frictionloss, joint_indices,
                    sim_config.joint_frictionloss, "joint_frictionloss");

    std::array<int, kArmDof> arm_dof{};
    std::array<int, kArmDof> arm_qpos{};
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      arm_dof[joint] = model->jnt_dofadr[joint_indices[joint]];
      arm_qpos[joint] = model->jnt_qposadr[joint_indices[joint]];
    }

    std::array<std::array<double, kCategoryCount>, kArmDof> max_abs_category{};
    std::array<double, kArmDof> max_reconstruction_error{};
    std::array<double, kArmDof> max_friction_bound_violation{};
    std::array<double, kArmDof> max_sliding_direction_violation{};
    std::array<std::uint64_t, kArmDof> sliding_samples{};
    std::array<std::uint64_t, kArmDof> saturated_sliding_samples{};
    std::array<std::array<std::uint64_t, 5>, kArmDof> friction_state_counts{};
    std::array<RepresentativeSample, 4> representative{};
    const std::array<double, 4> target_speeds{0.05, 0.06, 0.07, 0.08};
    for (std::size_t index = 0; index < representative.size(); ++index) {
      representative[index].target_speed = target_speeds[index];
    }

    double simulation_time = 0.0;
    std::size_t samples = 0;
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

      const auto q_before = sample.position;
      const auto qd_before = sample.velocity;
      mj_step(model.get(), data.get());
      const auto split = decomposeConstraintForce(model.get(), data.get());

      for (std::size_t joint = 0; joint < kArmDof; ++joint) {
        double reconstructed = 0.0;
        for (std::size_t category = 0; category < kCategoryCount; ++category) {
          const double contribution = split[category][static_cast<std::size_t>(arm_dof[joint])];
          reconstructed += contribution;
          max_abs_category[joint][category] =
              std::max(max_abs_category[joint][category], std::abs(contribution));
        }
        max_reconstruction_error[joint] = std::max(
            max_reconstruction_error[joint],
            std::abs(reconstructed - data->qfrc_constraint[arm_dof[joint]]));

        const double friction = split[static_cast<std::size_t>(ConstraintCategory::FRICTION)]
                                     [static_cast<std::size_t>(arm_dof[joint])];
        const double fc = sim_config.joint_frictionloss[joint];
        max_friction_bound_violation[joint] = std::max(
            max_friction_bound_violation[joint], std::max(0.0, std::abs(friction) - fc));
        if (std::abs(qd_before[joint]) >= 0.05) {
          ++sliding_samples[joint];
          const double force_along_motion =
              friction * (qd_before[joint] > 0.0 ? 1.0 : -1.0);
          max_sliding_direction_violation[joint] = std::max(
              max_sliding_direction_violation[joint],
              std::max(0.0, force_along_motion));
          if (std::abs(std::abs(friction) - fc) <= 1e-10) {
            ++saturated_sliding_samples[joint];
          }
        }
      }

      for (int row = 0; row < data->nefc; ++row) {
        if (data->efc_type[row] == mjCNSTR_FRICTION_DOF) {
          const int dof = data->efc_id[row];
          for (std::size_t joint = 0; joint < kArmDof; ++joint) {
            if (dof == arm_dof[joint] && data->efc_state[row] >= 0 && data->efc_state[row] < 5) {
              ++friction_state_counts[joint][static_cast<std::size_t>(data->efc_state[row])];
            }
          }
        }
      }

      for (auto &target : representative) {
        updateRepresentative(
            target, simulation_time, q_before[5], qd_before[5], data->qacc[arm_dof[5]],
            sim_config.joint_frictionloss[5], data->qfrc_constraint[arm_dof[5]],
            split, model.get(), data.get(), arm_dof[5]);
      }

      simulation_time += model->opt.timestep;
      ++samples;
    }

    std::cout << std::scientific << std::setprecision(12);
    std::cout << "seed=" << seed << " samples=" << samples
              << " accepted_scale=" << controller.acceptedTrajectoryScale()
              << " accepted_attempt=" << controller.acceptedTrajectoryAttempt() << "\n";
    std::cout << "category_order=[friction,equality,limit,contact,other]\n";
    double global_reconstruction_error = 0.0;
    double global_nonfriction_contribution = 0.0;
    double global_friction_bound_violation = 0.0;
    double global_sliding_direction_violation = 0.0;
    bool has_sliding_and_saturated_samples = true;
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      global_reconstruction_error = std::max(global_reconstruction_error,
                                             max_reconstruction_error[joint]);
      for (std::size_t category = 1; category < kCategoryCount; ++category) {
        global_nonfriction_contribution = std::max(
            global_nonfriction_contribution, max_abs_category[joint][category]);
      }
      global_friction_bound_violation = std::max(
          global_friction_bound_violation, max_friction_bound_violation[joint]);
      global_sliding_direction_violation = std::max(
          global_sliding_direction_violation,
          max_sliding_direction_violation[joint]);
      has_sliding_and_saturated_samples &=
          sliding_samples[joint] > 0 && saturated_sliding_samples[joint] > 0;
      std::cout << "J" << joint + 1 << " max_abs_contribution=[";
      for (std::size_t category = 0; category < kCategoryCount; ++category) {
        if (category) std::cout << ",";
        std::cout << max_abs_category[joint][category];
      }
      std::cout << "] reconstruction_error=" << max_reconstruction_error[joint]
                << " friction_bound_violation=" << max_friction_bound_violation[joint]
                << " sliding_samples=" << sliding_samples[joint]
                << " saturated_sliding_samples=" << saturated_sliding_samples[joint]
                << " sliding_direction_violation="
                << max_sliding_direction_violation[joint] << "\n";
      std::cout << "J" << joint + 1 << " friction_state_counts"
                << " satisfied=" << friction_state_counts[joint][mjCNSTRSTATE_SATISFIED]
                << " quadratic=" << friction_state_counts[joint][mjCNSTRSTATE_QUADRATIC]
                << " linear_negative=" << friction_state_counts[joint][mjCNSTRSTATE_LINEARNEG]
                << " linear_positive=" << friction_state_counts[joint][mjCNSTRSTATE_LINEARPOS]
                << " cone=" << friction_state_counts[joint][mjCNSTRSTATE_CONE] << "\n";
    }
    std::cout << "global_max_reconstruction_error=" << global_reconstruction_error << "\n";
    std::cout << "global_max_nonfriction_arm_contribution="
              << global_nonfriction_contribution << "\n";
    std::cout << "global_max_friction_bound_violation="
              << global_friction_bound_violation << "\n";
    std::cout << "global_max_sliding_direction_violation="
              << global_sliding_direction_violation << "\n";

    for (const auto &sample : representative) {
      std::cout << "representative target_abs_qd6=" << sample.target_speed
                << " time=" << sample.time << " q6=" << sample.q6
                << " qd6=" << sample.qd6 << " qdd6=" << sample.qdd6
                << " frictionloss6=" << sample.frictionloss6
                << " total6=" << sample.total6
                << " friction6=" << sample.category6[0]
                << " equality6=" << sample.category6[1]
                << " limit6=" << sample.category6[2]
                << " contact6=" << sample.category6[3]
                << " other6=" << sample.category6[4]
                << " expected_hard_coulomb=" << sample.expected_hard_coulomb << "\n";
      for (const auto &row : sample.rows) {
        std::cout << "  row type=" << constraintTypeName(row.type)
                  << "(" << row.type << ") id=" << row.id
                  << " state=" << constraintStateName(row.state)
                  << "(" << row.state << ") efc_force=" << row.force
                  << " J6_contribution=" << row.joint6_contribution << "\n";
      }
    }

    bool passed = true;
    if (global_reconstruction_error > kReconstructionTolerance) {
      std::cerr << "[FAIL] constraint-force decomposition reconstruction\n";
      passed = false;
    }
    if (global_nonfriction_contribution > kReconstructionTolerance) {
      std::cerr << "[FAIL] unexpected equality/limit/contact/other arm contribution\n";
      passed = false;
    }
    if (global_friction_bound_violation > kReconstructionTolerance) {
      std::cerr << "[FAIL] friction force exceeds frictionloss bound\n";
      passed = false;
    }
    if (global_sliding_direction_violation > kReconstructionTolerance) {
      std::cerr << "[FAIL] friction assists motion in the |qd|>=0.05 subset\n";
      passed = false;
    }
    if (!has_sliding_and_saturated_samples) {
      std::cerr << "[FAIL] missing sliding or saturated-sliding observations\n";
      passed = false;
    }
    if (!passed) {
      return 1;
    }
    std::cout << "[PASS] constraint-force decomposition and friction semantics\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "constraint-force diagnostic failed: " << error.what() << "\n";
    return 1;
  }
}
