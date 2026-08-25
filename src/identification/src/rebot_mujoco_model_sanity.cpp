/**
 * @file rebot_mujoco_model_sanity.cpp
 * @brief Validate the Phase 4A canonical reBot-DM MuJoCo model before Pinocchio is available.
 */

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

constexpr std::size_t kArmDof = 6;
constexpr std::size_t kGripperDof = 2;
constexpr double kScalarTolerance = 1e-12;
constexpr double kGripperLockPosition = 0.05;

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

struct ExpectedJoint {
  const char *name;
  std::array<double, 3> axis;
  double lower;
  double upper;
  double effort;
};

constexpr std::array<ExpectedJoint, kArmDof> kExpectedJoints{{
    {"joint1", {0.0, 0.0, 1.0}, -2.8, 2.8, 27.0},
    {"joint2", {0.0, 0.0, -1.0}, -3.14, 0.0, 27.0},
    {"joint3", {0.0, 0.0, 1.0}, -3.14, 0.0, 27.0},
    {"joint4", {0.0, 0.0, 1.0}, -1.87, 1.57, 7.0},
    {"joint5", {0.0, 0.0, 1.0}, -1.57, 1.57, 7.0},
    {"joint6", {0.0, 0.0, 1.0}, -3.14, 3.14, 7.0},
}};

/** Return true when two scalar model values agree to Phase 4A inspection precision. */
bool near(double lhs, double rhs) {
  return std::abs(lhs - rhs) <= kScalarTolerance;
}

/** Load one MuJoCo XML and surface the compiler error as an exception. */
ModelPtr loadModel(const std::filesystem::path &path) {
  char error[1024]{};
  ModelPtr model(mj_loadXML(path.string().c_str(), nullptr, error, sizeof(error)),
                 mj_deleteModel);
  if (!model) {
    throw std::runtime_error("Failed to load " + path.string() + ": " +
                             std::string(error));
  }
  return model;
}

/**
 * Validate J1-J6 ordering, axes, limits, explicit truth, and direct torque actuators.
 *
 * @param model Canonical reBot-DM MuJoCo model.
 * @return True when every arm mapping and actuator property matches the DM source contract.
 */
bool validateArmMapping(const mjModel *model) {
  if (model->nu != static_cast<int>(kArmDof)) {
    std::cerr << "Expected exactly 6 actuators, got " << model->nu << "\n";
    return false;
  }

  bool passed = true;
  std::cout << "joint  mj_id  qpos  dof  axis        lower    upper    effort  actuator\n";
  for (std::size_t index = 0; index < kExpectedJoints.size(); ++index) {
    const ExpectedJoint &expected = kExpectedJoints[index];
    const std::string actuator_name = "actuator" + std::to_string(index + 1);
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, expected.name);
    const int actuator_id =
        mj_name2id(model, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (joint_id < 0 || actuator_id < 0) {
      std::cerr << "Missing " << expected.name << " or " << actuator_name << "\n";
      passed = false;
      continue;
    }

    const int qpos_index = model->jnt_qposadr[joint_id];
    const int dof_index = model->jnt_dofadr[joint_id];
    const double *axis = &model->jnt_axis[3 * joint_id];
    const double lower = model->jnt_range[2 * joint_id];
    const double upper = model->jnt_range[2 * joint_id + 1];
    const double ctrl_lower = model->actuator_ctrlrange[2 * actuator_id];
    const double ctrl_upper = model->actuator_ctrlrange[2 * actuator_id + 1];
    const int transmitted_joint = model->actuator_trnid[2 * actuator_id];
    const double gear = model->actuator_gear[6 * actuator_id];

    std::cout << std::setw(6) << expected.name << std::setw(7) << joint_id
              << std::setw(6) << qpos_index << std::setw(5) << dof_index << "  ["
              << axis[0] << " " << axis[1] << " " << axis[2] << "]  "
              << std::setw(8) << lower << std::setw(9) << upper
              << std::setw(8) << expected.effort << std::setw(10) << actuator_id
              << "\n";

    const bool axis_ok = near(axis[0], expected.axis[0]) &&
                         near(axis[1], expected.axis[1]) &&
                         near(axis[2], expected.axis[2]);
    const bool limits_ok = near(lower, expected.lower) && near(upper, expected.upper);
    const bool actuator_ok = actuator_id == static_cast<int>(index) &&
                             transmitted_joint == joint_id && near(gear, 1.0) &&
                             near(ctrl_lower, -expected.effort) &&
                             near(ctrl_upper, expected.effort);
    const bool truth_ok = near(model->dof_armature[dof_index], 0.0) &&
                          near(model->dof_damping[dof_index], 0.0) &&
                          near(model->dof_frictionloss[dof_index], 0.0);
    passed = axis_ok && limits_ok && actuator_ok && truth_ok && passed;
  }
  return passed;
}

/** Verify unit-gear motor controls produce identical generalized actuator torque. */
bool validateDirectTorqueSemantics(const mjModel *model) {
  DataPtr data(mj_makeData(model), mj_deleteData);
  if (!data) {
    throw std::runtime_error("Failed to allocate MuJoCo data");
  }

  constexpr std::array<double, kArmDof> commands{{1.0, -1.2, 0.8, -0.5, 0.4, -0.3}};
  for (std::size_t index = 0; index < commands.size(); ++index) {
    data->ctrl[index] = commands[index];
  }
  mj_forward(model, data.get());

  double max_error = 0.0;
  for (std::size_t index = 0; index < kExpectedJoints.size(); ++index) {
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, kExpectedJoints[index].name);
    const int dof_index = model->jnt_dofadr[joint_id];
    max_error = std::max(max_error,
                         std::abs(data->qfrc_actuator[dof_index] - commands[index]));
  }
  std::cout << "max |qfrc_actuator - ctrl| = " << std::scientific
            << std::setprecision(12) << max_error << " Nm\n";
  return max_error <= kScalarTolerance;
}

/** Validate that the identification scene explicitly locks both gripper DOFs at 0.05 m. */
bool validateGripperLock(const std::filesystem::path &scene_path) {
  ModelPtr model = loadModel(scene_path);
  DataPtr data(mj_makeData(model.get()), mj_deleteData);
  if (!data) {
    throw std::runtime_error("Failed to allocate identification scene data");
  }
  const int key_id = mj_name2id(model.get(), mjOBJ_KEY, "identification_home");
  if (key_id < 0) {
    std::cerr << "Missing identification_home keyframe\n";
    return false;
  }
  mj_resetDataKeyframe(model.get(), data.get(), key_id);
  mj_forward(model.get(), data.get());

  bool passed = model->neq == static_cast<int>(kGripperDof);
  for (const char *joint_name : {"gripper_joint1", "gripper_joint2"}) {
    const int joint_id = mj_name2id(model.get(), mjOBJ_JOINT, joint_name);
    if (joint_id < 0) {
      std::cerr << "Missing " << joint_name << "\n";
      return false;
    }
    const int qpos_index = model->jnt_qposadr[joint_id];
    passed = near(data->qpos[qpos_index], kGripperLockPosition) && passed;
  }
  std::cout << "gripper_lock_position = [" << kGripperLockPosition << ", "
            << kGripperLockPosition << "] m, equality_count = " << model->neq
            << "\n";
  return passed;
}

/**
 * Check one candidate gripper opening using the external source MJCF geometry.
 *
 * This optional diagnostic is intentionally not part of ctest because the
 * source asset path lives outside this repository. It verifies only contacts
 * among gripper_link, gripper_left, and gripper_right at a safe arm pose.
 */
bool validateSourceGripperCollision(const std::filesystem::path &model_path,
                                   double lock_position) {
  ModelPtr model = loadModel(model_path);
  DataPtr data(mj_makeData(model.get()), mj_deleteData);
  if (!data) {
    throw std::runtime_error("Failed to allocate source-model MuJoCo data");
  }
  constexpr std::array<double, kArmDof> safe_arm{{0.0, -1.0, -1.0, 0.0, 0.0, 0.0}};
  for (std::size_t index = 0; index < kArmDof; ++index) {
    const std::string name = "joint" + std::to_string(index + 1);
    const int joint_id = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
    if (joint_id < 0) {
      throw std::runtime_error("Source model is missing " + name);
    }
    data->qpos[model->jnt_qposadr[joint_id]] = safe_arm[index];
  }
  for (const char *name : {"gripper_joint1", "gripper_joint2"}) {
    const int joint_id = mj_name2id(model.get(), mjOBJ_JOINT, name);
    if (joint_id < 0) {
      throw std::runtime_error(std::string("Source model is missing ") + name);
    }
    data->qpos[model->jnt_qposadr[joint_id]] = lock_position;
  }
  mj_forward(model.get(), data.get());

  const int gripper_link = mj_name2id(model.get(), mjOBJ_BODY, "gripper_link");
  const int gripper_left = mj_name2id(model.get(), mjOBJ_BODY, "gripper_left");
  const int gripper_right = mj_name2id(model.get(), mjOBJ_BODY, "gripper_right");
  if (gripper_link < 0 || gripper_left < 0 || gripper_right < 0) {
    throw std::runtime_error("Source model is missing gripper bodies");
  }
  const auto is_gripper_body = [&](int body_id) {
    return body_id == gripper_link || body_id == gripper_left ||
           body_id == gripper_right;
  };

  int gripper_self_contacts = 0;
  int left_right_contacts = 0;
  int left_link_contacts = 0;
  int right_link_contacts = 0;
  for (int contact_index = 0; contact_index < data->ncon; ++contact_index) {
    const mjContact &contact = data->contact[contact_index];
    const int body1 = model->geom_bodyid[contact.geom[0]];
    const int body2 = model->geom_bodyid[contact.geom[1]];
    if (body1 != body2 && is_gripper_body(body1) && is_gripper_body(body2)) {
      ++gripper_self_contacts;
      const auto is_pair = [body1, body2](int lhs, int rhs) {
        return (body1 == lhs && body2 == rhs) ||
               (body1 == rhs && body2 == lhs);
      };
      left_right_contacts += is_pair(gripper_left, gripper_right) ? 1 : 0;
      left_link_contacts += is_pair(gripper_left, gripper_link) ? 1 : 0;
      right_link_contacts += is_pair(gripper_right, gripper_link) ? 1 : 0;
      const char *body1_name = mj_id2name(model.get(), mjOBJ_BODY, body1);
      const char *body2_name = mj_id2name(model.get(), mjOBJ_BODY, body2);
      std::cout << "source gripper contact: "
                << (body1_name != nullptr ? body1_name : "<unnamed>") << " <-> "
                << (body2_name != nullptr ? body2_name : "<unnamed>") << "\n";
    }
  }
  std::cout << "source geometry gripper contacts at [" << lock_position << ", "
            << lock_position << "]: left<->right=" << left_right_contacts
            << " left<->link=" << left_link_contacts
            << " right<->link=" << right_link_contacts
            << " total=" << gripper_self_contacts << "\n";
  return gripper_self_contacts == 0;
}

} // namespace

int main(int argc, char **argv) {
  try {
    const std::filesystem::path root(PROJECT_ROOT_DIR);
    const std::filesystem::path model_path = root / "rebot_dm" / "rebot_dm.xml";
    const std::filesystem::path scene_path =
        root / "rebot_dm" / "scene_identification.xml";

    ModelPtr model = loadModel(model_path);
    std::cout << "reBot-DM MuJoCo dimensions: nq=" << model->nq
              << " nv=" << model->nv << " nu=" << model->nu << "\n";
    bool passed = validateArmMapping(model.get());
    passed = validateDirectTorqueSemantics(model.get()) && passed;
    passed = validateGripperLock(scene_path) && passed;
    if (argc == 4 && std::string(argv[1]) == "--source-gripper-check") {
      passed = validateSourceGripperCollision(argv[2], std::stod(argv[3])) && passed;
    } else if (argc != 1) {
      throw std::invalid_argument(
          "Usage: rebot_mujoco_model_sanity [--source-gripper-check <source.xml> <lock_position>]");
    }

    std::cout << (passed ? "[PASS] reBot-DM MuJoCo model sanity\n"
                        : "[FAIL] reBot-DM MuJoCo model sanity\n");
    return passed ? 0 : 1;
  } catch (const std::exception &exception) {
    std::cerr << "rebot_mujoco_model_sanity failed: " << exception.what() << "\n";
    return 1;
  }
}
