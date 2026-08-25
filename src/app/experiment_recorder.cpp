#include "app/experiment_recorder.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace app {

ExperimentRecorder::ExperimentRecorder(const std::filesystem::path &output_csv,
                                       std::size_t dof,
                                       bool simulation_truth_schema)
    : output_csv_(output_csv), dof_(dof),
      simulation_truth_schema_(simulation_truth_schema),
      previous_velocity_(dof, 0.0) {
  if (output_csv.has_parent_path()) {
    fs::create_directories(output_csv.parent_path());
  }
  file_.open(output_csv);
  if (!file_) {
    throw std::runtime_error("无法创建实验记录文件: " + output_csv.string());
  }

  if (simulation_truth_schema_) {
    writeSimulationHeader();
  } else {
    writeLegacyHeader();
  }
}

void ExperimentRecorder::writeSimulationHeader() {
  file_ << "time_begin,time_end";
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",q" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",qd" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",qdd_mujoco" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",qdd_diff" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",tau_cmd" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",tau_effort" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",tau_constraint" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",q_next" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",qd_next" << i;
  }
  file_ << ",saturated,contact_count\n";
}

void ExperimentRecorder::writeLegacyHeader() {
  file_ << "time";
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",q" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",qd" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",qdd" << i;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    file_ << ",tau" << i;
  }
  file_ << "\n";
}

ExperimentRecorder::~ExperimentRecorder() {
  if (file_.is_open()) {
    file_.close();
  }
}

void ExperimentRecorder::record(double time, const ExperimentState &state,
                                const force_node::ControlCommand &command) {
  if (!file_) {
    return;
  }

  if (simulation_truth_schema_) {
    recordSimulation(time, state, command);
  } else {
    recordLegacy(time, state, command);
  }
}

void ExperimentRecorder::recordSimulation(
    double time, const ExperimentState &state,
    const force_node::ControlCommand &command) {
  if (!state.simulation_truth) {
    throw std::runtime_error(
        "仿真真值 CSV 缺少当前积分步的 MuJoCo truth");
  }
  const auto &truth = *state.simulation_truth;
  const auto require_dof = [this](const std::vector<double> &values,
                                  const std::string &name) {
    if (values.size() < dof_) {
      throw std::runtime_error(name + " 维度不足，无法写入仿真真值 CSV");
    }
  };
  require_dof(truth.position, "q");
  require_dof(truth.velocity, "qd");
  require_dof(truth.acceleration, "qdd_mujoco");
  require_dof(truth.actuator_effort, "tau_effort");
  require_dof(truth.constraint_effort, "tau_constraint");
  require_dof(state.position, "q_next");
  require_dof(state.velocity, "qd_next");
  require_dof(command.torque, "tau_cmd");

  const double dt = time - truth.time_begin;
  if (dt <= 0.0) {
    throw std::runtime_error("仿真积分步时间戳非递增");
  }

  std::ostringstream line;
  line << std::setprecision(17) << truth.time_begin << "," << time;
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << truth.position[i];
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << truth.velocity[i];
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << truth.acceleration[i];
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << (state.velocity[i] - truth.velocity[i]) / dt;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << command.torque[i];
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << truth.actuator_effort[i];
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << truth.constraint_effort[i];
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << state.position[i];
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << state.velocity[i];
  }
  line << "," << (command.saturated ? 1 : 0) << ","
       << truth.contact_count << "\n";
  file_ << line.str();
}

void ExperimentRecorder::recordLegacy(
    double time, const ExperimentState &state,
    const force_node::ControlCommand &command) {
  if (command.saturated) {
    return;
  }

  std::vector<double> qdd(dof_, 0.0);
  if (has_previous_ && time > previous_time_) {
    const double dt = time - previous_time_;
    for (std::size_t i = 0; i < dof_; ++i) {
      qdd[i] = (state.velocity[i] - previous_velocity_[i]) / dt;
    }
  }

  std::ostringstream line;
  line << std::fixed << std::setprecision(6) << time;
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << state.position[i];
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << state.velocity[i];
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << qdd[i];
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    line << "," << command.torque[i];
  }
  line << "\n";
  file_ << line.str();
  file_.flush();

  previous_time_ = time;
  previous_velocity_ = state.velocity;
  has_previous_ = true;
}

void ExperimentRecorder::writeMetadata(
    const ExperimentMetadata &metadata) const {
  std::filesystem::path metadata_path = output_csv_;
  metadata_path.replace_extension(".meta.yaml");
  std::ofstream metadata_file(metadata_path);
  if (!metadata_file) {
    throw std::runtime_error("无法创建实验 metadata: " +
                             metadata_path.string());
  }
  metadata_file << std::setprecision(17);
  metadata_file << "schema_version: "
                << (simulation_truth_schema_ ? 2 : 1) << "\n";
  metadata_file << "robot: \"" << metadata.robot << "\"\n";
  metadata_file << "backend: \"" << metadata.backend << "\"\n";
  metadata_file << "scene: \"" << metadata.scene.string() << "\"\n";
  metadata_file << "controller_config: \""
                << metadata.controller_config.string() << "\"\n";
  metadata_file << "simulation_config: \""
                << metadata.simulation_config.string() << "\"\n";
  metadata_file << "time_step: " << metadata.time_step << "\n";
  metadata_file << "trajectory_seed: " << metadata.trajectory_seed << "\n";
  metadata_file << "trajectory_harmonics: "
                << metadata.trajectory_harmonics << "\n";
  metadata_file << "trajectory_coefficient_scale: "
                << metadata.trajectory_coefficient_scale << "\n";
  metadata_file << "trajectory_accepted_attempt: "
                << metadata.trajectory_accepted_attempt << "\n";
  metadata_file << "trajectory_replay_file: \""
                << metadata.trajectory_replay_file.string() << "\"\n";
  metadata_file << "trajectory_output_file: \""
                << metadata.trajectory_output_file.string() << "\"\n";
  metadata_file << "trajectory_sha256: \"" << metadata.trajectory_sha256
                << "\"\n";
  metadata_file << "joint_frictionloss: [";
  for (std::size_t joint = 0; joint < metadata.joint_frictionloss.size();
       ++joint) {
    if (joint > 0) {
      metadata_file << ", ";
    }
    metadata_file << metadata.joint_frictionloss[joint];
  }
  metadata_file << "]\n";
  if (simulation_truth_schema_) {
    metadata_file << "q_source: \"mujoco_qpos_pre_integration\"\n";
    metadata_file << "qd_source: \"mujoco_qvel_pre_integration\"\n";
    metadata_file << "qdd_mujoco_source: \"mujoco_qacc_pre_integration\"\n";
    metadata_file << "qdd_diff_source: \"forward_velocity_difference\"\n";
    metadata_file << "tau_cmd_source: \"control_command_torque\"\n";
    metadata_file << "tau_effort_source: \"mujoco_qfrc_actuator\"\n";
    metadata_file << "tau_constraint_source: \"mujoco_qfrc_constraint\"\n";
  } else {
    metadata_file << "data_semantics: \"legacy_ambiguous_schema\"\n";
  }
}

} // namespace app
