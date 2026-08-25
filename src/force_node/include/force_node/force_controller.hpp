#ifndef FORCE_NODE_FORCE_CONTROLLER_HPP_
#define FORCE_NODE_FORCE_CONTROLLER_HPP_

#include "robot/mujoco_collision_checker.hpp"
#include "trajectory/fourier_trajectory.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace force_node {

enum class ControllerMode {
  HOLD_POSITION,
  EXCITATION_TRAJECTORY
};

struct ForceControllerConfig {
  double control_rate_hz = 1000.0;
  double trajectory_duration = 30.0;
  std::size_t trajectory_harmonics = 5;
  std::uint32_t trajectory_seed = 20260824U;
  double trajectory_coefficient_scale = 0.04;
  std::filesystem::path trajectory_coefficients_file;
  std::vector<double> kp = {450.0, 450.0, 450.0, 450.0, 180.0, 100.0, 40.0};
  std::vector<double> kd = {35.0, 35.0, 35.0, 35.0, 20.0, 18.0, 10.0};
  std::vector<double> target_position = {0.0, 0.0, 0.0, -1.57079,
                                         0.0, 1.57079, -0.7853};
};

struct JointSample {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
};

struct ControlCommand {
  std::vector<double> torque;
  std::vector<double> desired_position;
  std::vector<double> desired_velocity;
  std::vector<double> kp;
  std::vector<double> kd;
  bool saturated{false};
};

class ForceController {
public:
  explicit ForceController(const ForceControllerConfig &config,
                           const std::filesystem::path &collision_model_path);

  static ForceControllerConfig
  loadConfig(const std::filesystem::path &config_path);

  ControlCommand computeCommand(const JointSample &sample, double current_time);
  std::vector<double> computeTorque(const JointSample &sample,
                                    double current_time);

  std::size_t armDOF() const { return arm_dof_; }
  double controlRateHz() const { return config_.control_rate_hz; }
  bool isTrajectoryFinished() const { return trajectory_finished_; }
  bool isTorqueSaturated() const { return torque_saturated_; }
  double trajectoryDuration() const { return trajectory_duration_; }

  /** Save the accepted or replayed Fourier coefficients to a CSV file. */
  void saveTrajectoryCoefficients(const std::filesystem::path &path) const;

  /** Return the seed used by the deterministic safety search. */
  std::uint32_t trajectorySeed() const { return config_.trajectory_seed; }

  /** Return the configured number of Fourier harmonics. */
  std::size_t trajectoryHarmonics() const {
    return config_.trajectory_harmonics;
  }

  /** Return the accepted one-based search attempt, or zero for replay. */
  std::size_t acceptedTrajectoryAttempt() const {
    return accepted_trajectory_attempt_;
  }

  /** Return the coefficient scale used by the accepted safety-search attempt. */
  double acceptedTrajectoryScale() const {
    return accepted_trajectory_scale_;
  }

  /** Return the configured replay file, empty when coefficients were generated. */
  const std::filesystem::path &trajectoryReplayFile() const {
    return config_.trajectory_coefficients_file;
  }

private:
  bool checkTrajectory(const trajectory::FourierTrajectory &traj);
  void initExcitationTrajectory();

  ForceControllerConfig config_;
  robot::MujocoCollisionChecker collision_checker_;
  std::unique_ptr<trajectory::FourierTrajectory> trajectory_;
  ControllerMode mode_{ControllerMode::EXCITATION_TRAJECTORY};
  std::vector<double> collision_home_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> actuator_limits_;
  std::size_t arm_dof_{0};

  double trajectory_start_time_{0.0};
  double trajectory_duration_{30.0};
  double last_print_time_{0.0};
  bool trajectory_started_{false};
  bool trajectory_finished_{false};
  bool torque_saturated_{false};
  std::size_t total_samples_{0};
  std::size_t saturated_samples_{0};
  std::size_t accepted_trajectory_attempt_{0};
  double accepted_trajectory_scale_{0.0};
};

} // namespace force_node

#endif
