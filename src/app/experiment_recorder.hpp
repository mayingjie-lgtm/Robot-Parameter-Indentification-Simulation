#ifndef APP_EXPERIMENT_RECORDER_HPP_
#define APP_EXPERIMENT_RECORDER_HPP_

#include "app/experiment_backend.hpp"

#include <filesystem>
#include <fstream>
#include <cstdint>
#include <string>
#include <vector>

namespace app {

/** Provenance written next to an experiment CSV. */
struct ExperimentMetadata {
  std::string robot;
  std::string backend;
  std::filesystem::path scene;
  std::filesystem::path controller_config;
  std::filesystem::path simulation_config;
  double time_step{0.0};
  std::uint32_t trajectory_seed{0};
  std::size_t trajectory_harmonics{0};
  double trajectory_coefficient_scale{0.0};
  std::size_t trajectory_accepted_attempt{0};
  std::filesystem::path trajectory_replay_file;
  std::filesystem::path trajectory_output_file;
  std::string trajectory_sha256;
  std::vector<double> joint_frictionloss;
};

class ExperimentRecorder {
public:
  /**
   * Open an experiment CSV using either the simulation-truth or legacy schema.
   *
   * @param output_csv Destination CSV path.
   * @param dof Number of recorded arm joints.
   * @param simulation_truth_schema Whether to require exact MuJoCo step truth.
   */
  ExperimentRecorder(const std::filesystem::path &output_csv, std::size_t dof,
                     bool simulation_truth_schema = false);
  ~ExperimentRecorder();

  /** Write the dataset provenance sidecar without changing CSV contents. */
  void writeMetadata(const ExperimentMetadata &metadata) const;

  /** Record one backend interval with explicit source semantics when available. */
  void record(double time, const ExperimentState &state,
              const force_node::ControlCommand &command);

private:
  void writeSimulationHeader();
  void writeLegacyHeader();
  void recordSimulation(double time, const ExperimentState &state,
                        const force_node::ControlCommand &command);
  void recordLegacy(double time, const ExperimentState &state,
                    const force_node::ControlCommand &command);

  std::ofstream file_;
  std::filesystem::path output_csv_;
  std::size_t dof_{0};
  bool simulation_truth_schema_{false};
  bool has_previous_{false};
  double previous_time_{0.0};
  std::vector<double> previous_velocity_;
};

} // namespace app

#endif
