#include "app/simulation_backend.hpp"

namespace app {

SimulationBackend::SimulationBackend(const sim_com_node::PandaSimConfig &config,
                                     const std::filesystem::path &scene_path,
                                     std::size_t dof)
    : simulator_(config, scene_path, std::filesystem::path(), dof) {}

ExperimentState SimulationBackend::initialize() {
  return convertState(simulator_.currentState());
}

ExperimentState
SimulationBackend::step(const force_node::ControlCommand &command) {
  const auto simulator_truth =
      simulator_.step(command.torque, command.saturated);
  ExperimentState state = convertState(simulator_.currentState());
  state.simulation_truth = SimulationStepTruth{
      simulator_truth.time_begin,
      simulator_truth.position,
      simulator_truth.velocity,
      simulator_truth.acceleration,
      simulator_truth.actuator_effort,
      simulator_truth.constraint_effort,
      simulator_truth.contact_count};
  return state;
}

double SimulationBackend::simulationTime() const {
  return simulator_.simulationTime();
}

double SimulationBackend::timeStep() const {
  return simulator_.timeStep();
}

ExperimentState
SimulationBackend::convertState(const sim_com_node::JointState &state) {
  return ExperimentState{state.position, state.velocity, state.effort,
                         std::nullopt};
}

} // namespace app
