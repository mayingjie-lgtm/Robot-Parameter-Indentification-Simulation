#ifndef APP_EXPERIMENT_BACKEND_HPP_
#define APP_EXPERIMENT_BACKEND_HPP_

#include "force_node/force_controller.hpp"

#include <optional>
#include <vector>

namespace app {

/** MuJoCo truth associated with the interval ending at the returned state. */
struct SimulationStepTruth {
  double time_begin{0.0};
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  std::vector<double> actuator_effort;
  std::vector<double> constraint_effort;
  int contact_count{0};
};

struct ExperimentState {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::optional<SimulationStepTruth> simulation_truth;
};

class ExperimentBackend {
public:
  virtual ~ExperimentBackend() = default;

  virtual ExperimentState initialize() = 0;
  virtual ExperimentState step(const force_node::ControlCommand &command) = 0;
  virtual double simulationTime() const = 0;
  virtual double timeStep() const = 0;
};

} // namespace app

#endif
