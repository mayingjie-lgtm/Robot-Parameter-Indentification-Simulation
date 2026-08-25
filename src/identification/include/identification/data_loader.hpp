#ifndef IDENTIFICATION_DATA_LOADER_HPP_
#define IDENTIFICATION_DATA_LOADER_HPP_

#include <Eigen/Dense>
#include <string>
#include <vector>

/** Exact CSV column contract used by parameter identification. */
struct DataColumnSelection {
  std::string time_column = "time_begin";
  std::string position_prefix = "q";
  std::string velocity_prefix = "qd";
  std::string acceleration_prefix = "qdd_mujoco";
  std::string torque_prefix = "tau_effort";
  bool require_quality_columns = true;
};

struct ExperimentData {
  std::vector<double> time;
  Eigen::MatrixXd q;              // N x DOF
  Eigen::MatrixXd qd;             // N x DOF
  Eigen::MatrixXd qdd;            // N x DOF, source selected explicitly
  Eigen::MatrixXd tau;            // N x DOF, source selected explicitly
  Eigen::MatrixXd tau_constraint; // N x DOF
  std::vector<int> saturated;
  std::vector<int> contact_count;
  std::size_t n_samples{0};
  std::size_t n_dof{0};
};

class DataLoader {
public:
  /** Load exact named columns and reject ambiguous or incomplete schemas. */
  static ExperimentData loadCSV(const std::string &filename,
                                std::size_t n_dof = 7,
                                const DataColumnSelection &selection = {});
};

#endif // IDENTIFICATION_DATA_LOADER_HPP_
