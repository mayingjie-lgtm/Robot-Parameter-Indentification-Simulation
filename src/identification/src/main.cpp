#include "identification/algorithms.hpp"
#include "identification/data_loader.hpp"
#include "identification/identification.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct IdentificationConfig {
  std::string data_mode = "simulation";
  bool enable_armature_columns = true;
  bool enable_damping_columns = true;
  bool enable_friction_columns = true;
  int algorithm = 1;
  std::string robot = "piper";
  fs::path training_data_file =
      fs::path(PROJECT_ROOT_DIR) / "data/phase3/piper_trajectory_A.csv";
  fs::path validation_data_file =
      fs::path(PROJECT_ROOT_DIR) / "data/phase3/piper_trajectory_B.csv";
  fs::path basis_data_file =
      fs::path(PROJECT_ROOT_DIR) / "data/phase3/piper_trajectory_A.csv";
  fs::path model_file = fs::path(PROJECT_ROOT_DIR) / "piper/piper.xml";
  fs::path output_file =
      fs::path(PROJECT_ROOT_DIR) / "results/piper_phase3_clean.yaml";
  DataColumnSelection columns;
  double constraint_tolerance = 1e-10;
  double rank_relative_tolerance = 1e-6;
  double friction_velocity_threshold = 0.05;
  std::string friction_observation_mode = "moving";
  std::vector<double> joint_armature;
  std::vector<double> joint_damping;
  std::vector<double> joint_frictionloss;
};

struct PreparedData {
  ExperimentData data;
  std::size_t excluded_samples{0};
};

struct BaseParameterSpace {
  Eigen::VectorXd scales;
  Eigen::MatrixXd directions;
  Eigen::VectorXd singular_values;
  std::size_t rank{0};
  double effective_condition{0.0};
};

struct ErrorMetrics {
  double rmse{0.0};
  double mae{0.0};
  double bias{0.0};
  double max_error{0.0};
  double r_squared{0.0};
  std::size_t included_count{0};
};

struct ObservationSelection {
  std::vector<Eigen::Index> rows;
  std::vector<std::size_t> per_joint_counts;
};

struct AggregateError {
  double rmse{0.0};
  double max_error{0.0};
  std::size_t included_count{0};
  std::size_t worst_sample{0};
  std::size_t worst_joint{0};
  double worst_time{0.0};
};

struct RobustDiagnostics {
  int iterations{0};
  double downweighted_fraction{0.0};
  double minimum_weight{1.0};
  double mean_weight{1.0};
};

/** Remove leading and trailing whitespace from a flat YAML token. */
std::string trim(const std::string &value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

/** Remove an unquoted trailing comment from the project's simple config. */
std::string removeComment(const std::string &value) {
  const auto position = value.find('#');
  return position == std::string::npos ? value : value.substr(0, position);
}

/** Parse a string field from the flat identification YAML file. */
bool parseString(const std::string &line, const std::string &key,
                 std::string &out) {
  const std::string stripped = trim(line);
  if (stripped.rfind(key + ":", 0) != 0) {
    return false;
  }
  std::string value = trim(removeComment(stripped.substr(key.size() + 1)));
  if (value.size() >= 2 &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    value = value.substr(1, value.size() - 2);
  }
  out = value;
  return true;
}

/** Parse an integer field from the flat identification YAML file. */
bool parseInt(const std::string &line, const std::string &key, int &out) {
  std::string value;
  if (!parseString(line, key, value)) {
    return false;
  }
  out = std::stoi(value);
  return true;
}

/** Parse a floating-point field from the flat identification YAML file. */
bool parseDouble(const std::string &line, const std::string &key,
                 double &out) {
  std::string value;
  if (!parseString(line, key, value)) {
    return false;
  }
  out = std::stod(value);
  return true;
}

/** Parse a bracketed double array from the flat identification YAML file. */
bool parseDoubleArray(const std::string &line, const std::string &key,
                      std::vector<double> &out) {
  std::string value;
  if (!parseString(line, key, value)) {
    return false;
  }
  if (value.size() < 2 || value.front() != '[' || value.back() != ']') {
    throw std::runtime_error(key + " 必须使用 [v0, ...] 数组格式");
  }
  value = value.substr(1, value.size() - 2);
  std::stringstream stream(value);
  std::string token;
  std::vector<double> parsed;
  while (std::getline(stream, token, ',')) {
    parsed.push_back(std::stod(trim(token)));
  }
  out = std::move(parsed);
  return true;
}

/** Resolve repository-relative paths while preserving absolute inputs. */
fs::path resolveRepoPath(const fs::path &path) {
  return path.empty() || path.is_absolute()
             ? path
             : fs::path(PROJECT_ROOT_DIR) / path;
}

/** Load only the formal A/B identification settings used by this executable. */
IdentificationConfig loadConfig(const fs::path &config_path) {
  IdentificationConfig config;
  std::ifstream file(config_path);
  if (!file) throw std::runtime_error("cannot open identification config: " + config_path.string());
  std::string line;
  while (std::getline(file, line)) {
    std::string value;
    parseInt(line, "algorithm", config.algorithm);
    parseString(line, "data_mode", config.data_mode);
    const auto parse_bool = [&line](const char *key, bool &target) {
      std::string value;
      if (parseString(line, key, value)) {
        if (value != "true" && value != "false")
          throw std::runtime_error(std::string(key) + " must be true or false");
        target = value == "true";
      }
    };
    parse_bool("enable_armature_columns", config.enable_armature_columns);
    parse_bool("enable_damping_columns", config.enable_damping_columns);
    parse_bool("enable_friction_columns", config.enable_friction_columns);
    if (parseString(line, "robot", value)) {
      config.robot = value;
    } else if (parseString(line, "training_data_file", value)) {
      config.training_data_file = value;
    } else if (parseString(line, "validation_data_file", value)) {
      config.validation_data_file = value;
    } else if (parseString(line, "basis_data_file", value)) {
      config.basis_data_file = value;
    } else if (parseString(line, "model_file", value)) {
      config.model_file = value;
    } else if (parseString(line, "output_file", value)) {
      config.output_file = value;
    } else if (parseString(line, "position_prefix", value)) {
      config.columns.position_prefix = value;
    } else if (parseString(line, "velocity_prefix", value)) {
      config.columns.velocity_prefix = value;
    } else if (parseString(line, "acceleration_prefix", value)) {
      config.columns.acceleration_prefix = value;
    } else if (parseString(line, "torque_prefix", value)) {
      config.columns.torque_prefix = value;
    } else if (parseString(line, "friction_observation_mode", value)) {
      config.friction_observation_mode = value;
    }
    parseDouble(line, "constraint_tolerance", config.constraint_tolerance);
    parseDouble(line, "rank_relative_tolerance",
                config.rank_relative_tolerance);
    parseDouble(line, "friction_velocity_threshold",
                config.friction_velocity_threshold);
    parseDoubleArray(line, "joint_armature", config.joint_armature);
    parseDoubleArray(line, "joint_damping", config.joint_damping);
    parseDoubleArray(line, "joint_frictionloss", config.joint_frictionloss);
  }
  config.training_data_file = resolveRepoPath(config.training_data_file);
  config.validation_data_file = resolveRepoPath(config.validation_data_file);
  config.basis_data_file = resolveRepoPath(config.basis_data_file);
  config.model_file = resolveRepoPath(config.model_file);
  config.output_file = resolveRepoPath(config.output_file);
  return config;
}

/** Return a CLI value following a flag, or an empty string when absent. */
std::string readArgument(int argc, char **argv, const std::string &flag) {
  for (int index = 1; index + 1 < argc; ++index) {
    if (argv[index] == flag) {
      return argv[index + 1];
    }
  }
  return "";
}

/** Apply supported command-line overrides without changing config semantics. */
void applyArguments(int argc, char **argv, IdentificationConfig &config) {
  const auto assign_path = [argc, argv](const std::string &flag,
                                        fs::path &target) {
    const std::string value = readArgument(argc, argv, flag);
    if (!value.empty()) {
      target = resolveRepoPath(value);
    }
  };
  assign_path("--training-data-file", config.training_data_file);
  assign_path("--validation-data-file", config.validation_data_file);
  assign_path("--basis-data-file", config.basis_data_file);
  assign_path("--model-file", config.model_file);
  assign_path("--output-file", config.output_file);
  const std::string algorithm = readArgument(argc, argv, "--algorithm");
  if (!algorithm.empty()) {
    config.algorithm = std::stoi(algorithm);
  }
  const std::string robot = readArgument(argc, argv, "--robot");
  if (!robot.empty()) {
    config.robot = robot;
  }
  const auto assign_string = [argc, argv](const std::string &flag,
                                          std::string &target) {
    const std::string value = readArgument(argc, argv, flag);
    if (!value.empty()) {
      target = value;
    }
  };
  assign_string("--position-prefix", config.columns.position_prefix);
  assign_string("--velocity-prefix", config.columns.velocity_prefix);
  assign_string("--acceleration-prefix", config.columns.acceleration_prefix);
  assign_string("--torque-prefix", config.columns.torque_prefix);
}

/** Return the controlled degrees of freedom for a supported robot. */
std::size_t robotDof(const std::string &robot) {
  if (robot == "piper" || robot == "rebot_dm") {
    return 6;
  }
  if (robot == "panda") {
    return 7;
  }
  throw std::runtime_error("robot 仅支持 piper、rebot_dm 或 panda");
}

/** Keep only finite, unsaturated, contact-free samples with valid constraints. */
PreparedData selectValidSamples(const ExperimentData &source,
                                double constraint_tolerance,
                                const std::vector<double> &frictionloss = {},
                                double friction_speed_threshold = 0.05,
                                const std::string &friction_observation_mode =
                                    "moving") {
  std::vector<std::size_t> indices;
  indices.reserve(source.n_samples);
  for (std::size_t sample = 0; sample < source.n_samples; ++sample) {
    const auto row = static_cast<Eigen::Index>(sample);
    const bool finite = source.q.row(row).allFinite() &&
                        source.qd.row(row).allFinite() &&
                        source.qdd.row(row).allFinite() &&
                        source.tau.row(row).allFinite() &&
                        source.tau_constraint.row(row).allFinite() &&
                        std::isfinite(source.time[sample]);
    bool constraint_clean = true;
    for (std::size_t joint = 0; joint < source.n_dof; ++joint) {
      const double constraint = source.tau_constraint(
          row, static_cast<Eigen::Index>(joint));
      if (frictionloss.empty()) {
        constraint_clean = constraint_clean &&
                           std::abs(constraint) <= constraint_tolerance;
      } else {
        const double velocity =
            source.qd(row, static_cast<Eigen::Index>(joint));
        if (friction_observation_mode == "saturated_sliding") {
          constraint_clean =
              constraint_clean &&
              std::abs(constraint) <= frictionloss[joint] +
                                          constraint_tolerance;
          if (std::abs(velocity) >= friction_speed_threshold) {
            const double motion_sign = velocity > 0.0 ? 1.0 : -1.0;
            constraint_clean = constraint_clean &&
                               constraint * motion_sign <= constraint_tolerance;
          }
        } else if (std::abs(velocity) >= friction_speed_threshold) {
          const double expected =
              velocity > 0.0 ? -frictionloss[joint] : frictionloss[joint];
          constraint_clean =
              constraint_clean &&
              std::abs(constraint - expected) <= constraint_tolerance;
        } else {
          constraint_clean =
              constraint_clean &&
              std::abs(constraint) <= frictionloss[joint] +
                                          constraint_tolerance;
        }
      }
    }
    const bool clean = source.saturated[sample] == 0 &&
                       source.contact_count[sample] == 0 && constraint_clean;
    if (finite && clean) {
      indices.push_back(sample);
    }
  }
  if (indices.empty()) {
    throw std::runtime_error("显式质量筛选后没有可用样本");
  }

  PreparedData result;
  result.excluded_samples = source.n_samples - indices.size();
  result.data.n_samples = indices.size();
  result.data.n_dof = source.n_dof;
  result.data.q.resize(indices.size(), source.n_dof);
  result.data.qd.resize(indices.size(), source.n_dof);
  result.data.qdd.resize(indices.size(), source.n_dof);
  result.data.tau.resize(indices.size(), source.n_dof);
  result.data.tau_constraint.resize(indices.size(), source.n_dof);
  result.data.saturated.assign(indices.size(), 0);
  result.data.contact_count.assign(indices.size(), 0);
  result.data.time.reserve(indices.size());
  for (std::size_t output = 0; output < indices.size(); ++output) {
    const auto input_row = static_cast<Eigen::Index>(indices[output]);
    const auto output_row = static_cast<Eigen::Index>(output);
    result.data.time.push_back(source.time[indices[output]]);
    result.data.q.row(output_row) = source.q.row(input_row);
    result.data.qd.row(output_row) = source.qd.row(input_row);
    result.data.qdd.row(output_row) = source.qdd.row(input_row);
    result.data.tau.row(output_row) = source.tau.row(input_row);
    result.data.tau_constraint.row(output_row) =
        source.tau_constraint.row(input_row);
  }
  return result;
}

/** Stack torque samples in the same sample-major order as the regressor. */
Eigen::VectorXd flattenTorque(const ExperimentData &data) {
  Eigen::VectorXd result(data.n_samples * data.n_dof);
  for (std::size_t sample = 0; sample < data.n_samples; ++sample) {
    for (std::size_t joint = 0; joint < data.n_dof; ++joint) {
      result(static_cast<Eigen::Index>(sample * data.n_dof + joint)) =
          data.tau(static_cast<Eigen::Index>(sample),
                   static_cast<Eigen::Index>(joint));
    }
  }
  return result;
}

/** Select observation rows using the configured hard-friction semantics. */
ObservationSelection selectObservationRows(
    const ExperimentData &data, const std::vector<double> &frictionloss,
    double minimum_speed, double constraint_tolerance,
    const std::string &mode) {
  ObservationSelection selection;
  selection.rows.reserve(data.n_samples * data.n_dof);
  selection.per_joint_counts.assign(data.n_dof, 0);
  for (std::size_t sample = 0; sample < data.n_samples; ++sample) {
    for (std::size_t joint = 0; joint < data.n_dof; ++joint) {
      const auto row = static_cast<Eigen::Index>(sample);
      const auto column = static_cast<Eigen::Index>(joint);
      const double velocity = data.qd(row, column);
      bool included = frictionloss.empty();
      if (!frictionloss.empty() && std::abs(velocity) >= minimum_speed) {
        if (mode == "moving") {
          included = true;
        } else if (mode == "saturated_sliding") {
          const double constraint = data.tau_constraint(row, column);
          const double motion_sign = velocity > 0.0 ? 1.0 : -1.0;
          included =
              std::abs(std::abs(constraint) - frictionloss[joint]) <=
                  constraint_tolerance &&
              constraint * motion_sign <= constraint_tolerance;
        } else {
          throw std::runtime_error("未知 friction_observation_mode: " + mode);
        }
      }
      if (included) {
        selection.rows.push_back(static_cast<Eigen::Index>(sample * data.n_dof +
                                                           joint));
        ++selection.per_joint_counts[joint];
      }
    }
  }
  if (selection.rows.empty()) {
    throw std::runtime_error("摩擦 observation policy 筛选后没有 observation row");
  }
  return selection;
}

/** Copy selected observation rows while preserving their original order. */
Eigen::MatrixXd takeRows(const Eigen::MatrixXd &matrix,
                         const std::vector<Eigen::Index> &rows) {
  Eigen::MatrixXd result(rows.size(), matrix.cols());
  for (std::size_t output = 0; output < rows.size(); ++output) {
    result.row(static_cast<Eigen::Index>(output)) = matrix.row(rows[output]);
  }
  return result;
}

/** Copy selected torque observations while preserving sample-joint mapping. */
Eigen::VectorXd takeRows(const Eigen::VectorXd &vector,
                         const std::vector<Eigen::Index> &rows) {
  Eigen::VectorXd result(rows.size());
  for (std::size_t output = 0; output < rows.size(); ++output) {
    result(static_cast<Eigen::Index>(output)) = vector(rows[output]);
  }
  return result;
}

/** Compute a fixed scaled SVD basis from clean trajectory A. */
BaseParameterSpace computeBaseSpace(const Eigen::MatrixXd &observation,
                                    double relative_tolerance) {
  BaseParameterSpace space;
  space.scales.resize(observation.cols());
  Eigen::MatrixXd scaled = observation;
  for (Eigen::Index column = 0; column < observation.cols(); ++column) {
    double scale = observation.col(column).norm();
    if (!(scale > std::numeric_limits<double>::epsilon())) {
      scale = 1.0;
    }
    space.scales(column) = scale;
    scaled.col(column) /= scale;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(scaled, Eigen::ComputeThinV);
  space.singular_values = svd.singularValues();
  const double threshold =
      relative_tolerance * space.singular_values(0);
  while (space.rank < static_cast<std::size_t>(space.singular_values.size()) &&
         space.singular_values(static_cast<Eigen::Index>(space.rank)) >
             threshold) {
    ++space.rank;
  }
  if (space.rank == 0) {
    throw std::runtime_error("训练回归矩阵数值秩为零");
  }
  space.directions =
      svd.matrixV().leftCols(static_cast<Eigen::Index>(space.rank));
  space.effective_condition =
      space.singular_values(0) /
      space.singular_values(static_cast<Eigen::Index>(space.rank - 1));
  return space;
}

/** Project a full observation matrix into the fixed base-parameter space. */
Eigen::MatrixXd baseObservation(const Eigen::MatrixXd &observation,
                                const BaseParameterSpace &space) {
  Eigen::MatrixXd scaled = observation;
  for (Eigen::Index column = 0; column < observation.cols(); ++column) {
    scaled.col(column) /= space.scales(column);
  }
  return scaled * space.directions;
}

/** Compute torque error statistics for one joint on selected rows only. */
ErrorMetrics jointMetrics(const Eigen::VectorXd &measured,
                          const Eigen::VectorXd &predicted,
                          const ObservationSelection &selection,
                          std::size_t n_dof, std::size_t joint) {
  ErrorMetrics metrics;
  double sum_squared = 0.0;
  double sum_absolute = 0.0;
  double sum_error = 0.0;
  double measured_mean = 0.0;
  for (const Eigen::Index index : selection.rows) {
    if (static_cast<std::size_t>(index) % n_dof != joint) {
      continue;
    }
    measured_mean += measured(index);
    ++metrics.included_count;
  }
  if (metrics.included_count == 0) {
    throw std::runtime_error("逐关节误差统计没有被 observation policy 选中的样本");
  }
  measured_mean /= static_cast<double>(metrics.included_count);
  double total_variance = 0.0;
  for (const Eigen::Index index : selection.rows) {
    if (static_cast<std::size_t>(index) % n_dof != joint) {
      continue;
    }
    const double error = predicted(index) - measured(index);
    sum_squared += error * error;
    sum_absolute += std::abs(error);
    sum_error += error;
    metrics.max_error = std::max(metrics.max_error, std::abs(error));
    const double centered = measured(index) - measured_mean;
    total_variance += centered * centered;
  }
  metrics.rmse =
      std::sqrt(sum_squared / static_cast<double>(metrics.included_count));
  metrics.mae = sum_absolute / static_cast<double>(metrics.included_count);
  metrics.bias = sum_error / static_cast<double>(metrics.included_count);
  metrics.r_squared = total_variance > 0.0
                          ? 1.0 - sum_squared / total_variance
                          : (sum_squared == 0.0 ? 1.0 : 0.0);
  return metrics;
}

/** Compute aggregate selected-row error and retain the worst sample location. */
AggregateError aggregateError(const Eigen::VectorXd &measured,
                              const Eigen::VectorXd &predicted,
                              const ExperimentData &data,
                              const ObservationSelection &selection) {
  AggregateError result;
  double sum_squared = 0.0;
  for (const Eigen::Index index : selection.rows) {
    const double error = predicted(index) - measured(index);
    const double absolute_error = std::abs(error);
    sum_squared += error * error;
    if (absolute_error > result.max_error) {
      result.max_error = absolute_error;
      result.worst_sample = static_cast<std::size_t>(index) / data.n_dof;
      result.worst_joint = static_cast<std::size_t>(index) % data.n_dof;
      result.worst_time = data.time[result.worst_sample];
    }
  }
  result.included_count = selection.rows.size();
  result.rmse = std::sqrt(sum_squared / static_cast<double>(result.included_count));
  return result;
}

/** Compute aggregate error when vectors are already packed in selection order. */
AggregateError aggregateSelectedError(const Eigen::VectorXd &measured,
                                      const Eigen::VectorXd &predicted,
                                      const ExperimentData &data,
                                      const ObservationSelection &selection) {
  if (measured.size() != static_cast<Eigen::Index>(selection.rows.size()) ||
      predicted.size() != measured.size()) {
    throw std::runtime_error("selected error vectors do not match row selection");
  }
  AggregateError result;
  double sum_squared = 0.0;
  for (std::size_t selected = 0; selected < selection.rows.size(); ++selected) {
    const double error = predicted(static_cast<Eigen::Index>(selected)) -
                         measured(static_cast<Eigen::Index>(selected));
    const double absolute_error = std::abs(error);
    sum_squared += error * error;
    if (absolute_error > result.max_error) {
      result.max_error = absolute_error;
      const std::size_t original =
          static_cast<std::size_t>(selection.rows[selected]);
      result.worst_sample = original / data.n_dof;
      result.worst_joint = original % data.n_dof;
      result.worst_time = data.time[result.worst_sample];
    }
  }
  result.included_count = selection.rows.size();
  result.rmse = std::sqrt(sum_squared / static_cast<double>(result.included_count));
  return result;
}

/** Return a selection containing every sample-joint observation row. */
ObservationSelection allObservationRows(const ExperimentData &data) {
  return selectObservationRows(data, {}, 0.0, 0.0, "moving");
}

/** Write an Eigen vector as an indented YAML sequence. */
void writeVector(std::ofstream &output, const std::string &name,
                 const Eigen::VectorXd &values) {
  output << name << ":\n";
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    output << "  - " << values(index) << "\n";
  }
}

/** Write an Eigen matrix as a YAML row sequence for basis-direction audit. */
void writeMatrix(std::ofstream &output, const std::string &name,
                 const Eigen::MatrixXd &values) {
  output << name << ":\n";
  for (Eigen::Index row = 0; row < values.rows(); ++row) {
    output << "  - [";
    for (Eigen::Index column = 0; column < values.cols(); ++column) {
      output << values(row, column);
      if (column + 1 < values.cols()) {
        output << ", ";
      }
    }
    output << "]\n";
  }
}

/** Write aggregate error fields beneath an already-emitted YAML mapping key. */
void writeAggregateError(std::ofstream &output, const AggregateError &error,
                         const std::string &indent = "  ") {
  output << indent << "aggregate_rmse: " << error.rmse << "\n";
  output << indent << "global_max_error: " << error.max_error << "\n";
  output << indent << "included_observation_count: " << error.included_count
         << "\n";
  output << indent << "worst_sample: " << error.worst_sample << "\n";
  output << indent << "worst_joint: " << (error.worst_joint + 1) << "\n";
  output << indent << "worst_time: " << error.worst_time << "\n";
}

/** Save validation torques with inclusion matching the validation row policy. */
fs::path savePredictions(const IdentificationConfig &config,
                         const ExperimentData &validation,
                         const Eigen::VectorXd &true_prediction,
                         const Eigen::VectorXd &identified_prediction,
                         const ObservationSelection &selection) {
  fs::path path = config.output_file;
  path.replace_extension(".prediction.csv");
  if (path.has_parent_path()) {
    fs::create_directories(path.parent_path());
  }
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("无法写入力矩预测明细: " + path.string());
  }
  output << "time,joint,tau_simulation,tau_regressor_true,tau_identified,"
            "included\n"
         << std::setprecision(17);
  std::vector<bool> included(validation.n_samples * validation.n_dof, false);
  for (const Eigen::Index index : selection.rows) {
    included[static_cast<std::size_t>(index)] = true;
  }
  for (std::size_t sample = 0; sample < validation.n_samples; ++sample) {
    for (std::size_t joint = 0; joint < validation.n_dof; ++joint) {
      const Eigen::Index index =
          static_cast<Eigen::Index>(sample * validation.n_dof + joint);
      output << validation.time[sample] << "," << (joint + 1) << ","
             << validation.tau(static_cast<Eigen::Index>(sample),
                               static_cast<Eigen::Index>(joint))
             << "," << true_prediction(index) << ","
             << identified_prediction(index) << ","
             << (included[static_cast<std::size_t>(index)] ? 1 : 0) << "\n";
    }
  }
  return path;
}

/** Persist clean A-only identification and independent B validation audit data. */
void saveResults(const IdentificationConfig &config,
                 const PreparedData &training,
                 const PreparedData &validation,
                 const ObservationSelection &training_selection,
                 const ObservationSelection &validation_selection,
                 const BaseParameterSpace &space,
                 const BaseParameterSpace &validation_space,
                 const Eigen::VectorXd &theta_true,
                 const Eigen::VectorXd &beta_true,
                 const Eigen::VectorXd &beta_hat,
                 const Eigen::VectorXd &minimum_norm_parameters,
                 const std::vector<ErrorMetrics> &training_model_metrics,
                 const std::vector<ErrorMetrics> &model_metrics,
                 const std::vector<ErrorMetrics> &estimate_metrics,
                 double beta_relative_error,
                 const RobustDiagnostics &robust,
                 const AggregateError &oracle_a,
                 const AggregateError &oracle_b,
                 const AggregateError &training_residual,
                 const AggregateError &validation_error,
                 const AggregateError &full_validation_diagnostic) {
  if (config.output_file.has_parent_path()) {
    fs::create_directories(config.output_file.parent_path());
  }
  std::ofstream output(config.output_file);
  if (!output) {
    throw std::runtime_error("无法写入辨识结果: " +
                             config.output_file.string());
  }
  output << std::setprecision(17);
  output << "schema_version: 4\n";
  output << "robot: \"" << config.robot << "\"\n";
  output << "algorithm: \"" << (config.algorithm == 3 ? "IRLS" : "OLS")
         << "\"\n";
  output << "training_data_file: \"" << config.training_data_file.string()
         << "\"\n";
  output << "validation_data_file: \""
         << config.validation_data_file.string() << "\"\n";
  output << "basis_data_file: \"" << config.basis_data_file.string()
         << "\"\n";
  output << "model_file: \"" << config.model_file.string() << "\"\n";
  fs::path prediction_file = config.output_file;
  prediction_file.replace_extension(".prediction.csv");
  output << "prediction_file: \"" << prediction_file.string() << "\"\n";
  output << "column_sources:\n";
  output << "  q: \"" << config.columns.position_prefix << "\"\n";
  output << "  qd: \"" << config.columns.velocity_prefix << "\"\n";
  output << "  qdd: \"" << config.columns.acceleration_prefix << "\"\n";
  output << "  tau: \"" << config.columns.torque_prefix << "\"\n";
  output << "training_samples: " << training.data.n_samples << "\n";
  output << "training_excluded_samples: " << training.excluded_samples << "\n";
  output << "validation_samples: " << validation.data.n_samples << "\n";
  output << "validation_excluded_samples: " << validation.excluded_samples
         << "\n";
  output << "training_observation_policy: \""
         << (config.joint_frictionloss.empty() ? "all"
                                               : config.friction_observation_mode)
         << "\"\n";
  output << "validation_observation_policy: \""
         << (config.joint_frictionloss.empty() ? "all"
                                               : config.friction_observation_mode)
         << "\"\n";
  output << "training_observation_count: " << training_selection.rows.size()
         << "\n";
  output << "validation_observation_count: " << validation_selection.rows.size()
         << "\n";
  output << "training_observation_count_per_joint: [";
  for (std::size_t joint = 0; joint < training_selection.per_joint_counts.size();
       ++joint) {
    if (joint > 0) output << ", ";
    output << training_selection.per_joint_counts[joint];
  }
  output << "]\n";
  output << "validation_observation_count_per_joint: [";
  for (std::size_t joint = 0; joint < validation_selection.per_joint_counts.size();
       ++joint) {
    if (joint > 0) output << ", ";
    output << validation_selection.per_joint_counts[joint];
  }
  output << "]\n";
  output << "full_parameter_count: " << space.scales.size() << "\n";
  output << "base_parameter_rank: " << space.rank << "\n";
  output << "rank_relative_tolerance: " << config.rank_relative_tolerance
         << "\n";
  output << "effective_condition_number: " << space.effective_condition
         << "\n";
  output << "validation_rank_diagnostic: " << validation_space.rank << "\n";
  output << "validation_condition_diagnostic: "
         << validation_space.effective_condition << "\n";
  output << "base_parameter_relative_error: " << beta_relative_error << "\n";
  output << "beta_relative_error: " << beta_relative_error << "\n";
  output << "friction_velocity_threshold: "
         << config.friction_velocity_threshold << "\n";
  output << "friction_observation_mode: \"" << config.friction_observation_mode
         << "\"\n";

  output << "oracle_model_error:\n";
  output << "  A:\n";
  writeAggregateError(output, oracle_a, "    ");
  output << "  B:\n";
  writeAggregateError(output, oracle_b, "    ");
  output << "parameter_estimation_error:\n";
  output << "  base_parameter_relative_error: " << beta_relative_error << "\n";
  output << "  training_residual:\n";
  writeAggregateError(output, training_residual, "    ");
  output << "independent_validation_error:\n";
  writeAggregateError(output, validation_error);
  output << "validation_model_valid_subset:\n";
  writeAggregateError(output, validation_error);
  output << "validation_full_forward_diagnostic:\n";
  writeAggregateError(output, full_validation_diagnostic);

  if (!config.joint_frictionloss.empty()) {
    const Eigen::Index count =
        static_cast<Eigen::Index>(config.joint_frictionloss.size());
    const Eigen::VectorXd friction_true = theta_true.tail(count);
    const Eigen::VectorXd friction_hat = minimum_norm_parameters.tail(count);
    output << "frictionloss_relative_error: "
           << (friction_hat - friction_true).norm() / friction_true.norm()
           << "\n";
    writeVector(output, "frictionloss_true", friction_true);
    writeVector(output, "frictionloss_hat", friction_hat);
  }
  output << "robust_diagnostics:\n";
  output << "  iterations: " << robust.iterations << "\n";
  output << "  downweighted_fraction: " << robust.downweighted_fraction
         << "\n";
  output << "  minimum_weight: " << robust.minimum_weight << "\n";
  output << "  mean_weight: " << robust.mean_weight << "\n";
  writeVector(output, "column_scales", space.scales);
  writeVector(output, "singular_values", space.singular_values);
  writeMatrix(output, "base_directions", space.directions);
  writeVector(output, "beta_true", beta_true);
  writeVector(output, "beta_hat", beta_hat);
  writeVector(output, "full_minimum_norm_parameters", minimum_norm_parameters);
  output << "oracle_A_per_joint:\n";
  for (std::size_t joint = 0; joint < training_model_metrics.size(); ++joint) {
    output << "  - joint: " << (joint + 1) << "\n";
    output << "    included_observation_count: "
           << training_model_metrics[joint].included_count << "\n";
    output << "    rmse: " << training_model_metrics[joint].rmse << "\n";
    output << "    max_error: " << training_model_metrics[joint].max_error << "\n";
  }
  output << "validation_per_joint:\n";
  for (std::size_t joint = 0; joint < estimate_metrics.size(); ++joint) {
    output << "  - joint: " << (joint + 1) << "\n";
    output << "    included_observation_count: "
           << estimate_metrics[joint].included_count << "\n";
    output << "    tau_regressor_true_rmse: " << model_metrics[joint].rmse
           << "\n";
    output << "    tau_regressor_true_max_error: "
           << model_metrics[joint].max_error << "\n";
    output << "    tau_identified_rmse: " << estimate_metrics[joint].rmse
           << "\n";
    output << "    tau_identified_mae: " << estimate_metrics[joint].mae
           << "\n";
    output << "    tau_identified_bias: " << estimate_metrics[joint].bias
           << "\n";
    output << "    tau_identified_max_error: "
           << estimate_metrics[joint].max_error << "\n";
    output << "    tau_identified_r_squared: "
           << estimate_metrics[joint].r_squared << "\n";
  }
}

/** Fit reported effort without treating any model parameter as hardware truth. */
int runReportedEffortIdentification(const IdentificationConfig &config) {
  if (config.robot != "rebot_dm" || config.algorithm != 1)
    throw std::runtime_error("real_reported_effort supports rebot_dm OLS(1) only");
  if (!config.joint_armature.empty() || !config.joint_damping.empty() ||
      !config.joint_frictionloss.empty() || config.friction_observation_mode != "moving")
    throw std::runtime_error("real mode rejects simulation truth and saturated_sliding");
  if (!std::isfinite(config.rank_relative_tolerance) || config.rank_relative_tolerance <= 0 ||
      config.rank_relative_tolerance >= 1 || !std::isfinite(config.friction_velocity_threshold) ||
      config.friction_velocity_threshold <= 0)
    throw std::runtime_error("invalid real-mode rank or velocity threshold");
  if (fs::weakly_canonical(config.training_data_file) == fs::weakly_canonical(config.validation_data_file) ||
      fs::weakly_canonical(config.basis_data_file) != fs::weakly_canonical(config.training_data_file))
    throw std::runtime_error("real mode requires independent A/B and basis == A");
  fs::path prediction_path = config.output_file;
  prediction_path.replace_extension(".prediction.csv");
  if (fs::exists(config.output_file) || fs::exists(prediction_path))
    throw std::runtime_error("real-mode outputs already exist; use a new run directory");
  DataColumnSelection columns;
  columns.time_column = "time";
  columns.acceleration_prefix = "qdd_est";
  columns.torque_prefix = "effort_filtered";
  columns.require_quality_columns = false;
  // Raw quality filtering belongs to the offline preprocessor. No fake MuJoCo
  // constraint/contact values participate in this mode's observation selection.
  const ExperimentData a = DataLoader::loadCSV(config.training_data_file.string(), 6, columns);
  const ExperimentData b = DataLoader::loadCSV(config.validation_data_file.string(), 6, columns);
  const auto validate = [](const ExperimentData &data) {
    if (data.n_samples < 2 || !data.q.allFinite() || !data.qd.allFinite() ||
        !data.qdd.allFinite() || !data.tau.allFinite())
      throw std::runtime_error("real mode requires finite preprocessed data");
    for (std::size_t i = 0; i < data.n_samples; ++i)
      if (!std::isfinite(data.time[i]) || (i && data.time[i] <= data.time[i-1]))
        throw std::runtime_error("processed time must be finite and strictly increasing");
  };
  validate(a);
  validate(b);
  const auto select = [&config](const ExperimentData &data) {
    ObservationSelection selection;
    selection.per_joint_counts.assign(6, 0);
    for (std::size_t i = 0; i < data.n_samples; ++i)
      for (std::size_t j = 0; j < 6; ++j)
        if (!config.enable_friction_columns ||
            std::abs(data.qd(i,j)) >= config.friction_velocity_threshold) {
          selection.rows.push_back(static_cast<Eigen::Index>(i*6+j));
          ++selection.per_joint_counts[j];
        }
    if (std::any_of(selection.per_joint_counts.begin(), selection.per_joint_counts.end(),
                    [](std::size_t n) { return n < 2; }))
      throw std::runtime_error("each joint needs at least two valid moving observations");
    return selection;
  };
  const auto sa = select(a);
  const auto sb = select(b);
  using mujoco_dynamics::MuJoCoParamFlags;
  auto flags = MuJoCoParamFlags::NONE;
  if (config.enable_armature_columns) flags = flags | MuJoCoParamFlags::ARMATURE;
  if (config.enable_damping_columns) flags = flags | MuJoCoParamFlags::DAMPING;
  if (config.enable_friction_columns) flags = flags | MuJoCoParamFlags::FRICTION_LOSS;
  rebot_dynamics::ReBotPinocchioRegressor regressor(config.model_file);
  const Eigen::MatrixXd wa = regressor.computeObservationMatrix(a.q.transpose(), a.qd.transpose(), a.qdd.transpose(), flags);
  const auto space = computeBaseSpace(takeRows(wa, sa.rows), config.rank_relative_tolerance);
  if (sa.rows.size() <= space.rank)
    throw std::runtime_error("insufficient A observations for fitted rank");
  auto solver = identification::createAlgorithm("OLS", 6);
  const Eigen::VectorXd ta = flattenTorque(a);
  const Eigen::VectorXd beta = solver->solve(baseObservation(takeRows(wa, sa.rows), space), takeRows(ta, sa.rows));
  const Eigen::VectorXd pa = baseObservation(wa, space) * beta;
  // B is evaluated only in A's frozen coordinates; it never defines a second basis.
  const Eigen::MatrixXd wb = regressor.computeObservationMatrix(b.q.transpose(), b.qd.transpose(), b.qdd.transpose(), flags);
  const Eigen::VectorXd tb = flattenTorque(b);
  const Eigen::VectorXd pb = baseObservation(wb, space) * beta;
  if (!beta.allFinite() || !pa.allFinite() || !pb.allFinite())
    throw std::runtime_error("nonfinite reported-effort fit");
  if (config.output_file.has_parent_path()) fs::create_directories(config.output_file.parent_path());
  std::ofstream output(config.output_file);
  std::ofstream predictions(prediction_path);
  if (!output || !predictions) throw std::runtime_error("cannot create real-mode outputs");
  output << std::setprecision(17)
         << "schema_version: rebot_reported_effort_result_v1\n"
         << "data_mode: real_reported_effort\nrobot: rebot_dm\nalgorithm: OLS\n"
         << "torque_source: reported_effort\ntorque_calibrated: false\n"
         << "physical_parameter_accuracy_claimed: false\n"
         << "training_data_file: " << std::quoted(config.training_data_file.string()) << "\n"
         << "validation_data_file: " << std::quoted(config.validation_data_file.string()) << "\n"
         << "model_file: " << std::quoted(config.model_file.string()) << "\n"
         << "prediction_file: " << std::quoted(prediction_path.string()) << "\n"
         << "full_parameter_count: " << wa.cols() << "\nbase_parameter_rank: " << space.rank << "\n"
         << "rank_relative_tolerance: " << config.rank_relative_tolerance << "\n"
         << "effective_condition_number: " << space.effective_condition << "\n"
         << "friction_velocity_threshold: " << config.friction_velocity_threshold << "\n"
         << "training_error:\n";
  writeAggregateError(output, aggregateError(ta, pa, a, sa));
  output << "validation_error:\n";
  writeAggregateError(output, aggregateError(tb, pb, b, sb));
  writeVector(output, "column_scales", space.scales);
  writeVector(output, "singular_values", space.singular_values);
  writeMatrix(output, "base_directions", space.directions);
  writeVector(output, "beta_hat", beta);
  writeVector(output, "full_minimum_norm_parameters", space.scales.cwiseInverse().asDiagonal() * space.directions * beta);
  output << "parameter_names:\n";
  for (const auto &name : regressor.getParameterNames(flags)) output << "  - " << std::quoted(name) << "\n";
  predictions << std::setprecision(17) << "split,time,joint,effort_filtered,effort_predicted,residual,included\n";
  const auto save_split = [&](const char *name, const ExperimentData &data, const Eigen::VectorXd &target,
                              const Eigen::VectorXd &prediction, const ObservationSelection &selection) {
    output << name << "_per_joint:\n";
    for (std::size_t j = 0; j < 6; ++j) {
      const auto m = jointMetrics(target, prediction, selection, 6, j);
      output << "  - joint: " << j+1 << "\n    count: " << m.included_count
             << "\n    rmse: " << m.rmse << "\n    mae: " << m.mae
             << "\n    bias: " << m.bias << "\n    max_error: " << m.max_error << "\n    r_squared: ";
      if (std::isfinite(m.r_squared)) output << m.r_squared; else output << "null";
      output << "\n";
    }
    std::vector<bool> included(data.n_samples * 6, false);
    for (const auto i : selection.rows) included[i] = true;
    for (std::size_t i = 0; i < data.n_samples; ++i)
      for (std::size_t j = 0; j < 6; ++j) {
        const auto row = static_cast<Eigen::Index>(i*6+j);
        predictions << name << ',' << data.time[i] << ',' << j+1 << ',' << target(row) << ','
                    << prediction(row) << ',' << prediction(row)-target(row) << ',' << included[row] << '\n';
      }
  };
  save_split("training", a, ta, pa, sa);
  save_split("validation", b, tb, pb, sb);
  std::cout << "reported-effort OLS: A rank=" << space.rank << "/" << wa.cols()
            << ", B RMSE=" << aggregateError(tb,pb,b,sb).rmse
            << "; torque_calibrated=false\n";
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  try {
    fs::path config_path =
        fs::path(PROJECT_ROOT_DIR) / "config/identification.yaml";
    const std::string config_argument =
        readArgument(argc, argv, "--config");
    if (!config_argument.empty()) {
      config_path = resolveRepoPath(config_argument);
    }
    IdentificationConfig config = loadConfig(config_path);
    applyArguments(argc, argv, config);
    std::transform(config.robot.begin(), config.robot.end(), config.robot.begin(),
                   [](unsigned char character) {
                     return static_cast<char>(std::tolower(character));
                   });
    if (config.data_mode == "real_reported_effort") return runReportedEffortIdentification(config);
    if (config.data_mode != "simulation") throw std::runtime_error("unknown data_mode");
    const std::size_t dof = robotDof(config.robot);
    if (fs::weakly_canonical(config.training_data_file) ==
        fs::weakly_canonical(config.validation_data_file)) {
      throw std::runtime_error(
          "正式辨识拒绝 training_data_file 与 validation_data_file 相同");
    }
    if (config.algorithm != 1 && config.algorithm != 3) {
      throw std::runtime_error("可信闭环当前只允许 OLS(1) 或 IRLS(3)");
    }
    const bool friction_enabled = !config.joint_frictionloss.empty();
    if (config.friction_observation_mode != "moving" &&
        config.friction_observation_mode != "saturated_sliding") {
      throw std::runtime_error(
          "friction_observation_mode 仅支持 moving 或 saturated_sliding");
    }
    if (config.friction_observation_mode == "saturated_sliding" &&
        (config.robot != "rebot_dm" || !friction_enabled)) {
      throw std::runtime_error(
          "saturated_sliding 仅用于带 friction truth 的 rebot_dm clean closure");
    }
    if (config.friction_observation_mode == "saturated_sliding" &&
        config.algorithm != 1) {
      throw std::runtime_error("reBot clean closure 只允许 OLS(1)");
    }
    if (config.friction_observation_mode == "saturated_sliding" &&
        fs::weakly_canonical(config.basis_data_file) !=
            fs::weakly_canonical(config.training_data_file)) {
      throw std::runtime_error("reBot clean closure 要求 basis_data_file == training_data_file");
    }
    const auto validate_truth_size = [dof](const std::vector<double> &values,
                                           const char *name) {
      if (!values.empty() && values.size() != dof) {
        throw std::runtime_error(std::string(name) +
                                 " 数量必须等于机械臂自由度");
      }
    };
    validate_truth_size(config.joint_armature, "joint_armature");
    validate_truth_size(config.joint_damping, "joint_damping");
    validate_truth_size(config.joint_frictionloss, "joint_frictionloss");

    ExperimentData raw_training = DataLoader::loadCSV(
        config.training_data_file.string(), dof, config.columns);
    const DataColumnSelection clean_validation_columns;
    ExperimentData raw_validation = DataLoader::loadCSV(
        config.validation_data_file.string(), dof, clean_validation_columns);
    const DataColumnSelection clean_basis_columns;
    ExperimentData raw_basis = DataLoader::loadCSV(
        config.basis_data_file.string(), dof, clean_basis_columns);
    PreparedData training =
        selectValidSamples(raw_training, config.constraint_tolerance,
                           config.joint_frictionloss,
                           config.friction_velocity_threshold,
                           config.friction_observation_mode);
    PreparedData validation =
        selectValidSamples(raw_validation, config.constraint_tolerance,
                           config.joint_frictionloss,
                           config.friction_velocity_threshold,
                           config.friction_observation_mode);
    PreparedData basis_data =
        selectValidSamples(raw_basis, config.constraint_tolerance,
                           config.joint_frictionloss,
                           config.friction_velocity_threshold,
                           config.friction_observation_mode);
    std::cout << "Quality filter: A excluded " << training.excluded_samples
              << ", B excluded " << validation.excluded_samples << std::endl;

    Identification identifier(config.robot, config.model_file,
                              config.joint_frictionloss,
                              config.joint_armature, config.joint_damping);
    const auto flags =
        friction_enabled
            ? mujoco_dynamics::MuJoCoParamFlags::ALL_WITH_FRICTION
            : mujoco_dynamics::MuJoCoParamFlags::ALL;
    const Eigen::VectorXd theta_true =
        identifier.getGroundTruthParameters(flags);
    const ObservationSelection basis_selection = selectObservationRows(
        basis_data.data, config.joint_frictionloss,
        config.friction_velocity_threshold, config.constraint_tolerance,
        config.friction_observation_mode);
    const ObservationSelection training_selection = selectObservationRows(
        training.data, config.joint_frictionloss,
        config.friction_velocity_threshold, config.constraint_tolerance,
        config.friction_observation_mode);
    const ObservationSelection validation_selection = selectObservationRows(
        validation.data, config.joint_frictionloss,
        config.friction_velocity_threshold, config.constraint_tolerance,
        config.friction_observation_mode);
    std::cout << "Observation rows A=" << training_selection.rows.size()
              << " B=" << validation_selection.rows.size() << std::endl;
    for (std::size_t joint = 0; joint < dof; ++joint) {
      std::cout << "  J" << (joint + 1)
                << " A=" << training_selection.per_joint_counts[joint]
                << " B=" << validation_selection.per_joint_counts[joint]
                << std::endl;
    }

    BaseParameterSpace space;
    {
      Eigen::MatrixXd basis_observation =
          identifier.computeObservationMatrix(
              basis_data.data.q.transpose(), basis_data.data.qd.transpose(),
              basis_data.data.qdd.transpose(), flags);
      basis_observation = takeRows(basis_observation, basis_selection.rows);
      space = computeBaseSpace(basis_observation,
                               config.rank_relative_tolerance);
    }
    Eigen::MatrixXd training_observation =
        identifier.computeObservationMatrix(
            training.data.q.transpose(), training.data.qd.transpose(),
            training.data.qdd.transpose(), flags);
    const Eigen::VectorXd training_torque_full = flattenTorque(training.data);
    const Eigen::VectorXd training_true_prediction =
        training_observation * theta_true;
    const AggregateError oracle_a = aggregateError(
        training_torque_full, training_true_prediction, training.data,
        training_selection);
    training_observation = takeRows(training_observation, training_selection.rows);
    const Eigen::VectorXd training_torque =
        takeRows(training_torque_full, training_selection.rows);
    const Eigen::MatrixXd training_base =
        baseObservation(training_observation, space);
    const Eigen::VectorXd beta_true =
        space.directions.transpose() * space.scales.asDiagonal() * theta_true;

    auto solver = identification::createAlgorithm(
        config.algorithm == 3 ? "IRLS" : "OLS", static_cast<int>(dof));
    const Eigen::VectorXd beta_hat =
        solver->solve(training_base, training_torque);
    RobustDiagnostics robust;
    if (const auto *irls =
            dynamic_cast<const identification::IRLS *>(solver.get())) {
      robust.iterations = irls->iterations();
      const Eigen::VectorXd &weights = irls->lastWeights();
      if (weights.size() > 0) {
        robust.minimum_weight = weights.minCoeff();
        robust.mean_weight = weights.mean();
        Eigen::Index downweighted = 0;
        for (Eigen::Index index = 0; index < weights.size(); ++index) {
          if (weights(index) < 0.999999) {
            ++downweighted;
          }
        }
        robust.downweighted_fraction =
            static_cast<double>(downweighted) /
            static_cast<double>(weights.size());
      }
    }
    const Eigen::VectorXd minimum_norm_parameters =
        space.scales.cwiseInverse().asDiagonal() * space.directions * beta_hat;
    const double beta_relative_error =
        (beta_hat - beta_true).norm() /
        std::max(beta_true.norm(), std::numeric_limits<double>::epsilon());

    const Eigen::VectorXd training_identified_prediction =
        training_base * beta_hat;
    const AggregateError training_residual = aggregateSelectedError(
        training_torque, training_identified_prediction, training.data,
        training_selection);
    std::vector<ErrorMetrics> training_model_metrics;
    for (std::size_t joint = 0; joint < dof; ++joint) {
      training_model_metrics.push_back(jointMetrics(
          training_torque_full, training_true_prediction, training_selection,
          dof, joint));
    }

    const Eigen::MatrixXd validation_observation =
        identifier.computeObservationMatrix(
            validation.data.q.transpose(), validation.data.qd.transpose(),
            validation.data.qdd.transpose(), flags);
    const Eigen::MatrixXd validation_selected_observation =
        takeRows(validation_observation, validation_selection.rows);
    const BaseParameterSpace validation_space = computeBaseSpace(
        validation_selected_observation, config.rank_relative_tolerance);
    const Eigen::MatrixXd validation_base =
        baseObservation(validation_observation, space);
    const Eigen::VectorXd validation_torque = flattenTorque(validation.data);
    const Eigen::VectorXd true_prediction =
        validation_observation * theta_true;
    const Eigen::VectorXd identified_prediction = validation_base * beta_hat;
    const AggregateError oracle_b = aggregateError(
        validation_torque, true_prediction, validation.data,
        validation_selection);
    const AggregateError validation_error = aggregateError(
        validation_torque, identified_prediction, validation.data,
        validation_selection);
    const AggregateError full_validation_diagnostic = aggregateError(
        validation_torque, true_prediction, validation.data,
        allObservationRows(validation.data));
    std::vector<ErrorMetrics> model_metrics;
    std::vector<ErrorMetrics> estimate_metrics;
    for (std::size_t joint = 0; joint < dof; ++joint) {
      model_metrics.push_back(jointMetrics(
          validation_torque, true_prediction, validation_selection, dof,
          joint));
      estimate_metrics.push_back(jointMetrics(
          validation_torque, identified_prediction, validation_selection, dof,
          joint));
    }

    std::cout << std::setprecision(10)
              << "A oracle RMSE=" << oracle_a.rmse
              << " max=" << oracle_a.max_error
              << " worst_sample=" << oracle_a.worst_sample
              << " worst_joint=J" << (oracle_a.worst_joint + 1)
              << " worst_time=" << oracle_a.worst_time << std::endl;
    std::cout << "B oracle RMSE=" << oracle_b.rmse
              << " max=" << oracle_b.max_error
              << " worst_sample=" << oracle_b.worst_sample
              << " worst_joint=J" << (oracle_b.worst_joint + 1)
              << " worst_time=" << oracle_b.worst_time << std::endl;
    std::cout << "A W rank: " << space.rank << "/" << space.scales.size()
              << ", effective condition: " << space.effective_condition
              << ", beta relative error: " << beta_relative_error << std::endl;
    std::cout << "B rank diagnostic: " << validation_space.rank << "/"
              << validation_space.scales.size()
              << ", effective condition: "
              << validation_space.effective_condition << std::endl;
    std::cout << "B aggregate RMSE: " << validation_error.rmse
              << " Nm, global max: " << validation_error.max_error << " Nm"
              << std::endl;
    for (std::size_t joint = 0; joint < dof; ++joint) {
      std::cout << "Joint " << (joint + 1)
                << " B count: " << estimate_metrics[joint].included_count
                << ", RMSE: " << estimate_metrics[joint].rmse
                << " Nm, max: " << estimate_metrics[joint].max_error
                << " Nm, R2: " << estimate_metrics[joint].r_squared
                << std::endl;
    }
    savePredictions(config, validation.data, true_prediction,
                    identified_prediction, validation_selection);
    saveResults(config, training, validation, training_selection,
                validation_selection, space, validation_space, theta_true,
                beta_true, beta_hat, minimum_norm_parameters,
                training_model_metrics, model_metrics, estimate_metrics,
                beta_relative_error, robust, oracle_a, oracle_b,
                training_residual, validation_error,
                full_validation_diagnostic);
    std::cout << "结果已保存到: " << config.output_file << std::endl;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "identify 失败: " << error.what() << std::endl;
    return 1;
  }
}
