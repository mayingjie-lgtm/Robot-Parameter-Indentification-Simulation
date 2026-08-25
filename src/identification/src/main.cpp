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
  std::string line;
  while (std::getline(file, line)) {
    std::string value;
    parseInt(line, "algorithm", config.algorithm);
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

/** Keep only finite, unsaturated, contact-free, constraint-free samples. */
PreparedData selectValidSamples(const ExperimentData &source,
                                double constraint_tolerance,
                                const std::vector<double> &frictionloss = {},
                                double friction_speed_threshold = 0.05) {
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
        if (std::abs(velocity) >= friction_speed_threshold) {
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
  result.data.tau_constraint =
      Eigen::MatrixXd::Zero(indices.size(), source.n_dof);
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

/** Select sample-major torque rows whose corresponding joint is moving. */
std::vector<Eigen::Index>
movingObservationRows(const Eigen::MatrixXd &velocity,
                      double minimum_speed) {
  std::vector<Eigen::Index> rows;
  rows.reserve(static_cast<std::size_t>(velocity.size()));
  for (Eigen::Index sample = 0; sample < velocity.rows(); ++sample) {
    for (Eigen::Index joint = 0; joint < velocity.cols(); ++joint) {
      if (std::abs(velocity(sample, joint)) >= minimum_speed) {
        rows.push_back(sample * velocity.cols() + joint);
      }
    }
  }
  if (rows.empty()) {
    throw std::runtime_error("摩擦速度阈值筛选后没有 observation row");
  }
  return rows;
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

/** Compute torque error statistics for one joint in sample-major vectors. */
ErrorMetrics jointMetrics(const Eigen::VectorXd &measured,
                          const Eigen::VectorXd &predicted,
                          const Eigen::MatrixXd &velocity,
                          std::size_t n_dof, std::size_t joint,
                          double minimum_speed) {
  ErrorMetrics metrics;
  const std::size_t total_samples =
      static_cast<std::size_t>(measured.size()) / n_dof;
  std::size_t samples = 0;
  double sum_squared = 0.0;
  double sum_absolute = 0.0;
  double sum_error = 0.0;
  double measured_mean = 0.0;
  for (std::size_t sample = 0; sample < total_samples; ++sample) {
    if (std::abs(velocity(static_cast<Eigen::Index>(sample),
                          static_cast<Eigen::Index>(joint))) < minimum_speed) {
      continue;
    }
    measured_mean += measured(
        static_cast<Eigen::Index>(sample * n_dof + joint));
    ++samples;
  }
  if (samples == 0) {
    throw std::runtime_error("逐关节误差统计没有满足速度阈值的样本");
  }
  measured_mean /= static_cast<double>(samples);
  double total_variance = 0.0;
  for (std::size_t sample = 0; sample < total_samples; ++sample) {
    if (std::abs(velocity(static_cast<Eigen::Index>(sample),
                          static_cast<Eigen::Index>(joint))) < minimum_speed) {
      continue;
    }
    const Eigen::Index index =
        static_cast<Eigen::Index>(sample * n_dof + joint);
    const double error = predicted(index) - measured(index);
    sum_squared += error * error;
    sum_absolute += std::abs(error);
    sum_error += error;
    metrics.max_error = std::max(metrics.max_error, std::abs(error));
    const double centered = measured(index) - measured_mean;
    total_variance += centered * centered;
  }
  metrics.rmse = std::sqrt(sum_squared / static_cast<double>(samples));
  metrics.mae = sum_absolute / static_cast<double>(samples);
  metrics.bias = sum_error / static_cast<double>(samples);
  metrics.r_squared = total_variance > 0.0
                          ? 1.0 - sum_squared / total_variance
                          : (sum_squared == 0.0 ? 1.0 : 0.0);
  return metrics;
}

/** Write an Eigen vector as an indented YAML sequence. */
void writeVector(std::ofstream &output, const std::string &name,
                 const Eigen::VectorXd &values) {
  output << name << ":\n";
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    output << "  - " << values(index) << "\n";
  }
}

/** Save validation torques for direct per-joint plotting and audit. */
fs::path savePredictions(const IdentificationConfig &config,
                         const ExperimentData &validation,
                         const Eigen::VectorXd &true_prediction,
                         const Eigen::VectorXd &identified_prediction,
                         double minimum_speed) {
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
  for (std::size_t sample = 0; sample < validation.n_samples; ++sample) {
    for (std::size_t joint = 0; joint < validation.n_dof; ++joint) {
      const Eigen::Index index =
          static_cast<Eigen::Index>(sample * validation.n_dof + joint);
      const bool included =
          std::abs(validation.qd(static_cast<Eigen::Index>(sample),
                                 static_cast<Eigen::Index>(joint))) >=
          minimum_speed;
      output << validation.time[sample] << "," << (joint + 1) << ","
             << validation.tau(static_cast<Eigen::Index>(sample),
                               static_cast<Eigen::Index>(joint))
             << "," << true_prediction(index) << ","
             << identified_prediction(index) << "," << (included ? 1 : 0)
             << "\n";
    }
  }
  return path;
}

/** Persist the complete clean/noisy A-to-B identification result. */
void saveResults(const IdentificationConfig &config,
                 const PreparedData &training,
                 const PreparedData &validation,
                 const BaseParameterSpace &space,
                 const Eigen::VectorXd &theta_true,
                 const Eigen::VectorXd &beta_true,
                 const Eigen::VectorXd &beta_hat,
                 const Eigen::VectorXd &minimum_norm_parameters,
                 const std::vector<ErrorMetrics> &model_metrics,
                 const std::vector<ErrorMetrics> &estimate_metrics,
                 double beta_relative_error,
                 const RobustDiagnostics &robust) {
  if (config.output_file.has_parent_path()) {
    fs::create_directories(config.output_file.parent_path());
  }
  std::ofstream output(config.output_file);
  if (!output) {
    throw std::runtime_error("无法写入辨识结果: " +
                             config.output_file.string());
  }
  output << std::setprecision(17);
  output << "schema_version: 3\n";
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
  output << "full_parameter_count: " << space.scales.size() << "\n";
  output << "base_parameter_rank: " << space.rank << "\n";
  output << "rank_relative_tolerance: " << config.rank_relative_tolerance
         << "\n";
  output << "effective_condition_number: " << space.effective_condition
         << "\n";
  output << "beta_relative_error: " << beta_relative_error << "\n";
  output << "friction_velocity_threshold: "
         << config.friction_velocity_threshold << "\n";
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
  writeVector(output, "singular_values", space.singular_values);
  writeVector(output, "beta_true", beta_true);
  writeVector(output, "beta_hat", beta_hat);
  writeVector(output, "full_minimum_norm_parameters", minimum_norm_parameters);
  output << "validation_per_joint:\n";
  for (std::size_t joint = 0; joint < estimate_metrics.size(); ++joint) {
    output << "  - joint: " << (joint + 1) << "\n";
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
                           config.friction_velocity_threshold);
    PreparedData validation =
        selectValidSamples(raw_validation, config.constraint_tolerance,
                           config.joint_frictionloss,
                           config.friction_velocity_threshold);
    PreparedData basis_data =
        selectValidSamples(raw_basis, config.constraint_tolerance,
                           config.joint_frictionloss,
                           config.friction_velocity_threshold);
    std::cout << "Quality filter: A excluded " << training.excluded_samples
              << ", B excluded " << validation.excluded_samples << std::endl;

    Identification identifier(config.robot, config.model_file,
                              config.joint_frictionloss,
                              config.joint_armature, config.joint_damping);
    const auto flags =
        friction_enabled
            ? mujoco_dynamics::MuJoCoParamFlags::ALL_WITH_FRICTION
            : mujoco_dynamics::MuJoCoParamFlags::ALL;
    BaseParameterSpace space;
    {
      Eigen::MatrixXd basis_observation =
          identifier.computeObservationMatrix(
              basis_data.data.q.transpose(), basis_data.data.qd.transpose(),
              basis_data.data.qdd.transpose(), flags);
      if (friction_enabled) {
        basis_observation = takeRows(
            basis_observation,
            movingObservationRows(basis_data.data.qd,
                                  config.friction_velocity_threshold));
      }
      space = computeBaseSpace(basis_observation,
                               config.rank_relative_tolerance);
    }
    Eigen::MatrixXd training_observation =
        identifier.computeObservationMatrix(
            training.data.q.transpose(), training.data.qd.transpose(),
            training.data.qdd.transpose(), flags);
    Eigen::VectorXd training_torque = flattenTorque(training.data);
    if (friction_enabled) {
      const auto moving_rows = movingObservationRows(
          training.data.qd, config.friction_velocity_threshold);
      training_observation = takeRows(training_observation, moving_rows);
      training_torque = takeRows(training_torque, moving_rows);
    }
    const Eigen::MatrixXd training_base =
        baseObservation(training_observation, space);
    const Eigen::VectorXd theta_true =
        identifier.getGroundTruthParameters(flags);
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

    const Eigen::MatrixXd validation_observation =
        identifier.computeObservationMatrix(
            validation.data.q.transpose(), validation.data.qd.transpose(),
            validation.data.qdd.transpose(), flags);
    const Eigen::MatrixXd validation_base =
        baseObservation(validation_observation, space);
    const Eigen::VectorXd validation_torque = flattenTorque(validation.data);
    const Eigen::VectorXd true_prediction =
        validation_observation * theta_true;
    const Eigen::VectorXd identified_prediction = validation_base * beta_hat;
    std::vector<ErrorMetrics> model_metrics;
    std::vector<ErrorMetrics> estimate_metrics;
    for (std::size_t joint = 0; joint < dof; ++joint) {
      model_metrics.push_back(jointMetrics(
          validation_torque, true_prediction, validation.data.qd, dof, joint,
          friction_enabled ? config.friction_velocity_threshold : 0.0));
      estimate_metrics.push_back(jointMetrics(
          validation_torque, identified_prediction, validation.data.qd, dof,
          joint,
          friction_enabled ? config.friction_velocity_threshold : 0.0));
    }

    std::cout << std::setprecision(10)
              << "W rank: " << space.rank << "/" << space.scales.size()
              << ", effective condition: " << space.effective_condition
              << ", beta relative error: " << beta_relative_error << std::endl;
    for (std::size_t joint = 0; joint < dof; ++joint) {
      std::cout << "Joint " << (joint + 1)
                << " B RMSE: " << estimate_metrics[joint].rmse
                << " Nm, max: " << estimate_metrics[joint].max_error
                << " Nm, R2: " << estimate_metrics[joint].r_squared
                << std::endl;
    }
    savePredictions(config, validation.data, true_prediction,
                    identified_prediction,
                    friction_enabled ? config.friction_velocity_threshold
                                     : 0.0);
    saveResults(config, training, validation, space, theta_true, beta_true, beta_hat,
                minimum_norm_parameters, model_metrics, estimate_metrics,
                beta_relative_error, robust);
    std::cout << "结果已保存到: " << config.output_file << std::endl;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "identify 失败: " << error.what() << std::endl;
    return 1;
  }
}
