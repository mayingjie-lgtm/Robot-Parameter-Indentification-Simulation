#include "force_node/force_controller.hpp"
#include "mujoco_regressor.hpp"
#include "rebot_pinocchio_regressor.hpp"
#include "robot/mujoco_collision_checker.hpp"
#include "trajectory/fourier_trajectory.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr std::size_t kDof = 6;
constexpr double kRelativeRankThreshold = 1e-6;
constexpr double kStructuralZeroTolerance = 1e-14;
constexpr double kNearZeroColumnRelativeTolerance = 1e-10;

struct Options {
  fs::path controller_config =
      fs::path(PROJECT_ROOT_DIR) / "config" / "rebot_dm_excitation_controller.yaml";
  fs::path qualification_config =
      fs::path(PROJECT_ROOT_DIR) / "config" / "rebot_trajectory_preview.yaml";
  fs::path model =
      fs::path(PROJECT_ROOT_DIR) / "rebot_dm" / "scene_runtime.xml";
  fs::path urdf =
      fs::path(PROJECT_ROOT_DIR) / "rebot_dm" / "rebot_dm.urdf";
  fs::path baseline_coefficients =
      fs::path(PROJECT_ROOT_DIR) / "results" /
      "rebot_real_ab_servo_safe_100hz" / "A" / "coefficients.csv";
  fs::path output_coefficients;
  fs::path report;
  std::uint32_t seed_start = 20260909U;
  std::size_t seed_count = 64;
  std::size_t attempts_per_seed = 64;
  std::size_t pass_candidates = 12;
  double design_target_step_rad = 0.0028;
  double maximum_condition_ratio = 1.25;
  double minimum_joint_peak_velocity_rad_s = 0.05;
  bool overwrite = false;
};

struct QualificationConfig {
  double sample_rate_hz = 100.0;
  std::size_t sample_count = 3001;
  double duration_s = 30.0;
  std::array<double, kDof> expected_start{};
  double start_position_tolerance_rad = 1e-9;
  double start_velocity_tolerance_rad_s = 1e-9;
  double start_acceleration_tolerance_rad_s2 = 1e-9;
  bool require_end_matches_start = true;
  double endpoint_position_tolerance_rad = 1e-8;
  double endpoint_velocity_tolerance_rad_s = 1e-8;
  double endpoint_acceleration_tolerance_rad_s2 = 1e-8;
  std::array<double, kDof> q_min{};
  std::array<double, kDof> q_max{};
  std::array<double, kDof> qd_max{};
  std::array<double, kDof> qdd_max{};
  std::array<double, kDof> jerk_max{};
  double hard_target_step_rad = 0.003125;
  double design_target_step_rad = 0.0028;
};

struct ReplayPoint {
  double time = 0.0;
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd qdd;
};

struct SafetyMetrics {
  bool pass = true;
  bool collision_pass = false;
  std::vector<std::string> failures;
  std::array<double, kDof> q_min{};
  std::array<double, kDof> q_max{};
  std::array<double, kDof> qd_abs_max{};
  std::array<double, kDof> qdd_abs_max{};
  std::array<double, kDof> jerk_abs_max{};
  std::array<double, kDof> target_step_abs_max{};
  std::array<std::size_t, kDof> target_step_sample_index{};
  std::array<double, kDof> position_margin{};
};

struct QualityMetrics {
  std::size_t raw_columns = 0;
  std::size_t rank = 0;
  double effective_condition = std::numeric_limits<double>::infinity();
  double sigma_max = 0.0;
  double sigma_effective_min = 0.0;
  Eigen::VectorXd singular_values;
};

struct Candidate {
  std::uint32_t seed = 0;
  std::size_t attempt = 0;
  SafetyMetrics safety;
  QualityMetrics quality;
  Eigen::VectorXd parameters;
};

std::string trim(const std::string &value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string removeComment(const std::string &value) {
  const auto pos = value.find('#');
  return pos == std::string::npos ? value : value.substr(0, pos);
}

std::optional<std::string> yamlValue(const std::string &line,
                                     const std::string &key) {
  const std::string cleaned = trim(removeComment(line));
  if (cleaned.rfind(key + ":", 0) != 0) return std::nullopt;
  return trim(cleaned.substr(key.size() + 1));
}

double parsePositiveDouble(const std::string &text, const std::string &name) {
  std::size_t consumed = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception &) {
    throw std::runtime_error(name + " must be numeric");
  }
  if (consumed != text.size() || !std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(name + " must be positive and finite");
  }
  return value;
}

std::uint64_t parseUnsigned(const std::string &text, const std::string &name) {
  std::size_t consumed = 0;
  unsigned long long value = 0;
  try {
    value = std::stoull(text, &consumed);
  } catch (const std::exception &) {
    throw std::runtime_error(name + " must be an unsigned integer");
  }
  if (consumed != text.size()) {
    throw std::runtime_error(name + " must be an unsigned integer");
  }
  return static_cast<std::uint64_t>(value);
}

bool parseBool(const std::string &text, const std::string &name) {
  if (text == "true") return true;
  if (text == "false") return false;
  throw std::runtime_error(name + " must be true or false");
}

std::array<double, kDof> parseSix(const std::string &text,
                                  const std::string &name) {
  std::string value = trim(text);
  if (value.size() < 2 || value.front() != '[' || value.back() != ']') {
    throw std::runtime_error(name + " must be a six-element YAML array");
  }
  value = value.substr(1, value.size() - 2);
  std::stringstream stream(value);
  std::string token;
  std::array<double, kDof> result{};
  std::size_t index = 0;
  while (std::getline(stream, token, ',')) {
    if (index >= kDof) {
      throw std::runtime_error(name + " must contain exactly six values");
    }
    const std::string item = trim(token);
    std::size_t consumed = 0;
    result[index] = std::stod(item, &consumed);
    if (consumed != item.size() || !std::isfinite(result[index])) {
      throw std::runtime_error(name + " must contain finite numeric values");
    }
    ++index;
  }
  if (index != kDof) {
    throw std::runtime_error(name + " must contain exactly six values");
  }
  return result;
}

QualificationConfig loadQualificationConfig(const fs::path &path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open qualification config: " + path.string());
  }

  QualificationConfig config;
  bool saw_rate = false;
  bool saw_count = false;
  bool saw_duration = false;
  bool saw_start = false;
  bool saw_q_min = false;
  bool saw_q_max = false;
  bool saw_qd = false;
  bool saw_qdd = false;
  bool saw_jerk = false;
  bool saw_hard_step = false;
  bool saw_design_step = false;

  std::string line;
  while (std::getline(input, line)) {
    if (const auto value = yamlValue(line, "expected_sample_rate_hz")) {
      config.sample_rate_hz = parsePositiveDouble(*value, "expected_sample_rate_hz");
      saw_rate = true;
    } else if (const auto value = yamlValue(line, "expected_sample_count")) {
      config.sample_count =
          static_cast<std::size_t>(parseUnsigned(*value, "expected_sample_count"));
      saw_count = true;
    } else if (const auto value = yamlValue(line, "expected_duration_s")) {
      config.duration_s = parsePositiveDouble(*value, "expected_duration_s");
      saw_duration = true;
    } else if (const auto value = yamlValue(line, "expected_start_position_rad")) {
      config.expected_start = parseSix(*value, "expected_start_position_rad");
      saw_start = true;
    } else if (const auto value = yamlValue(line, "start_position_tolerance_rad")) {
      config.start_position_tolerance_rad =
          parsePositiveDouble(*value, "start_position_tolerance_rad");
    } else if (const auto value = yamlValue(line, "start_velocity_tolerance_rad_s")) {
      config.start_velocity_tolerance_rad_s =
          parsePositiveDouble(*value, "start_velocity_tolerance_rad_s");
    } else if (const auto value = yamlValue(line, "start_acceleration_tolerance_rad_s2")) {
      config.start_acceleration_tolerance_rad_s2 =
          parsePositiveDouble(*value, "start_acceleration_tolerance_rad_s2");
    } else if (const auto value = yamlValue(line, "require_end_matches_start")) {
      config.require_end_matches_start =
          parseBool(*value, "require_end_matches_start");
    } else if (const auto value = yamlValue(line, "endpoint_position_tolerance_rad")) {
      config.endpoint_position_tolerance_rad =
          parsePositiveDouble(*value, "endpoint_position_tolerance_rad");
    } else if (const auto value = yamlValue(line, "endpoint_velocity_tolerance_rad_s")) {
      config.endpoint_velocity_tolerance_rad_s =
          parsePositiveDouble(*value, "endpoint_velocity_tolerance_rad_s");
    } else if (const auto value = yamlValue(line, "endpoint_acceleration_tolerance_rad_s2")) {
      config.endpoint_acceleration_tolerance_rad_s2 =
          parsePositiveDouble(*value, "endpoint_acceleration_tolerance_rad_s2");
    } else if (const auto value = yamlValue(line, "joint_position_min_rad")) {
      config.q_min = parseSix(*value, "joint_position_min_rad");
      saw_q_min = true;
    } else if (const auto value = yamlValue(line, "joint_position_max_rad")) {
      config.q_max = parseSix(*value, "joint_position_max_rad");
      saw_q_max = true;
    } else if (const auto value = yamlValue(line, "maximum_command_velocity_rad_s")) {
      config.qd_max = parseSix(*value, "maximum_command_velocity_rad_s");
      saw_qd = true;
    } else if (const auto value = yamlValue(line, "maximum_command_acceleration_rad_s2")) {
      config.qdd_max = parseSix(*value, "maximum_command_acceleration_rad_s2");
      saw_qdd = true;
    } else if (const auto value = yamlValue(line, "maximum_command_jerk_rad_s3")) {
      config.jerk_max = parseSix(*value, "maximum_command_jerk_rad_s3");
      saw_jerk = true;
    } else if (const auto value = yamlValue(line, "maximum_servo_target_delta_rad")) {
      config.hard_target_step_rad =
          parsePositiveDouble(*value, "maximum_servo_target_delta_rad");
      saw_hard_step = true;
    } else if (const auto value = yamlValue(line, "servo_target_delta_design_limit_rad")) {
      config.design_target_step_rad =
          parsePositiveDouble(*value, "servo_target_delta_design_limit_rad");
      saw_design_step = true;
    }
  }

  if (!(saw_rate && saw_count && saw_duration && saw_start && saw_q_min &&
        saw_q_max && saw_qd && saw_qdd && saw_jerk && saw_hard_step &&
        saw_design_step)) {
    throw std::runtime_error("qualification config is missing required replay gates");
  }
  const double expected_count_real =
      config.duration_s * config.sample_rate_hz + 1.0;
  if (std::abs(expected_count_real - static_cast<double>(config.sample_count)) >
      1e-9) {
    throw std::runtime_error(
        "qualification sample count does not match duration/rate fixed grid");
  }
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    if (!(config.q_min[joint] < config.q_max[joint]) ||
        config.qd_max[joint] <= 0.0 || config.qdd_max[joint] <= 0.0 ||
        config.jerk_max[joint] <= 0.0) {
      throw std::runtime_error("qualification joint limits must be valid");
    }
  }
  return config;
}

fs::path absolutePath(const fs::path &path) {
  return path.is_absolute() ? path : fs::path(PROJECT_ROOT_DIR) / path;
}

void ensureWritable(const fs::path &path, bool overwrite) {
  if (fs::exists(path) && !overwrite) {
    throw std::runtime_error("output exists; use --overwrite: " + path.string());
  }
  if (path.has_parent_path()) fs::create_directories(path.parent_path());
}

std::string sha256File(const fs::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot read for SHA-256: " + path.string());
  }
  using DigestContext =
      std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
  DigestContext context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("cannot initialize SHA-256");
  }
  std::array<char, 8192> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = input.gcount();
    if (count > 0 &&
        EVP_DigestUpdate(context.get(), buffer.data(),
                         static_cast<std::size_t>(count)) != 1) {
      throw std::runtime_error("cannot update SHA-256");
    }
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1) {
    throw std::runtime_error("cannot finish SHA-256");
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < digest_size; ++index) {
    output << std::setw(2) << static_cast<unsigned>(digest[index]);
  }
  return output.str();
}

void printUsage() {
  std::cout
      << "Usage: rebot_hardware_excitation_search --output-coefficients <csv> "
         "--report <yaml> [options]\n"
      << "  --controller-config <yaml>\n"
      << "  --qualification-config <yaml>\n"
      << "  --model <xml>\n"
      << "  --urdf <urdf>\n"
      << "  --baseline-coefficients <csv>\n"
      << "  --seed-start <uint32>\n"
      << "  --seed-count <integer>\n"
      << "  --attempts-per-seed <integer>\n"
      << "  --pass-candidates <integer>\n"
      << "  --design-target-step-rad <value>\n"
      << "  --maximum-condition-ratio <value>\n"
      << "  --minimum-joint-peak-velocity-rad-s <value>\n"
      << "  --overwrite\n";
}

std::string requireValue(int argc, char **argv, int &index) {
  if (index + 1 >= argc) {
    throw std::runtime_error(std::string("missing value for ") + argv[index]);
  }
  return argv[++index];
}

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--controller-config") {
      options.controller_config = requireValue(argc, argv, index);
    } else if (argument == "--qualification-config") {
      options.qualification_config = requireValue(argc, argv, index);
    } else if (argument == "--model") {
      options.model = requireValue(argc, argv, index);
    } else if (argument == "--urdf") {
      options.urdf = requireValue(argc, argv, index);
    } else if (argument == "--baseline-coefficients") {
      options.baseline_coefficients = requireValue(argc, argv, index);
    } else if (argument == "--output-coefficients") {
      options.output_coefficients = requireValue(argc, argv, index);
    } else if (argument == "--report") {
      options.report = requireValue(argc, argv, index);
    } else if (argument == "--seed-start") {
      const auto value = parseUnsigned(requireValue(argc, argv, index),
                                       "--seed-start");
      if (value > UINT32_MAX) {
        throw std::runtime_error("--seed-start exceeds uint32 range");
      }
      options.seed_start = static_cast<std::uint32_t>(value);
    } else if (argument == "--seed-count") {
      options.seed_count = static_cast<std::size_t>(
          parseUnsigned(requireValue(argc, argv, index), "--seed-count"));
    } else if (argument == "--attempts-per-seed") {
      options.attempts_per_seed = static_cast<std::size_t>(
          parseUnsigned(requireValue(argc, argv, index),
                        "--attempts-per-seed"));
    } else if (argument == "--pass-candidates") {
      options.pass_candidates = static_cast<std::size_t>(
          parseUnsigned(requireValue(argc, argv, index),
                        "--pass-candidates"));
    } else if (argument == "--design-target-step-rad") {
      options.design_target_step_rad =
          parsePositiveDouble(requireValue(argc, argv, index),
                              "--design-target-step-rad");
    } else if (argument == "--maximum-condition-ratio") {
      options.maximum_condition_ratio =
          parsePositiveDouble(requireValue(argc, argv, index),
                              "--maximum-condition-ratio");
    } else if (argument == "--minimum-joint-peak-velocity-rad-s") {
      options.minimum_joint_peak_velocity_rad_s =
          parsePositiveDouble(requireValue(argc, argv, index),
                              "--minimum-joint-peak-velocity-rad-s");
    } else if (argument == "--overwrite") {
      options.overwrite = true;
    } else if (argument == "--help") {
      printUsage();
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + argument);
    }
  }

  if (options.output_coefficients.empty() || options.report.empty()) {
    throw std::runtime_error("--output-coefficients and --report are required");
  }
  if (options.seed_count == 0 || options.attempts_per_seed == 0 ||
      options.pass_candidates == 0) {
    throw std::runtime_error("search counts must be positive");
  }
  return options;
}

std::vector<ReplayPoint> sampleTrajectory(
    const trajectory::FourierTrajectory &trajectory,
    const QualificationConfig &qualification) {
  std::vector<ReplayPoint> samples;
  samples.reserve(qualification.sample_count);
  for (std::size_t index = 0; index < qualification.sample_count; ++index) {
    const double time =
        static_cast<double>(index) / qualification.sample_rate_hz;
    const auto point = trajectory.evaluate(time);
    samples.push_back({time, point.q, point.qd, point.qdd});
  }
  return samples;
}

SafetyMetrics evaluateSafety(
    const std::vector<ReplayPoint> &samples,
    const QualificationConfig &qualification,
    double target_step_limit_rad,
    robot::MujocoCollisionChecker *checker,
    bool run_collision) {
  SafetyMetrics metrics;
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    metrics.q_min[joint] = std::numeric_limits<double>::infinity();
    metrics.q_max[joint] = -std::numeric_limits<double>::infinity();
    metrics.position_margin[joint] = std::numeric_limits<double>::infinity();
  }

  if (samples.size() != qualification.sample_count) {
    metrics.pass = false;
    metrics.failures.push_back("sample_count mismatch");
    return metrics;
  }

  const double dt = 1.0 / qualification.sample_rate_hz;
  for (std::size_t sample_index = 0; sample_index < samples.size();
       ++sample_index) {
    const auto &point = samples[sample_index];
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      const Eigen::Index j = static_cast<Eigen::Index>(joint);
      const double q = point.q(j);
      const double qd = point.qd(j);
      const double qdd = point.qdd(j);
      if (!std::isfinite(q) || !std::isfinite(qd) || !std::isfinite(qdd)) {
        metrics.pass = false;
        metrics.failures.push_back("non-finite trajectory value");
        return metrics;
      }
      metrics.q_min[joint] = std::min(metrics.q_min[joint], q);
      metrics.q_max[joint] = std::max(metrics.q_max[joint], q);
      metrics.qd_abs_max[joint] =
          std::max(metrics.qd_abs_max[joint], std::abs(qd));
      metrics.qdd_abs_max[joint] =
          std::max(metrics.qdd_abs_max[joint], std::abs(qdd));
      metrics.position_margin[joint] = std::min(
          metrics.position_margin[joint],
          std::min(q - qualification.q_min[joint],
                   qualification.q_max[joint] - q));

      if (q < qualification.q_min[joint] ||
          q > qualification.q_max[joint]) {
        metrics.pass = false;
        metrics.failures.push_back("J" + std::to_string(joint + 1) +
                                   " position limit violated");
      }
      if (std::abs(qd) > qualification.qd_max[joint] + 1e-12) {
        metrics.pass = false;
        metrics.failures.push_back("J" + std::to_string(joint + 1) +
                                   " velocity limit violated");
      }
      if (std::abs(qdd) > qualification.qdd_max[joint] + 1e-12) {
        metrics.pass = false;
        metrics.failures.push_back("J" + std::to_string(joint + 1) +
                                   " acceleration limit violated");
      }

      if (sample_index > 0) {
        const auto &previous = samples[sample_index - 1];
        const double step =
            std::abs(q - previous.q(static_cast<Eigen::Index>(joint)));
        if (step > metrics.target_step_abs_max[joint]) {
          metrics.target_step_abs_max[joint] = step;
          metrics.target_step_sample_index[joint] = sample_index;
        }
        if (step > target_step_limit_rad + 1e-12) {
          metrics.pass = false;
          const std::string failure =
              "J" + std::to_string(joint + 1) +
              " target-step limit violated";
          if (std::find(metrics.failures.begin(), metrics.failures.end(),
                        failure) == metrics.failures.end()) {
            metrics.failures.push_back(failure);
          }
        }

        const double jerk =
            std::abs((qdd -
                      previous.qdd(static_cast<Eigen::Index>(joint))) /
                     dt);
        metrics.jerk_abs_max[joint] =
            std::max(metrics.jerk_abs_max[joint], jerk);
        if (jerk > qualification.jerk_max[joint] + 1e-9) {
          metrics.pass = false;
          metrics.failures.push_back("J" + std::to_string(joint + 1) +
                                     " jerk limit violated");
        }
      }
    }
  }

  // Finish scanning the full fixed grid even after a numerical failure so the
  // report contains the true per-joint maxima (especially the old-A blocking
  // J4 peak), rather than only the first gate crossing.
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    const Eigen::Index j = static_cast<Eigen::Index>(joint);
    if (std::abs(samples.front().q(j) - qualification.expected_start[joint]) >
        qualification.start_position_tolerance_rad) {
      metrics.pass = false;
      metrics.failures.push_back("J" + std::to_string(joint + 1) +
                                 " start position mismatch");
    }
    if (std::abs(samples.front().qd(j)) >
        qualification.start_velocity_tolerance_rad_s) {
      metrics.pass = false;
      metrics.failures.push_back("J" + std::to_string(joint + 1) +
                                 " start velocity not near zero");
    }
    if (std::abs(samples.front().qdd(j)) >
        qualification.start_acceleration_tolerance_rad_s2) {
      metrics.pass = false;
      metrics.failures.push_back("J" + std::to_string(joint + 1) +
                                 " start acceleration not near zero");
    }
    if (qualification.require_end_matches_start) {
      if (std::abs(samples.back().q(j) - samples.front().q(j)) >
          qualification.endpoint_position_tolerance_rad) {
        metrics.pass = false;
        metrics.failures.push_back("J" + std::to_string(joint + 1) +
                                   " endpoint position mismatch");
      }
      if (std::abs(samples.back().qd(j)) >
          qualification.endpoint_velocity_tolerance_rad_s) {
        metrics.pass = false;
        metrics.failures.push_back("J" + std::to_string(joint + 1) +
                                   " endpoint velocity not near zero");
      }
      if (std::abs(samples.back().qdd(j)) >
          qualification.endpoint_acceleration_tolerance_rad_s2) {
        metrics.pass = false;
        metrics.failures.push_back("J" + std::to_string(joint + 1) +
                                   " endpoint acceleration not near zero");
      }
    }
  }
  if (!metrics.pass) return metrics;

  if (run_collision) {
    if (checker == nullptr) {
      throw std::runtime_error("collision checker is required");
    }
    std::vector<double> q(kDof);
    for (const auto &point : samples) {
      for (std::size_t joint = 0; joint < kDof; ++joint) {
        q[joint] = point.q(static_cast<Eigen::Index>(joint));
      }
      if (checker->checkCollision(q)) {
        metrics.pass = false;
        metrics.failures.push_back("collision/model precheck failed");
        return metrics;
      }
    }
    metrics.collision_pass = true;
  }

  return metrics;
}

QualityMetrics analyzeQuality(
    rebot_dynamics::ReBotPinocchioRegressor &regressor,
    const std::vector<ReplayPoint> &samples) {
  const std::size_t sample_count = samples.size();
  Eigen::MatrixXd q(kDof, sample_count);
  Eigen::MatrixXd qd(kDof, sample_count);
  Eigen::MatrixXd qdd(kDof, sample_count);
  for (std::size_t sample = 0; sample < sample_count; ++sample) {
    q.col(static_cast<Eigen::Index>(sample)) = samples[sample].q;
    qd.col(static_cast<Eigen::Index>(sample)) = samples[sample].qd;
    qdd.col(static_cast<Eigen::Index>(sample)) = samples[sample].qdd;
  }

  const auto flags = mujoco_dynamics::MuJoCoParamFlags::ALL_WITH_FRICTION;
  const Eigen::MatrixXd observation =
      regressor.computeObservationMatrix(q, qd, qdd, flags);
  Eigen::MatrixXd scaled = observation;

  double max_column_norm = 0.0;
  Eigen::VectorXd column_norms(observation.cols());
  for (Eigen::Index column = 0; column < observation.cols(); ++column) {
    const double norm = observation.col(column).norm();
    column_norms(column) = norm;
    max_column_norm = std::max(max_column_norm, norm);
  }
  const double near_zero_tolerance =
      kNearZeroColumnRelativeTolerance * max_column_norm;
  (void)near_zero_tolerance;
  for (Eigen::Index column = 0; column < observation.cols(); ++column) {
    const double norm = column_norms(column);
    if (norm <= kStructuralZeroTolerance) {
      continue;
    }
    // Match the trusted Phase 5B quality diagnostic exactly: near-zero
    // non-structural columns are reported as weak there, but they are still
    // column-normalized before the Gram/SVD rank calculation.
    scaled.col(column) /= norm;
  }

  const Eigen::MatrixXd gram = scaled.transpose() * scaled;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(gram);
  if (eigen_solver.info() != Eigen::Success) {
    throw std::runtime_error("failed to eigendecompose regressor Gram matrix");
  }

  QualityMetrics result;
  result.raw_columns = static_cast<std::size_t>(observation.cols());
  const Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
  result.singular_values.resize(eigenvalues.size());
  for (Eigen::Index index = 0; index < eigenvalues.size(); ++index) {
    const double value = eigenvalues(eigenvalues.size() - 1 - index);
    result.singular_values(index) = std::sqrt(std::max(0.0, value));
  }
  result.sigma_max = result.singular_values(0);
  const double threshold = kRelativeRankThreshold * result.sigma_max;
  result.rank = static_cast<std::size_t>(
      (result.singular_values.array() > threshold).count());
  if (result.rank > 0) {
    result.sigma_effective_min =
        result.singular_values(static_cast<Eigen::Index>(result.rank - 1));
    result.effective_condition =
        result.sigma_max / result.sigma_effective_min;
  }
  return result;
}

void writeArray(std::ostream &output,
                const std::array<double, kDof> &values) {
  output << "[";
  for (std::size_t index = 0; index < kDof; ++index) {
    if (index) output << ", ";
    output << std::setprecision(17) << values[index];
  }
  output << "]";
}

void writeIndexArray(std::ostream &output,
                     const std::array<std::size_t, kDof> &values) {
  output << "[";
  for (std::size_t index = 0; index < kDof; ++index) {
    if (index) output << ", ";
    output << values[index];
  }
  output << "]";
}

void writeSingularValues(std::ostream &output,
                         const Eigen::VectorXd &values) {
  output << "[";
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    if (index) output << ", ";
    output << std::setprecision(17) << values(index);
  }
  output << "]";
}

void writeSafetyBlock(std::ostream &output,
                      const std::string &indent,
                      const SafetyMetrics &metrics,
                      double hard_step,
                      double design_step) {
  output << indent << "status: " << (metrics.pass ? "PASS" : "FAIL") << "\n";
  output << indent << "collision_result: "
         << (metrics.collision_pass ? "PASS" : "NOT_RUN_OR_FAIL") << "\n";
  output << indent << "max_q_ref_step_per_joint: ";
  writeArray(output, metrics.target_step_abs_max);
  output << "\n";
  output << indent << "max_q_ref_step_sample_index_per_joint: ";
  writeIndexArray(output, metrics.target_step_sample_index);
  output << "\n";
  std::array<double, kDof> hard_margin{};
  std::array<double, kDof> design_margin{};
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    hard_margin[joint] = hard_step - metrics.target_step_abs_max[joint];
    design_margin[joint] = design_step - metrics.target_step_abs_max[joint];
  }
  output << indent << "hard_gate_margin_per_joint: ";
  writeArray(output, hard_margin);
  output << "\n";
  output << indent << "design_target_margin_per_joint: ";
  writeArray(output, design_margin);
  output << "\n";
  output << indent << "qd_abs_max_per_joint: ";
  writeArray(output, metrics.qd_abs_max);
  output << "\n";
  output << indent << "qdd_abs_max_per_joint: ";
  writeArray(output, metrics.qdd_abs_max);
  output << "\n";
  output << indent << "jerk_abs_max_per_joint: ";
  writeArray(output, metrics.jerk_abs_max);
  output << "\n";
  output << indent << "position_margin_per_joint: ";
  writeArray(output, metrics.position_margin);
  output << "\n";
  output << indent << "failures: [";
  for (std::size_t index = 0; index < metrics.failures.size(); ++index) {
    if (index) output << ", ";
    output << "'" << metrics.failures[index] << "'";
  }
  output << "]\n";
}

void writeQualityBlock(std::ostream &output,
                       const std::string &indent,
                       const QualityMetrics &quality) {
  output << indent << "raw_columns: " << quality.raw_columns << "\n";
  output << indent << "rank: " << quality.rank << "\n";
  output << indent << "relative_rank_threshold: "
         << kRelativeRankThreshold << "\n";
  output << indent << "effective_condition_number: "
         << std::setprecision(17) << quality.effective_condition << "\n";
  output << indent << "sigma_max: " << quality.sigma_max << "\n";
  output << indent << "minimum_effective_singular_value: "
         << quality.sigma_effective_min << "\n";
  output << indent << "singular_values: ";
  writeSingularValues(output, quality.singular_values);
  output << "\n";
}

} // namespace

int main(int argc, char **argv) {
  try {
    Options options = parseOptions(argc, argv);
    options.controller_config = absolutePath(options.controller_config);
    options.qualification_config = absolutePath(options.qualification_config);
    options.model = absolutePath(options.model);
    options.urdf = absolutePath(options.urdf);
    options.baseline_coefficients = absolutePath(options.baseline_coefficients);
    options.output_coefficients = absolutePath(options.output_coefficients);
    options.report = absolutePath(options.report);

    for (const auto &path : {options.controller_config,
                             options.qualification_config, options.model,
                             options.urdf, options.baseline_coefficients}) {
      if (!fs::is_regular_file(path)) {
        throw std::runtime_error("required input missing: " + path.string());
      }
    }
    ensureWritable(options.output_coefficients, options.overwrite);
    ensureWritable(options.report, options.overwrite);

    const auto controller =
        force_node::ForceController::loadConfig(options.controller_config);
    const auto qualification =
        loadQualificationConfig(options.qualification_config);
    if (controller.robot != "rebot_dm" ||
        controller.controller_mode != "excitation_trajectory" ||
        controller.target_position.size() != kDof ||
        controller.trajectory_harmonics < 2 ||
        controller.trajectory_duration != qualification.duration_s) {
      throw std::runtime_error(
          "controller config does not match the 6-DoF reBot excitation design");
    }
    if (options.design_target_step_rad >= qualification.hard_target_step_rad) {
      throw std::runtime_error(
          "design target step must remain below the Lower hard target-step gate");
    }
    if (std::abs(options.design_target_step_rad -
                 qualification.design_target_step_rad) > 1e-12) {
      throw std::runtime_error(
          "search design target must match servo_target_delta_design_limit_rad "
          "in the qualification config");
    }

    Eigen::VectorXd q0(static_cast<Eigen::Index>(kDof));
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      q0(static_cast<Eigen::Index>(joint)) =
          controller.target_position[joint];
      if (std::abs(controller.target_position[joint] -
                   qualification.expected_start[joint]) >
          qualification.start_position_tolerance_rad) {
        throw std::runtime_error(
            "controller q0 does not match qualification expected start");
      }
    }

    robot::MujocoCollisionChecker collision_checker;
    collision_checker.initialize(options.model.string());
    const auto &model_lower = collision_checker.jointLowerLimits();
    const auto &model_upper = collision_checker.jointUpperLimits();
    if (model_lower.size() < kDof || model_upper.size() < kDof) {
      throw std::runtime_error("collision model exposes fewer than six joints");
    }
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      if (std::abs(model_lower[joint] - qualification.q_min[joint]) > 1e-12 ||
          std::abs(model_upper[joint] - qualification.q_max[joint]) > 1e-12) {
        throw std::runtime_error(
            "qualification position limits do not match canonical model");
      }
    }

    rebot_dynamics::ReBotPinocchioRegressor regressor(options.urdf);

    trajectory::FourierTrajectory baseline(
        kDof, controller.trajectory_harmonics,
        controller.trajectory_duration, q0);
    baseline.loadCoefficients(options.baseline_coefficients);
    const auto baseline_samples =
        sampleTrajectory(baseline, qualification);
    auto baseline_hard_safety =
        evaluateSafety(baseline_samples, qualification,
                       std::numeric_limits<double>::infinity(),
                       &collision_checker, true);
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      if (baseline_hard_safety.target_step_abs_max[joint] >
          qualification.hard_target_step_rad + 1e-12) {
        baseline_hard_safety.pass = false;
        baseline_hard_safety.failures.push_back(
            "J" + std::to_string(joint + 1) +
            " Lower ServoCore hard target-step gate violated");
      }
    }
    const auto baseline_quality =
        analyzeQuality(regressor, baseline_samples);

    if (baseline_quality.rank == 0 ||
        !std::isfinite(baseline_quality.effective_condition)) {
      throw std::runtime_error("baseline excitation quality is invalid");
    }

    std::vector<Candidate> accepted;
    std::size_t evaluated_candidates = 0;
    std::size_t design_safety_pass_candidates = 0;
    std::size_t quality_rank_pass_candidates = 0;
    std::size_t quality_condition_pass_candidates = 0;

    for (std::size_t seed_offset = 0;
         seed_offset < options.seed_count &&
         accepted.size() < options.pass_candidates;
         ++seed_offset) {
      const std::uint64_t seed_value =
          static_cast<std::uint64_t>(options.seed_start) + seed_offset;
      if (seed_value > UINT32_MAX) break;
      const std::uint32_t seed = static_cast<std::uint32_t>(seed_value);
      std::mt19937 generator(seed);
      trajectory::FourierTrajectory candidate(
          kDof, controller.trajectory_harmonics,
          controller.trajectory_duration, q0);

      for (std::size_t attempt = 1;
           attempt <= options.attempts_per_seed &&
           accepted.size() < options.pass_candidates;
           ++attempt) {
        ++evaluated_candidates;
        candidate.setRandomCoefficients(
            controller.trajectory_coefficient_scale, generator);
        const auto samples = sampleTrajectory(candidate, qualification);
        auto safety =
            evaluateSafety(samples, qualification,
                           options.design_target_step_rad,
                           nullptr, false);
        if (!safety.pass) continue;

        bool peak_velocity_pass = true;
        for (double value : safety.qd_abs_max) {
          if (value < options.minimum_joint_peak_velocity_rad_s) {
            peak_velocity_pass = false;
            break;
          }
        }
        if (!peak_velocity_pass) continue;

        safety = evaluateSafety(samples, qualification,
                                options.design_target_step_rad,
                                &collision_checker, true);
        if (!safety.pass) continue;
        ++design_safety_pass_candidates;

        const auto quality = analyzeQuality(regressor, samples);
        if (quality.rank != baseline_quality.rank) continue;
        ++quality_rank_pass_candidates;

        if (quality.effective_condition >
            baseline_quality.effective_condition *
                options.maximum_condition_ratio) {
          continue;
        }
        ++quality_condition_pass_candidates;

        accepted.push_back(
            Candidate{seed, attempt, safety, quality,
                      candidate.getParameterVector()});
        std::cout << "PASS candidate seed=" << seed
                  << " attempt=" << attempt
                  << " condition=" << std::setprecision(12)
                  << quality.effective_condition
                  << " sigma_min=" << quality.sigma_effective_min
                  << " max_step="
                  << *std::max_element(
                         safety.target_step_abs_max.begin(),
                         safety.target_step_abs_max.end())
                  << "\n";
      }
    }

    if (accepted.empty()) {
      std::ofstream report(options.report);
      report << "schema_version: rebot_hardware_excitation_search_v1\n";
      report << "status: BLOCKED\n";
      report << "evaluated_candidates: " << evaluated_candidates << "\n";
      report << "design_safety_pass_candidates: "
             << design_safety_pass_candidates << "\n";
      report << "quality_rank_pass_candidates: "
             << quality_rank_pass_candidates << "\n";
      report << "quality_condition_pass_candidates: "
             << quality_condition_pass_candidates << "\n";
      report << "lower_hard_gate_rad: "
             << qualification.hard_target_step_rad << "\n";
      report << "design_target_rad: "
             << options.design_target_step_rad << "\n";
      report << "baseline:\n";
      writeSafetyBlock(report, "  ", baseline_hard_safety,
                       qualification.hard_target_step_rad,
                       options.design_target_step_rad);
      report << "  quality:\n";
      writeQualityBlock(report, "    ", baseline_quality);
      throw std::runtime_error(
          "no hardware-compatible candidate met safety and excitation-quality gates");
    }

    const auto best = std::min_element(
        accepted.begin(), accepted.end(),
        [](const Candidate &lhs, const Candidate &rhs) {
          if (lhs.quality.effective_condition !=
              rhs.quality.effective_condition) {
            return lhs.quality.effective_condition <
                   rhs.quality.effective_condition;
          }
          return lhs.quality.sigma_effective_min >
                 rhs.quality.sigma_effective_min;
        });

    trajectory::FourierTrajectory selected(
        kDof, controller.trajectory_harmonics,
        controller.trajectory_duration, q0);
    selected.setParameterVector(best->parameters);
    selected.saveCoefficients(options.output_coefficients);
    const std::string coefficient_sha =
        sha256File(options.output_coefficients);

    std::ofstream report(options.report);
    if (!report) {
      throw std::runtime_error("cannot write search report");
    }
    report << std::setprecision(17);
    report << "schema_version: rebot_hardware_excitation_search_v1\n";
    report << "status: PASS\n";
    report << "hardware_authorized: false\n";
    report << "controller_config: '" << options.controller_config.string()
           << "'\n";
    report << "qualification_config: '"
           << options.qualification_config.string() << "'\n";
    report << "model_file: '" << options.model.string() << "'\n";
    report << "urdf_file: '" << options.urdf.string() << "'\n";
    report << "sample_rate_hz: " << qualification.sample_rate_hz << "\n";
    report << "sample_count: " << qualification.sample_count << "\n";
    report << "duration_s: " << qualification.duration_s << "\n";
    report << "harmonics: " << controller.trajectory_harmonics << "\n";
    report << "q0: ";
    writeArray(report, qualification.expected_start);
    report << "\n";
    report << "coefficient_scale: "
           << controller.trajectory_coefficient_scale << "\n";
    report << "lower_hard_gate_rad: "
           << qualification.hard_target_step_rad << "\n";
    report << "design_target_rad: "
           << options.design_target_step_rad << "\n";
    report << "maximum_condition_ratio: "
           << options.maximum_condition_ratio << "\n";
    report << "minimum_joint_peak_velocity_rad_s: "
           << options.minimum_joint_peak_velocity_rad_s << "\n";
    report << "seed_start: " << options.seed_start << "\n";
    report << "seed_count: " << options.seed_count << "\n";
    report << "attempts_per_seed: " << options.attempts_per_seed << "\n";
    report << "requested_pass_candidates: "
           << options.pass_candidates << "\n";
    report << "evaluated_candidates: " << evaluated_candidates << "\n";
    report << "design_safety_pass_candidates: "
           << design_safety_pass_candidates << "\n";
    report << "quality_rank_pass_candidates: "
           << quality_rank_pass_candidates << "\n";
    report << "quality_condition_pass_candidates: "
           << quality_condition_pass_candidates << "\n";
    report << "accepted_candidate_count: " << accepted.size() << "\n";
    report << "baseline:\n";
    report << "  coefficient_file: '"
           << options.baseline_coefficients.string() << "'\n";
    report << "  coefficient_sha256: "
           << sha256File(options.baseline_coefficients) << "\n";
    writeSafetyBlock(report, "  ", baseline_hard_safety,
                     qualification.hard_target_step_rad,
                     options.design_target_step_rad);
    report << "  design_target_status: "
           << ((*std::max_element(
                    baseline_hard_safety.target_step_abs_max.begin(),
                    baseline_hard_safety.target_step_abs_max.end()) <=
                options.design_target_step_rad + 1e-12)
                   ? "PASS"
                   : "FAIL")
           << "\n";
    report << "  quality:\n";
    writeQualityBlock(report, "    ", baseline_quality);
    report << "selected:\n";
    report << "  seed: " << best->seed << "\n";
    report << "  attempt: " << best->attempt << "\n";
    report << "  coefficient_file: '"
           << options.output_coefficients.string() << "'\n";
    report << "  coefficient_sha256: " << coefficient_sha << "\n";
    writeSafetyBlock(report, "  ", best->safety,
                     qualification.hard_target_step_rad,
                     options.design_target_step_rad);
    report << "  design_target_status: PASS\n";
    report << "  quality:\n";
    writeQualityBlock(report, "    ", best->quality);
    report << "comparison:\n";
    report << "  effective_condition_ratio_new_over_old: "
           << best->quality.effective_condition /
                  baseline_quality.effective_condition
           << "\n";
    report << "  minimum_effective_singular_ratio_new_over_old: "
           << best->quality.sigma_effective_min /
                  baseline_quality.sigma_effective_min
           << "\n";

    std::cout << "selected_seed=" << best->seed
              << " selected_attempt=" << best->attempt << "\n"
              << "coefficient_sha256=" << coefficient_sha << "\n"
              << "evaluated_candidates=" << evaluated_candidates << "\n"
              << "accepted_candidate_count=" << accepted.size() << "\n"
              << "old_condition=" << baseline_quality.effective_condition
              << " new_condition=" << best->quality.effective_condition << "\n"
              << "report=" << options.report << std::endl;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "rebot_hardware_excitation_search failed: "
              << error.what() << std::endl;
    return 1;
  }
}
