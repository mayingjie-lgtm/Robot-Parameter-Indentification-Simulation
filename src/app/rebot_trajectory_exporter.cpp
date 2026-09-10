#include "force_node/force_controller.hpp"
#include "robot/mujoco_collision_checker.hpp"
#include "trajectory/fourier_trajectory.hpp"

#include <openssl/evp.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr std::size_t kDof = 6;
constexpr std::size_t kQuinticCollisionSubdivisions = 10;

struct Options {
  fs::path coefficients;
  fs::path generated_coefficients;
  fs::path controller_config =
      fs::path(PROJECT_ROOT_DIR) / "config" / "rebot_dm_excitation_controller.yaml";
  fs::path model =
      fs::path(PROJECT_ROOT_DIR) / "rebot_dm" / "scene_runtime.xml";
  fs::path output;
  fs::path metadata;
  double sample_rate_hz = 100.0;
  std::optional<double> duration_s;
  std::optional<std::uint32_t> seed;
  std::optional<std::size_t> accepted_attempt;
  std::string expected_coefficient_sha256;
  bool overwrite = false;
};

struct ReplayPoint {
  double time = 0.0;
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd qdd;
};

void printUsage() {
  std::cout
      << "Usage: rebot_trajectory_exporter (--coefficients <csv> | "
         "--generate-coefficients <csv>) --output <csv> --metadata <yaml> "
         "[options]\n"
      << "Options:\n"
      << "  --controller-config <yaml>     accepted reBot excitation config\n"
      << "  --model <xml>                  canonical MuJoCo collision/runtime model\n"
      << "  --sample-rate-hz <value>       replay rate (default: 100 candidate)\n"
      << "  --duration <seconds>           default: controller trajectory duration\n"
      << "  --seed <uint>                  generation seed override\n"
      << "  --accepted-attempt <integer>   replay random stream to accepted attempt\n"
      << "  --expected-coefficient-sha256 <hex>  accepted coefficient hash\n"
      << "  --overwrite                    replace outputs\n"
      << "  --help                         show this message\n";
}

std::string requireValue(int argc, char **argv, int &index) {
  if (index + 1 >= argc) {
    throw std::runtime_error(std::string("missing value for ") + argv[index]);
  }
  return argv[++index];
}

double finitePositive(const std::string &text, const std::string &name) {
  std::size_t parsed = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &parsed);
  } catch (const std::exception &) {
    throw std::runtime_error(name + " must be numeric");
  }
  if (parsed != text.size() || !std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(name + " must be positive and finite");
  }
  return value;
}

std::uint64_t parseUnsigned(const std::string &text, const std::string &name) {
  std::size_t parsed = 0;
  unsigned long long value = 0;
  try {
    value = std::stoull(text, &parsed);
  } catch (const std::exception &) {
    throw std::runtime_error(name + " must be unsigned integer");
  }
  if (parsed != text.size()) {
    throw std::runtime_error(name + " must be unsigned integer");
  }
  return static_cast<std::uint64_t>(value);
}

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--coefficients") {
      options.coefficients = requireValue(argc, argv, index);
    } else if (argument == "--generate-coefficients") {
      options.generated_coefficients = requireValue(argc, argv, index);
    } else if (argument == "--controller-config") {
      options.controller_config = requireValue(argc, argv, index);
    } else if (argument == "--model") {
      options.model = requireValue(argc, argv, index);
    } else if (argument == "--output") {
      options.output = requireValue(argc, argv, index);
    } else if (argument == "--metadata") {
      options.metadata = requireValue(argc, argv, index);
    } else if (argument == "--sample-rate-hz") {
      options.sample_rate_hz =
          finitePositive(requireValue(argc, argv, index), "--sample-rate-hz");
    } else if (argument == "--duration") {
      options.duration_s =
          finitePositive(requireValue(argc, argv, index), "--duration");
    } else if (argument == "--seed") {
      const auto value = parseUnsigned(requireValue(argc, argv, index), "--seed");
      if (value > UINT32_MAX) {
        throw std::runtime_error("--seed exceeds uint32 range");
      }
      options.seed = static_cast<std::uint32_t>(value);
    } else if (argument == "--accepted-attempt") {
      const auto value =
          parseUnsigned(requireValue(argc, argv, index), "--accepted-attempt");
      if (value == 0) {
        throw std::runtime_error("--accepted-attempt must be >=1");
      }
      options.accepted_attempt = static_cast<std::size_t>(value);
    } else if (argument == "--expected-coefficient-sha256") {
      options.expected_coefficient_sha256 = requireValue(argc, argv, index);
    } else if (argument == "--overwrite") {
      options.overwrite = true;
    } else if (argument == "--help") {
      printUsage();
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + argument);
    }
  }
  const bool existing = !options.coefficients.empty();
  const bool generate = !options.generated_coefficients.empty();
  if (existing == generate || options.output.empty() || options.metadata.empty()) {
    throw std::runtime_error(
        "choose exactly one coefficient source and specify --output/--metadata");
  }
  if (generate &&
      (!options.accepted_attempt ||
       options.expected_coefficient_sha256.size() != 64)) {
    throw std::runtime_error(
        "generation mode requires --accepted-attempt and 64-char "
        "--expected-coefficient-sha256");
  }
  return options;
}

fs::path absolutePath(const fs::path &path) {
  return path.is_absolute() ? path : fs::path(PROJECT_ROOT_DIR) / path;
}

std::string sha256File(const fs::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot read file for SHA-256: " + path.string());
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
    const std::streamsize count = input.gcount();
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

std::string yamlQuote(const std::string &value) {
  std::string result = "'";
  for (const char c : value) {
    result += c == '\'' ? "''" : std::string(1, c);
  }
  result += "'";
  return result;
}

void writeVector(std::ostream &output, const Eigen::VectorXd &values) {
  output << "[";
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    if (index != 0) {
      output << ", ";
    }
    output << std::setprecision(17) << values(index);
  }
  output << "]";
}

void ensureOutputAllowed(const fs::path &path, bool overwrite) {
  if (fs::exists(path) && !overwrite) {
    throw std::runtime_error("output already exists; use --overwrite: " +
                             path.string());
  }
  if (path.has_parent_path()) {
    fs::create_directories(path.parent_path());
  }
}

void validateControllerConfig(const force_node::ForceControllerConfig &config) {
  if (config.robot != "rebot_dm" ||
      config.controller_mode != "excitation_trajectory" ||
      config.kp.size() != kDof || config.kd.size() != kDof ||
      config.target_position.size() != kDof ||
      config.trajectory_harmonics < 2 ||
      !std::isfinite(config.trajectory_duration) ||
      config.trajectory_duration <= 0.0 ||
      !std::isfinite(config.trajectory_coefficient_scale) ||
      config.trajectory_coefficient_scale <= 0.0) {
    throw std::runtime_error("invalid reBot excitation controller config");
  }
}

std::vector<ReplayPoint>
sampleTrajectory(const trajectory::FourierTrajectory &trajectory,
                 double duration_s, double sample_rate_hz) {
  const double intervals_real = duration_s * sample_rate_hz;
  const auto intervals =
      static_cast<std::size_t>(std::llround(intervals_real));
  if (std::abs(intervals_real - static_cast<double>(intervals)) > 1e-9) {
    throw std::runtime_error(
        "duration * sample-rate must be integer for an exact final sample");
  }

  std::vector<ReplayPoint> samples;
  samples.reserve(intervals + 1);
  for (std::size_t sample = 0; sample <= intervals; ++sample) {
    const double time = static_cast<double>(sample) / sample_rate_hz;
    const auto point = trajectory.evaluate(time);
    samples.push_back(ReplayPoint{time, point.q, point.qd, point.qdd});
  }
  return samples;
}

ReplayPoint quinticHermitePoint(const ReplayPoint &left,
                                const ReplayPoint &right, double ratio) {
  const double h = right.time - left.time;
  if (!(h > 0.0) || ratio < 0.0 || ratio > 1.0) {
    throw std::runtime_error("invalid quintic interpolation interval");
  }
  ReplayPoint result;
  result.time = left.time + ratio * h;
  result.q.resize(kDof);
  result.qd.resize(kDof);
  result.qdd.resize(kDof);
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    const Eigen::Index j = static_cast<Eigen::Index>(joint);
    const double q0 = left.q(j), q1 = right.q(j);
    const double v0 = left.qd(j), v1 = right.qd(j);
    const double a0 = left.qdd(j), a1 = right.qdd(j);
    const double delta = q1 - q0;
    const double c0 = q0;
    const double c1 = h * v0;
    const double c2 = 0.5 * h * h * a0;
    const double c3 = 10.0 * delta - h * (6.0 * v0 + 4.0 * v1) -
                      h * h * (1.5 * a0 - 0.5 * a1);
    const double c4 = -15.0 * delta + h * (8.0 * v0 + 7.0 * v1) +
                      h * h * (1.5 * a0 - a1);
    const double c5 = 6.0 * delta - 3.0 * h * (v0 + v1) -
                      0.5 * h * h * (a0 - a1);
    const double s = ratio;
    result.q(j) = c0 + s * (c1 + s * (c2 + s * (c3 + s * (c4 + s * c5))));
    result.qd(j) =
        (c1 + s * (2.0 * c2 + s * (3.0 * c3 +
                                    s * (4.0 * c4 + s * 5.0 * c5)))) /
        h;
    result.qdd(j) =
        (2.0 * c2 + s * (6.0 * c3 +
                          s * (12.0 * c4 + s * 20.0 * c5))) /
        (h * h);
  }
  return result;
}

std::vector<ReplayPoint>
denseQuinticPath(const std::vector<ReplayPoint> &samples) {
  std::vector<ReplayPoint> dense;
  dense.reserve((samples.size() - 1) * kQuinticCollisionSubdivisions + 1);
  for (std::size_t interval = 0; interval + 1 < samples.size(); ++interval) {
    for (std::size_t subdivision = 0;
         subdivision < kQuinticCollisionSubdivisions; ++subdivision) {
      dense.push_back(quinticHermitePoint(
          samples[interval], samples[interval + 1],
          static_cast<double>(subdivision) /
              static_cast<double>(kQuinticCollisionSubdivisions)));
    }
  }
  dense.push_back(samples.back());
  return dense;
}

void runEverySampleModelPrecheck(
    const fs::path &model_path,
    const force_node::ForceControllerConfig &controller_config,
    const std::vector<ReplayPoint> &samples) {
  robot::MujocoCollisionChecker checker;
  checker.initialize(model_path.string());
  const auto &lower = checker.jointLowerLimits();
  const auto &upper = checker.jointUpperLimits();
  if (lower.size() < kDof || upper.size() < kDof) {
    throw std::runtime_error("collision model exposes fewer than six scalar joints");
  }

  for (std::size_t sample_index = 0; sample_index < samples.size();
       ++sample_index) {
    const auto &sample = samples[sample_index];
    std::vector<double> q(kDof);
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      q[joint] = sample.q(static_cast<Eigen::Index>(joint));
      if (!std::isfinite(q[joint]) ||
          q[joint] < lower[joint] || q[joint] > upper[joint]) {
        throw std::runtime_error(
            "sampled q_ref violates canonical model limit at sample " +
            std::to_string(sample_index) + ", joint " +
            std::to_string(joint + 1));
      }
      if (!controller_config.joint_velocity_safety_limits.empty() &&
          std::abs(sample.qd(static_cast<Eigen::Index>(joint))) >
              controller_config.joint_velocity_safety_limits[joint] + 1e-12) {
        throw std::runtime_error(
            "sampled qd_ref violates controller velocity safety limit at sample " +
            std::to_string(sample_index) + ", joint " +
            std::to_string(joint + 1));
      }
    }
    if (checker.checkCollision(q)) {
      checker.printCollisions();
      throw std::runtime_error(
          "sampled q_ref collision/contact precheck failed at sample " +
          std::to_string(sample_index));
    }
  }
}

} // namespace

int main(int argc, char **argv) {
  try {
    Options options = parseOptions(argc, argv);
    options.controller_config = absolutePath(options.controller_config);
    options.model = absolutePath(options.model);
    options.output = absolutePath(options.output);
    options.metadata = absolutePath(options.metadata);
    if (!options.coefficients.empty()) {
      options.coefficients = absolutePath(options.coefficients);
    }
    if (!options.generated_coefficients.empty()) {
      options.generated_coefficients =
          absolutePath(options.generated_coefficients);
    }

    if (!fs::is_regular_file(options.controller_config) ||
        !fs::is_regular_file(options.model)) {
      throw std::runtime_error("controller config/model input is missing");
    }
    ensureOutputAllowed(options.output, options.overwrite);
    ensureOutputAllowed(options.metadata, options.overwrite);

    auto controller_config =
        force_node::ForceController::loadConfig(options.controller_config);
    validateControllerConfig(controller_config);

    Eigen::VectorXd q0(static_cast<Eigen::Index>(kDof));
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      q0(static_cast<Eigen::Index>(joint)) =
          controller_config.target_position[joint];
    }

    fs::path coefficient_path = options.coefficients;
    std::string provenance = "existing_coefficient_file";
    std::uint32_t source_seed =
        options.seed.value_or(controller_config.trajectory_seed);
    std::size_t source_attempt = options.accepted_attempt.value_or(0);
    if (!options.generated_coefficients.empty()) {
      coefficient_path = options.generated_coefficients;
      ensureOutputAllowed(coefficient_path, options.overwrite);
      trajectory::FourierTrajectory generated(
          kDof, controller_config.trajectory_harmonics,
          controller_config.trajectory_duration, q0);
      std::mt19937 generator(source_seed);
      for (std::size_t attempt = 1; attempt <= source_attempt; ++attempt) {
        generated.setRandomCoefficients(
            controller_config.trajectory_coefficient_scale, generator);
      }
      generated.saveCoefficients(coefficient_path);
      const std::string generated_sha = sha256File(coefficient_path);
      if (generated_sha != options.expected_coefficient_sha256) {
        throw std::runtime_error(
            "generated coefficient SHA-256 does not match accepted provenance: " +
            generated_sha);
      }
      provenance = "phase5b_accepted_random_stream_attempt_sha256_verified";
    }

    if (!fs::is_regular_file(coefficient_path)) {
      throw std::runtime_error("coefficient file does not exist: " +
                               coefficient_path.string());
    }
    const std::string coefficient_sha = sha256File(coefficient_path);
    if (!options.expected_coefficient_sha256.empty() &&
        coefficient_sha != options.expected_coefficient_sha256) {
      throw std::runtime_error(
          "coefficient SHA-256 does not match expected accepted hash");
    }

    // Reuse ForceController's accepted replay gate. It reloads the coefficient
    // file and checks model position limits, velocity safety and collision.
    auto controller_precheck_config = controller_config;
    controller_precheck_config.trajectory_coefficients_file = coefficient_path;
    force_node::ForceController controller_precheck(controller_precheck_config,
                                                    options.model);
    (void)controller_precheck;

    // Reload the exact accepted bytes before sampling. No in-memory generated
    // coefficient object is reused for the replay artifact.
    trajectory::FourierTrajectory trajectory(
        kDof, controller_config.trajectory_harmonics,
        controller_config.trajectory_duration, q0);
    trajectory.loadCoefficients(coefficient_path);

    const double duration =
        options.duration_s.value_or(controller_config.trajectory_duration);
    if (duration > controller_config.trajectory_duration + 1e-12) {
      throw std::runtime_error(
          "--duration cannot exceed the coefficient/controller period");
    }
    const auto samples =
        sampleTrajectory(trajectory, duration, options.sample_rate_hz);
    const auto dense_quintic_samples = denseQuinticPath(samples);

    // Stronger replay-specific check: every frozen q_ref sample is applied to
    // the canonical collision model before any artifact is accepted.
    runEverySampleModelPrecheck(options.model, controller_config,
                                dense_quintic_samples);

    {
      std::ofstream output(options.output);
      if (!output) {
        throw std::runtime_error("cannot write replay artifact");
      }
      output << "time";
      for (std::size_t joint = 0; joint < kDof; ++joint) {
        output << ",q_ref" << joint;
      }
      for (std::size_t joint = 0; joint < kDof; ++joint) {
        output << ",qd_ref" << joint;
      }
      for (std::size_t joint = 0; joint < kDof; ++joint) {
        output << ",qdd_ref" << joint;
      }
      output << "\n" << std::setprecision(17);
      for (const auto &sample : samples) {
        output << sample.time;
        for (std::size_t joint = 0; joint < kDof; ++joint) {
          output << "," << sample.q(static_cast<Eigen::Index>(joint));
        }
        for (std::size_t joint = 0; joint < kDof; ++joint) {
          output << "," << sample.qd(static_cast<Eigen::Index>(joint));
        }
        for (std::size_t joint = 0; joint < kDof; ++joint) {
          output << "," << sample.qdd(static_cast<Eigen::Index>(joint));
        }
        output << "\n";
      }
    }

    const std::string trajectory_sha = sha256File(options.output);
    const auto &start = samples.front();
    {
      std::ofstream metadata(options.metadata);
      if (!metadata) {
        throw std::runtime_error("cannot write replay metadata");
      }
      metadata << std::setprecision(17);
      metadata << "schema_version: rebot_replay_trajectory_v1\n";
      metadata << "robot: rebot_dm\n";
      metadata << "dof: " << kDof << "\n";
      metadata << "duration_s: " << duration << "\n";
      metadata << "sample_rate_hz: " << options.sample_rate_hz << "\n";
      metadata << "sample_count: " << samples.size() << "\n";
      metadata << "source_type: cxx_fourier_trajectory\n";
      metadata << "source_provenance: " << yamlQuote(provenance) << "\n";
      metadata << "source_seed: " << source_seed << "\n";
      metadata << "source_accepted_attempt: " << source_attempt << "\n";
      metadata << "source_coefficient_scale: "
               << controller_config.trajectory_coefficient_scale << "\n";
      metadata << "source_coefficient_file: "
               << yamlQuote(coefficient_path.string()) << "\n";
      metadata << "source_coefficient_sha256: " << coefficient_sha << "\n";
      metadata << "trajectory_sha256: " << trajectory_sha << "\n";
      metadata << "generator_git_commit: " << yamlQuote(PROJECT_GIT_COMMIT)
               << "\n";
      metadata << "q_start: ";
      writeVector(metadata, start.q);
      metadata << "\n";
      metadata << "j1_convention: "
               << yamlQuote(
                      "canonical reBot-DM model convention; SDK J1 "
                      "principal-angle convention remains unresolved")
               << "\n";
      metadata << "j6_geometry_status: "
               << yamlQuote(
                      "canonical reBot-DM geometry retained; SDK/canonical "
                      "J6 geometry difference remains unresolved")
               << "\n";
      metadata << "model_file: " << yamlQuote(options.model.string()) << "\n";
      metadata << "model_hash: " << sha256File(options.model) << "\n";
      metadata << "limits_config_file: "
               << yamlQuote(options.controller_config.string()) << "\n";
      metadata << "limits_config_hash: "
               << sha256File(options.controller_config) << "\n";
      metadata << "source_controller_precheck: PASS\n";
      metadata << "collision_model_file: "
               << yamlQuote(options.model.string()) << "\n";
      metadata << "collision_model_hash: " << sha256File(options.model) << "\n";
      metadata << "collision_precheck: PASS\n";
      metadata << "collision_precheck_sample_count: " << samples.size() << "\n";
      metadata << "continuous_quintic_collision_precheck: PASS\n";
      metadata << "continuous_quintic_collision_subdivisions_per_interval: "
               << kQuinticCollisionSubdivisions << "\n";
      metadata << "continuous_quintic_collision_precheck_sample_count: "
               << dense_quintic_samples.size() << "\n";
      metadata << "sample_rate_status: "
                  "candidate_replay_rate_not_hardware_certified\n";
    }

    std::cout << "coefficients=" << coefficient_path << "\n"
              << "coefficient_sha256=" << coefficient_sha << "\n"
              << "artifact=" << options.output << "\n"
              << "metadata=" << options.metadata << "\n"
              << "trajectory_sha256=" << trajectory_sha << "\n"
              << "source_controller_precheck=PASS\n"
              << "continuous_quintic_collision_precheck=PASS samples="
              << dense_quintic_samples.size() << "\n"
              << "sample_count=" << samples.size()
              << " sample_rate_hz=" << options.sample_rate_hz
              << " duration_s=" << duration << std::endl;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "rebot_trajectory_exporter failed: " << error.what()
              << std::endl;
    return 1;
  }
}
