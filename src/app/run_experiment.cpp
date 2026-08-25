#include "app/experiment_backend.hpp"
#include "app/experiment_recorder.hpp"
#include "app/piper_hardware_backend.hpp"
#include "app/simulation_backend.hpp"
#include "force_node/force_controller.hpp"

#include <openssl/evp.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

fs::path repoRoot() {
  return fs::path(PROJECT_ROOT_DIR);
}

fs::path readArgPath(int argc, char **argv, const std::string &flag,
                     const fs::path &default_value) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == flag) {
      return fs::path(argv[i + 1]);
    }
  }
  return default_value;
}

bool hasFlag(int argc, char **argv, const std::string &flag) {
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == flag) {
      return true;
    }
  }
  return false;
}

/** Read an optional unsigned command-line value following a flag. */
std::optional<std::uint32_t> readArgUnsigned(int argc, char **argv,
                                            const std::string &flag) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == flag) {
      return static_cast<std::uint32_t>(std::stoul(argv[i + 1]));
    }
  }
  return std::nullopt;
}

/** Derive the trajectory sidecar name from the dataset path. */
fs::path defaultTrajectoryFile(const fs::path &dataset_path) {
  fs::path result = dataset_path;
  result.replace_extension(".trajectory.csv");
  return result;
}

/** Compute the SHA-256 digest of an accepted Fourier coefficient file. */
std::string sha256File(const fs::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("无法读取待哈希的 Fourier 系数: " +
                             path.string());
  }
  using DigestContext = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
  DigestContext context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("无法初始化 SHA-256");
  }
  std::array<char, 8192> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = input.gcount();
    if (count > 0 &&
        EVP_DigestUpdate(context.get(), buffer.data(),
                         static_cast<std::size_t>(count)) != 1) {
      throw std::runtime_error("无法更新 Fourier 系数 SHA-256");
    }
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1) {
    throw std::runtime_error("无法完成 Fourier 系数 SHA-256");
  }
  std::ostringstream hexadecimal;
  hexadecimal << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < digest_size; ++index) {
    hexadecimal << std::setw(2) << static_cast<unsigned>(digest[index]);
  }
  return hexadecimal.str();
}

fs::path defaultRecordFile() {
  return repoRoot() / "data" / "benchmark_data.csv";
}

struct ExperimentConfig {
  std::string robot = "panda";
  std::string backend = "sim";
  fs::path controller_config;
  fs::path sim_config;
  fs::path scene_path;
  fs::path collision_model;
  fs::path hardware_config;
};

std::string trimCopy(const std::string &input) {
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

std::optional<std::string> parseYamlString(const std::string &trimmed,
                                           const std::string &key) {
  if (trimmed.rfind(key, 0) != 0) {
    return std::nullopt;
  }

  std::size_t pos = key.size();
  while (pos < trimmed.size() &&
         std::isspace(static_cast<unsigned char>(trimmed[pos]))) {
    ++pos;
  }
  if (pos >= trimmed.size() || trimmed[pos] != ':') {
    return std::nullopt;
  }

  std::string value = trimCopy(trimmed.substr(pos + 1));
  const auto comment = value.find('#');
  if (comment != std::string::npos) {
    value = trimCopy(value.substr(0, comment));
  }
  if (value.size() >= 2 &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    value = value.substr(1, value.size() - 2);
  }
  return value.empty() ? std::nullopt : std::optional<std::string>(value);
}

fs::path resolveRepoPath(const fs::path &path_value) {
  if (path_value.empty() || path_value.is_absolute()) {
    return path_value;
  }
  return repoRoot() / path_value;
}

ExperimentConfig defaultExperimentConfigForRobot(const std::string &robot) {
  ExperimentConfig config;
  config.robot = robot;
  config.backend = "sim";
  config.sim_config = repoRoot() / "config" / "panda_sim_node.yaml";
  config.hardware_config = repoRoot() / "config" / "piper_real_experiment.yaml";

  if (robot == "piper") {
    config.controller_config =
        repoRoot() / "config" / "piper_force_controller_node.yaml";
    config.scene_path = repoRoot() / "piper" / "scene_identification.xml";
    config.sim_config = repoRoot() / "config" / "piper_sim_node.yaml";
    config.collision_model = repoRoot() / "piper" / "piper.xml";
    return config;
  }

  if (robot == "rebot_dm") {
    config.controller_config =
        repoRoot() / "config" / "rebot_dm_force_controller_node.yaml";
    config.scene_path = repoRoot() / "rebot_dm" / "scene_runtime.xml";
    config.sim_config = repoRoot() / "config" / "rebot_dm_sim_node.yaml";
    config.collision_model = repoRoot() / "rebot_dm" / "rebot_dm_runtime.xml";
    return config;
  }

  if (robot != "panda") {
    throw std::runtime_error("不支持的机械臂类型: " + robot);
  }

  config.controller_config = repoRoot() / "config" / "force_controller_node.yaml";
  config.scene_path = repoRoot() / "franka_emika_panda" / "scene.xml";
  config.collision_model = repoRoot() / "franka_emika_panda" / "panda.xml";
  return config;
}

ExperimentConfig loadExperimentConfig(const fs::path &config_path) {
  ExperimentConfig parsed;

  std::ifstream file(config_path);
  if (file) {
    std::string line;
    while (std::getline(file, line)) {
      const std::string trimmed = trimCopy(line);
      if (trimmed.empty() || trimmed.front() == '#') {
        continue;
      }
      if (const auto robot = parseYamlString(trimmed, "robot")) {
        parsed.robot = *robot;
      } else if (const auto backend = parseYamlString(trimmed, "backend")) {
        parsed.backend = *backend;
      } else if (const auto controller =
                     parseYamlString(trimmed, "controller_config")) {
        parsed.controller_config = resolveRepoPath(*controller);
      } else if (const auto sim = parseYamlString(trimmed, "sim_config")) {
        parsed.sim_config = resolveRepoPath(*sim);
      } else if (const auto scene = parseYamlString(trimmed, "scene")) {
        parsed.scene_path = resolveRepoPath(*scene);
      } else if (const auto collision =
                     parseYamlString(trimmed, "collision_model")) {
        parsed.collision_model = resolveRepoPath(*collision);
      } else if (const auto hardware =
                     parseYamlString(trimmed, "hardware_config")) {
        parsed.hardware_config = resolveRepoPath(*hardware);
      }
    }
  }

  auto resolved = defaultExperimentConfigForRobot(parsed.robot);
  if (!parsed.controller_config.empty()) {
    resolved.controller_config = parsed.controller_config;
  }
  if (!parsed.sim_config.empty()) {
    resolved.sim_config = parsed.sim_config;
  }
  if (!parsed.scene_path.empty()) {
    resolved.scene_path = parsed.scene_path;
  }
  if (!parsed.collision_model.empty()) {
    resolved.collision_model = parsed.collision_model;
  }
  if (!parsed.backend.empty()) {
    resolved.backend = parsed.backend;
  }
  if (!parsed.hardware_config.empty()) {
    resolved.hardware_config = parsed.hardware_config;
  }
  return resolved;
}

} // namespace

int main(int argc, char **argv) {
  try {
    const fs::path experiment_config_path = readArgPath(
        argc, argv, "--experiment-config",
        repoRoot() / "config" / "experiment.yaml");
    const ExperimentConfig experiment_config =
        loadExperimentConfig(experiment_config_path);

    const fs::path force_config = readArgPath(
        argc, argv, "--controller-config",
        experiment_config.controller_config);
    const fs::path sim_config = readArgPath(
        argc, argv, "--sim-config",
        experiment_config.sim_config);
    const fs::path scene_path = readArgPath(
        argc, argv, "--scene",
        experiment_config.scene_path);
    const fs::path collision_model = readArgPath(
        argc, argv, "--collision-model",
        experiment_config.collision_model);
    const fs::path output_csv =
        readArgPath(argc, argv, "--output", defaultRecordFile());
    const fs::path trajectory_input =
        readArgPath(argc, argv, "--trajectory-input", {});
    const fs::path trajectory_output = readArgPath(
        argc, argv, "--trajectory-output", defaultTrajectoryFile(output_csv));
    const bool headless = hasFlag(argc, argv, "--headless");

    auto controller_config = force_node::ForceController::loadConfig(force_config);
    if (!controller_config.robot.empty() &&
        controller_config.robot != experiment_config.robot) {
      throw std::runtime_error("controller config robot 与 experiment robot 不一致");
    }
    if (const auto seed = readArgUnsigned(argc, argv, "--trajectory-seed")) {
      controller_config.trajectory_seed = *seed;
    }
    if (!trajectory_input.empty()) {
      controller_config.trajectory_coefficients_file = trajectory_input;
    }
    force_node::ForceController controller(controller_config, collision_model);
    if (experiment_config.robot == "rebot_dm") {
      if (experiment_config.backend != "sim") {
        throw std::runtime_error("rebot_dm Phase 4B 仅支持 sim 后端");
      }
      if (controller.armDOF() != 6) {
        throw std::runtime_error("rebot_dm 必须严格控制 J1-J6 六个机械臂自由度");
      }
    }

    std::string trajectory_sha256;
    fs::path recorded_trajectory_output;
    if (controller.usesExcitationTrajectory()) {
      controller.saveTrajectoryCoefficients(trajectory_output);
      trajectory_sha256 = sha256File(trajectory_output);
      recorded_trajectory_output = trajectory_output;
    }
    std::unique_ptr<app::ExperimentBackend> backend;
    std::vector<double> joint_armature;
    std::vector<double> joint_damping;
    std::vector<double> joint_frictionloss;

    if (experiment_config.backend == "piper_real") {
      if (experiment_config.robot != "piper") {
        throw std::runtime_error("piper_real 后端仅支持 piper 机械臂");
      }
      backend = std::make_unique<app::PiperHardwareBackend>(
          repoRoot() / "src" / "app" / "piper_backend_bridge.py",
          experiment_config.hardware_config, 1.0 / controller.controlRateHz());
    } else {
      auto sim_config_loaded = sim_com_node::PandaSimulator::loadConfig(sim_config);
      joint_armature = sim_config_loaded.joint_armature;
      joint_damping = sim_config_loaded.joint_damping;
      joint_frictionloss = sim_config_loaded.joint_frictionloss;
      sim_config_loaded.enable_viewer =
          !headless && sim_config_loaded.enable_viewer;
      backend = std::make_unique<app::SimulationBackend>(
          sim_config_loaded, scene_path, controller.armDOF());
    }

    std::vector<double> gripper_lock_position;
    std::vector<double> armature_truth;
    std::vector<double> damping_truth;
    if (experiment_config.robot == "rebot_dm") {
      gripper_lock_position = {0.05, 0.05};
      armature_truth = joint_armature.empty() ? std::vector<double>(6, 0.0)
                                               : joint_armature;
      damping_truth = joint_damping.empty() ? std::vector<double>(6, 0.0)
                                             : joint_damping;
      if (joint_frictionloss.empty()) {
        joint_frictionloss.assign(6, 0.0);
      }
    }

    app::ExperimentRecorder recorder(
        output_csv, controller.armDOF(), experiment_config.backend == "sim");
    recorder.writeMetadata(app::ExperimentMetadata{
        experiment_config.robot, experiment_config.backend, scene_path,
        collision_model, force_config, sim_config, PROJECT_GIT_COMMIT,
        backend->timeStep(), controller.controllerMode(), controller.trajectorySeed(),
        controller.trajectoryHarmonics(), controller.acceptedTrajectoryScale(),
        controller.acceptedTrajectoryAttempt(), controller.trajectoryReplayFile(),
        recorded_trajectory_output, trajectory_sha256, gripper_lock_position,
        armature_truth, damping_truth, joint_frictionloss});

    const double end_time = controller.trajectoryDuration();
    auto state = backend->initialize();
    while (backend->simulationTime() < end_time) {
      force_node::JointSample sample{state.position, state.velocity, state.effort};
      const auto command =
          controller.computeCommand(sample, backend->simulationTime());
      state = backend->step(command);
      recorder.record(backend->simulationTime(), state, command);
    }

    std::cout << "实验完成，数据已保存到: " << output_csv << std::endl;
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "run_experiment 失败: " << e.what() << std::endl;
    return 1;
  }
}
