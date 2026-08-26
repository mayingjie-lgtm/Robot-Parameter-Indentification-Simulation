#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <sys/wait.h>

namespace fs = std::filesystem;

namespace {

constexpr std::size_t kDof = 6;
constexpr double kContinuityTolerance = 1e-9;

struct RenderOptions {
  fs::path input;
  fs::path output;
  fs::path scene =
      fs::path(PROJECT_ROOT_DIR) / "rebot_dm" / "scene_runtime.xml";
  int fps = 60;
  int width = 1280;
  int height = 720;
  double playback_speed = 1.0;
  std::optional<double> start_time;
  std::optional<double> duration;
  bool overwrite = false;
};

struct TrajectoryPoint {
  double time{0.0};
  std::array<double, kDof> position{};
};

/** Print the supported command-line interface. */
void printUsage() {
  std::cout
      << "Usage: rebot_trajectory_renderer --input <csv> --output <mp4> "
         "[options]\n"
      << "Options:\n"
      << "  --scene <xml>             MuJoCo scene (default: reBot runtime "
         "scene)\n"
      << "  --fps <integer>           Video frame rate (default: 60)\n"
      << "  --width <even integer>    Frame width (default: 1280)\n"
      << "  --height <even integer>   Frame height (default: 720)\n"
      << "  --playback-speed <value>  Simulation seconds per video second "
         "(default: 1)\n"
      << "  --start-time <seconds>    Absolute CSV time at which to start\n"
      << "  --duration <seconds>      Simulation-time duration to render\n"
      << "  --overwrite               Replace an existing output file\n"
      << "  --help                    Show this message\n";
}

/** Return the value following one command-line option. */
std::string requireArgumentValue(int argc, char **argv, int &index) {
  if (index + 1 >= argc) {
    throw std::runtime_error(std::string("参数缺少值: ") + argv[index]);
  }
  return argv[++index];
}

/** Parse a finite floating-point command-line value. */
double parseDouble(const std::string &text, const std::string &name) {
  std::size_t parsed = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &parsed);
  } catch (const std::exception &) {
    throw std::runtime_error(name + " 必须是数值");
  }
  if (parsed != text.size() || !std::isfinite(value)) {
    throw std::runtime_error(name + " 必须是有限数值");
  }
  return value;
}

/** Parse a positive integer command-line value. */
int parsePositiveInteger(const std::string &text, const std::string &name) {
  std::size_t parsed = 0;
  long value = 0;
  try {
    value = std::stol(text, &parsed);
  } catch (const std::exception &) {
    throw std::runtime_error(name + " 必须是正整数");
  }
  if (parsed != text.size() || value <= 0 || value > 16384) {
    throw std::runtime_error(name + " 超出有效范围");
  }
  return static_cast<int>(value);
}

/** Parse and validate renderer command-line options. */
RenderOptions parseOptions(int argc, char **argv) {
  RenderOptions options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--input") {
      options.input = requireArgumentValue(argc, argv, index);
    } else if (argument == "--output") {
      options.output = requireArgumentValue(argc, argv, index);
    } else if (argument == "--scene") {
      options.scene = requireArgumentValue(argc, argv, index);
    } else if (argument == "--fps") {
      options.fps = parsePositiveInteger(
          requireArgumentValue(argc, argv, index), "--fps");
    } else if (argument == "--width") {
      options.width = parsePositiveInteger(
          requireArgumentValue(argc, argv, index), "--width");
    } else if (argument == "--height") {
      options.height = parsePositiveInteger(
          requireArgumentValue(argc, argv, index), "--height");
    } else if (argument == "--playback-speed") {
      options.playback_speed = parseDouble(
          requireArgumentValue(argc, argv, index), "--playback-speed");
    } else if (argument == "--start-time") {
      options.start_time =
          parseDouble(requireArgumentValue(argc, argv, index), "--start-time");
    } else if (argument == "--duration") {
      options.duration =
          parseDouble(requireArgumentValue(argc, argv, index), "--duration");
    } else if (argument == "--overwrite") {
      options.overwrite = true;
    } else if (argument == "--help") {
      printUsage();
      std::exit(0);
    } else {
      throw std::runtime_error("未知参数: " + argument);
    }
  }

  if (options.input.empty() || options.output.empty()) {
    throw std::runtime_error("必须同时指定 --input 和 --output");
  }
  if (options.playback_speed <= 0.0) {
    throw std::runtime_error("--playback-speed 必须大于 0");
  }
  if (options.duration && *options.duration <= 0.0) {
    throw std::runtime_error("--duration 必须大于 0");
  }
  if (options.width % 2 != 0 || options.height % 2 != 0) {
    throw std::runtime_error("--width 和 --height 必须是正偶数");
  }
  if (options.output.extension() != ".mp4") {
    throw std::runtime_error("--output 必须使用 .mp4 扩展名");
  }
  return options;
}

/** Split one project-generated CSV row. */
std::vector<std::string> splitCsv(const std::string &line) {
  std::vector<std::string> values;
  std::stringstream stream(line);
  std::string value;
  while (std::getline(stream, value, ',')) {
    if (!value.empty() && value.back() == '\r') {
      value.pop_back();
    }
    values.push_back(value);
  }
  return values;
}

/** Resolve one required CSV column by exact header name. */
std::size_t
requiredColumn(const std::unordered_map<std::string, std::size_t> &columns,
               const std::string &name) {
  const auto found = columns.find(name);
  if (found == columns.end()) {
    throw std::runtime_error("CSV 缺少明确要求的列: " + name);
  }
  return found->second;
}

/** Parse a finite CSV value and include the source line in errors. */
double parseCsvValue(const std::vector<std::string> &row, std::size_t column,
                     std::size_t line_number) {
  if (column >= row.size()) {
    throw std::runtime_error("CSV 第 " + std::to_string(line_number) +
                             " 行列数不足");
  }
  const double value =
      parseDouble(row[column], "CSV 第 " + std::to_string(line_number) + " 行");
  return value;
}

/** Load the exact reBot simulation states needed for visual replay. */
std::vector<TrajectoryPoint> loadTrajectory(const fs::path &path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("无法读取轨迹 CSV: " + path.string());
  }

  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("轨迹 CSV 为空: " + path.string());
  }
  const auto header = splitCsv(line);
  std::unordered_map<std::string, std::size_t> columns;
  for (std::size_t index = 0; index < header.size(); ++index) {
    if (!columns.emplace(header[index], index).second) {
      throw std::runtime_error("CSV header 包含重复列: " + header[index]);
    }
  }

  const std::size_t begin_column = requiredColumn(columns, "time_begin");
  const std::size_t end_column = requiredColumn(columns, "time_end");
  std::array<std::size_t, kDof> q_columns{};
  std::array<std::size_t, kDof> q_next_columns{};
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    q_columns[joint] = requiredColumn(columns, "q" + std::to_string(joint));
    q_next_columns[joint] =
        requiredColumn(columns, "q_next" + std::to_string(joint));
  }

  std::vector<TrajectoryPoint> points;
  std::array<double, kDof> previous_next{};
  double previous_end = 0.0;
  std::size_t line_number = 1;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty()) {
      continue;
    }
    const auto row = splitCsv(line);
    const double begin = parseCsvValue(row, begin_column, line_number);
    const double end = parseCsvValue(row, end_column, line_number);
    if (end <= begin) {
      throw std::runtime_error("CSV 第 " + std::to_string(line_number) +
                               " 行 time_end 必须大于 time_begin");
    }

    TrajectoryPoint point;
    point.time = begin;
    std::array<double, kDof> next{};
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      point.position[joint] = parseCsvValue(row, q_columns[joint], line_number);
      next[joint] = parseCsvValue(row, q_next_columns[joint], line_number);
    }

    if (!points.empty()) {
      if (begin <= points.back().time) {
        throw std::runtime_error("CSV time_begin 必须严格递增");
      }
      if (std::abs(begin - previous_end) > kContinuityTolerance) {
        throw std::runtime_error("CSV 相邻积分区间时间不连续");
      }
      for (std::size_t joint = 0; joint < kDof; ++joint) {
        if (std::abs(point.position[joint] - previous_next[joint]) >
            kContinuityTolerance) {
          throw std::runtime_error("CSV q 与上一行 q_next 不连续");
        }
      }
    }

    points.push_back(point);
    previous_end = end;
    previous_next = next;
  }

  if (points.empty()) {
    throw std::runtime_error("轨迹 CSV 不包含数据行");
  }
  points.push_back(TrajectoryPoint{previous_end, previous_next});
  return points;
}

/** Interpolate six joint positions at one simulation timestamp. */
std::array<double, kDof>
interpolatePosition(const std::vector<TrajectoryPoint> &points, double time) {
  if (time <= points.front().time) {
    return points.front().position;
  }
  if (time >= points.back().time) {
    return points.back().position;
  }

  const auto upper =
      std::upper_bound(points.begin(), points.end(), time,
                       [](double value, const TrajectoryPoint &point) {
                         return value < point.time;
                       });
  const auto lower = std::prev(upper);
  const double alpha = (time - lower->time) / (upper->time - lower->time);
  std::array<double, kDof> result{};
  for (std::size_t joint = 0; joint < kDof; ++joint) {
    result[joint] = lower->position[joint] +
                    alpha * (upper->position[joint] - lower->position[joint]);
  }
  return result;
}

/** Quote one path for the POSIX shell used by popen. */
std::string shellQuote(const std::string &value) {
  std::string result = "'";
  for (const char character : value) {
    if (character == '\'') {
      result += "'\\''";
    } else {
      result += character;
    }
  }
  result += "'";
  return result;
}

/** Own the FFmpeg stdin pipe and verify encoder completion. */
class FfmpegPipe {
public:
  FfmpegPipe(const RenderOptions &options) {
    std::signal(SIGPIPE, SIG_IGN);
    const std::string overwrite = options.overwrite ? "-y" : "-n";
    const std::string dimensions =
        std::to_string(options.width) + "x" + std::to_string(options.height);
    const std::string command =
        "ffmpeg -loglevel error -nostdin " + overwrite +
        " -f rawvideo -pixel_format rgb24 -video_size " + dimensions +
        " -framerate " + std::to_string(options.fps) +
        " -i pipe:0 -vf vflip -an -c:v libx264 -preset medium -crf 18"
        " -pix_fmt yuv420p -movflags +faststart " +
        shellQuote(options.output.string());
    pipe_ = popen(command.c_str(), "w");
    if (!pipe_) {
      throw std::runtime_error(
          "无法启动 FFmpeg，请确认 ffmpeg 已安装并位于 PATH");
    }
  }

  FfmpegPipe(const FfmpegPipe &) = delete;
  FfmpegPipe &operator=(const FfmpegPipe &) = delete;

  ~FfmpegPipe() {
    if (pipe_) {
      pclose(pipe_);
    }
  }

  /** Write one tightly packed RGB frame to FFmpeg. */
  void writeFrame(const std::vector<unsigned char> &rgb) {
    if (std::fwrite(rgb.data(), 1, rgb.size(), pipe_) != rgb.size()) {
      throw std::runtime_error("FFmpeg 提前关闭，视频帧写入失败");
    }
  }

  /** Close the encoder and reject unsuccessful FFmpeg exit status. */
  void finish() {
    if (!pipe_) {
      return;
    }
    FILE *pipe = pipe_;
    pipe_ = nullptr;
    const int status = pclose(pipe);
    if (status == -1 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      throw std::runtime_error("FFmpeg 编码失败，请检查上方错误信息");
    }
  }

private:
  FILE *pipe_{nullptr};
};

/** Own one GLFW process lifetime for the hidden OpenGL context. */
class GlfwSession {
public:
  GlfwSession() {
    if (!glfwInit()) {
      throw std::runtime_error("GLFW 初始化失败；离线渲染仍需要可用的显示环境");
    }
  }

  GlfwSession(const GlfwSession &) = delete;
  GlfwSession &operator=(const GlfwSession &) = delete;

  ~GlfwSession() { glfwTerminate(); }
};

/** Destroy a GLFW window before terminating the GLFW session. */
struct GlfwWindowDeleter {
  void operator()(GLFWwindow *window) const {
    if (window) {
      glfwDestroyWindow(window);
    }
  }
};

/** Render deterministic reBot poses into an offscreen RGB framebuffer. */
class RebotRenderer {
public:
  RebotRenderer(const fs::path &scene_path, int width, int height)
      : width_(width), height_(height) {
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window_.reset(
        glfwCreateWindow(64, 64, "reBot offline renderer", nullptr, nullptr));
    if (!window_) {
      throw std::runtime_error("无法创建隐藏 OpenGL 上下文");
    }
    glfwMakeContextCurrent(window_.get());

    char error[1024]{};
    model_.reset(
        mj_loadXML(scene_path.string().c_str(), nullptr, error, sizeof(error)));
    if (!model_) {
      throw std::runtime_error("MuJoCo 场景加载失败: " + std::string(error));
    }
    data_.reset(mj_makeData(model_.get()));
    if (!data_) {
      throw std::runtime_error("无法分配 MuJoCo 数据结构");
    }

    const int keyframe = mj_name2id(model_.get(), mjOBJ_KEY, "runtime_home");
    if (keyframe < 0) {
      throw std::runtime_error("reBot scene 缺少 runtime_home keyframe");
    }
    mj_resetDataKeyframe(model_.get(), data_.get(), keyframe);
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      const std::string name = "joint" + std::to_string(joint + 1);
      const int id = mj_name2id(model_.get(), mjOBJ_JOINT, name.c_str());
      if (id < 0 || model_->jnt_type[id] == mjJNT_FREE ||
          model_->jnt_type[id] == mjJNT_BALL) {
        throw std::runtime_error("reBot scene 缺少标量关节: " + name);
      }
      qpos_indices_[joint] = model_->jnt_qposadr[id];
    }

    model_->vis.global.offwidth = width_;
    model_->vis.global.offheight = height_;
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&visual_option_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);
    mjv_makeScene(model_.get(), &scene_, 1000);
    scene_ready_ = true;
    mjr_makeContext(model_.get(), &context_, mjFONTSCALE_150);
    context_ready_ = true;
    mjr_setBuffer(mjFB_OFFSCREEN, &context_);

    // A model-relative free camera stays reproducible if mesh bounds change.
    camera_.type = mjCAMERA_FREE;
    for (int axis = 0; axis < 3; ++axis) {
      camera_.lookat[axis] = model_->stat.center[axis];
    }
    camera_.distance = 1.8 * model_->stat.extent;
    camera_.azimuth = 135.0;
    camera_.elevation = -20.0;
  }

  RebotRenderer(const RebotRenderer &) = delete;
  RebotRenderer &operator=(const RebotRenderer &) = delete;

  ~RebotRenderer() {
    if (context_ready_) {
      mjr_freeContext(&context_);
    }
    if (scene_ready_) {
      mjv_freeScene(&scene_);
    }
  }

  /** Apply one arm position and render it without advancing simulation time. */
  void render(const std::array<double, kDof> &position,
              std::vector<unsigned char> &rgb) {
    for (std::size_t joint = 0; joint < kDof; ++joint) {
      data_->qpos[qpos_indices_[joint]] = position[joint];
    }
    // mj_forward updates geometry poses only; it does not integrate dynamics.
    mj_forward(model_.get(), data_.get());
    mjv_updateScene(model_.get(), data_.get(), &visual_option_, nullptr,
                    &camera_, mjCAT_ALL, &scene_);
    const mjrRect viewport{0, 0, width_, height_};
    mjr_render(viewport, &scene_, &context_);
    mjr_readPixels(rgb.data(), nullptr, viewport, &context_);
  }

private:
  GlfwSession glfw_;
  std::unique_ptr<GLFWwindow, GlfwWindowDeleter> window_;
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr,
                                                             mj_deleteModel};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr,
                                                          mj_deleteData};
  int width_{0};
  int height_{0};
  std::array<int, kDof> qpos_indices_{};
  mjvCamera camera_{};
  mjvOption visual_option_{};
  mjvScene scene_{};
  mjrContext context_{};
  bool scene_ready_{false};
  bool context_ready_{false};
};

/** Validate paths, output setup, and the FFmpeg runtime dependency. */
void preparePaths(const RenderOptions &options) {
  if (!fs::is_regular_file(options.input)) {
    throw std::runtime_error("输入 CSV 不存在: " + options.input.string());
  }
  if (!fs::is_regular_file(options.scene)) {
    throw std::runtime_error("MuJoCo scene 不存在: " + options.scene.string());
  }
  if (fs::exists(options.output) && !options.overwrite) {
    throw std::runtime_error("输出文件已存在；如需替换请添加 --overwrite: " +
                             options.output.string());
  }
  if (options.output.has_parent_path()) {
    fs::create_directories(options.output.parent_path());
  }

  const int ffmpeg_status = std::system("ffmpeg -version >/dev/null 2>&1");
  if (ffmpeg_status == -1 || !WIFEXITED(ffmpeg_status) ||
      WEXITSTATUS(ffmpeg_status) != 0) {
    throw std::runtime_error(
        "无法调用 FFmpeg，请确认 ffmpeg 已安装并位于 PATH");
  }
}

/** Render the selected trajectory interval and encode all frames. */
void renderVideo(const RenderOptions &options,
                 const std::vector<TrajectoryPoint> &points) {
  const double trajectory_begin = points.front().time;
  const double trajectory_end = points.back().time;
  const double clip_begin = options.start_time.value_or(trajectory_begin);
  const double clip_end =
      options.duration ? clip_begin + *options.duration : trajectory_end;
  if (clip_begin < trajectory_begin || clip_begin >= trajectory_end) {
    throw std::runtime_error("--start-time 超出 CSV 时间范围");
  }
  if (clip_end > trajectory_end + kContinuityTolerance ||
      clip_end <= clip_begin) {
    throw std::runtime_error("请求的渲染区间超出 CSV 时间范围");
  }

  const double video_duration =
      (clip_end - clip_begin) / options.playback_speed;
  // Recorded 1 kHz timestamps accumulate tiny binary error; keep an exact
  // 30 s, 60 fps trajectory at 1800 frames instead of creating a spurious one.
  const double nominal_frame_count = video_duration * options.fps;
  const std::size_t frame_count = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(nominal_frame_count - 1e-9)));
  std::vector<unsigned char> rgb(static_cast<std::size_t>(options.width) *
                                 static_cast<std::size_t>(options.height) * 3);

  RebotRenderer renderer(options.scene, options.width, options.height);
  FfmpegPipe encoder(options);
  const std::size_t progress_interval =
      static_cast<std::size_t>(options.fps) * 5;
  for (std::size_t frame = 0; frame < frame_count; ++frame) {
    const double simulation_time =
        std::min(clip_end, clip_begin + static_cast<double>(frame) *
                                            options.playback_speed /
                                            static_cast<double>(options.fps));
    renderer.render(interpolatePosition(points, simulation_time), rgb);
    encoder.writeFrame(rgb);
    if ((frame + 1) % progress_interval == 0 || frame + 1 == frame_count) {
      std::cout << "Rendered " << (frame + 1) << "/" << frame_count
                << " frames\r" << std::flush;
    }
  }
  std::cout << std::endl;
  encoder.finish();
  std::cout << "视频已保存到: " << options.output << "\n"
            << "frames=" << frame_count << " fps=" << options.fps
            << " playback_speed=" << options.playback_speed << std::endl;
}

} // namespace

/** Run the reBot-DM CSV-to-MP4 renderer. */
int main(int argc, char **argv) {
  try {
    const RenderOptions options = parseOptions(argc, argv);
    preparePaths(options);
    const auto trajectory = loadTrajectory(options.input);
    renderVideo(options, trajectory);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "rebot_trajectory_renderer 失败: " << error.what()
              << std::endl;
    return 1;
  }
}
