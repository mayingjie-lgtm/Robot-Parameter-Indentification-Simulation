# Robot Parameter Identification Simulation

机器人动力学参数辨识工程，包含 MuJoCo 仿真辨识、reBot-DM 动力学/回归器验证，以及 reBot 真机 A/B 激励采集与离线辨识。

> 本项目不依赖 ROS 作为运行中间层；ROS Humble 只是 Ubuntu 22.04 上获取 Pinocchio 的一种可选方式。项目参考了 [BIRDy](https://github.com/TUM-ICS/BIRDY)。

## 环境与依赖

### 支持范围

- 仿真、C++ 离线辨识：Linux/Windows；本仓库当前实机验证环境为 Ubuntu 22.04。
- reBot 真机 A/B：Linux、Python 3.10+，并需要单独提供的 reBot SDK、已验收轨迹产物与机器人网络。
- `rebot_trajectory_renderer` 只在 Linux 上构建，导出 MP4 还需要 FFmpeg 和可用的 GLFW 显示环境。

### 必需依赖

- CMake 3.21+、C++17 编译器、Git；
- Eigen3、OpenSSL development headers、GLFW3；
- MuJoCo；
- Pinocchio 4.x（reBot-DM dynamics/regressor）；
- Python 3.10+，以及 `numpy`、`scipy`、`matplotlib`、`PyYAML`；
- `pytest` 用于回归测试，FFmpeg 或 `imageio-ffmpeg` 用于轨迹视频生成。

### Ubuntu 22.04 安装示例

```bash
git clone https://github.com/mayingjie-lgtm/Robot-Parameter-Indentification-Simulation.git
cd Robot-Parameter-Indentification-Simulation

sudo apt update
sudo apt install -y \
  build-essential cmake git \
  libeigen3-dev libssl-dev libglfw3-dev \
  python3 python3-venv python3-pip ffmpeg

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install numpy scipy matplotlib PyYAML pytest imageio-ffmpeg
```

Pinocchio 需要另行安装。已配置 ROS Humble 软件源时，可使用：

```bash
sudo apt install ros-humble-pinocchio

# Bash
source /opt/ros/humble/setup.bash

# Zsh 改用：source /opt/ros/humble/setup.zsh
```

也可使用 conda-forge 等方式安装 Pinocchio；关键是 `pinocchioConfig.cmake` 必须能被 CMake 找到。若不在默认搜索路径，配置时传入 `-Dpinocchio_DIR=<.../lib/cmake/pinocchio>`。

MuJoCo 请从官方 release 安装。若未安装到 `/usr` 或 `/usr/local`，设置：

```bash
export MUJOCO_DIR=/path/to/mujoco
export LD_LIBRARY_PATH="$MUJOCO_DIR/lib:${LD_LIBRARY_PATH:-}"
```

### 配置与构建

推荐统一构建到 `build_rebot/`，因为默认真机配置会在该目录查找 `identify`：

```bash
cmake -S . -B build_rebot -DCMAKE_BUILD_TYPE=Release
cmake --build build_rebot --parallel
```

若 MuJoCo、Pinocchio 或 GLFW 位于非标准路径，可显式配置：

```bash
cmake -S . -B build_rebot -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_INCLUDE_DIR="$MUJOCO_DIR/include" \
  -DMUJOCO_LIB="$MUJOCO_DIR/lib/libmujoco.so" \
  -Dpinocchio_DIR=/path/to/pinocchio/lib/cmake/pinocchio \
  -Dglfw3_DIR=/path/to/glfw/lib/cmake/glfw3
cmake --build build_rebot --parallel
```

构建完成后先运行不接触硬件的检查：

```bash
python3 -c "import numpy, scipy, matplotlib, yaml"
ctest --test-dir build_rebot --output-on-failure
```

### 真机额外前置条件

Git 仓库本身不包含外部 reBot SDK；`data/`、`results/` 和构建目录也被 `.gitignore` 忽略。因此，全新 clone 可构建仿真链路，但不能仅凭 Git 内容直接启动正式真机 A/B。执行 `--preflight-only` 前必须：

1. 获取与机器人匹配的外部 reBot SDK checkout，不要复制进本仓库或在本项目中修改它；
2. 将 `config/rebot_real_ab.yaml` 的 `connection.sdk_root`、`host`、TCP/UDP port 改成当前机器人环境；SDK root 下需存在 `upper/python/wlsea_arm_sdk/client.py` 或 `src/wlsea_arm_sdk/client.py`；
3. 按下一节使用项目自带工具生成 A/B trajectory、metadata、preview report/MP4 和 acceptance template，完成人工预览验收后更新配置路径；
4. 确认 `build_rebot/identify` 已构建，并复核 joint mapping、J1 convention、软限位、授权开关和网络地址。

`--mock` 不会创建真实 SDK client，但仍会先执行完整 preflight，所以也需要上述 SDK 目录、冻结产物和 `identify` binary 存在。

完整 Python 回归测试中还有少量用例会读取被忽略的历史 `results/` fixture；先放置项目证据包，再运行 `python3 -m pytest tests/rebot_real -q`。

## 使用项目生成 A/B 激励轨迹

项目可以从 C++ Fourier 轨迹源自行重建当前两条独立 A/B 激励轨迹，不需要向项目负责人索取轨迹文件。先完成 `build_rebot/` 构建，再在仓库根目录执行：

```bash
python3 scripts/prepare_rebot_ab.py \
  --build-directory build_rebot \
  --output-directory results/rebot_real_ab_generated
```

输出目录必须是不存在的新目录，脚本不会覆盖旧证据。它会对 A 和 B 分别完成：

```text
C++ Fourier seed/attempt 确定性生成 coefficients.csv
  -> 导出 100 Hz / 30 s frozen trajectory.csv
  -> 写入 trajectory.meta.yaml 和 SHA-256 provenance
  -> position/velocity/acceleration/jerk/Servo step/collision 数值门禁
  -> 生成 preview_report.yaml
  -> 渲染 actual_time_quintic_v1 preview.mp4
  -> 生成默认 accepted_for_hardware: false 的 preview_acceptance.yaml
```

典型产物为：

```text
results/rebot_real_ab_generated/
  qualification.snapshot.yaml
  manifest.yaml
  A/
    coefficients.csv
    trajectory.csv
    trajectory.meta.yaml
    preview_report.yaml
    preview.mp4
    preview_acceptance.yaml
    hardware.pending.yaml
  B/
    ...
```

没有显示环境时可以加 `--skip-video` 只做数值生成与检查，但这种输出没有 preview MP4/acceptance，**不能用于真机授权**。

若本地没有被 `.gitignore` 排除的历史实机 timing CSV，脚本会在控制台明确说明，并在 `qualification.snapshot.yaml` 中保留本次实际使用的 nominal/alternating timing 检查配置。这仍只是离线 candidate qualification，不替代后续人工视频验收、真机 preflight 和现场安全确认。

### 人工预览与验收

对 A、B 分别检查 `preview_report.yaml` 的 `preview_status: PASS`，并完整观看对应 `preview.mp4`。确认起点、全程运动范围、连续性、终点和周边环境均可接受后，才能在各自的 `preview_acceptance.yaml` 中人工填写：

```yaml
operator: <reviewer name>
review_date: <YYYY-MM-DD>
accepted_for_hardware: true
notes: <what was reviewed>
```

不要手工修改 trajectory、metadata、report 或 MP4 中的 hash。任一上游文件改变后，必须使用新输出目录重新生成并重新人工验收。

最后将 `config/rebot_real_ab.yaml` 指向新产物：

```yaml
trajectories:
  A:
    artifact: results/rebot_real_ab_generated/A/trajectory.csv
    metadata: results/rebot_real_ab_generated/A/trajectory.meta.yaml
    preview_acceptance: results/rebot_real_ab_generated/A/preview_acceptance.yaml
  B:
    artifact: results/rebot_real_ab_generated/B/trajectory.csv
    metadata: results/rebot_real_ab_generated/B/trajectory.meta.yaml
    preview_acceptance: results/rebot_real_ab_generated/B/preview_acceptance.yaml
```

然后先运行后文的 `--preflight-only`。只有 A/B artifact hash 不同、数值门禁通过、preview/report/MP4 hash 一致且两条轨迹均完成人工验收时，正式入口才会继续。

## 当前推荐入口

reBot 正式真机实验只需要维护一个配置：

`config/rebot_real_ab.yaml`

在仓库根目录执行现场唯一启动命令：

```bash
python3 scripts/run_rebot_real_ab.py \
  --config config/rebot_real_ab.yaml
```

同一次进程内会依次完成：离线 preflight → A 人工 ENTER gate → MoveJ 到 A `q0` → A Servo excitation → controlled cleanup → B 人工 ENTER gate → MoveJ 到 B `q0` → B Servo excitation → controlled cleanup → preprocessing → A fit → B validation → `summary.yaml`。

不再要求操作者手工设置 `SDK_ROOT/ORIN_IP/TCP_PORT/UDP_PORT/RUN_ID/RUN_DIR`，也不需要复制/修改 A/B `hardware.yaml` 或辨识配置。每次运行的底层配置会自动 materialize 到本次 run directory，保留 provenance。

## 安全的离线检查

只做配置、依赖、artifact/hash、preview acceptance 和数值门禁：

```bash
python3 scripts/run_rebot_real_ab.py \
  --config config/rebot_real_ab.yaml \
  --preflight-only
```

`--preflight-only` 不创建硬件 client。

完整 Mock 编排：

```bash
python3 scripts/run_rebot_real_ab.py \
  --config config/rebot_real_ab.yaml \
  --mock
```

`--mock` 只使用进程内 `MockArmClient`，不会加载或创建真实 SDK client。Mock 没有真实机械臂动力学/物理 effort；若其 raw 数据无法满足真实辨识器的运动观测要求，流程会明确记录这一点，并使用仓库已有 synthetic RNEA fixture 验证离线 preprocessing/identify 软件链，不会伪造“Mock 真机参数辨识成功”。

## 当前已验证真机事实

2026-09-10 已完成两条相互独立 frozen trajectory 的真实执行：

- A：`be2faa1d58fc834efbca030aea7ec9fbb28d57895aae8e449f36164e6141f131`，`motion_status=completed`，2946 条采样，实际 Servo dispatch 约 98.15 Hz；
- B：`73c0ad08360b0723ed13ba9e0e78d6cc924715bc567548fd5201c77a58457cc4`，`motion_status=completed`，2940 条采样，实际 Servo dispatch 约 97.95 Hz；
- 两次真实运行均完成 MoveJ 预定位、Servo excitation 和 controlled park/disable/close，无 primary hard fault；
- tracking threshold exceedance 保持 monitor-only 数据质量 warning，不会单独触发 excitation 立即失能；
- 实际 lower feedback 独立更新约 10 Hz，不能把约 100 Hz command/CSV rate 解释为 100 Hz 独立物理测量。

上述历史成功目录仅作为已验证证据；下一次正式实验应使用新统一入口生成新的完整 A/B run directory。

## 真机数据解释边界

真机辨识当前使用 SDK `JointState.torque_nm` 映射得到的 `effort_reported`。它是 SDK reported joint-side effort estimate，目前尚未完成独立物理 torque calibration。因此：

```text
reported-effort prediction RMSE 很低
!=
已经证明质量、惯量、摩擦等真实物理参数逐项正确
```

A 只用于 preprocessing rate freeze、基础参数空间与参数拟合；B 只用于独立 validation，不允许 B 反向调节 A 的 rank threshold、basis 或拟合参数。

## 仿真辨识

reBot-DM 仿真仍使用现有 C++ / MuJoCo / Pinocchio 路径。典型流程：

```bash
cmake -S . -B build_rebot -DCMAKE_BUILD_TYPE=Release
cmake --build build_rebot --parallel 4

./build_rebot/rebot_mujoco_model_sanity
./build_rebot/rebot_model_consistency_test
./build_rebot/rebot_phase5a_regressor_test

./build_rebot/run_experiment \
  --experiment-config config/rebot_dm_excitation_experiment.yaml \
  --trajectory-seed 20260826 \
  --headless \
  --output /tmp/rebot_A.csv \
  --trajectory-output /tmp/rebot_A.trajectory.csv

./build_rebot/run_experiment \
  --experiment-config config/rebot_dm_excitation_experiment.yaml \
  --trajectory-seed 20260829 \
  --headless \
  --output /tmp/rebot_B.csv \
  --trajectory-output /tmp/rebot_B.trajectory.csv
```

仿真 reBot augmented regressor 为 `60 rigid + 6 armature + 6 damping + 6 frictionloss = 78` raw columns。冻结 synthetic clean baseline 的 A-only base rank 为 52；这只是仿真闭环基准，不是实际硬件参数声明。

## 文档

长期维护文档：

- `doc/ARCHITECTURE.md`：当前系统架构与 source-of-truth；
- `doc/DEVELOPMENT_RULES.md`：开发规则；
- `doc/REBOT_HARDWARE_CONTROL_CONTRACT.md`：真机控制/安全语义契约；
- `doc/REBOT_HARDWARE_DATA_CONTRACT.md`：真机数据字段与可解释性契约；
- `doc/REBOT_SYSTEM_IDENTIFICATION_GUIDE.md`：唯一面向操作者的完整系统辨识指南。

正式使用优先阅读 `doc/REBOT_SYSTEM_IDENTIFICATION_GUIDE.md`。

## 底层工具

以下脚本保留用于诊断、测试或高级调试，不是普通正式实验入口：

- `scripts/run_rebot_hardware.py`
- `scripts/run_rebot_identification.py`
- `scripts/preprocess_rebot_hardware_identification.py`
- `scripts/prepare_rebot_ab.py`
- 其它 state/fault/reset/preview diagnostic tools

不要为日常正式实验重新拼接这些底层命令；统一入口已经负责配置派生和阶段依赖关系。

## 回归测试

```bash
python3 -m pytest tests/rebot_real -q
git diff --check
```

真机控制链、artifact/hash/preview gates、Mock 行为以及 orchestration/config 均有独立测试。修改安全/数据语义前请先阅读对应 contract。
