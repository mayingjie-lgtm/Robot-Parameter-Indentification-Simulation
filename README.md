# 机器人动力学参数辨识系统

本分支是一个**无 ROS 中间层**的机器人动力学参数辨识实验版本。仿真与离线辨识核心采用 **MuJoCo + C++ + CMake**；Piper 真机后端通过 Python bridge 调用 `piper_sdk`。项目目标是在 Linux/Windows 上运行仿真与离线辨识，并在支持的 Linux 真机环境中接入硬件实验。

> **声明**：本项目参考了 [BIRDy (Benchmark for Identification of Robot Dynamics)](https://github.com/TUM-ICS/BIRDY) 开源项目。

---

## 快速开始

### 1. 安装依赖

需要预先安装并配置：

- MuJoCo
- GLFW
- Eigen3
- CMake 3.21+
- C++17 编译器
- Pinocchio 4.x（reBot-DM dynamics / regressor；当前 Ubuntu 22.04 主机使用 `ros-humble-pinocchio`）
- FFmpeg（可选；Linux 下将 reBot-DM 仿真 CSV 离线渲染为 MP4 时需要）

如果 MuJoCo 未安装在系统默认路径，请设置环境变量 `MUJOCO_DIR`。使用 ROS Humble 提供的 Pinocchio 时，配置前先加载对应 shell 的环境：zsh 执行 `source /opt/ros/humble/setup.zsh`，Bash 执行 `source /opt/ros/humble/setup.bash`，让 CMake 能找到 Pinocchio 及其依赖。

### 2. 配置项目

```bash
cmake -S . -B build
```

### 3. 构建

```bash
cmake --build build --parallel
```

### 4. 运行统一实验入口

```bash
./build/run_experiment
```

默认会：

- 读取 [`config/experiment.yaml`](config/experiment.yaml)
- 按机器人选择对应 controller config，例如 [`config/force_controller_node.yaml`](config/force_controller_node.yaml) 或 [`config/piper_force_controller_node.yaml`](config/piper_force_controller_node.yaml)
- 按机器人读取 simulator config；Piper 使用 [`config/piper_sim_node.yaml`](config/piper_sim_node.yaml)
- 按机器人选择对应 MuJoCo scene
- 在 `data/benchmark_data.csv` 输出采样数据

通过修改 [`config/experiment.yaml`](config/experiment.yaml) 中的 `robot` 和 `backend` 字段，可以在仿真后端和 Piper 真机后端之间切换。切到 `piper` 且 `backend: sim` 时，会自动改用 [`config/piper_force_controller_node.yaml`](config/piper_force_controller_node.yaml) 和 `piper` 对应的 MuJoCo 模型。`robot: rebot_dm` 当前严格限制为仿真后端；所需 10 个 accepted binary STL 已完成 repository-local vendoring，Phase 4B runtime、Phase 5A regressor、Phase 5B A/B excitation 和 Phase 5C clean identification 均已 PASS。完整复现流程见 [`doc/robot_parameter_identification_manual.md`](doc/robot_parameter_identification_manual.md#13-rebot-dm-仿真辨识完整运行流程)，各阶段数值证据见 [`doc/PHASE4B_REBOT_RUNTIME_BASELINE.md`](doc/PHASE4B_REBOT_RUNTIME_BASELINE.md)、[`doc/PHASE5A_REBOT_REGRESSOR_BASELINE.md`](doc/PHASE5A_REBOT_REGRESSOR_BASELINE.md)、[`doc/PHASE5B_REBOT_EXCITATION_BASELINE.md`](doc/PHASE5B_REBOT_EXCITATION_BASELINE.md) 和 [`doc/PHASE5C_REBOT_CLEAN_IDENTIFICATION_BASELINE.md`](doc/PHASE5C_REBOT_CLEAN_IDENTIFICATION_BASELINE.md)。

推荐的后端切换方式：

- `backend: sim`：使用 MuJoCo 仿真后端
- `backend: piper_real`：使用 Piper 真机后端

可选参数：

```bash
./build/run_experiment --headless --output data/my_run.csv
```

也可以显式指定实验配置：

```bash
./build/run_experiment --experiment-config config/experiment.yaml
```

仿真输出现在同时包含 `qdd_mujoco/qdd_diff` 与 `tau_cmd/tau_effort/tau_constraint`，并生成 `.meta.yaml`。真实 Piper backend 仍使用明确标为 `legacy_ambiguous_schema` 的旧格式。完整字段语义和可信 Piper 基线见 [`doc/PHASE3_PIPER_BASELINE.md`](doc/PHASE3_PIPER_BASELINE.md)。

### 5. 运行参数辨识

```bash
./build/identify
```

默认读取 [`config/identification.yaml`](config/identification.yaml)。
默认配置使用独立 Piper 轨迹 A 训练、轨迹 B 验证，按 header 名称选择 `q/qd/qdd_mujoco/tau_effort`。正式可信闭环当前只允许无 ridge OLS (`algorithm: 1`) 或 Huber IRLS (`algorithm: 3`)。

可选参数：

```bash
./build/identify --config config/identification.yaml
./build/identify --config config/identification_friction.yaml
python3 scripts/verify_phase3_gates.py
```

#### reBot-DM 仿真辨识

reBot-DM 当前可信流程使用固定 seed 生成独立轨迹 A/B，只用 A 建立 52 维基础参数空间并执行无 ridge OLS，再用 B 做独立力矩预测。快速复核已有基准产物：

```bash
python3 scripts/verify_rebot_excitation_data.py \
  --csv data/rebot_dm/excitation_A.csv
python3 scripts/verify_rebot_excitation_data.py \
  --csv data/rebot_dm/excitation_B.csv
./build/rebot_excitation_data_quality data/rebot_dm/excitation_A.csv
./build/rebot_excitation_data_quality data/rebot_dm/excitation_B.csv
python3 scripts/verify_rebot_clean_identification.py \
  --result results/rebot_dm_clean_identification.yaml
```

从模型门禁、A/B 重新采集到 OLS 和独立验证的完整可复制命令、产物说明与通过标准，统一记录在 [`doc/robot_parameter_identification_manual.md`](doc/robot_parameter_identification_manual.md#13-rebot-dm-仿真辨识完整运行流程)。该闭环仅代表 clean synthetic simulation，不代表真实 reBot 硬件参数。

#### 将 reBot-DM 轨迹保存为视频（Linux）

GUI viewer 不会限制仿真主循环速度。需要稳定观察运动时，可以直接读取已有
simulation-truth CSV，以固定相机离线生成 MP4，而不重新执行仿真：

```bash
./build/rebot_trajectory_renderer \
  --input data/rebot_dm/excitation_A.csv \
  --output results/rebot_dm_excitation_A.mp4
```

默认输出 `1280x720`、`60 fps`、`1.0` 倍速的 H.264 MP4。慢放或只导出一段：

```bash
./build/rebot_trajectory_renderer \
  --input data/rebot_dm/excitation_A.csv \
  --output results/rebot_dm_excitation_A_slow_clip.mp4 \
  --playback-speed 0.5 \
  --start-time 10 \
  --duration 5
```

已有输出默认不会被覆盖；确认替换时显式添加 `--overwrite`。该工具需要可用的
GLFW 显示环境以及 PATH 中的 `ffmpeg`，目前只构建 Linux target。

### 6. reBot 真机离线接入（当前只完成软件/Mock 验收）

reBot-DM 当前**没有**接入 C++ `ExperimentBackend`。真实 SDK 的 Servo command 经审计只有：

```text
servo_sequence
host_timestamp_ns
target_position_rad[6]
```

也就是只发送 `q_target`，没有 `qd/kp/kd/tau_cmd`。因此 reBot 真机使用独立的 Python
control adapter / runner / hardware recorder，不能把现有 C++ legacy recorder 的 `tau` 当成
真实 reBot torque command。

当前安全默认配置是 [`config/rebot_real_experiment.yaml`](config/rebot_real_experiment.yaml)：

```yaml
control_mode: state_only
allow_hardware: false
allow_motion: false
joint_mapping_verified: false
j1_convention: UNRESOLVED
```

本机可直接执行离线 Mock smoke，不会连接真实机械臂：

```bash
python3 scripts/run_rebot_hardware.py \
  --config config/rebot_real_experiment.yaml \
  --mock
```

输出使用独立 schema `rebot_hardware_experiment_v1`，记录 `q/qd/effort_reported/q_cmd`、
feedback validity/age、Servo 状态、host/lower timestamp 与 Servo sequence；**不记录**在线
`qdd`、`tau_cmd`、`current` 或 `q_raw`。完整控制、门禁、CSV 和 metadata 语义见
[`doc/REBOT_HARDWARE_CONTROL_CONTRACT.md`](doc/REBOT_HARDWARE_CONTROL_CONTRACT.md)。
接上实机后的逐步检查、状态采集、关节映射和 Servo 保持操作见
[`doc/REBOT_REAL_HARDWARE_RUNBOOK.md`](doc/REBOT_REAL_HARDWARE_RUNBOOK.md)。

control mode 当前状态：

- `state_only`：软件流程与 Mock 已实现；真实硬件验收仍 `PENDING`。
- `servo_hold`：完整 Servo lifecycle 与 fault cleanup 已用 Mock 验证；真实执行仍被默认门禁阻断。
- `joint_jog`：单轴平滑小步往返与自动记录已实现，真实硬件验收 PENDING。离线运行
  `python3 scripts/run_rebot_hardware.py --config config/rebot_joint_jog_mock.yaml --mock`。
  Mock 参数不是实机限值；旧 hold-only 映射不能放行，详见操作手册 §6.2。
- `excitation`：门禁已定义，但可信 Fourier source 仍在现有 C++ `ForceController`；为避免复制第二套数学实现，trajectory source integration 仍 `PENDING`，当前会在创建 hardware session 前明确拒绝执行。

Phase 6A 的 UDP-only raw state capture `rebot_hardware_state_v1` 保持独立且不变，见
[`doc/REBOT_HARDWARE_DATA_CONTRACT.md`](doc/REBOT_HARDWARE_DATA_CONTRACT.md)。特别注意：当前
SDK 的 TCP session 在 lower disconnect 时会触发 `disable()`，所以 Phase 6B `state_only`
只能保证上位机不调用 motor-changing command，不能称为完全无硬件副作用的 observation
contract。

未来切到真机电脑后，至少需要确认/修改：`sdk_root`、`host`、TCP/UDP ports、
`joint_mapping_verified`、`j1_convention`、`joint_direction`、`joint_offset_rad`、joint limits、
command limits、feedback freshness threshold 与 control rate。J1 当前 canonical `[-2.8, 2.8]`
与 SDK `[0, 2*pi]` 冲突仍未解决，禁止自动 wrap 或猜测。

### 7. 运行 Piper 真机实验

真机实验与仿真实验共用同一个入口程序，只是后端不同：

```bash
./build/run_experiment --experiment-config config/experiment.yaml --headless
```

使用前请先确认：

- Python 环境中能够正常 `import piper_sdk`，并已按所使用 Piper SDK 版本准备 CAN 环境（当前仓库本身不包含 `piper_sdk/` 源码目录）
- 已安装真机 bridge 所需 Python 依赖，例如 `python-can`
- 已在 [`config/experiment.yaml`](config/experiment.yaml) 中设置 `backend: piper_real`
- 已检查 [`config/piper_real_experiment.yaml`](config/piper_real_experiment.yaml) 中的 `control_mode`、软限位和初始位

默认配置使用 `excitation_trajectory`，会通过 MIT 接口执行 Fourier 激励轨迹，并记录与仿真相同结构的 `time / q / qd / qdd / tau`。

在真正开始激励轨迹之前，bridge 会先执行“回安全初始位”流程：

- 若 `move_to_home_before_start: true`
- 且当前关节位置不在 `home_position_tolerance` 容差窗口内
- bridge 会先用 `home_speed_rate` 把机械臂移动到 `home_position`
- 只有进入容差窗口后，统一实验主循环才会开始执行激励轨迹

这里的“回零位”在工程上等价为“回到配置中的安全初始位 `home_position`”，不是重新写入电机零点标定。

可选模式：

- `excitation_trajectory`：真机参数辨识主模式，执行 Fourier 激励轨迹
- `mit_identification`：与激励轨迹相同的 MIT 接口实验模式，便于后续扩展不同控制策略
- `position_validation`：小幅关节位置轨迹验证
- `state_only`：只连接与采样，不下发运动命令

激励轨迹相关参数位于 [`config/piper_real_experiment.yaml`](config/piper_real_experiment.yaml)：

- `trajectory_period`
- `trajectory_harmonics`
- `trajectory_coefficient_scale`
- `trajectory_seed`

这些参数决定真机实验使用的 Fourier 轨迹形状。当前实现会用固定随机种子生成可复现的轨迹系数，并自动满足 `t=0` 时的初始位置/零速度约束。

真机 bridge 还会额外检查：

- `joint_soft_limits`
- `max_command_velocity`
- `max_command_torque`
- `home_position_tolerance`

任一关节命令超过这些阈值时，bridge 会拒绝该步命令并触发急停，避免上层统一主循环把明显异常的控制量直接发送到真机。

---

## 项目结构

```text
├── franka_emika_panda/   # MuJoCo Panda 模型与资源
├── piper/                # MuJoCo Piper 模型与资源
├── rebot_dm/             # reBot-DM canonical dynamics、runtime scene 与 repository-local mesh
├── src/app/              # 统一实验入口、backend 与 recorder
├── src/sim_com_node/     # MuJoCo 仿真器
├── src/force_node/       # C++ 控制器、轨迹与碰撞检查
├── src/identification/   # 离线参数辨识与诊断工具
├── src/piper_real/       # Piper 真机 Python SDK adapter / bridge 逻辑
├── src/rebot_real/       # reBot raw state capture + 独立 hardware control/Mock integration
├── tests/piper_real/     # Piper 真机后端单元测试
├── tests/rebot_real/     # reBot hardware contract / lifecycle / fault-injection tests
├── config/               # 运行配置
└── doc/                  # 架构、baseline、数学与实验说明
```

---

## 系统架构

当前分支采用统一实验调度架构，不再依赖 ROS 中间层。实验链路分为
“配置层 -> 控制层 -> 执行后端层 -> 数据层 -> 辨识层” 五个部分。

```text
config/*.yaml
    |
    v
run_experiment
    |
    +--> ForceController --------------------+
    |                                        |
    |                                        | 统一控制命令
    |                                        v
    +--> SimulationBackend / PiperHardwareBackend
             |
             | 关节状态 q / qd
             +---------------------------> ForceController
             |
             | 统一 CSV 记录
             v
        benchmark_data.csv
             |
             v
     identify / mujoco_identify
             |
             v
   results/identification.yaml
```

- `run_experiment` 是统一实验调度入口，负责读取配置、创建控制器、选择执行后端，并驱动唯一的一套时间循环。
- `ForceController` 负责激励轨迹生成、PD 跟踪、力矩限幅，并输出统一控制命令。
- `SimulationBackend` 与 `PiperHardwareBackend` 分别负责仿真步进和真机命令执行，但不改变上层实验流程。
- `identify` 与 `mujoco_identify` 负责离线辨识，读取实验 CSV，构造观测矩阵并输出参数结果。

如果按运行时序看，单步循环可以概括为：

1. 后端返回当前关节状态。
2. `ForceController` 根据当前时刻和关节状态计算统一控制命令。
3. 后端执行一步仿真或一步真机命令下发。
4. 仿真记录器将同一步的 pre-integration truth、post-integration state 和质量标记写入 CSV；真机记录器保持 legacy 语义。

---

## 主要可执行文件

- `run_experiment`：统一实验入口，可选择 MuJoCo 仿真或 Piper 真机 backend，并生成 CSV
- `identify`：读取独立 A/B CSV，在固定 SVD 基础参数空间执行 OLS 或 IRLS
- `mujoco_identify`：快速执行一次 MuJoCo 回归辨识
- `dynamics_diagnostic`：对比动力学模型与记录数据
- `model_comparison`：对比不同动力学模型
- `regressor_test`：检查 Piper 回归矩阵与 MuJoCo 动力学一致性
- `rebot_mujoco_model_sanity`：检查 reBot-DM canonical MJCF 的 J1–J6 mapping、六个 torque actuator、显式 simulation truth 与固定夹爪状态；可选读取外部源 MJCF 验证指定夹爪开度的 mesh 自碰撞
- `rebot_model_consistency_test`：执行 reBot-DM MuJoCo↔Pinocchio joint mapping、gravity、`M(q)`、inverse dynamics 与 `Y*theta` 六项 Phase 4A 门禁；当前基线见 [`doc/PHASE4_REBOT_DM_MODEL_BASELINE.md`](doc/PHASE4_REBOT_DM_MODEL_BASELINE.md)
- `rebot_phase5a_regressor_test`：检查 reBot-DM 60/72/78 列参数布局，以及 rigid/armature/damping/frictionloss simulation-truth 闭环
- `rebot_constraint_force_diagnostic`：分解 reBot forward rollout 的 friction、equality、limit、contact 等约束力来源
- `rebot_excitation_data_quality`：检查 reBot A/B 实际轨迹的 60/72/78 列 rank、condition 和结构零列
- `rebot_identification_model_closure`：解释指定 trajectory seed 的 MuJoCo forward/inverse 与 Pinocchio regressor oracle floor
- `rebot_trajectory_renderer`（Linux）：将 reBot simulation-truth CSV 离线回放并保存为 H.264 MP4
