# Robot Parameter Identification Simulation — Architecture

> 目标：让项目首先服务于“机械臂系统辨识实验”，而不是为了通用性进行过度软件重构。
>
> 当前阶段原则：**先把现有 Panda/Piper 链路完全搞清楚并建立可重复 baseline，再以最小改动接入 reBot。**
>
> 本文同时包含“当前代码事实”和“目标数据契约”。凡是两者不一致的地方，必须显式标注；如果本文与可重复运行的当前代码冲突，以 `AGENTS.md` 定义的 source-of-truth 顺序为准，先报告差异，不得把目标态当成已经实现。

> 2026-08-24 状态：完整 Piper 固定夹爪的 Phase 3 闭环已经实现并通过门禁。本文中保留的 Phase 1 历史问题应结合 [`PHASE3_PIPER_BASELINE.md`](PHASE3_PIPER_BASELINE.md) 阅读；以下“当前实现”段落已按新数据契约更新。

---

# 1. 项目目标

本项目最终要完成：

```text
机械臂模型
   ↓
设计激励轨迹
   ↓
仿真 / 真机执行
   ↓
采集 q, qd, qdd, tau
   ↓
构造动力学回归矩阵 W
   ↓
参数辨识
   ↓
独立轨迹验证
   ↓
辨识参数回灌模型
   ↓
验证动力学预测 / 重力补偿 / 控制效果
```

当前阶段不追求“支持任意机器人”的完美框架。

当前优先目标是：

1. 完全理解现有代码；
2. 建立可信的 Panda/Piper baseline；
3. 搞清楚每一个数据字段的物理意义；
4. 能解释系统辨识算法；
5. 再接入 reBot；
6. 只有出现真实重复和维护问题后，才做有针对性的重构。

---

# 2. 当前系统主链路（代码事实）

```text
┌─────────────────────┐
│ experiment config   │
└──────────┬──────────┘
           ↓
┌─────────────────────┐
│ ForceController     │
│ Fourier excitation  │
└──────────┬──────────┘
           ↓
┌─────────────────────────────┐
│ ExperimentBackend           │
│                             │
│  SimulationBackend          │
│       ↓                     │
│    MuJoCo                   │
│                             │
│  PiperHardwareBackend       │
│       ↓                     │
│    Piper SDK bridge         │
└──────────┬──────────────────┘
           ↓
┌─────────────────────┐
│ ExperimentRecorder  │
│ CSV data            │
└──────────┬──────────┘
           ↓
┌─────────────────────┐
│ DataLoader          │
└──────────┬──────────┘
           ↓
┌─────────────────────┐
│ Regressor           │
│ W(q,qd,qdd)         │
└──────────┬──────────┘
           ↓
┌─────────────────────┐
│ Identification      │
│ OLS/WLS/IRLS/...    │
└──────────┬──────────┘
           ↓
┌─────────────────────┐
│ Validation          │
│ tau_hat vs tau      │
└─────────────────────┘
```

---

# 3. 系统辨识的数学主线

当前项目最核心的模型不是软件类，而是：

\[
\tau =
M(q)\ddot q +
C(q,\dot q)\dot q +
g(q) +
\tau_f
\]

刚体动力学对惯性参数可以写成线性形式：

\[
\tau = Y(q,\dot q,\ddot q)\beta
\]

多组采样堆叠：

\[
T = W\beta
\]

其中：

- `q`：关节位置；
- `qd`：关节速度；
- `qdd`：关节加速度；
- `tau`：用于辨识的关节力矩；
- `Y`：单时刻动力学 regressor；
- `W`：整段轨迹 observation matrix；
- `beta`：待辨识动力学参数；
- `beta_hat`：估计参数；
- `tau_hat = W beta_hat`：模型预测力矩。

对项目的任何代码修改，都必须能够回答：

> 它改变了这条数学链路中的哪一部分？

如果回答不了，一般不应该在当前阶段修改。

---

# 4. 数据定义

这一部分是整个项目最重要的接口协议。下面先区分**当前实现**和**目标数据契约**。

## 4.0 当前统一实验链路的真实数据语义

当前 `run_experiment -> ExperimentRecorder -> CSV` 在仿真 backend 下的实际行为是：

| CSV / state 字段 | 当前来源 | 当前语义 |
|---|---|---|
| `q0..` / `qd0..` | MuJoCo pre-integration `qpos` / `qvel` | 区间起点状态 |
| `qdd_mujoco0..` | 同一步 forward dynamics 的 MuJoCo `qacc` | 仿真辨识加速度真值 |
| `qdd_diff0..` | `(qd_next-qd)/(time_end-time_begin)` | 前向速度差分，仅用于误差诊断 |
| `tau_cmd0..` | `ControlCommand.torque` | 命令力矩 |
| `tau_effort0..` | 同一步 MuJoCo `qfrc_actuator` | actuator 广义力；当前 unit gear 未饱和时等于命令 |
| `tau_constraint0..` | MuJoCo `qfrc_constraint` | 接触、限位、equality 或 frictionloss 的约束力 |
| `q_next0..` / `qd_next0..` | 积分后的 `qpos` / `qvel` | 区间终点状态 |
| `saturated` / `contact_count` | controller / MuJoCo | 显式质量标记，不静默丢弃 |

仿真 schema 使用 17 位有效数字并生成 `.meta.yaml`。真机 backend 仍保留 legacy `qdd/tau` schema，并在 metadata 中标记 `legacy_ambiguous_schema`；不能把真机 legacy 列解释成上述 MuJoCo 真值。

## 4.1 q

```text
单位：rad
含义：关节实际位置
```

仿真：

```text
MuJoCo qpos
```

真机：

```text
编码器 / 电机反馈的位置
```

---

## 4.2 qd

```text
单位：rad/s
含义：关节实际速度
```

必须记录来源：

```text
sim_exact
motor_feedback
position_differentiated
filtered
```

真机中不能默认电机 SDK 提供的 velocity 一定是精确 rad/s。

---

## 4.3 qdd

```text
单位：rad/s^2
含义：关节加速度
```

**当前实现：**仿真同时记录 `qdd_mujoco` 与 `qdd_diff`，正式仿真辨识显式选择前者；真机 legacy schema 仍使用速度差分。

```text
仿真：qdd_mujoco = MuJoCo qacc（pre-integration）

真机：
优先离线对 q / qd 做滤波和求导，并明确 preprocessing 方法
```

不要把未经滤波的差分结果当成可信真机 `qdd`，也不要因为 CSV 已有 `qdd` 列就声称它来自 physics engine。

---

## 4.4 tau_cmd

```text
单位：Nm
含义：发送给 backend / actuator 的命令力矩
```

注意：

```text
tau_cmd != 实际关节力矩
```

特别是在 MIT / PD / firmware control 模式下。

---

## 4.5 tau_effort

```text
单位：Nm（如果 SDK 已校准）
含义：backend 返回的 effort / estimated torque
```

必须确认：

```text
真实扭矩传感器？
电流换算？
firmware estimate？
PD estimate？
其他？
```

在没有确认之前，统一称为：

```text
reported effort
```

不能直接称为“真实关节力矩”。

---

# 5. 模块职责

## 5.1 ForceController

负责：

```text
生成激励参考轨迹
计算控制命令
检查基本位置/碰撞/力矩安全约束
```

不负责：

```text
动力学参数辨识
数据后处理
真机 qdd 估计
```

---

## 5.2 ExperimentBackend

当前负责：

```text
向执行环境发送命令
读取机器人状态
提供统一的 step / initialize 接口
```

当前 `ExperimentState` 包含：

```text
position
velocity
effort
optional simulation_truth
```

`simulation_truth` 只由仿真 backend 填充，包含区间起点时间、pre-integration `q/qd/qacc/qfrc_actuator/qfrc_constraint` 和接触数；真实 Piper backend 的返回语义未改变。时间仍由 backend 的 `simulationTime()` / `timeStep()` 提供。

目标上，如果后续为了保留真实数据来源而扩展接口，可以显式增加 `qdd`、timestamp 或 source metadata，但必须由实际需求驱动，不能为了形式统一提前扩展。

Backend 可以是：

```text
MuJoCo
Piper real
reBot real（未来）
```

上层不应该关心具体通信方式。

---

## 5.3 ExperimentRecorder

目标职责：

```text
把状态和命令按明确物理语义写入文件
```

仿真 schema 已按 `qdd_mujoco/qdd_diff/tau_cmd/tau_effort/tau_constraint` 分列，并通过 sidecar metadata 记录来源。真机 legacy schema 仍需在进入真实参数辨识前单独完成传感器来源和预处理契约。

Recorder 不应负责：

```text
复杂滤波
动力学计算
辨识
机器人专用逻辑
```

原则：

> Recorder 记录事实，不“猜测”数据；当同名字段可能有多个物理来源时，应在 schema 或实验 metadata 中显式记录 source。

---

## 5.4 DataLoader / Preprocessing

当前 `DataLoader` 按配置中的精确 header 前缀读取 position、velocity、acceleration 和 torque；缺列、重复列或不完整关节列组直接报错，不再按列数猜测来源。正式辨识只依据 finite、`saturated`、`contact_count` 和已知约束语义筛选数据，不再使用 `qdd < 10` 之类隐式阈值。

目标职责：

```text
读取数据
检查维度和时间戳
读取/选择明确的 torque source
根据 source metadata 决定 qd/qdd preprocessing
```

这一层是仿真数据和真机数据的重要分界。

---

## 5.5 Regressor

负责：

```text
给定 q, qd, qdd
生成 Y / W
```

不负责：

```text
采集数据
控制机械臂
求解参数
```

---

## 5.6 Identification Algorithm

负责：

```text
给定 W 和 tau
求 beta_hat
```

例如：

```text
OLS
WLS
IRLS
TLS
EKF
ML
CLOE
```

算法层原则上不应该知道：

```text
Panda
Piper
reBot
MuJoCo
真实机械臂 SDK
```

---

## 5.7 Evaluation

当前 Piper 正式模式要求不同路径的 trajectory A training CSV 与 trajectory B validation CSV，然后计算：

```text
tau_hat = W beta_hat
```

训练 A 定义固定的列缩放、SVD 数值秩和基础参数方向；所有噪声/IRLS 实验复用这组基础坐标，B 始终保持干净且不参与估计。

目标重点指标：

```text
train RMSE
validation RMSE
per-joint RMSE
max error
W rank
singular values
condition number
```

最终评价重点应升级为：

```text
独立于辨识激励轨迹的 validation trajectory 上的 torque prediction
```

而不是：

```text
每一个原始惯性参数都精确恢复
```

---

# 6. Robot-specific 与 Robot-agnostic 边界

## 可以 robot-specific

```text
robot model files
URDF / MJCF
joint limits
home pose
controller gains
hardware SDK adapter
specific hardware safety limits
robot-specific validation config
```

## 应尽量 robot-agnostic

```text
ExperimentRecorder
DataLoader
Identification algorithms
metrics
diagnostics
experiment pipeline
result format
```

## 暂时允许 robot-specific

```text
regressor implementation
inverse dynamics implementation
```

原因：

当前 Panda/Piper 已有经过现有项目验证的 MuJoCo-specific 实现。

不要为了“统一”立刻替换它们。

---

# 7. reBot 的接入位置

> 2026-08-25 Phase 4A 已完成：仓库已加入 `rebot_dm/` dynamics-only canonical URDF/MJCF、显式零 armature/damping/friction simulation truth、J1–J6 六个 direct torque actuator，以及最小 `ReBotPinocchioDynamics` wrapper。full URDF 先加载 8-DoF 模型，再把两个 gripper joint 固定在经源 mesh 验证无自碰撞的 `[0.05, 0.05] m`，得到 `nq=6,nv=6` reduced model。MuJoCo↔Pinocchio gravity、`M(q)`、inverse dynamics 与 rigid-body `Y*theta` 均达到约 `1e-15` 数值闭环，Phase 3 Piper gates 保持不变。详见 [`PHASE4_REBOT_DM_MODEL_BASELINE.md`](PHASE4_REBOT_DM_MODEL_BASELINE.md)。
>
> 2026-08-25 Phase 4B 已完成最小 `rebot_dm -> ForceController(hold_position) -> SimulationBackend -> MuJoCo -> ExperimentRecorder` 代码接入，并用 accepted 完整 mesh 验证 6DOF actuator mapping、无饱和/无意外 contact、`tau_cmd == qfrc_actuator` 和 Phase 3 CSV/metadata semantics。后续 STL 已补齐，Phase 5A/5B/5C 的 regressor、激励质量与 clean identification gates 也已完成；详见对应 Phase 4B/5 baseline 文档。

目标结构：

```text
                  ┌── Panda model
                  ├── Piper model
Robot model ──────┤
                  └── reBot model
                         │
                         ↓
              experiment pipeline
                         │
              ┌──────────┴──────────┐
              ↓                     ↓
          MuJoCo sim            real backend
                                    │
                                    ↓
                              reBot SDK
```

动力学辨识侧：

```text
reBot URDF
   ↓
Pinocchio
   ↓
inverse dynamics / torque regressor
   ↓
Identification algorithms
```

2026-08-26 起，reBot 真机接入先增加一个**独立 state-only 旁路**，而不是直接实现
`reBot ExperimentBackend`：

```text
reBot SDK public UDP JointState
        ↓
UDP-only state subscriber
        ↓
reBot hardware CSV + metadata
        ↓
offline acceptance analyzer
```

该旁路不进入 `ForceController`，不发送任何 motor-changing command，也不复用
simulation-truth CSV schema；`qdd` 只允许在后续 preprocessing 中离线生成。正式字段、
时间戳/力矩/丢包语义和 J1 unresolved mapping 见
[`REBOT_HARDWARE_DATA_CONTRACT.md`](REBOT_HARDWARE_DATA_CONTRACT.md)。只有 state-only 真机
验收完成后，才讨论是否将 reBot SDK 接入 `ExperimentBackend`。

优先采用 Pinocchio，是为了避免继续手写第三套机器人动力学和 regressor。

---

# 8. 当前明确不做的架构工作

现阶段不做：

```text
通用插件系统
复杂 factory hierarchy
dependency injection framework
ROS abstraction
多层 service architecture
统一所有 robot_core 源文件
手写任意机器人 regressor generator
```

如果增加一个抽象层不能解决当前真实问题，就不增加。

---

# 9. 架构演进原则

项目采用：

```text
需求驱动抽象
```

而不是：

```text
预测未来需求后提前抽象
```

正确顺序：

```text
先 Panda/Piper baseline
        ↓
最小方式加入 reBot
        ↓
观察真实重复代码
        ↓
抽象重复部分
        ↓
继续 regression test
```

---

# 10. 当前阶段成功标准

在开始 reBot 代码开发前，项目负责人应能够独立解释：

1. `run_experiment` 从哪里开始；
2. Fourier 激励如何产生；
3. command torque 如何计算；
4. MuJoCo 状态从哪里读取；
5. CSV 每一列是什么；
6. `qdd` 从哪里来；
7. `tau` 从哪里来；
8. `DataLoader` 如何组织数据；
9. `W` 如何生成；
10. `OLS` 在求什么；
11. 为什么 `W` 可能 rank deficient；
12. 为什么 full inertial parameter 不一定逐项可辨识；
13. validation torque RMSE 表示什么；
14. reBot 应该从项目哪两个位置接入：
    - simulation/model
    - real backend

如果这 14 个问题仍有无法解释的部分，优先继续阅读/实验，而不是继续重构。
