# Robot Parameter Identification Simulation — Architecture

> 目标：让项目首先服务于“机械臂系统辨识实验”，而不是为了通用性进行过度软件重构。
>
> 当前阶段原则：**先把现有 Panda/Piper 链路完全搞清楚并建立可重复 baseline，再以最小改动接入 reBot。**
>
> 本文同时包含“当前代码事实”和“目标数据契约”。凡是两者不一致的地方，必须显式标注；如果本文与可重复运行的当前代码冲突，以 `AGENTS.md` 定义的 source-of-truth 顺序为准，先报告差异，不得把目标态当成已经实现。

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

截至当前 Phase 1 审计，`run_experiment -> ExperimentRecorder -> CSV` 的实际行为是：

| CSV / state 字段 | 当前来源 | 当前语义 |
|---|---|---|
| `q` | `ExperimentState.position`；仿真来自 MuJoCo `qpos` | 关节位置 |
| `qd` | `ExperimentState.velocity`；仿真来自 MuJoCo `qvel` | 关节速度 |
| `qdd` | `ExperimentRecorder` 对相邻 `state.velocity` 做一阶差分 | 数值微分加速度，不是当前统一链路中的 MuJoCo `qacc` |
| CSV `tau` | `ControlCommand.torque` | 命令力矩 `tau_cmd`，不是 `state.effort` |
| `state.effort`（sim） | MuJoCo `qfrc_actuator` | actuator reported effort，当前未写入统一 CSV |
| `state.effort`（Piper real） | Piper SDK bridge 返回的 effort | SDK reported effort，当前未写入统一 CSV |

因此当前 CSV 中的通用列名 `qdd` 和 `tau` **不能仅凭列名推断数据来源**。特别是：

```text
CSV 存在 qdd 列 != qdd 来自 physics engine
CSV tau != 已证明的实际关节扭矩
```

这两点属于 Phase 1 需要优先确认和修正的数据可信度问题。

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

**当前实现：**统一 `ExperimentRecorder` 对相邻速度样本做一阶差分，仿真和真机都走这一路径。

**目标数据契约：**

```text
仿真：
优先记录 MuJoCo qacc，并明确 source=sim_qacc

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

当前 `ExperimentState` 实际只包含：

```text
position
velocity
effort
```

时间由 backend 的 `simulationTime()` / `timeStep()` 单独提供；当前接口**没有直接传递 qdd 或原始硬件 timestamp**。

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

当前实现仍有两处待收口：

```text
qdd = velocity finite difference
tau = command.torque
```

而 CSV header 只写 `qdd` / `tau`，没有携带 source metadata。因此当前实现尚未完全达到“Recorder 记录事实且不猜测数据”的目标。

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

当前 `DataLoader` 负责读取固定 CSV schema，并根据是否存在 `qdd` 列决定是否需要后续数值微分；当前还没有真正的 `torque source` 选择机制。

已知问题：当前 loader 只要检测到 `qdd` 列，就会把它描述为“from physics engine”，但统一 `ExperimentRecorder` 生成的 `qdd` 实际来自速度差分。这个描述不能作为数据来源证据。

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

当前 `identify` 将同一 CSV 按时间顺序切分为前 80% training、后 20% validation，然后计算：

```text
tau_hat = W beta_hat
```

这属于**同一条激励轨迹内部的 hold-out validation**，还不是最终目标中的独立验证轨迹。

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

# 7. reBot 的未来接入位置

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
