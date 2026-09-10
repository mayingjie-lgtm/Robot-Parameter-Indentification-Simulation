# Robot Parameter Identification Simulation — Architecture

> 目标：服务于可审计、可复现的机械臂动力学系统辨识。当前主对象是 reBot-DM；Panda/Piper 仿真链作为既有回归基线保留。
>
> 本文只记录当前长期架构，不记录开发阶段、Codex prompt 或阶段性 TODO。若本文与代码/运行证据冲突，以根目录 `AGENTS.md` 中的 source-of-truth 顺序为准。

## 1. 系统目标

系统辨识主线为：

```text
机器人模型
  -> 设计并冻结安全激励轨迹
  -> 仿真或真机执行
  -> 采集 q / qd / effort
  -> 离线预处理与 qdd 估计
  -> 构造动力学回归矩阵 W
  -> A-only 基础空间与参数拟合
  -> B-only 独立验证
  -> 输出参数、预测、误差和 provenance
```

核心约束：数据物理语义、轨迹 provenance、A/B 独立性和真机安全优先于软件抽象。

## 2. 仓库的两条主链路

### 2.1 仿真辨识链路

```text
config/*.yaml
  -> run_experiment
  -> ForceController / Fourier excitation
  -> SimulationBackend
  -> MuJoCo
  -> ExperimentRecorder
  -> simulation CSV + metadata
  -> identify
  -> result.yaml + prediction.csv
```

reBot-DM 仿真模型由 MuJoCo 与 Pinocchio 共同校验。augmented regressor 的 raw columns 固定为：

```text
60 rigid-body
+ 6 armature
+ 6 viscous damping
+ 6 frictionloss
= 78 raw columns
```

冻结 synthetic clean baseline 在 `rank_relative_tolerance=1e-6` 下的 A-only base rank 为 52。该结果用于软件/数学闭环回归，不代表真实 reBot 物理参数。

### 2.2 reBot 真机 A/B 链路

面向操作者的唯一正式入口：

```bash
python3 scripts/run_rebot_real_ab.py --config config/rebot_real_ab.yaml
```

其内部只做编排，不复制底层控制或辨识算法：

```text
config/rebot_real_ab.yaml
  -> strict config validation
  -> offline preflight
       - SDK/model/binary dependency check
       - A/B frozen artifact + metadata hash check
       - preview acceptance check
       - runtime trajectory numerical limits check
       - A/B independence check
  -> create unique run directory
  -> materialize A/hardware.yaml
  -> operator ENTER gate A
  -> RebotHardwareRunner
       MoveJ(q0) -> settle -> Servo excitation -> controlled cleanup
  -> validate A completion
  -> materialize B/hardware.yaml
  -> operator ENTER gate B
  -> RebotHardwareRunner
       MoveJ(q0) -> settle -> Servo excitation -> controlled cleanup
  -> validate B completion
  -> materialize identification config
  -> run_rebot_identification.run_pipeline()
       preprocessing(A) -> freeze A resample rate
       preprocessing(B, frozen A rate)
       A-only SVD/base-space + OLS
       B-only reported-effort validation
  -> summary.yaml
```

A 失败时 B 不得执行；B 失败时辨识不得执行。

## 3. 统一配置职责

`config/rebot_real_ab.yaml` 是正式 A/B 实验唯一长期人工维护配置。它包含：

- SDK checkout、host、TCP/UDP port 和 timeout；
- 真机 authorization；
- joint mapping / J1 convention；
- position/velocity/acceleration/jerk/Servo step limits；
- feedback freshness/recovery 语义；
- MoveJ 参数和 controlled park 策略；
- A、B 各自 frozen artifact、metadata、preview acceptance；
- offline identification 模型、binary 和 preprocessing/solver 参数。

以下内容必须自动派生，不要求操作者手工填写：

- `RUN_ID` / run directory；
- A/B `output_csv`；
- trajectory duration；
- trajectory SHA-256；
- 每次运行的底层 `hardware.yaml`；
- identification 的 A/B raw 路径和 output directory；
- provenance snapshot。

## 4. 真机控制边界

真实控制链由 `src/rebot_real/runner.py` 与 `control_adapter.py` 实现。正式编排脚本不得复制或改写以下语义：

1. 真实 client 创建前完成 preflight 和 authorization gate；
2. excitation 开始前用 SDK MoveJ 从当前姿态移动到 frozen artifact 的 `q0`；
3. MoveJ 完成后要求新鲜反馈、位置误差和速度 settle；
4. 进入 Servo 后立即发送初始 hold target；
5. excitation 使用 `actual_time_quintic_v1`，按真实 host dispatch 时间重采样冻结连续轨迹；
6. 不允许 catch-up burst；
7. Servo accepted target 后才提交本地 finite-difference envelope history；
8. hard fault、state stream 超时、Servo reject、ownership 丢失等进入 fail-safe；
9. excitation 普通 `q_ref-q` tracking error 只作为 monitor-only 数据质量 warning，不单独触发立即失能；
10. 正常结束或可恢复的软件侧 abort 使用明确的 controlled park 策略，最终 disable/close。

外部 SDK 不属于本仓库，不得在本项目任务中静默修改。

## 5. 真机数据语义

真机 raw 数据由 `HardwareExperimentRecorder` 写出。关键字段：

- `timestamp_host_rx_ns`：upper host 收到/发布 SDK state snapshot 的 monotonic 时间；
- `timestamp_lower_ns`：SDK `JointState.monotonic_time_ns`，表示 lower 侧最新有效 driver feedback 的 steady-clock 时间，不是电机硬件 timestamp；
- `timestamp_host_command_ns` / `actual_dispatch_timestamp_ns`：实际传入 `ArmClient.servo_joint()` 的 upper-host monotonic dispatch 时间；
- `q`：SDK joint position，经 `joint_direction` / `joint_offset_rad` 映射后的 runner coordinate；
- `qd`：SDK joint velocity，经 `joint_direction` 映射；
- `effort_reported`：SDK `JointState.torque_nm` 的 joint-side reported effort estimate；
- `q_cmd`：本次 Servo 的 runner-coordinate target position。

当前没有：独立 hardware timestamp、raw motor encoder/current、`tau_cmd` 或独立力矩传感器真值。

`effort_reported` 目前没有独立完成物理 torque calibration，因此系统辨识输出的正确表述是“对 SDK reported effort 的预测拟合”，不能仅凭低 RMSE 宣称逐项恢复了真实质量/惯量/摩擦参数。

## 6. 命令频率与反馈频率必须分开

当前 2026-09-10 成功实机证据显示：

- A：2946 command rows，effective Servo dispatch 约 98.15 Hz；
- B：2940 command rows，effective Servo dispatch 约 97.95 Hz；
- 两次 lower/signal 独立更新约 10 Hz；
- CSV/command 接近 100 Hz 不等价于 100 Hz 独立物理观测。

因此 preprocessing、辨识解释和文档不得把 raw row cadence 当作传感器独立更新率。

## 7. A-only / B-only 识别原则

真实与 synthetic pipeline 都必须维持：

```text
A:
  choose/freeze preprocessing resample rate
  build column scales
  choose SVD rank/base directions
  fit beta_hat

B:
  reuse A preprocessing decision where required
  reuse A scales/base directions/beta_hat
  only evaluate independent prediction
```

B 可以报告自己的诊断统计，但不得反向修改 A 的 rank threshold、basis、参数或超参数。

## 8. Mock 边界

`--mock` 只用于验证 A/B orchestration、runner lifecycle、artifact provenance、输出目录和 failure dependency。

`MockArmClient` 的 ideal following 不包含真实机械臂动力学和 identification-grade physical effort。因此：

- Mock A/B motion completion 可以作为编排回归；
- Mock raw 若不满足辨识运动观测条件，应被真实 identification pipeline 拒绝；
- 此时可使用仓库已有 synthetic RNEA fixture 验证 preprocessing/identify 软件链；
- 不得把 fixture 或 Mock 结果表述成真机参数辨识结果。

## 9. 输出与 provenance

正式一次 A/B run 的结构：

```text
<RUN>/
  experiment_config.snapshot.yaml
  provenance.yaml
  A/
    hardware.yaml
    raw.csv
    raw.meta.yaml
  B/
    hardware.yaml
    raw.csv
    raw.meta.yaml
  identification/
    config.yaml
    A.csv
    B.csv
    identify.yaml
    identify.log
    result.yaml
    result.prediction.csv
    training.png
    validation.png
  summary.yaml
```

`summary.yaml` 只做汇总，不替代 raw/meta/result provenance。任何数值结论都应能追溯到本次 run directory 和 frozen trajectory SHA-256。

## 10. 长期维护边界

长期 source documents 只保留：

- `AGENTS.md`
- `README.md`
- `doc/ARCHITECTURE.md`
- `doc/DEVELOPMENT_RULES.md`
- `doc/REBOT_HARDWARE_CONTROL_CONTRACT.md`
- `doc/REBOT_HARDWARE_DATA_CONTRACT.md`
- `doc/REBOT_SYSTEM_IDENTIFICATION_GUIDE.md`

开发阶段、阶段 baseline 和 Codex prompt 不应继续作为当前架构文档存在；历史细节由 Git history 保存。
