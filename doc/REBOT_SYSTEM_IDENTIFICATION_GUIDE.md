# reBot-DM 系统辨识操作指南

> 面向现场操作者与后续维护者的唯一长期操作文档。
>
> 正式真机 A/B 实验使用一个配置文件和一个入口脚本。底层 hardware runner、预处理脚本、trajectory 工具和 identify CLI 继续保留用于调试，但不应再由操作者手工拼成正式流程。

## 1. 当前状态

截至 2026-09-10，A/B 两条独立 frozen trajectory 均已在真实 reBot-DM 上完整执行成功：

- A SHA-256：`be2faa1d58fc834efbca030aea7ec9fbb28d57895aae8e449f36164e6141f131`；2946 条 command rows；effective Servo dispatch 约 98.15 Hz；
- B SHA-256：`73c0ad08360b0723ed13ba9e0e78d6cc924715bc567548fd5201c77a58457cc4`；2940 条 command rows；effective Servo dispatch 约 97.95 Hz；
- 两次均完成 SDK MoveJ 预定位、Servo excitation、controlled park、disable 与 close；
- 两次 lower/signal 独立更新约 10 Hz；
- excitation tracking threshold exceedance 是 monitor-only 数据质量 warning，不是单独的立即失能条件。

这些成功记录证明当前控制链可以完成 A/B 激励执行，但不等价于已经证明 `effort_reported` 是独立标定的真实关节 torque。

## 2. 正式实验只维护一个配置

配置文件：

```text
config/rebot_real_ab.yaml
```

长期人工维护内容分为八组：

1. `experiment`：实验名、输出根目录、正式 Servo nominal rate；
2. `connection`：SDK root、robot host、TCP/UDP port、timeouts；
3. `authorization`：是否允许连接真机、是否允许运动；
4. `mapping`：J1-J6 direction/offset、mapping acceptance scope、J1 convention；
5. `safety`：position/velocity/acceleration/jerk、Servo packet delta、timestamp interval；
6. `feedback`：motion-ready、disabled、lower feedback 和 host snapshot freshness/recovery；
7. `movej`：SDK MoveJ 限制、settle tolerance、controlled park；
8. `trajectories` + `identification`：A/B frozen evidence 与离线辨识参数。

不再手工设置：

```text
SDK_ROOT
ORIN_IP
TCP_PORT
UDP_PORT
RUN_ID
RUN_DIR
A_RUN_DIR
B_RUN_DIR
output_csv
trajectory_hash
trajectory duration
identification A/B raw path
identification output directory
```

这些都由统一入口从 YAML 和 frozen artifact 自动派生。

## 3. 实验前只做离线 preflight

建议每次正式真机前先运行：

```bash
cd /home/j/j_ws/src/Robot-Parameter-Indentification-Simulation
python3 scripts/run_rebot_real_ab.py \
  --config config/rebot_real_ab.yaml \
  --preflight-only
```

该模式不会创建任何 hardware client。它会在连接机械臂前检查：

- YAML schema 是否完全匹配；
- boolean、数组长度、数值范围和 mapping 是否有效；
- SDK checkout 是否存在且包含 `ArmClient`；
- `build_rebot/identify` 与 reBot URDF 是否存在；
- A/B trajectory CSV 与 metadata 是否可读；
- trajectory CSV 的实际 SHA-256 是否匹配 metadata；
- preview report/MP4/acceptance hashes 是否完整一致；
- `accepted_for_hardware=true`，且 operator/review date 已填写；
- frozen trajectory runtime position/velocity/acceleration/jerk/Servo delta 等数值门禁；
- A/B trajectory hash 是否独立。

任何一项失败都应停止，不得通过临时改脚本绕过。

## 4. 可选 Mock 回归

需要检查整个编排而不接真机时：

```bash
python3 scripts/run_rebot_real_ab.py \
  --config config/rebot_real_ab.yaml \
  --mock
```

`--mock` 只使用进程内 `MockArmClient`，不会加载或实例化真实 SDK client。

Mock 会完整验证：

```text
preflight
  -> A materialization / MoveJ / Servo / cleanup
  -> A completion gate
  -> B materialization / MoveJ / Servo / cleanup
  -> B completion gate
  -> identification orchestration
  -> summary
```

Mock 的 ideal following 不提供真实 mechanical dynamics/physical effort，因此 Mock raw 可能被真实 identify 正确拒绝。当前 orchestration 会明确保存这一结果，并用 synthetic RNEA fixture 继续验证离线 preprocessing/identify 软件链；fixture 不代表真机辨识。

## 5. 正式真机唯一命令

确认 preflight 通过、机械臂周围安全、急停与断电措施可用后，执行：

```bash
cd /home/j/j_ws/src/Robot-Parameter-Indentification-Simulation
python3 scripts/run_rebot_real_ab.py \
  --config config/rebot_real_ab.yaml
```

不需要另开脚本执行 A/B，也不需要修改本次 `hardware.yaml`。

### 5.1 A gate

程序会先显示：

- trajectory label；
- duration；
- control rate；
- trajectory SHA-256；
- target host/ports；
- output path。

此时程序尚未进入 A 的真实 runner 执行。操作者确认现场安全后按 ENTER；不确认则 Ctrl-C 退出。

A 内部顺序固定为：

```text
connect
  -> read disabled state
  -> configure existing SDK MoveJ PVT policy
  -> enable
  -> wait motion-ready fresh feedback
  -> SDK MoveJ to frozen A q0
  -> require settled position + low qd + idle/ready
  -> enter Servo
  -> immediate initial hold target
  -> actual-time quintic excitation
  -> exit Servo
  -> controlled park
  -> disable
  -> close
```

A 必须满足完整 endpoint、无 catch-up/timestamp mismatch、raw/meta 存在、cleanup 完成，程序才允许进入 B。

### 5.2 B gate

B 不复用 A 的运动 session；它会重新 materialize B 配置，并再次要求独立 ENTER gate。

B 的控制生命周期与 A 相同，但 frozen artifact/hash/preview acceptance 不同。

B 失败时程序不得启动 identification。

## 6. 为什么开始前要先 MoveJ 到 q0

frozen trajectory 本身不需要增加“当前姿态到 q0”的激励段。

正确做法是：

```text
current physical posture
  -> SDK MoveJ preposition
  -> q0 settled
  -> Servo owns q0
  -> frozen excitation starts at t=0
```

这样既不改变 frozen excitation 的数学内容和 SHA-256，也保证实际执行与 preview 的激励起点一致。

MoveJ 是 preposition/cleanup 机制，不属于系统辨识的 excitation samples。

## 7. 正式运行目录

preflight 完成后，脚本才创建新的唯一目录：

```text
data/rebot_real/<timestamp>_rebot_real_ab/
```

正式结构：

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

脚本使用 `exist_ok=False` 创建 run directory；正式 evidence 不允许静默覆盖历史 run。

## 8. raw 数据怎么理解

真机 CSV 不是仿真 truth schema。主要字段：

### 8.1 `q`

来源：SDK public `JointState.position_rad`，再应用本项目 mapping：

```text
q = joint_direction * q_sdk + joint_offset_rad
```

单位：rad。

### 8.2 `qd`

来源：SDK public `JointState.velocity_rad_s`，再应用 `joint_direction`。

单位：rad/s。

### 8.3 `effort_reported`

来源：SDK public `JointState.torque_nm` 经 direction mapping 后的 joint-side reported effort estimate。

单位字段由 SDK 表述为 Nm，但当前项目尚未通过独立 torque sensor/标定实验证明它等于高精度真实物理 joint torque。因此结果中统一使用：

```text
reported effort
reported-effort prediction
uncalibrated physical torque interpretation
```

而不是直接宣称“真实 torque parameter recovery”。

### 8.4 `q_cmd`

runner-coordinate Servo target position。adapter 会在调用 `ArmClient.servo_joint()` 前映射回 SDK coordinate。

### 8.5 时间戳

- `timestamp_host_command_ns`：发送 Servo command 使用的真实 upper-host monotonic 时间；
- `actual_dispatch_timestamp_ns`：实际 dispatch 时间，正常情况下应与 command timestamp 一致；
- `timestamp_host_rx_ns`：upper host 接收/发布最新 state snapshot 的 monotonic 时间；
- `timestamp_lower_ns`：lower 最新有效 driver feedback 的 steady-clock 时间；不是 motor device hardware timestamp。

## 9. 为什么 100 Hz CSV 不等于 100 Hz 传感器数据

当前成功实机记录中，Servo command/CSV 接近 100 Hz，但 `timestamp_lower_ns`、`q`、`qd`、`effort_reported` 的独立更新约 10 Hz。

因此：

- 不能用 raw row count 直接声称传感器为 100 Hz；
- 预处理必须识别 duplicate/latest-state publications；
- 系统辨识有效独立观测的时间分辨率受 lower/signal update cadence 约束；
- command timing 与 measurement timing 必须分开报告。

`raw.meta.yaml -> feedback_cadence` 已专门记录这些统计。

## 10. excitation 期间什么会停、什么不会停

### 10.1 会进入立即 fail-safe 的典型问题

- primary fault；
- lower safety state 为 protective/fault/emergency；
- state snapshot host receive age 超过通信 timeout；
- Servo ownership 意外丢失；
- Servo target 被 SDK/lower reject；
- non-finite/非法 state；
- feedback invalid/stale 超过已经审计的 lower timeout + recovery 语义；
- 实际 command finite-difference envelope 超出 position/delta/velocity/acceleration/jerk/timestamp interval 限制。

### 10.2 不会仅凭它立即失能

正式 excitation 中：

```text
abs(q_cmd - q) > maximum_tracking_error_rad
```

是 tracking/identification quality warning。它会写入 metadata 和 summary，但不会单独立即停止 excitation。

原因是系统辨识本来就需要观测真实 tracking lag；只要没有出现 hard safety/communication/Servo fault，就应尽量完成整条 excitation，再离线判断这组数据是否适合辨识。

## 11. A/B 离线辨识逻辑

B 不参与 A 的拟合决策。

顺序：

```text
A raw
  -> duplicate/gap/validity handling
  -> low-pass/filter + edge trim
  -> select A resample rate
  -> derive qdd estimate
  -> build W_A
  -> column scaling
  -> SVD rank/base directions
  -> OLS beta_hat

B raw
  -> same semantics
  -> reuse frozen A resample rate
  -> build W_B
  -> reuse A scales/base directions/beta_hat
  -> independent reported-effort prediction
```

当前 real-mode identifier 使用：

```text
60 rigid + 6 armature + 6 damping + 6 frictionloss = 78 raw columns
```

但 raw parameter count 不是“78 个参数都能逐项恢复”的保证。真正可观测的是 A 数据与 rank threshold 决定的 base parameter space。

## 12. 结果重点看什么

先看顶层：

```text
<RUN>/summary.yaml
```

每条 trajectory 至少关注：

- `motion_status`；
- `sample_count`；
- `trajectory_sha256`；
- effective `dispatch_rate_hz`；
- lower feedback update rate；
- tracking quality/max lag；
- fault；
- cleanup status。

辨识关注：

- `base_parameter_rank / full_parameter_count`；
- A reported-effort RMSE；
- B reported-effort RMSE；
- per-joint validation error；
- `validation.png`；
- `result.yaml` 中 preprocessing/provenance/interpretation。

低 B RMSE 只证明在当前数据和模型假设下，对 SDK reported effort 有较好的独立预测能力。是否得到可信真实质量、质心、惯量、摩擦参数，还需要额外物理标定/约束/实验验证。

## 13. 失败后怎么处理

正式脚本是 fail-closed：

- preflight 失败：不会建立真实 ArmClient session；
- A 失败：写本次 `summary.yaml`，不进入 B；
- B 失败：写本次 `summary.yaml`，不进入 identification；
- identification 失败：A/B raw evidence 保留，summary 标记 FAIL，不覆盖旧结果。

排障时优先查看：

```text
<RUN>/summary.yaml
<RUN>/A/raw.meta.yaml
<RUN>/B/raw.meta.yaml
<RUN>/identification/identify.log
```

只有需要单独定位底层 runner/SDK 行为时，再使用 `scripts/run_rebot_hardware.py` 等高级工具。不要把调试命令重新变成正式操作流程。

## 14. frozen trajectory 生成与更换流程

仓库可以从 C++ Fourier 源确定性重建当前 A/B trajectory，不需要向项目负责人索取轨迹文件。完成 `build_rebot/` 构建后，在仓库根目录执行：

```bash
python3 scripts/prepare_rebot_ab.py \
  --build-directory build_rebot \
  --output-directory results/rebot_real_ab_generated
```

输出目录必须为新目录。脚本会生成 A/B 各自的 `coefficients.csv`、`trajectory.csv`、`trajectory.meta.yaml`、`preview_report.yaml`、`preview.mp4` 和默认未授权的 `preview_acceptance.yaml`，并在顶层保存 `qualification.snapshot.yaml` 与 `manifest.yaml`。

无显示环境时可加 `--skip-video` 进行纯数值检查，但该模式不会产生可授权的 preview acceptance。历史 measured timing CSV 不存在时，脚本会明确退回 nominal/alternating timing profiles 并将实际配置写入 snapshot；这不替代人工预览和现场安全确认。

生成后的正确验收流程：

1. 确认 A/B 均从可信 C++ 生成器导出，且 trajectory SHA-256 不同；
2. 做连续 quintic 数值安全和 collision qualification；
3. 用 exact frozen excitation 生成 preview MP4；
4. 人工观看；
5. 在 `preview_acceptance.yaml` 中填写 operator/review date/notes，并仅对 matching trajectory/report/MP4 hashes 将 `accepted_for_hardware` 设为 true；
6. 再把新的 `artifact`、`metadata`、`preview_acceptance` 三个路径填入 `config/rebot_real_ab.yaml`；
7. 重新执行 `--preflight-only`；
8. 只有 preflight PASS 后才允许现场实验。

A/B 必须保持不同 trajectory SHA-256。

## 15. 开发/回归检查

修改正式 A/B orchestration、runner、recorder、preprocessing 或 identification 后至少执行：

```bash
python3 -m pytest tests/rebot_real -q
git diff --check
```

仅修改统一配置/文档时，也应至少运行：

```bash
python3 scripts/run_rebot_real_ab.py \
  --config config/rebot_real_ab.yaml \
  --preflight-only
```

涉及实际控制语义时必须同时遵守：

- `doc/REBOT_HARDWARE_CONTROL_CONTRACT.md`
- `doc/REBOT_HARDWARE_DATA_CONTRACT.md`

## 16. 最终操作原则

正式现场流程应保持简单：

```text
维护 config/rebot_real_ab.yaml
  -> preflight-only
  -> 确认现场安全
  -> 单次启动正式脚本
  -> A ENTER
  -> B ENTER
  -> 查看 summary / identification outputs
```

不要再手工创建 run id、复制 hardware YAML、拼接 A/B raw 路径或单独启动 identification。所有派生配置都应由 orchestration 自动生成，并保存在本次 run directory 供审计。
