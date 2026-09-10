# Robot Parameter Identification Simulation

机器人动力学参数辨识工程，包含 MuJoCo 仿真辨识、reBot-DM 动力学/回归器验证，以及 reBot 真机 A/B 激励采集与离线辨识。

## 当前推荐入口

reBot 正式真机实验只需要维护一个配置：

`config/rebot_real_ab.yaml`

现场唯一启动命令：

```bash
cd /home/j/j_ws/src/Robot-Parameter-Indentification-Simulation
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
cmake -S . -B build
cmake --build build --parallel 4

./build/rebot_mujoco_model_sanity
./build/rebot_model_consistency_test
./build/rebot_phase5a_regressor_test

./build/run_experiment \
  --experiment-config config/rebot_dm_excitation_experiment.yaml \
  --trajectory-seed 20260826 \
  --headless \
  --output /tmp/rebot_A.csv \
  --trajectory-output /tmp/rebot_A.trajectory.csv

./build/run_experiment \
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
