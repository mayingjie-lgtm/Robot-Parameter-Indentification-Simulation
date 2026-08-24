# Phase 1 Baseline — Robot Parameter Identification Simulation

> 本文记录 Phase 1 审计中**已经在当前主机实际验证过**的行为。
>
> 这里首先保存的是 `smoke baseline`，用于证明当前链路可以构建、运行和完成一次辨识。它还不是最终论文/正式实验 baseline；没有实际验证的项目统一标记为 `NOT YET VERIFIED`。

---

# 1. Baseline Reference

审计时仓库：

```text
branch: main
HEAD: 802ea4778d58cf1d8f775c22267dabcc8899b335
subject: 准备piper真机
```

审计开始前 working tree 已存在用户修改：

```text
M  results/identification.yaml
?? AGENTS.md / doc/*.md（文档整理阶段逐步加入）
```

这些用户修改未被 reset、stash、clean 或覆盖。

---

# 2. Current Runtime Architecture Verified

当前统一主链路已确认存在：

```text
config/experiment.yaml
        ↓
run_experiment
        ↓
ForceController
        ↓
ExperimentBackend
   ├── SimulationBackend → MuJoCo
   └── PiperHardwareBackend → Python bridge → Piper SDK
        ↓
ExperimentRecorder
        ↓
CSV
        ↓
DataLoader
        ↓
Identification / MuJoCo regressor
        ↓
solver
        ↓
validation
```

当前 `ExperimentState` 包含：

```text
position
velocity
effort
```

`qdd` 不由 backend 直接传入统一 recorder。

---

# 3. Current Data Semantics Verified

| Field | Simulation source | Piper real source | Current CSV behavior | Confidence |
|---|---|---|---|---|
| `q` | MuJoCo `qpos` | SDK joint position，经 adapter 转为 rad | 写入 `q*` | code verified |
| `qd` | MuJoCo `qvel` | SDK high-speed motor speed，经 adapter 当前换算后返回 | 写入 `qd*` | code verified；真机物理单位仍需 SDK 文档/实机确认 |
| `qdd` | backend 未直接提供 | backend 未直接提供 | `ExperimentRecorder` 对相邻 `state.velocity` 做一阶差分 | code verified |
| `tau_cmd` | `ControlCommand.torque` | `ControlCommand.torque` 作为 MIT feedforward torque 发送 | 当前写入 CSV `tau*` | code verified |
| `state.effort` | MuJoCo `qfrc_actuator` | Piper SDK reported effort | 当前统一 CSV 未记录 | code verified |

因此当前必须牢记：

```text
CSV qdd != 已证明的 MuJoCo qacc
CSV tau = command torque
CSV tau != 已证明的 measured joint torque
```

另外，当前 `DataLoader` 只要检测到 CSV 含 `qdd` 列，就会打印类似“with qdd from physics engine”的描述。对于统一 `ExperimentRecorder` 生成的数据，这个描述与实际来源不一致，应视为已知数据语义问题，而不是来源证据。

---

# 4. Build Smoke Baseline

执行过：

```bash
cmake --build build --parallel 4
```

结果：成功。

已确认构建的主要 target：

```text
force_controller_core
panda_simulator_core
identification_core
run_experiment
identify
mujoco_identify
dynamics_diagnostic
model_comparison
regressor_test
```

从全新空 build directory 执行完整 `cmake -S . -B build` 的 clean build：

```text
NOT YET VERIFIED as formal Phase 1 baseline
```

---

# 5. Piper Simulation Smoke Baseline

使用当前 `config/experiment.yaml` 的 Piper simulation 配置进行 headless smoke test，并将输出写到临时文件以避免覆盖仓库数据。

实际链路：

```text
robot: piper
backend: sim
```

观察结果：

```text
安全激励轨迹搜索成功
trajectory duration: 30 s
experiment end time: 32 s
force saturation ratio: 0%
CSV rows: 32000
CSV columns: 25
first timestamp: 0.001 s
last timestamp: 32.000 s
NaN/Inf: not observed in smoke check
```

CSV schema：

```text
time
q0..q5
qd0..qd5
qdd0..qdd5
tau0..tau5
```

这次 smoke run 中控制器日志还显示：

```text
找到安全激励轨迹，尝试次数: 37
幅值: 0.0694937
```

注意：该临时数据集用于 smoke validation，不作为正式论文实验数据集。

---

# 6. OLS Identification Smoke Baseline

对上述 Piper simulation smoke dataset 执行 OLS。

观察结果：

```text
samples: 32000
train/validation split: same CSV, first 80% / last 20%
filtered training outliers: 235
valid training samples: 25365
W(all): 152190 x 72
W(armature): 152190 x 66
```

MuJoCo dynamics diagnostic printed by `identify`：

```text
RMSE: 0.529249 Nm
Max Error: 85.6476 Nm
```

OLS validation：

```text
RMSE: 0.00870139 Nm
Max Error: 0.034033 Nm
```

重要解释限制：

1. 当前 validation 是同一条 CSV 的时间切分，不是独立激励轨迹；
2. 当前 CSV `qdd` 来自速度差分；
3. 当前 CSV `tau` 来自 command torque；
4. 因此不能只根据 `0.0087 Nm` 的 hold-out RMSE 就断言真实动力学参数已经高精度恢复；
5. `MuJoCo dynamics RMSE 0.529249 Nm / Max 85.6476 Nm` 与 OLS 的低 validation RMSE 之间存在明显差异，后续需要结合数据语义、outlier、regressor 参数化和评价对象继续审计。

---

# 7. Piper Real Tests

执行过：

```bash
python3 -m pytest tests/piper_real -q
```

结果：

```text
13 passed
```

这证明当前 Python bridge / safety / config 相关单元测试通过，但**不等价于真机环境已经验证**。

以下项目仍为：

```text
真实 CAN 连接: NOT YET VERIFIED
真实 Piper SDK 安装/版本兼容: NOT YET VERIFIED
真机 velocity 单位和精度: NOT YET VERIFIED
真机 reported effort 的物理定义/标定: NOT YET VERIFIED
真实 Fourier excitation: NOT YET VERIFIED
```

当前仓库顶层没有 `piper_sdk/` 源码目录；真机运行前需要保证 Python 环境能够正常 `import piper_sdk`，并按实际使用的 SDK 版本完成 CAN 配置。

---

# 8. Diagnostic Executables

## `regressor_test`

当前源码中存在旧机器硬编码数据路径：

```text
src/identification/src/regressor_test.cpp
/home/windiff/Code/Simulation/data/...
```

实际尝试运行时无法形成可重复的当前主机 baseline，因此：

```text
status: FAIL / NOT PORTABLE
priority: P2/P3 depending on whether it blocks reBot regression work
```

## `dynamics_diagnostic`

```text
formal standalone run: NOT YET VERIFIED
```

## `model_comparison`

```text
formal standalone run: NOT YET VERIFIED
```

## `mujoco_identify`

```text
formal standalone run: NOT YET VERIFIED
```

Phase 1 后续应逐个确认这些工具的输入、输出、用途和是否值得保留。

---

# 9. Validation Model Currently Implemented

当前 `identify`：

```text
one CSV
  ├── first 80% → training
  └── last 20%  → validation
```

因此当前 validation 正确名称应是：

```text
temporal hold-out validation on the same excitation run
```

目标正式实验还需要：

```text
identification trajectory A
        ↓
parameter estimation
        ↓
independent validation trajectory B
        ↓
torque prediction metrics
```

---

# 10. Known Issues

## P0 — 会直接导致辨识结论错误

当前没有在 smoke audit 中直接判定为 P0 的问题；需要完成更严格数学和数据来源验证后再定级。

## P1 — 数据可信度 / 结果解释

1. CSV `qdd` 当前来自速度差分，但 loader 会把“存在 qdd 列”描述成 physics-engine qdd；
2. CSV `tau` 当前是 `command.torque`，没有区分 `tau_cmd` 与 `state.effort`；
3. Piper real effort 的真实物理定义、单位和标定尚未通过 SDK 文档/真机验证；
4. validation 仍是同轨迹 80/20 split，不是独立轨迹验证；
5. MuJoCo dynamics diagnostic error 与 OLS hold-out error 差异较大，需要解释后才能把 RMSE 当成可信辨识指标。

## P2 — reBot 接入前应处理/确认

1. 需要明确 reBot simulation/model 与 real backend 的最小接入点；
2. 需要决定 reBot regressor 使用可靠库（目标优先 Pinocchio）还是复用现有 robot-specific MuJoCo 路径；
3. diagnostics 工具中仍有旧绝对路径，影响跨机器人 regression workflow；
4. joint mapping、重力方向、模型惯量和执行器参数需要独立 consistency test。

## P3 — 架构 / 文档 / 命名

1. `PandaSimulator` 名称已承担 Piper MuJoCo model 的通用 simulator 角色，但 Phase 1 不为命名做重构；
2. 历史 `plan.md` 的“当前状态”已经过期，应仅作为历史设计记录；
3. README 曾包含旧 `/home/windiff/...` 绝对路径，文档整理阶段已改为仓库相对引用。

---

# 11. Formal Phase 1 Baseline Still Required

在宣布 Phase 1 完成前，还应至少补齐：

```text
clean configure/build command and result
Panda simulation baseline（如果仍作为必须支持的 baseline）
Piper simulation dataset statistics: per-joint min/max and sampling jitter
diagnostics executables status
W rank / singular value / condition information（若下一阶段加入指标）
independent validation trajectory design
exact experiment metadata record
```

完成后再把本文从 `smoke baseline` 更新为正式的 `Phase 1 reproducible baseline`。
