# Phase 1 — 最小执行计划：理解项目 + 建立 Baseline

> Phase 1 不做核心架构重构。
>
> 目标：让项目负责人可以完整解释现有系统辨识链路，并得到一份可重复运行的 Panda/Piper baseline。

---

# 1. Phase 1 的目标

最终应得到：

```text
当前代码架构图
当前实验数据流
当前辨识数学链路
当前配置说明
Panda/Piper baseline
当前已知问题清单
```

而不是：

```text
大量新代码
复杂 abstraction
reBot 接入
```

---

# 2. Step 1：Repository Baseline

只读。

记录：

```bash
git status --short
git branch --show-current
git rev-parse HEAD
git log -5 --oneline
```

确认：

```text
是否存在用户未提交修改
build directory
当前可执行文件
当前 config
```

---

# 3. Step 2：建立代码地图

重点阅读：

```text
CMakeLists.txt

src/app/run_experiment.cpp
src/app/experiment_backend.hpp
src/app/simulation_backend.*
src/app/experiment_recorder.*

src/force_node/include/force_node/force_controller.hpp
src/force_node/src/force_controller.cpp

src/sim_com_node/.../panda_simulator.*

src/identification/include/identification/*
src/identification/src/main.cpp
src/identification/src/identification.cpp
src/identification/src/data_loader.cpp

mujoco_*_dynamics
mujoco_*_regressor
```

输出：

```text
文件
主要类/函数
输入
输出
上下游调用关系
```

---

# 4. Step 3：画出完整 Runtime Data Flow

必须按**当前代码真实行为**画链路，不得把目标态写成现状。当前已确认的统一仿真链路至少应体现：

```text
FourierTrajectory
     ↓
desired q / qd
     ↓
PD controller
     ↓
command torque
     ↓
MuJoCo actuator
     ↓
ExperimentState(q / qd / effort)
     ↓
ExperimentRecorder
     ├── qdd = velocity finite difference
     └── CSV tau = command torque
     ↓
CSV
     ↓
DataLoader
     ↓
W
     ↓
solver
     ↓
beta_hat
     ↓
tau_hat
     ↓
RMSE
```

同时记录：MuJoCo simulator 内部可以访问 `qacc` / `qfrc_actuator`，但它们当前并未按这些物理来源进入统一 CSV。

---

# 5. Step 4：跑一份现有 Simulation Baseline

优先使用当前已经成功运行的 Piper 或 Panda 配置。

保存：

```text
完整 command
config
dataset path
sample count
runtime
```

检查 CSV：

```text
columns
q range
qd range
qdd range
tau range
NaN
timestamp interval
```

---

# 6. Step 5：跑现有 Identification Baseline

至少先跑：

```text
OLS
```

如果 `algorithm: 0` 当前会跑全部算法，也记录。

必须保存：

```text
W rows
W cols
beta size
validation RMSE
max error
```

当前代码没有 rank/SVD 也没关系。

Phase 1 第一轮只记录现有行为。

---

# 7. Step 6：人工验证数学关系

随机选择若干 sample。

确认概念上：

```text
q, qd, qdd
      ↓
regressor
      ↓
Y
```

然后：

```text
W beta_hat
```

和：

```text
tau measured/recorded
```

是当前 validation 的实际比较对象。

不用马上改数学。

---

# 8. Step 7：确认当前数据语义

重点查清：

```text
CSV tau 到底来自哪里？
CSV qdd 到底来自哪里？
Simulation state.effort 是什么？
Piper real effort 是什么？
```

输出一张表：

| Field | Simulation source | Real source | Unit | Confidence |
|---|---|---|---|---|

如果发现：

```text
tau 实际是 command
```

只记录问题。

Phase 1 审计阶段先不急着改。

---

# 9. Step 8：检查现有 regression / diagnostics 工具

查：

```text
regressor_test
dynamics_diagnostic
model_comparison
mujoco_identify
tests/
```

对每个 executable 写一句：

```text
它验证什么？
当前是否能运行？
对 reBot 是否有价值？
```

---

# 10. Step 9：收口项目学习文档

Codex 最终应：

```text
复核并增量更新 doc/ARCHITECTURE.md
复核并增量更新 doc/DEVELOPMENT_RULES.md
生成/更新 doc/PHASE1_BASELINE.md
```

不得为了完成任务而覆盖已经人工确认的架构/开发规则。

`PHASE1_BASELINE.md` 必须包含实际本地运行结果，并区分：

```text
smoke baseline
formal reproducible baseline
```

若某项尚未完成正式验证，明确写 `NOT YET VERIFIED`，不要推断。

---

# 11. Step 10：只允许给出下一步建议

Phase 1 完成后，根据实际结果，把问题分成：

```text
P0：会导致当前辨识结论错误
P1：影响数据可信度
P2：影响 reBot 接入
P3：纯架构/代码质量
```

优先解决：

```text
P0/P1
```

暂时不解决：

```text
P3
```

---

# 12. Phase 1 禁止修改

本阶段禁止：

```text
新增 regressor factory
新增 plugin architecture
删除旧代码
rename PandaSimulator
重写 Identification
统一两套 robot core
接 Pinocchio
接 reBot
改动力学数学
```

---

# 13. Phase 1 完成标准

项目负责人能够回答：

```text
1. run_experiment 的入口在哪里？
2. controller 的 torque 怎么得到？
3. MuJoCo state 怎么取得？
4. CSV 的 q/qd/qdd/tau 分别来自哪里？
5. identify 的入口在哪里？
6. W 是在哪生成？
7. beta 是怎么求？
8. validation 怎么算？
9. 当前 tau 是 command 还是 effort？
10. 当前 qdd 是 qacc 还是 differentiation？
11. Panda 与 Piper 在哪里发生代码分支？
12. reBot 最小需要接入哪些位置？
```

并且有可重复的 baseline command 和结果。
