# Codex Prompt — Phase 1 只读审计 + Baseline + 架构文档

项目路径：

`/home/wlsea1/j_ws/src/Robot-Parameter-Indentification-Simulation`

## 任务目标

这一轮**不要进行核心代码重构**。

你的角色首先是：

```text
代码审计员
+
实验工程师
```

而不是架构重构工程师。

本轮目标：

```text
完整理解当前项目
建立可重复 baseline
复核并增量更新已有架构/开发协议文档
生成 PHASE1_BASELINE.md
输出下一步最小修改建议
```

---

# 0. 强制约束

本轮禁止：

```text
新增 factory
新增 plugin system
新增复杂 interface
rename PandaSimulator
重写 Identification
统一两套 robot core
修改动力学数学
修改 regressor 数学
接 Pinocchio
接 reBot
接 reBot 真机
删除 legacy Panda/Piper code
大规模代码格式化
```

默认：

```text
不修改核心 .cpp/.hpp
```

除非为了让现有 baseline 能正常运行而存在明确 build bug。

如果发现 bug：

```text
先报告
不要直接修
```

---

# 1. 先读项目规则

必须按根目录 `AGENTS.md` 的文档优先级读取：

```text
AGENTS.md
doc/ARCHITECTURE.md
doc/DEVELOPMENT_RULES.md
doc/PHASE1_MINIMAL_PLAN.md
doc/PHASE1_BASELINE.md（若已存在）
README.md
```

`plan.md` 当前属于历史 Piper 真机接入计划，**不得作为当前架构 source of truth**。只有需要追溯历史设计决策时才读取，并且必须与当前代码核对。

如果文档与当前代码/可重复运行结果冲突，先报告冲突，再按 `AGENTS.md` 的 source-of-truth 顺序处理。

---

# 2. Repository Status

执行：

```bash
git status --short
git branch --show-current
git rev-parse HEAD
git log -5 --oneline
git diff --stat
```

输出：

```text
HEAD
branch
是否干净
是否存在用户改动
```

严禁：

```text
git reset
git checkout .
git clean
git stash
```

---

# 3. 确认 Build Baseline

检查已有 build 目录和 README。

优先复用用户当前已经成功使用的 build 方式。

记录：

```text
CMake command
build command
build result
主要 executable
```

不要擅自升级依赖。

---

# 4. 做代码级调用链审计

重点阅读：

```text
CMakeLists.txt

src/app/run_experiment.cpp
src/app/experiment_backend.hpp
src/app/simulation_backend.hpp
src/app/simulation_backend.cpp
src/app/experiment_recorder.hpp
src/app/experiment_recorder.cpp

src/force_node/include/force_node/force_controller.hpp
src/force_node/src/force_controller.cpp

src/sim_com_node/include/sim_com_node/panda_simulator.hpp
src/sim_com_node/src/panda_simulator.cpp

src/identification/include/identification/data_loader.hpp
src/identification/include/identification/identification.hpp
src/identification/src/data_loader.cpp
src/identification/src/identification.cpp
src/identification/src/main.cpp

src/identification/include/mujoco_regressor.hpp
src/identification/include/mujoco_piper_regressor.hpp
对应 cpp

mujoco_panda_dynamics
mujoco_piper_dynamics
```

生成调用链：

```text
config
→ run_experiment
→ controller
→ backend
→ recorder
→ CSV
→ loader
→ regressor
→ solver
→ validation
```

每一层写：

```text
input
output
关键 class/function
robot-specific 部分
```

---

# 5. 特别审查数据语义

必须回答：

## q

```text
Simulation 来源？
Real 来源？
```

## qd

```text
Simulation 来源？
Real 来源？
单位？
```

## qdd

确认：

```text
run_experiment 统一 recorder 最终写的是 MuJoCo qacc
还是 velocity differentiation？
```

## tau

确认：

```text
CSV tau 最终来自 command.torque
还是 state.effort
还是 data->ctrl
还是 data->qfrc_actuator
```

把结论写清楚，不允许模糊描述。

同时追踪 Piper real：

```text
SDK effort 的来源和单位
```

如果仅能从代码确认换算方式，明确写：

```text
这是 SDK reported effort，不等于已经证明的真实关节扭矩。
```

---

# 6. 运行 Simulation Baseline

选择当前默认、最容易成功的一套 Panda 或 Piper simulation config。

执行：

```text
run_experiment
```

优先 headless。

记录：

```text
command
robot
backend
scene
controller config
output CSV
sample count
```

检查 CSV：

```text
header
time interval
q min/max
qd min/max
qdd min/max
tau min/max
NaN/Inf
```

不要改参数来追求更好结果。

目标是记录当前真实 baseline。

---

# 7. 运行 Identification Baseline

先使用现有数据和现有 config。

至少运行：

```text
OLS
```

如果现有 algorithm=0 可运行所有算法，可以额外跑。

记录：

```text
command
robot
data file
sample count
W shape
beta dimension
RMSE
max error
```

如果当前程序已经有 train/validation split，说明具体逻辑。

---

# 8. 审查现有 Diagnostic Executables

检查：

```text
regressor_test
dynamics_diagnostic
model_comparison
mujoco_identify
```

尽可能运行已有安全的 simulation-only diagnostic。

对每一个给出：

```text
purpose
input
output
是否通过
是否应该保留
```

---

# 9. 审查代码重复与 robot-specific 分支

只记录，不重构。

重点回答：

```text
Panda/Piper 分支在哪里？
哪些代码已经 robot-agnostic？
哪些代码名字特化但实现其实可以泛化？
哪些 robot/* 源码存在重复？
两套 robot core 是否已经内容分叉？
```

特别检查：

```text
src/force_node/include/robot
src/identification/include/robot
```

不要合并。

---

# 10. 审查 generic regressor 的可信度

检查：

```text
src/identification/src/robot/regressor.cpp
```

如果存在：

```cpp
R_loc = Identity
```

或者：

```text
HYPOTHESIS TEST
```

明确记录：

```text
generic DH regressor 当前仍带实验性实现，
不应该直接作为 reBot 的最终动力学 regressor。
```

不要修改它。

---

# 11. 在仓库生成文档

如果 `doc/` 存在，将文档放入：

```text
doc/ARCHITECTURE.md
doc/DEVELOPMENT_RULES.md
doc/PHASE1_BASELINE.md
```

如果文件已经存在：

```text
先阅读
区分 current behavior 与 target behavior
保留已有有价值内容
只做必要的增量编辑
```

不要直接覆盖用户文档，也不要把目标架构描述成当前已经实现。

## ARCHITECTURE.md

至少包含：

```text
项目目标
当前 runtime data flow
系统辨识数学链路
模块职责
数据字段物理语义
robot-specific / agnostic 边界
未来 reBot 接入点
当前明确不做的重构
```

## DEVELOPMENT_RULES.md

至少包含：

```text
最小修改原则
修改前审计协议
动力学代码保护规则
数据语义规则
baseline regression 规则
AI/Codex 修改规模限制
不做过度抽象原则
```

## PHASE1_BASELINE.md

必须用本机真实执行结果填写，并区分 `smoke baseline` 与 `formal reproducible baseline`：

```text
git HEAD
build
run_experiment command
dataset
CSV schema
identify command
W shape
RMSE
diagnostic results
known issues
```

如果某项尚未在本机验证，写 `NOT YET VERIFIED`；禁止根据 README、旧 plan 或文件名猜测“已经通过”。

---

# 12. 本轮允许写入的范围

优先只写：

```text
doc/*.md
```

如果不需要修 build bug：

```text
不要修改 src/
不要修改 config/
不要修改 CMakeLists.txt
```

---

# 13. 最终问题分级

完成审计后，把问题分成：

## P0

```text
会直接导致当前辨识结论错误
```

## P1

```text
影响实验数据可信度或结果解释
```

## P2

```text
未来 reBot 接入会遇到
```

## P3

```text
代码结构/命名/重复问题
```

必须优先建议：

```text
P0/P1
```

不能因为 P3 看起来更容易重构就优先做 P3。

---

# 14. 下一阶段建议限制

最终最多提出：

```text
3 个下一步任务
```

每个任务必须满足：

```text
目的明确
改动小
可以单独验证
```

不要提出一轮十几个文件的大重构。

---

# 15. 最终输出格式

## 1. Repository status

## 2. Current architecture

用 ASCII 图。

## 3. Data semantics

表格：

```text
field
simulation source
real source
unit
current reliability
```

## 4. Baseline

```text
build
experiment
identification
diagnostics
```

## 5. Robot-specific audit

## 6. Known problems

按 P0/P1/P2/P3。

## 7. Documents created/updated

## 8. Recommended next 3 minimal tasks

## 9. Modified files

原则上应只有：

```text
doc/*.md
```

如果不是，解释原因。
