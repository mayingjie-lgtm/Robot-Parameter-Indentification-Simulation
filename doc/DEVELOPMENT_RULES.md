# Robot Parameter Identification Simulation — Development Rules

> 本文件约束人工开发和 Codex/AI 辅助开发。
>
> 核心目标：**代码必须能够被项目负责人掌握、解释、验证。**

---

# 1. 总原则

## Rule 1：先理解，再修改

任何任务开始前必须：

```text
读取相关代码
确认当前数据流
确认当前 baseline
说明要解决的具体问题
```

禁止：

```text
看到重复就立即重构
看到旧代码就“顺手优化”
为了更现代而改架构
```

如果文档、注释和当前代码/可重复运行结果冲突：

```text
先报告冲突
区分 current behavior 与 target behavior
不得为了“让代码符合文档”而直接改代码
```

文档权威顺序以根目录 `AGENTS.md` 为准。

---

## Rule 2：默认采用最小修改

优先级：

```text
修改现有配置
>
修改现有函数
>
新增小型辅助函数
>
新增一个独立类
>
新增 abstraction/interface/factory
>
重构整个模块
```

必须从上往下选择。

---

## Rule 3：每一次修改只解决一个主要问题

推荐一个任务只解决：

```text
一个 bug
一个数据语义问题
一个 metric
一个 backend 接口问题
一个机器人接入问题
```

禁止一轮任务同时：

```text
修 bug + rename + 重构 + 加 framework + 改数学模型
```

---

# 2. AI / Codex 修改前置协议

Codex 在写代码前必须先输出：

```text
1. Current behavior
2. Problem
3. Minimal proposed change
4. Files to modify
5. Files explicitly not modified
6. Risks
7. Validation method
```

如果预计修改超过：

```text
8 个现有文件
```

或新增超过：

```text
3 个核心类
```

必须停止编码并先说明为什么不能再缩小。

---

# 3. 禁止过度抽象

新增 interface / factory / manager / service 前必须回答：

```text
当前重复代码在哪里？
现在已经出现几个 concrete implementation？
不抽象会产生什么具体 bug / 维护问题？
一个简单 if/else 是否已经足够？
```

如果答案只是：

```text
未来可能扩展
看起来更优雅
符合设计模式
```

则不得新增抽象。

---

# 4. 可读性优先

优先：

```cpp
if (robot == "piper") {
    ...
} else if (robot == "rebot") {
    ...
}
```

而不是在只有两个实现时立即创建：

```text
Factory
Registry
Provider
Resolver
Plugin
Adapter chain
```

当重复真正出现后再抽象。

---

# 5. 动力学代码特殊保护

以下代码属于高风险区域：

```text
inverse dynamics
regressor
inertia transformation
coordinate transformation
gravity
friction model
parameter vector ordering
```

任何修改都必须：

1. 单独 commit / task；
2. 明确数学公式；
3. 给出 old vs new numerical comparison；
4. 有 regression test；
5. 不允许仅因为代码风格问题进行修改。

---

# 6. 数据物理语义协议

以下字段不得混用：

```text
q
qd
qdd
tau_cmd
tau_effort
tau_sensor
motor_current
```

每个数据源都必须能够回答：

```text
单位是什么？
来源是什么？
是否经过滤波？
是否经过换算？
是否是真实测量还是估计？
```

严禁：

```text
把 tau_cmd 重命名成 tau 后默认是真实 torque
```

---

# 7. 仿真与真机必须显式区分

仿真可用：

```text
qpos
qvel
qacc
qfrc_actuator
```

真机可能只有：

```text
q
velocity estimate
motor effort estimate
```

因此：

```text
sim data path
```

和：

```text
real data path
```

允许使用不同 preprocessing。

不要为了接口统一，隐藏真实的数据质量差异。

---

# 8. 保持 baseline

每次功能修改必须确保：

```text
现有 Panda/Piper baseline 不退化
```

修改前保存：

```text
build result
run command
W shape
RMSE
关键输出
```

修改后重新执行。

如果数值变化：

```text
必须解释原因
```

不能把变化简单归因于“重构”。

---

# 9. 不删除旧实现

新实现刚加入时：

```text
旧路径保留
```

直到：

```text
新旧数值验证通过
项目负责人已经理解新路径
```

才允许删除旧路径。

---

# 10. 每一步保持可运行

禁止：

```text
先改 20 个文件
最后一起编译
```

推荐：

```text
修改 1
→ build/test
→ 修改 2
→ build/test
```

任何中间 commit 都应尽可能：

```text
可编译
可运行
可回滚
```

---

# 11. Robot 接入原则

新增机器人时优先添加：

```text
model
config
hardware adapter
```

而不是复制：

```text
whole experiment pipeline
whole identification pipeline
whole algorithm implementation
```

但如果现有架构强行抽象会增加理解成本：

```text
允许短期保留少量 robot-specific branch
```

---

# 12. reBot 特别规则

第一版 reBot：

```text
只处理 J1-J6
不辨识 gripper
```

优先顺序：

```text
模型加载
→ joint mapping
→ gravity consistency
→ inverse dynamics consistency
→ excitation simulation
→ identification
→ validation
→ real robot
```

禁止直接跳到：

```text
real Fourier excitation
```

---

# 13. 真机安全规则

任何真机轨迹上线前必须检查：

```text
joint position limits
velocity limits
torque limits
collision
home pose
emergency stop
communication timeout
control mode
```

第一轮真机实验优先：

```text
静态
低速
小幅值
```

---

# 14. 实验复现协议

每次辨识实验至少记录：

```text
robot
model version
git commit
config
trajectory seed
sampling frequency
control frequency
dataset path
torque source
qdd source
algorithm
parameter flags
train/validation split
```

没有这些 metadata 的结果不能作为正式实验结果。

允许存在用于快速确认链路可运行的 smoke baseline，但必须明确标注为 `smoke`，不能与正式可复现实验结果混用。

---

# 15. 评价指标协议

不得只报告：

```text
训练集拟合误差
```

至少要有：

```text
train RMSE
validation RMSE
per-joint validation RMSE
max error
W rank
singular values / condition information
```

最终评价重点：

```text
独立轨迹上的 torque prediction
```

---

# 16. 提交说明协议

Codex 每次任务完成后输出：

```text
Modified files
Why each file changed
What was intentionally not changed
Build/test commands
Observed numerical changes
Remaining risks
Next minimal step
```

---

# 17. 项目负责人掌握度原则

如果一次修改后出现：

```text
项目负责人无法在 5~10 分钟内解释新增架构
```

默认说明抽象程度过高。

应该优先：

```text
简化实现
补文档
拆分任务
```

而不是继续增加层次。

---

# 18. 当前阶段禁止事项

在 P1 baseline 完成前禁止：

```text
大规模 factory 架构
plugin system
统一所有 robot model code
大改 regressor 数学
直接接 reBot 真机
删除 Panda/Piper legacy implementation
为了 clean code 进行全仓 rename
```

---

# 19. AI 工作模式

默认让 Codex：

```text
先当代码审计员
再当实验工程师
最后才当重构工程师
```

优先级：

```text
理解正确
>
实验可信
>
结果可复现
>
代码简洁
>
架构通用
```
