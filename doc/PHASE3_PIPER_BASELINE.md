# Phase 3 Piper 仿真辨识可信基线

> 基线日期：2026-08-24  
> 模型：完整 `piper/piper.xml` + `piper/scene_identification.xml`  
> 控制/辨识自由度：`joint1..joint6`

## 1. 固定动力学范围

- `joint7=0.035 m`、`joint8=-0.035 m`。
- 两个手指的质量、一阶矩和惯量变换到 link6 body-local 坐标系后合入 link6。
- 无摩擦线性参数顺序为 72 维：六个 10 维刚体块、六个 armature、六个 damping。
- equality 使用稳定的软约束参数；实测最大手指位置误差为 `6.84213e-7 m`。原计划的 `1e-9 m` 会要求过硬约束并导致 1 ms 步长下的数值不稳定，因此稳定门禁采用 `1e-6 m`。
- 安全姿态下六个机械臂 DOF 的 `qfrc_constraint=0`，接触数为零。

## 2. 仿真数据契约

仿真 recorder 记录同一积分区间：

```text
time_begin: q, qd, qdd_mujoco=qacc, tau_cmd,
            tau_effort=qfrc_actuator, tau_constraint=qfrc_constraint
time_end:   q_next, qd_next
qdd_diff = (qd_next - qd) / (time_end - time_begin)
```

所有浮点列使用 17 位有效数字；饱和样本不再静默丢弃，而由 `saturated` 标记。数据旁的 `.meta.yaml` 记录模型、步长、轨迹和各列来源。

轨迹 A 的数据语义诊断：

- 六关节 `tau_cmd-tau_effort` 的 RMSE 和最大误差均为 `0 Nm`；
- `qdd_diff-qdd_mujoco` 的 RMSE 依关节约为 `1.67e-4` 至 `1.77e-3 rad/s²`；
- 最大瞬时差异为 `8.49e-2 rad/s²`，说明速度前向差分仍不是 MuJoCo `qacc` 的同义词。

仿真辨识固定使用 `qdd_mujoco` 和 `tau_effort`。

## 3. 回归矩阵一致性

`regressor_test` 不依赖外部 CSV，直接比较 MuJoCo：

```text
tau_mujoco_inverse
tau_regressor = Y * theta_model
tau_simulation (inverse torque fed through actuator, then mj_forward)
```

结果：

- gravity、inertia + armature、Coriolis + damping 和 100 个固定 seed 随机状态，最大力矩误差约 `1e-15 Nm`；
- forward acceleration closure 约 `1e-15 rad/s²`；
- 256/512/1024 随机状态在相对阈值 `1e-6` 下数值秩稳定为 47；
- 72 列中有 10 个结构零列；相对阈值 `1e-8` 会保留一个约 `1e-6` 量级的弱方向而得到 rank 48，因此正式基础空间固定记录阈值而不把“72”当作可辨识维度。

惯量从 `body_iquat` 主惯量坐标旋转到 body-local；armature 列为 `qdd_i`；damping 所需 actuator compensation 列为 `+qd_i`。

## 4. 可复现独立轨迹 A/B

固定参数：

```text
harmonics = 5
period = duration = 30 s
coefficient scale = 0.04
trajectory_A seed = 20260824
trajectory_B seed = 20260825
```

0.15 的初始尺度曾使 A 在闭环跟踪时产生接触；0.04 下两条轨迹均在第一次安全搜索尝试通过，30,000 个样本中饱和、接触和非摩擦机械臂约束力均为零。

辨识 scene 使用专用 `identification_home=[0,0.8,-0.8,0,0,0,0.035,-0.035]`，与 Fourier `q0` 完全一致；完整 `piper.xml` 的通用 `home` 未修改。这消除了启动追踪瞬态，并使摩擦数据所有样本都通过预期 `qfrc_constraint=-Fc*sign(qd)` 语义门禁。

同 seed 独立运行的系数文件和完整 CSV 字节一致。当前 A/B 数据 SHA-256：

```text
A 80143b21e1915e3ce1bae1986cf45e437147f035175bc5a19e5243125d2428d2
B 0adf7a820d47e0fd7666a614ddac9d3b83ca2e3e7ae61370c5b5a8f6d1d7ddaf
```

系数 CSV 的 SHA-256（也写入每个 dataset metadata）：

```text
A 358693ba5398cd8acf2b4e79b6ec8cde3f3aa344ca277e7522cd4ad6193eb9bb
B 24ce77066140897366bb937993822e4a5fec69856f84029a9e12811898106392
```

## 5. 干净 OLS 闭环

OLS 使用干净 A 的列范数缩放和 SVD 基础空间，不添加 ridge。B 从未参与参数估计。

```text
rank                         47 / 72
effective condition          85.4800485
base-parameter relative err  4.0581e-7
B aggregate RMSE             5.6238e-8 Nm
B worst joint RMSE           9.3206e-8 Nm
B worst max error            3.3449e-7 Nm
```

六关节均通过 `RMSE <= 1e-6 Nm` 和 `max <= 1e-5 Nm`。

## 6. 噪声、异常点与 IRLS

所有噪声实验只污染 A，并复用干净 A 的缩放和 47 维基础方向；B 始终干净。

| 实验 | OLS B aggregate RMSE | IRLS B aggregate RMSE | IRLS/OLS |
|---|---:|---:|---:|
| clean | `5.6238e-8` | `5.6238e-8` | `1.000` |
| Gaussian torque, `sigma=0.01 Nm` | `1.2856e-3` | `1.2799e-3` | `0.996` |
| 1% sample-joint `±1 Nm` outlier | `6.0424e-3` | `3.2594e-3` | `0.539` |

异常点实验中 IRLS 改善 `46.1%`。最终迭代为 8 次，降权比例 `18.13%`，最小权重 `0.01354`；改善来自 Huber 对异常观测降权，而不是增加了新的可辨识信息。clean 上 IRLS 不改变结果。

## 7. 显式库仑摩擦

只有在上述闭环通过后，`piper_friction_sim_node.yaml` 才把以下真值写入并校验到 `model->dof_frictionloss`：

```text
[0.20, 0.20, 0.15, 0.10, 0.08, 0.05] Nm
```

MuJoCo `frictionloss` 是约束求解器中的干摩擦，因此它会出现在 `qfrc_constraint`。摩擦质量门禁验证其为 `-Fc*sign(qd)`，而不是错误要求它为零。只在对应关节 `|qd| >= 0.05 rad/s` 的 observation row 上验证。

```text
rank                              53 / 78
effective condition               98.8565711
Y*theta vs simulation max error   < 3.0e-7 Nm on B
frictionloss relative error        1.4618e-7
B worst joint RMSE                 9.4802e-8 Nm
```

随机状态的直接 MuJoCo inverse/forward 摩擦测试误差为 `3.55e-15 Nm`。现有 tanh `NLS_FRICTION` 没有用于此门禁。

## 8. 为什么旧误差不能证明参数正确

旧 `0.529 Nm` 来自错误或不完整的手写动力学基准以及混合采样语义；旧 `0.0087 Nm` 是同一轨迹时间切分、秩亏回归空间中的 hold-out 拟合误差。二者比较的不是同一个可信物理闭环，小残差也可能来自同轨迹相关性、秩亏参数补偿或错误数据源，因此都不能单独证明参数正确。

本基线改用：MuJoCo 单状态 inverse/forward 身份验证、独立 A/B、固定基础参数坐标和逐关节 B 预测。

## 9. 复现命令

```bash
cmake -S . -B build
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure

./build/run_experiment --headless --trajectory-seed 20260824 \
  --output data/phase3/piper_trajectory_A.csv
./build/run_experiment --headless --trajectory-seed 20260825 \
  --output data/phase3/piper_trajectory_B.csv
./build/identify --config config/identification.yaml
./build/identify --config config/identification_friction.yaml
python3 scripts/verify_phase3_gates.py
```

`data/` 和 `results/` 由 `.gitignore` 排除；上述命令和固定配置用于重建产物。
