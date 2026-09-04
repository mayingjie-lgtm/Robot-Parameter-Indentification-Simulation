# reBot 实机接入与辨识操作手册

> 当前可执行到：`state_only`、`servo_hold`。
>
> 当前不可执行：小幅轨迹、Fourier 激励、真机数据预处理、真机参数辨识。
>
> 任一步不通过，立即停止，不跳过门禁。

## 1. 现场准备

连接关系：

```text
上位机 ── Ethernet ── Orin ── USB2CANFD ── reBot J1-J6
                                           └── 独立急停/断电
```

上电前确认：

- 机械臂固定可靠，首次测试时架空，运动范围内无人和障碍物；
- 独立急停或断电装置可用，并由一人全程值守；
- J1-J6 接线、电机 ID、反馈 ID 和供电符合设备资料；
- DMTool、`damiao_joint_angle_monitor`、只读检查程序和 lower service 不同时占用 USB2CANFD；
- 不使用 `--auto-enable`；
- 当前 SDK 的关节限位、碰撞、速度、力矩、温度保护仍有未标定项，禁止直接执行激励。

软件 `stop`、位置保持和失能不等于硬件急停、STO 或机械制动。

## 2. 统一现场变量

以下命令均在仓库根目录执行。按现场修改这一处即可：

```bash
export UPPER_IP=192.168.50.10
export ORIN_IP=192.168.50.20
export TCP_PORT=5000
export UDP_PORT=5001
export SDK_ROOT=/path/to/wlsea_arm_sdk
export RUN_ID=$(date +%Y%m%d_%H%M%S)
export RUN_DIR="$PWD/data/rebot_real/$RUN_ID"
mkdir -p "$RUN_DIR"
```

要求：

- `SDK_ROOT/upper/python/wlsea_arm_sdk/client.py` 必须存在；
- Orin 必须使用包含厂商 `dmcan.h` 和 `libdm_device.so` 的完整 SDK；
- 当前本机 handoff 包不包含这两个厂商文件，不能代替 Orin 真机运行环境；
- 每次正式验收使用新的 `RUN_DIR`，不得覆盖旧数据。

## 3. 网络与 lower service

### 3.1 上位机检查

```bash
ip -brief address
ip route get "$ORIN_IP"
ping -c 3 "$ORIN_IP"
```

预期：上位机网口地址为 `${UPPER_IP}/24`，能访问 `${ORIN_IP}`。

### 3.2 Orin 检查

在 Orin 执行：

```bash
ip -brief address
systemctl status wlsea-arm.service --no-pager
journalctl -u wlsea-arm.service -n 100 --no-pager
ss -ltnp | grep ':5000'
lsusb | grep -i '34b7:6877'
```

预期：

- Orin 地址为 `${ORIN_IP}/24`；
- `wlsea-arm.service` 正常，TCP `5000` 正在监听；
- USB2CANFD `34b7:6877` 可见；
- 日志没有 USB、CAN、配置或锁存故障。

lower 的 UDP 目标必须等于实际上位机地址。SDK 主文档使用
`192.168.50.10`，当前服务脚本默认值可能是 `192.168.50.30`。在 Orin 用 systemd
drop-in 显式设置，不要依赖默认值：

```bash
sudo systemctl edit wlsea-arm.service
```

填入现场值：

```ini
[Service]
Environment=WLSEA_UDP_TARGET_IP=192.168.50.10
Environment=WLSEA_UDP_TARGET_PORT=5001
```

然后执行：

```bash
sudo systemctl daemon-reload
sudo systemctl restart wlsea-arm.service
systemctl show wlsea-arm.service -p Environment
systemctl status wlsea-arm.service --no-pager
```

若上位机不是 `192.168.50.10`，将 drop-in 中的地址改成实际 `UPPER_IP`。

## 4. 软件基线

### 4.1 单元测试

```bash
python3 -m pytest tests/rebot_real -q
```

当前基线：`48 passed`。失败时不连接实机。

### 4.2 Mock smoke

```bash
python3 scripts/run_rebot_hardware.py \
  --config config/rebot_real_experiment.yaml \
  --mock \
  --output "$RUN_DIR/mock_state_only.csv"
```

必须看到：

```text
backend=rebot_sdk_mock
control_mode=state_only
schema_version=rebot_hardware_experiment_v1
mock_only=true; no real hardware was contacted
```

## 5. 第一阶段：只读状态

### 5.1 优先尝试 UDP-only 采集

该路径不创建 TCP session，不发送电机命令：

```bash
python3 scripts/capture_rebot_state.py \
  --sdk-root "$SDK_ROOT" \
  --bind-ip 0.0.0.0 \
  --udp-port "$UDP_PORT" \
  --duration 10 \
  --output "$RUN_DIR/udp_state.csv"
```

分析数据：

```bash
python3 scripts/analyze_rebot_state_capture.py \
  --csv "$RUN_DIR/udp_state.csv" \
  --output "$RUN_DIR/udp_state.summary.yaml"
```

PASS 条件：

- `sample_count > 0`，采样率接近 lower 配置；
- host 时间严格递增，lower 时间不递减；
- J1-J6 `feedback_invalid_count = 0`；
- 有效通道 `non_finite_count = 0`；
- 反馈年龄稳定且不过期；
- 原始 CSV 中所有 `primary_fault_code = 0`。

`udp_sequence_gap` 是跳过的 lower controller state 数，不等于 UDP 丢包数。

若脚本返回码为 `2` 且样本数为零，停止该路径。当前 lower 可能只在 TCP client
存在时发布 UDP；不得临时建立不受控 TCP 会话绕过。

### 5.2 必要时使用受控 `state_only`

先复制安全配置，不直接修改仓库默认文件：

```bash
cp config/rebot_real_experiment.yaml "$RUN_DIR/state_only.yaml"
```

编辑 `$RUN_DIR/state_only.yaml`，仅设置：

```yaml
sdk_root: "/path/to/wlsea_arm_sdk"
host: "192.168.50.20"
tcp_port: 5000
udp_port: 5001
control_mode: state_only
duration_s: 10.0
max_samples: null
allow_hardware: true
allow_motion: false
joint_mapping_verified: false
j1_convention: UNRESOLVED
```

映射未确认时，`joint_direction` 和 `joint_offset_rad` 保持原样。观测用位置范围必须
与当前部署 SDK 一致；若 SDK 的 J1 仍为 `[0, 2*pi]`，仅在这份观测配置中使用：

```yaml
joint_position_min_rad: [0.0, -3.14, -3.14, -1.87, -1.57, -3.14]
joint_position_max_rad: [6.283185307180, 0.0, 0.0, 1.57, 1.57, 3.14]
```

这只是允许观察 SDK 坐标，不代表 J1 映射已通过。

执行：

```bash
python3 scripts/run_rebot_hardware.py \
  --config "$RUN_DIR/state_only.yaml" \
  --sdk-root "$SDK_ROOT" \
  --host "$ORIN_IP" \
  --tcp-port "$TCP_PORT" \
  --udp-port "$UDP_PORT" \
  --output "$RUN_DIR/state_only.csv"
```

PASS 条件：正常结束、样本数大于零、CSV/metadata 已生成、无反馈过期和活动故障。

注意：该模式不会由上位机调用 `enable`、`enter_servo` 或 `servo_joint`，但会创建
TCP session；当前 lower 在 TCP 断开时会执行 `disable()`。

## 6. 第二阶段：关节映射与 J1

保持电机失能。只有设备允许失能拖动时，才逐轴小角度人工移动；否则按硬件厂商的
低风险标定流程执行。一次只动一个关节，并记录：

| 关节 | SDK 字段 | 实物关节 | SDK 增大时实物方向 | 零位依据 | 实测范围 | 结论 |
|---|---|---|---|---|---|---|
| J1 | `joint_1` |  |  |  |  |  |
| J2 | `joint_2` |  |  |  |  |  |
| J3 | `joint_3` |  |  |  |  |  |
| J4 | `joint_4` |  |  |  |  |  |
| J5 | `joint_5` |  |  |  |  |  |
| J6 | `joint_6` |  |  |  |  |  |

J1 必须依据机械零位标记、标定记录或硬件负责人确认。禁止：

```text
wrap_to_pi
q += 2*pi
q -= 2*pi
根据 URDF/MJCF 猜测
为了通过门禁扩大软限位
```

核验完成后，才在新的运行配置中填写：

```yaml
joint_direction: [<六轴 ±1>]
joint_offset_rad: [<六轴偏置>]
joint_position_min_rad: [<六轴实机限位>]
joint_position_max_rad: [<六轴实机限位>]
joint_mapping_verified: true
j1_convention: <明确且可追溯的约定名称>
```

同时保存日期、操作者、机械零位依据、SDK commit、`damiao_motors.csv` 和
`safety.json` 版本。任一轴不确定时保持 `joint_mapping_verified: false`。

## 7. 第三阶段：Servo 当前位置保持

仅在以下条件全部满足后执行：

- 六轴映射和 J1 约定已签字确认；
- 急停、断电、工作区和机械限位已检查；
- 六轴反馈有效、无活动故障、反馈年龄不超过阈值；
- lower 电机配置、方向、零偏和软限位与验收记录一致；
- 操作者站在机械臂运动范围外，另一人值守急停。

复制上一阶段已核验的配置为 `$RUN_DIR/servo_hold.yaml`，保持：

```yaml
control_mode: servo_hold
control_rate_hz: 100.0
duration_s: 0.1
max_samples: 5
allow_hardware: true
allow_motion: true
joint_mapping_verified: true
j1_convention: <已核验约定>
maximum_command_velocity_rad_s: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
maximum_feedback_age_ms: 50.0
```

执行：

```bash
python3 scripts/run_rebot_hardware.py \
  --config "$RUN_DIR/servo_hold.yaml" \
  --sdk-root "$SDK_ROOT" \
  --host "$ORIN_IP" \
  --tcp-port "$TCP_PORT" \
  --udp-port "$UDP_PORT" \
  --output "$RUN_DIR/servo_hold.csv"
```

正常生命周期：

```text
connect -> fresh state -> enable -> fresh q -> enter Servo
        -> hold fresh q -> exit Servo -> disable -> close
```

PASS 条件：

- 进入 Servo 时无可见位置跳变；
- CSV 中 `command_valid=1`，`q_cmd` 等于 Servo 前重新读取的当前位置；
- `servo_active=true`，无反馈过期和活动故障；
- 程序正常退出并失能；
- `servo_hold.csv` 和 `servo_hold.meta.yaml` 完整。

发生异常运动时直接使用物理急停/断电。不要把 `Ctrl+C`、关闭终端或软件 `stop`
当作急停。未通过时不得继续小幅运动或激励。

## 8. 第四阶段：小幅运动与完整辨识

### 8.1 当前停止点

当前仓库没有可运行的小幅运动模式。`control_mode: excitation` 会在创建 hardware
session 前报错：

```text
trajectory source integration remains pending
```

不要用 `MoveJ`、手写 Python Fourier 或仿真 `run_experiment` 绕过该限制。

### 8.2 继续前必须补齐

1. 将 C++ `FourierTrajectory` 已接受的系数通过 replay/provider 接入真机
   `q_target`，保留系数文件和 hash，不复制第二套 Fourier 数学；
2. 先完成单关节、极小幅、低速运动验收，再做六轴小幅轨迹；
3. 生成独立轨迹 A（训练）和 B（验证），分别保存配置、系数、hash 和 metadata；
4. 实现真机专用离线预处理：时间对齐、滤波、重采样和 `qdd` 估计；
5. 独立核验 `effort_reported` 的来源、方向、比例、单位和物理标定；
6. 为真机 schema 增加明确的数据转换/加载配置和样本质量门禁。

`effort_reported` 是 DM 反馈力矩估计，不是已独立标定的真实关节力矩。标定通过前，
不得将其改名为 `tau_measured`，也不得宣称得到可信实机物理参数。

仿真配置 `config/rebot_dm_clean_identification.yaml` 不能直接用于真机，因为：

- 真机没有 `qdd_mujoco`；
- `effort_reported` 未证明等于仿真的 `tau_effort`；
- `saturated_sliding` 依赖 MuJoCo truth，真机没有该 oracle。

### 8.3 正式辨识顺序

```text
静态采集
  -> 单关节小幅运动
  -> 六轴低速小幅轨迹
  -> 独立轨迹 A
  -> 独立轨迹 B
  -> 真机离线预处理
  -> 用 A 建立基础参数空间并拟合
  -> 固定 A 的缩放、基础方向和参数
  -> 只在 B 上预测力矩
  -> 检查 per-joint RMSE、max error、rank、singular values、condition
```

第一版先建立无 ridge OLS baseline；鲁棒算法不能替代数据语义、映射、力矩标定和
独立 B 验证。

## 9. 故障停止表

| 现象 | 立即动作 | 禁止操作 |
|---|---|---|
| 无反馈或 state timeout | 保持/确认失能，检查 UDP 目标、端口和 lower 日志 | 不使能、不运动 |
| 反馈年龄超限 | 停止实验，检查网络、lower 和 CAN 反馈 | 不放宽阈值掩盖问题 |
| `primary_fault_code != 0` | 保持失能，按 SDK fault catalog 恢复并复位 | 不靠重连清故障 |
| `protective_stop/fault_latched/emergency_stop` | 物理确认安全，排除根因 | 不继续当前实验 |
| J1 越界或方向不符 | 停止，重新核验零位和映射 | 不 wrap、不扩大限位 |
| `servo_active=false` | 停止并确认失能 | 不发送 Servo target |
| 通信中断 | 使用物理急停/断电确认安全，检查 cleanup 日志 | 不把软件 stop 当急停 |
| USB2CANFD 断开 | 断电检查连接，处理锁存故障 `200205` | 不反复热插拔后直接运动 |
| 任何异常运动/异响 | 立即物理急停或断电 | 不等待软件自行恢复 |

## 10. 每次验收必须保存

```text
git commit
SDK commit/version
Orin/上位机 IP 和端口
damiao_motors.csv
safety.json
运行 YAML
CSV + meta.yaml + analyzer summary
操作者、日期、机械臂安装和负载
关节映射/J1 验收记录
急停测试记录
```

缺少上述记录的数据只能作为 smoke，不得作为正式参数辨识结果。
