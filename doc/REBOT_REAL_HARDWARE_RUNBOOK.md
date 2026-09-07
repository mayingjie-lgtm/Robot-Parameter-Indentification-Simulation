# reBot 实机接入与辨识操作手册

> 本文同时给出 **2026-09-07 当前已可执行能力** 与 **reBot 真机系统辨识完整目标流程**。
>
> 当前仓库可直接执行到：`state_only`、`servo_hold`；小幅轨迹、Fourier 激励、真机数据预处理和真机参数辨识仍有明确软件/标定门禁，文中统一标记为 `PENDING`，不得把目标步骤误认为已经实现。
>
> 任一步不通过，立即停止，不跳过门禁。仿真数值门限、simulation truth 和 MuJoCo-only 字段不得直接搬到真机。

## 0. 2026-09-07 当前现场快路径

本节记录当前这台 old-arm 的实际部署，优先于下文的示例 IP 和占位值。它只授权
`state_only` 和一次 `0.1 s / 最多 5 样本` 的 Servo 当前位置保持 smoke，不表示真机
参数辨识已经具备条件。

```text
上位机 IP             192.168.50.11
Orin IP               192.168.50.24
TCP / UDP             5000 / 5001
本机 SDK_ROOT          /home/j/j_ws/src/wlsea_rebot_b601_upper_20260904
Orin lower service    wlsea-arm.service
Orin motor config     /home/nvidia/wlsea_arm_sdk/config/damiao_motors_old_arm.csv
motor config SHA-256  563afebb845c5b9691e03eae7c7d7f4ed338ed0ed37f5eb41a4bce48dff902ce
safety.json SHA-256   c105404b7b9ab028803624e4c02cea1d15448a3c208fe43aab50b42a4ba89c50
canonical URDF        rebot_dm/rebot_dm.urdf
canonical URDF SHA-256 97c8c5a23a637dea894cfa731dd6d2676af89dd91001ff73f2bc1ffa403955a5
```

当前已确认：

- `tests/rebot_real`：`50 passed`；
- lower 正常监听 `0.0.0.0:5000`，UDP 目标为 `192.168.50.11:5001`；
- USB2CANFD 为 `34b7:6877`，CAN TX 已观察到 `ack=1`；
- 10 秒受控 `state_only` 已通过；
- J1 从顶部看顺时针移动时 SDK 值减小；J2 向上抬时 SDK 值减小；
- J1 采用 `PHYSICAL_MARK_PI_CENTERED_VISUAL_20260907`：
  `q_model_J1 = q_sdk_J1 - pi`；
- J1 机械刻线为目测对齐，双向接近读数相差约 `1.85784 deg`，只允许用于 Servo
  当前位保持 smoke，不得声称是辨识级零位标定；
- disabled 六轴反馈约每 100 ms 刷新一次，因此使能前使用 110 ms 门限；使能后和
  Servo 期间仍使用 50 ms 门限；
- canonical URDF 与 SDK 组合 URDF 的 J6 origin x 相差 `0.004316 m`，真机辨识前必须
  解决；
- `effort_reported` 仍未独立标定，不能视为可信 `tau_measured`。

本次已有记录位于：

```text
data/rebot_real/20260907_100125/joint_mapping_acceptance.yaml
data/rebot_real/20260907_100125/j1_zero_aligned*.csv
data/rebot_real/20260907_100125/state_only_retry2.csv
```

当前 `scripts/analyze_rebot_state_capture.py` 只接受 UDP-only state schema，不能直接分析
`run_rebot_hardware.py` 生成的 experiment schema。不要把该 schema 错误误判为硬件失败；
在 analyzer 支持 experiment schema 前，Servo hold 只能记为 smoke，不能记为正式辨识
数据验收。

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
export UPPER_IP=192.168.50.11
export ORIN_IP=192.168.50.24
export TCP_PORT=5000
export UDP_PORT=5001
export SDK_ROOT=/home/j/j_ws/src/wlsea_rebot_b601_upper_20260904
export RUN_ID=$(date +%Y%m%d_%H%M%S)
export RUN_DIR="$PWD/data/rebot_real/$RUN_ID"
mkdir -p "$RUN_DIR"
```

要求：

- `SDK_ROOT/upper/python/wlsea_arm_sdk/client.py` 或
  `SDK_ROOT/src/wlsea_arm_sdk/client.py` 必须存在；当前 handoff 使用后者；
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

lower 的 UDP 目标必须等于实际上位机地址。当前现场上位机是 `192.168.50.11`，
服务脚本或历史文档中的默认值可能不同。在 Orin 用 systemd drop-in 显式设置，
不要依赖默认值：

```bash
sudo systemctl edit wlsea-arm.service
```

填入现场值：

```ini
[Service]
Environment=WLSEA_UDP_TARGET_IP=192.168.50.11
Environment=WLSEA_UDP_TARGET_PORT=5001
```

然后执行：

```bash
sudo systemctl daemon-reload
sudo systemctl restart wlsea-arm.service
systemctl show wlsea-arm.service -p Environment
systemctl status wlsea-arm.service --no-pager
```

若上位机不是 `192.168.50.11`，将 drop-in 中的地址改成实际 `UPPER_IP`。

## 4. 软件基线

### 4.1 单元测试

```bash
python3 -m pytest tests/rebot_real -q
```

当前基线：`50 passed`。失败时不连接实机。

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
sdk_root: "/home/j/j_ws/src/wlsea_rebot_b601_upper_20260904"
host: "192.168.50.24"
tcp_port: 5000
udp_port: 5001
control_mode: state_only
duration_s: 10.0
max_samples: null
allow_hardware: true
allow_motion: false
joint_mapping_verified: false
j1_convention: UNRESOLVED
maximum_disabled_feedback_age_ms: 110.0
maximum_feedback_age_ms: 50.0
```

映射未确认时，`joint_direction` 和 `joint_offset_rad` 保持原样。观测用位置范围必须
与当前部署 SDK 一致；若 SDK 的 J1 仍为 `[0, 2*pi]`，仅在这份观测配置中使用：

```yaml
joint_position_min_rad: [0.0, -3.174906585, -3.174906585, -1.904906585, -1.604906585, -3.174906585]
joint_position_max_rad: [6.283185307180, 0.087266463, 0.087266463, 1.604906585, 1.604906585, 3.174906585]
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

### 6.1 当前 old-arm 的 Servo-hold-only 映射

当前操作者接受以下映射仅用于当前位置保持 smoke：

```yaml
joint_direction: [1, 1, 1, 1, 1, 1]
joint_offset_rad: [-3.141592653589793, 0, 0, 0, 0, 0]
joint_mapping_verified: true
j1_convention: PHYSICAL_MARK_PI_CENTERED_VISUAL_20260907
```

这不是辨识级映射签字。J1 目测零位不确定性、J3/J4 人工移动串扰、J6 几何差异和
力矩语义未解决前，`identification_ready` 必须保持 `false`。

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
joint_direction: [<六轴 ±1>]
joint_offset_rad: [<六轴偏置>]
joint_position_min_rad: [<Servo smoke 使用的实机软限位>]
joint_position_max_rad: [<Servo smoke 使用的实机软限位>]
maximum_command_velocity_rad_s: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
maximum_disabled_feedback_age_ms: 110.0
maximum_feedback_age_ms: 50.0
```

`maximum_disabled_feedback_age_ms` 只用于 `state_only` 和 Servo 调用 `enable()` 前的
失能状态检查。当前 lower 的失能反馈实测约每 100 ms 刷新一次，且驱动的六轴轮询
时序要求最低合法 timeout 为 60 ms，因此本机验收使用 110 ms。`enable()` 返回后立即
切换回 `maximum_feedback_age_ms: 50.0`；不得用 110 ms 放宽 Servo 运行期门禁。

### 7.1 当前 old-arm 的精确配置

当前运行目录已有：

```bash
export RUN_DIR="$PWD/data/rebot_real/20260907_100125"
```

执行前打开 `$RUN_DIR/servo_hold.yaml`，确认至少包含以下值：

```yaml
sdk_root: "/home/j/j_ws/src/wlsea_rebot_b601_upper_20260904"
host: "192.168.50.24"
tcp_port: 5000
udp_port: 5001

control_mode: servo_hold
control_rate_hz: 100.0
duration_s: 0.1
max_samples: 5
output_csv: data/rebot_real/20260907_100125/servo_hold.csv

allow_hardware: true
allow_motion: true
joint_mapping_verified: true
j1_convention: PHYSICAL_MARK_PI_CENTERED_VISUAL_20260907
joint_direction: [1, 1, 1, 1, 1, 1]
joint_offset_rad: [-3.141592653589793, 0, 0, 0, 0, 0]

# 这是 Jetson 当前部署的实机安全范围映射到 runner 坐标后的 Servo-smoke 门限，
# 不是 canonical 辨识范围。当前位置保持目标仍必须等于使能后重新读取的 q。
joint_position_min_rad: [-3.141592653590, -3.174906585, -3.174906585, -1.904906585, -1.604906585, -3.174906585]
joint_position_max_rad: [3.141592653590, 0.087266463, 0.087266463, 1.604906585, 1.604906585, 3.174906585]

maximum_command_velocity_rad_s: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
maximum_disabled_feedback_age_ms: 110.0
maximum_feedback_age_ms: 50.0
connect_timeout_s: 3.0
command_timeout_s: 3.0
state_timeout_s: 0.25
trajectory_source: pending_cxx_fourier_integration
trajectory_hash: null
```

这里采用的是已部署 old-arm 的实机软限位，所以 J2 在 `(0, 0.087266463] rad` 内不会
阻断“当前位置保持”检查。不得把这段正向裕量当作 canonical 模型有效范围，也不得
据此执行 MoveJ、轨迹或辨识。

### 7.2 操作者执行前检查

先在仓库根目录执行：

```bash
python3 -m pytest tests/rebot_real -q

ssh nvidia@192.168.50.24 \
  'systemctl is-active wlsea-arm.service; \
   ss -ltnp | grep ":5000"; \
   test ! -e /home/nvidia/wlsea_arm_sdk/log/external_fault_latch'
```

必须得到 `50 passed`、service 为 `active`、端口 5000 正在监听，并且最后一个
`test` 返回成功。紧接执行命令前再次口头确认：急停就绪、工作区清空、人员在机械臂
运动范围外。任一项不满足，不执行下一节。

建议另开一个终端只观察 lower 日志：

```bash
ssh nvidia@192.168.50.24 \
  'journalctl -u wlsea-arm.service -f --no-pager -l -o cat'
```

观察终端不得运行第二个 SDK client、DMTool 或其他占用 USB2CANFD/UDP 5001 的程序。

### 7.3 由操作者执行一次 Servo hold

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

这是本手册中唯一授权的真实电机使能命令。不要重复运行来“碰运气”通过门限。若程序
因使能后的 50 ms feedback-age 门禁退出，应确认已自动失能并停止，不得把门限改成
110 ms 后重试。

执行结束后立即检查：

```bash
ssh nvidia@192.168.50.24 \
  'systemctl status wlsea-arm.service --no-pager; \
   journalctl -u wlsea-arm.service -n 80 --no-pager -l -o cat; \
   if test -e /home/nvidia/wlsea_arm_sdk/log/external_fault_latch; then \
     echo "FAULT LATCH:"; cat /home/nvidia/wlsea_arm_sdk/log/external_fault_latch; \
   else echo "fault_latch_absent"; fi'
```

只有程序返回 0、机械臂无可见跳变、末态失能、无 fault latch，并且 CSV/metadata
完整时才记为 PASS。

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

# 11. reBot 真机系统辨识完整闭环

这一节是本手册的最终主流程。前 0～10 节解决“能否安全连接和保持”，本节开始解决
“怎样得到可以用于论文、控制和模型更新的真实机械臂动力学参数”。

当前项目的辨识模型为：

```text
tau =
Y_rigid(q, qd, qdd) * theta_rigid
+ diag(qdd) * theta_armature
+ diag(qd)  * theta_damping
+ sign(qd)  * theta_coulomb
```

当前 reBot regressor 的列结构为：

```text
60 rigid-body inertial columns
+ 6 armature columns
+ 6 viscous-damping columns
+ 6 Coulomb-friction columns
= 78 raw columns
```

仿真 Phase 5C 的 78 列矩阵在轨迹 A 上得到 52 维 base space。这个结果只说明
**当前模型结构和充分激励下的仿真数值秩**，真机不得把“rank 必须等于 52”写死成
通过条件；应重新计算奇异值并检查 rank 对阈值和预处理是否稳定。

正式真机闭环固定为：

```text
R0  固定基座、版本冻结和仿真回归
R1  六轴映射、机械零位和几何模型冻结
R2  effort_reported 力矩语义与独立标定
R3  可信轨迹 replay/provider 接入
R4  单关节极小幅 commissioning
R5  六轴低速 commissioning
R6  冻结独立 trajectory A / B
R7  采集 A/B 原始真机数据
R8  离线时间对齐、滤波、qdd 估计和质量筛选
R9  真机 identification loader / model semantics 验收
R10 仅用 A 建立 base space + OLS
R11 固定 A 的全部结果，只在 B 上独立预测
R12 物理一致性检查、参数回灌和控制效果验证
```

任何一步出现新的映射、模型、传感器或数据语义变化，都回到对应门禁重新验收，不能
在后续拟合中“调参数把错误吃掉”。

# 12. R0：实验对象、基座、负载和软件版本冻结

## 12.1 首轮辨识必须是固定基座

当前 `rebot_dm.urdf` 和 Pinocchio regressor 是固定基座六自由度模型。首轮正式辨识时：

- reBot 基座必须刚性固定；
- ROV/水下航行器不得运动；
- 若机械臂装在 ROV 上，先把 ROV 可靠固定到试验架；
- 不得一边摇摆/航行一边使用固定基座 regressor 辨识。

若未来要在移动 ROV 上辨识，需要额外引入基座六维运动、IMU/位姿和 floating-base
动力学；那是另一套模型，不属于当前 78 列固定基座问题。

## 12.2 固定末端和夹爪状态

每组 A/B 实验都必须记录：

```text
gripper position
tool / payload mass
payload COM
安装方式
是否带水下工具
是否带线缆拖曳
空气 / 水下环境
```

当前 Pinocchio wrapper 固定两个 gripper joint 为 `[0.05, 0.05] m`。如果实机夹爪状态
与此不一致，或者末端加了工具/负载，必须先更新辨识模型或把负载作为明确参数处理，
不能直接沿用仿真模型。

## 12.3 每次正式实验前冻结版本

在新的 `RUN_DIR` 中保存：

```bash
git rev-parse HEAD > "$RUN_DIR/identification_repo_commit.txt"
sha256sum rebot_dm/rebot_dm.urdf   > "$RUN_DIR/rebot_dm.urdf.sha256"

sha256sum "$SDK_ROOT/config/damiao_motors.csv"   "$SDK_ROOT/config/safety.json"   > "$RUN_DIR/sdk_runtime_config.sha256"
```

如果 Orin 实际使用的是 `damiao_motors_old_arm.csv`，保存 **Orin 实际文件**，不能只保存
本机 handoff 中的同名候选文件。

## 12.4 先复核仿真基线

正式真机辨识前至少重新运行：

```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure

python3 scripts/verify_rebot_excitation_data.py   --csv data/rebot_dm/excitation_A.csv

python3 scripts/verify_rebot_excitation_data.py   --csv data/rebot_dm/excitation_B.csv

python3 scripts/verify_rebot_clean_identification.py   --result results/rebot_dm_clean_identification.yaml
```

若基线产物不存在，按 Phase 5B/5C 文档先重新生成。真机实验不能建立在已经回归失败的
regressor 上。

# 13. R1：六轴映射、零位和几何模型必须先达到辨识级

Servo-hold-only 的

```yaml
joint_offset_rad: [-pi, 0, 0, 0, 0, 0]
j1_convention: PHYSICAL_MARK_PI_CENTERED_VISUAL_20260907
```

只能证明当前位置保持 smoke 可做，**不能**作为辨识级零位。

## 13.1 六轴逐轴确认

J1～J6 每一轴都需要独立完成：

1. SDK 字段与物理关节一一对应；
2. 正方向确认；
3. 机械零位确认；
4. 实际安全范围确认；
5. 双向接近同一参考位置，检查回差/零位重复性；
6. 保存原始状态 CSV、照片/工装记录和签字结论。

最终冻结：

```yaml
joint_direction: [...]
joint_offset_rad: [...]
joint_position_min_rad: [...]
joint_position_max_rad: [...]
joint_mapping_verified: true
j1_convention: <辨识级且可追溯的约定>
```

## 13.2 解决当前 J6 几何冲突

当前两个项目中的运动学源存在明确差异：

```text
identification canonical:
joint6 origin x = 0.023692 m

SDK combined URDF:
arm_joint6 origin x = 0.028008 m

difference = 0.004316 m
```

并且后续末端固定连接长度也不同。正式辨识前必须通过机械图纸、CAD 或实物测量确定
哪一个才是当前 old-arm 的真实几何源，并让：

```text
辨识 URDF
SDK/控制运动学模型
实物机械臂
```

在关节轴位置、轴方向和固定变换上保持一致。

**禁止**在这个冲突未解决时通过动力学参数拟合去补偿几何误差。几何错误会进入
`Y(q,qd,qdd)` 本身，不是一个合理的惯性参数误差。

## 13.3 R1 PASS 条件

只有以下全部完成才进入 R2：

- J1～J6 mapping 有实机证据；
- 六轴零位重复性已记录；
- J6 几何冲突已关闭；
- 固定夹爪/工具/负载模型已确认；
- Pinocchio FK 与实机若干基准姿态可核对；
- 新的模型和 mapping 已保存 hash。

# 14. R2：把 effort_reported 变成可用于辨识的力矩量

当前 raw state 中的：

```text
effort_reported
<- SDK JointState.torque_nm
<- DM feedback torque field
```

项目目前只确认它是映射后的 DM 力矩估计，尚未证明其绝对比例、零偏、方向、温漂和
“Nm”物理精度。因此正式参数辨识前必须做独立力矩标定。

## 14.1 优先标定方法

优先使用经过校准的外部扭矩传感器；若只能使用力传感器/称重传感器，则使用已测量的
力臂形成参考扭矩：

```text
tau_ref = r_perpendicular * F
```

每个关节至少覆盖：

- 正、反两个方向；
- 多个力矩幅值；
- 多次加载/卸载；
- 多个关节姿态；
- 静态和低速区分别记录。

不要用当前未知动力学模型输出的重力矩作为“真值”再去标定同一模型。若使用悬挂砝码，
应通过外部已知负载差分或专用工装尽量消除机械臂自身重力项的影响。

## 14.2 每轴拟合的最小标定模型

先检查是否可用：

```text
tau_ref_j = scale_j * effort_reported_j + bias_j
```

若正反方向明显不一致，需要显式记录 hysteresis/dead-zone，不要强行用一个直线比例。
最终至少保存：

```text
scale[6]
bias[6]
calibration residual
repeatability
reference sensor / load cell serial & calibration date
ambient condition
```

## 14.3 无独立力矩标定时的结论边界

如果当前只能获得 `effort_reported`：

- 可以继续做控制/算法 smoke 和“reported-effort prediction”研究；
- 可以比较重复性和相对模型改进；
- **不能**把拟合结果称为经过物理标定的真实惯性/摩擦参数；
- 论文中必须明确 torque source 是电机反馈估计而非独立扭矩传感器。

R2 未通过时不得把 `effort_reported` 改名为 `tau_measured`。

# 15. R3：接入可信 trajectory provider，禁止复制第二套 Fourier 数学

当前可信 Fourier 实现位于 C++ `ForceController/FourierTrajectory`。仿真已冻结的候选：

```text
A seed = 20260826
B seed = 20260829
duration = 30 s
harmonics = 5
q0 = [0, -1, -1, 0, 0, -0.6]
```

它们只能作为 **真机轨迹设计起点**，不能直接等价为已批准真机命令。仿真中的
`1000 Hz`、PD gains、力矩限位和 coefficient scale 均不是实机授权值。

## 15.1 正确的软件接法

推荐的数据流是：

```text
accepted C++ FourierTrajectory
        |
        +-> coefficient/replay artifact
        |      + hash
        |      + q_ref(t)
        |      + qd_ref(t)
        |      + qdd_ref(t)
        |
        v
reBot hardware trajectory provider
        |
        v
RebotHardwareRunner
        |
        v
position-only ArmClient.servo_joint(q_target)
```

不要在 Python runner 中重新实现一套 Fourier 方程。必须让仿真和真机能够证明使用的是
同一份冻结系数/replay artifact。

## 15.2 当前必须完成的软件门禁

在允许 `control_mode: excitation` 前，代码需要做到：

- 读取冻结 trajectory artifact；
- 校验机器人、DOF、初始位、采样率、duration、hash；
- 起点必须与实机 fresh measured q 连续；
- 每个目标做 position limit 检查；
- 每周期做 velocity-derived delta 检查；
- 加入 acceleration/jerk 上位机门禁，不能只依赖 lower；
- 记录每个发送的 `q_cmd`、command timestamp、sequence；
- feedback stale / fault / servo ownership 立即 fail closed；
- 正常退出和异常退出都执行明确的 Servo cleanup。

当前 runner 对 `excitation` 明确报：

```text
excitation trajectory source integration remains pending
```

在上述实现完成前，不得用 MoveJ、临时脚本或手写 Fourier 绕过。

# 16. R4～R6：从 commissioning 到冻结 A/B

正式 A/B 前建议增加一条 **trajectory C / commissioning trajectory**。C 只用于调试安全、
采样率、跟踪和预处理，不能作为最后独立 B 验证。

## 16.1 R4：单关节极小幅

一次只激励一个关节，其余关节保持当前位置。顺序建议 J6 → J5 → J4 → J3 → J2 → J1，
但现场可按机械安全性调整。

每一轴先做：

```text
极小幅
低速度
低加速度
短时间
正反方向
```

逐级扩大前检查：

- 无可见跳变/异响；
- 无 fault latch；
- feedback 全有效；
- `effort_reported` 符号与 R2 标定一致；
- q_cmd 与 q 的方向一致；
- 位置、速度、加速度、跟踪误差有足够余量；
- command/feedback 时间序列连续。

任何一轴不通过就停在单关节阶段。

## 16.2 R5：六轴低速 commissioning

六轴组合前先使用显著低于仿真幅值的 replay scale，并逐级增加。每一级必须重新检查：

```text
q limits
qd limits
estimated qdd
tracking error
feedback age
UDP/TCP health
primary_fault_code
servo_active
effort range
机械自碰/环境碰撞
```

当前 production `safety.json` 中 torque monitor、measured velocity monitor、collision、
temperature numeric thresholds 等仍有未标定/禁用项，所以“lower 没报警”不能替代上位机
实验验收。

## 16.3 控制频率不能照搬仿真 1000 Hz

当前硬件 runner 为 100 Hz 候选，SDK 开发 ServoJ 有 200 Hz 使用路径，lower 内部还有自己的
高频控制。正式轨迹频率必须根据实测：

- upper command acceptance；
- UDP feedback rate；
- jitter；
- feedback age；
- Servo watchdog；

选择并冻结。不能因为仿真是 1000 Hz 就要求 Python 上位机也发送 1000 Hz。

## 16.4 R6：冻结 A/B

当 C 通过后再生成/接受正式 A 与 B：

```text
A = identification/training only
B = final independent validation only
A != B
```

每条轨迹必须保存：

```text
coefficient/replay file
SHA-256
start q
duration
hardware command rate
position/velocity/acceleration/jerk envelope
generation source commit
accepted scale
审核记录
```

B 不得用于：

- 选择滤波截止频率；
- 选择 qdd 方法；
- 调 rank threshold；
- 调摩擦速度阈值；
- 选择 OLS/IRLS；
- 修改模型结构。

这些选择应在 C 和 A 上冻结后，再只跑一次正式 B 评价。

# 17. R7：正式 A/B 原始数据采集

## 17.1 每条轨迹建议的采集结构

每次运行保存成独立目录：

```text
A_run01/
  config.yaml
  trajectory.csv
  trajectory.sha256
  raw.csv
  raw.meta.yaml
  lower.log
  versions.txt
  operator_notes.md

A_run02/
A_run03/
B_run01/
...
```

至少做重复运行来观察参数稳定性。不要只保留“最好的一次”。

## 17.2 raw CSV 的权威含义

当前 `rebot_hardware_experiment_v1` 原始数据包含：

```text
timestamp_host_rx_ns
timestamp_lower_ns
timestamp_host_command_ns
servo_sequence
q0..q5
qd0..qd5
effort_reported0..5
q_cmd0..5
feedback_valid0..5
torque_valid0..5
feedback_age_ms0..5
robot_mode
safety_state
primary_fault_code
servo_active
servo_mode
command_valid
control_mode
```

明确没有：

```text
online qdd
tau_cmd
motor current
hardware timestamp
raw encoder count
```

因此 raw 文件永远保留，不在 recorder 内做滤波或伪造缺失信号。

## 17.3 时间戳使用原则

辨识状态的首选采样时基是：

```text
timestamp_host_rx_ns
```

因为 `q/qd/effort_reported` 来自同一个 decoded JointState snapshot，并且
`timestamp_host_rx_ns` 是 upper host 的接收时间。

`timestamp_lower_ns` 来自 lower 的 steady clock，不是电机硬件时间；在没有明确时钟同步
模型时，不要直接与 upper 的 command timestamp 相减得到“网络延迟”。

`timestamp_host_command_ns` 与 host receive time 在同一主机时钟域，可用于 q_cmd 跟踪分析，
但 q_cmd 不是辨识力矩。

# 18. R8：真机离线预处理

必须新增真机专用预处理产物，不能把 raw CSV 直接喂给当前仿真 clean-identification 配置。

推荐链路：

```text
raw hardware CSV
-> schema/metadata validation
-> time-window extraction
-> bad-row masking
-> uniform resampling
-> q / qd consistency inspection
-> zero-phase smoothing
-> qdd estimation
-> torque calibration
-> final hardware-identification CSV
```

## 18.1 先做质量 mask

任何满足下列条件的 observation 必须排除或按 joint-row 显式失效：

- `feedback_valid=false`；
- `torque_valid=false`；
- NaN/Inf；
- active primary fault；
- unsafe safety state；
- Servo 应激励时 `servo_active=false`；
- feedback age 超过冻结阈值；
- 时间戳回退/重复异常；
- 进入/退出 Servo 的启动与停止瞬态；
- 人工急停/碰撞/外力干扰。

不要为了增加样本数放宽这些条件。

## 18.2 重采样

先统计：

```text
median dt
P95/P99 dt
max dt
effective feedback rate
missing interval
```

再把有效区间重采样到冻结的均匀时间网格。重采样率不得高于原始反馈能够支持的有效带宽。

## 18.3 qd 与 qdd

SDK 的 `qd` 是反馈速度，但正式辨识前仍应做：

```text
qd_sdk
vs
d/dt(filtered q)
```

一致性对比。

`qdd` 只允许离线得到，推荐：

```text
qd_filtered = zero_phase_filter(qd)
qdd_est     = derivative(qd_filtered, t)
```

也可以对 q 做 Savitzky-Golay/样条后同时求一、二阶导，但必须保存：

- 方法；
- 阶数；
- window/cutoff；
- 边界处理；
- 重采样率；
- 参数敏感性结果。

不得使用 raw 一阶速度差分作为最终 qdd，只因为它“能跑”。

## 18.4 torque

若 R2 已完成：

```text
tau_calibrated_j =
scale_j * effort_reported_j + bias_j
```

并使用同一方向映射。若存在明确的温漂、死区或方向依赖，预处理模型也必须版本化。

滤波不能只处理 qdd 而完全不检查 torque 带宽；状态与力矩应采用相容的离线带宽，并用
A/C 做滤波敏感性分析。

## 18.5 近零速度摩擦

当前 78 列 dry-friction regressor 使用 `sign(qd)`。真机在近零速有静摩擦、死区和控制器
内部效应，不能使用仿真的 `saturated_sliding` oracle。

第一版真机应采用：

```text
moving observations only
abs(qd_j) >= v_min_j
```

其中 `v_min_j` 应根据 C/A 的速度噪声和摩擦数据确定。仿真 `0.05 rad/s` 可作为分析参考，
不能未经验证直接当成真机物理阈值。

# 19. R9：当前 identify 程序在真机前还必须修的接口语义

这是当前项目最关键的软件阻塞之一。

现有 `./build/identify` 是围绕仿真/Piper clean 数据契约建立的，当前仍存在以下
真机不兼容点：

1. 默认需要 `time_begin/q/qd/qdd_mujoco/tau_effort`；
2. 默认要求 `tau_constraint/saturated/contact_count` 这类仿真质量列；
3. training 的列名前缀可配置，但 validation/basis 当前重新使用默认列契约；
4. `saturated_sliding` 依赖已知 simulation `frictionloss`；
5. 是否启用 friction columns 当前与 `joint_frictionloss` 非空绑定；
6. 程序会计算 `theta_true` 和 simulation-truth 相关误差，而真机没有 ground truth；
7. `joint_armature/joint_damping/joint_frictionloss` 在当前 reBot 路径中带有
   simulation-truth 语义，不能把仿真数值原样填进真机配置。

因此正式真机辨识前应做一个 **最小 real-hardware identification mode**，而不是伪造
`qdd_mujoco`、`tau_constraint` 等列名去欺骗现有 loader。

## 19.1 真机 mode 最小要求

建议新增的真实数据列契约为：

```text
time
q0..q5
qd0..qd5
qdd_est0..qdd_est5
tau_calibrated0..tau_calibrated5
valid0..valid5 / row-quality fields
```

并做到：

- A/B/basis 使用同一个显式 `DataColumnSelection`；
- quality policy 不依赖 MuJoCo oracle；
- `enable_armature_columns`、`enable_damping_columns`、
  `enable_friction_columns` 与“已知真值”解耦；
- real mode 不要求 `theta_true`；
- real mode 不输出“raw parameter recovery error”；
- base space 只从 A 构建；
- B 永不进入 scale/SVD/rank decision/solver；
- 所有 preprocessing metadata 写入最终 result。

建议的未来配置名可以是：

```text
config/rebot_real_identification.yaml
```

建议的未来预处理工具可以是：

```text
scripts/preprocess_rebot_hardware_identification.py
```

**这两个名字是后续实现建议，当前仓库尚不存在，不得现在直接执行。**

# 20. R10：只用 A 做第一版 OLS

完成 R9 后，第一版正式算法固定为无 ridge OLS。

## 20.1 构造 observation matrix

对 A：

```text
W_A = W(q_A, qd_A, qdd_A)
T_A = tau_A
```

然后：

```text
column-norm scaling
-> SVD
-> numerical base space
-> W_A_base
-> beta_hat = OLS(W_A_base, T_A)
```

当前仿真使用：

```text
relative rank threshold = 1e-6 * sigma_max
```

真机可以以 `1e-6` 为起点，但必须在 A 上检查 `1e-5 / 1e-6 / 1e-7` 等邻近阈值下：

- rank 是否稳定；
- condition 是否稳定；
- beta/prediction 是否稳定。

B 不参与这个选择。

## 20.2 不要把 78 个 raw 参数逐项当成最终答案

当前模型结构天然存在不可辨识方向。最终应优先报告：

```text
base rank
singular values
effective condition number
base coordinates
A torque residual
parameter/run repeatability
```

而不是宣称 78 个 raw inertia/armature/damping/friction scalar 都被独立准确识别。

## 20.3 OLS 后再决定是否需要 IRLS

只有 OLS baseline 和数据质量报告完成后，才考虑 Huber IRLS 等鲁棒算法。

IRLS 不能用来掩盖：

- 零位错；
- 关节方向错；
- 几何模型错；
- torque scale 错；
- qdd 噪声；
- feedback 丢包；
- 外力/碰撞。

# 21. R11：独立 trajectory B 是最终可信门禁

A 完成后冻结：

```text
preprocessing settings
torque calibration
model file/hash
column scales
rank threshold
base directions
beta_hat
friction moving threshold
solver
```

然后才加载 B。

预测：

```text
tau_hat_B = W_B_base * beta_hat_A
```

至少输出：

- aggregate RMSE；
- 每关节 RMSE；
- MAE；
- bias；
- P95 absolute error；
- max absolute error；
- torque RMS-normalized error；
- R²；
- 残差与 q/qd/qdd 的相关性；
- 每关节有效 observation 数；
- A/B condition 和 rank diagnostic。

仿真 Phase 5C 的 `1e-5 Nm` 级阈值是 clean synthetic oracle 水平，**绝不能**作为真机
通过线。真机阈值必须结合 R2 力矩标定残差、反馈噪声、qdd 不确定性和最终控制需求制定。

如果 B 明显差于 A：

1. 先查映射、时序、滤波、qdd、torque calibration；
2. 再查 trajectory coverage；
3. 再查模型缺项，如柔性、减速器、Stribeck、温度或线缆拖曳；
4. 最后才考虑更复杂估计器。

禁止使用 B 反复调参直到“看起来好看”，否则 B 不再独立。

# 22. R12：参数物理一致性、回灌与控制验证

## 22.1 不直接把 minimum-norm raw vector 写回 URDF

rank deficient 时 minimum-norm raw inertial parameters 可能非物理。回灌前必须检查：

- mass > 0；
- COM 在合理机械范围；
- inertia matrix positive definite；
- principal inertia triangle inequalities；
- 相邻 link 参数没有明显异常；
- actuator/friction 参数符号和数量级合理。

如需得到完整可回灌参数，应使用物理一致性约束优化或从等价 base dynamics 中寻找
physically-consistent parameter set，并保存变换过程。

## 22.2 创建独立 identified model

不要覆盖 canonical baseline。建议：

```text
rebot_dm/identified/<RUN_ID>/rebot_dm_identified.urdf
results/rebot_real/<RUN_ID>/identification.yaml
results/rebot_real/<RUN_ID>/prediction_B.csv
results/rebot_real/<RUN_ID>/parameter_mapping.yaml
```

记录原模型 hash、identified model hash 和生成脚本版本。

## 22.3 回灌后至少做三类验证

**动力学预测：**

```text
新的独立 trajectory / repeated B
tau_hat vs tau_calibrated
```

**静态重力：**

在多个静态姿态比较重力矩预测与已标定 effort，qdd≈0、qd≈0 时不让动态项掩盖错误。

**控制效果：**

在同一低风险控制任务上比较：

```text
原模型
vs
辨识模型
```

观察跟踪误差、前馈力矩残差和控制输出变化。控制验证必须另行做安全门禁，不能因为离线
RMSE 变小就直接扩大运动范围。

# 23. 推荐的现场执行顺序清单

真正开始做 reBot 系统辨识时，按下面顺序逐项打勾：

- [ ] 固定基座，固定工具/负载/夹爪状态；
- [ ] 保存 Git、SDK、motor config、safety、URDF hash；
- [ ] 仿真 Phase 5A/5B/5C regression PASS；
- [ ] `state_only` PASS；
- [ ] J1～J6 mapping/zero 达到辨识级；
- [ ] J6 4.316 mm 几何冲突关闭；
- [ ] Servo hold PASS；
- [ ] `effort_reported` 独立力矩标定完成；
- [ ] trajectory replay/provider 实现并测试；
- [ ] 单关节极小幅 C commissioning PASS；
- [ ] 六轴低速 C commissioning PASS；
- [ ] 冻结 A/B trajectory artifact + hash；
- [ ] A 重复采集完成；
- [ ] B 独立采集完成；
- [ ] raw CSV/meta/lower logs 全部归档；
- [ ] 真机 preprocessing 实现并冻结；
- [ ] q/qd/qdd/torque 频谱和滤波敏感性检查；
- [ ] real-hardware identification loader/mode 验收；
- [ ] A-only SVD/base-space/OLS 完成；
- [ ] B-only independent prediction 完成；
- [ ] 参数重复性和物理一致性检查；
- [ ] identified model 单独保存，不覆盖 canonical；
- [ ] 静态重力 + 独立轨迹 + 低风险控制验证完成。

# 24. 当前项目离“可以正式辨识”还差什么

截至 2026-09-07，按两个项目的实际代码和现场记录，状态为：

| 项目 | 状态 | 是否阻塞正式辨识 |
|---|---|---|
| reBot canonical dynamics / Pinocchio regressor | 已有仿真闭环 | 否 |
| 仿真 Fourier A/B + clean OLS | PASS | 否 |
| 真机 state_only | 已有现场记录 | 否 |
| Servo current-position hold | smoke 级 | 后续需正式验收 |
| J1 目测零位 | smoke-only | **是** |
| J2～J6 辨识级 mapping/zero 记录 | 未冻结 | **是** |
| J6 几何模型一致性 | 4.316 mm 冲突 | **是** |
| effort_reported 独立 Nm 标定 | 未完成 | **是** |
| excitation trajectory provider | runner 明确 PENDING | **是** |
| 单关节/六轴 commissioning | 未执行 | **是** |
| 真机 A/B 正式采集 | 未执行 | **是** |
| 真机 qdd preprocessing | 未实现 | **是** |
| 真机 identification data contract | 未实现 | **是** |
| current identify 的 real mode | 未实现 | **是** |
| 独立 B 真机验证 | 未执行 | **是** |

因此当前最高价值顺序不是直接“跑辨识”，而是：

```text
1. 关闭 J1～J6 mapping/zero + J6 geometry
2. 完成 effort_reported 独立力矩标定
3. 接入 trusted trajectory replay/provider
4. 单关节 -> 六轴 commissioning
5. 实现 hardware preprocessing + real identification mode
6. 冻结 A/B，正式采集
7. A-only OLS -> B-only validation
8. physically-consistent model reinjection
```

完成第 1～5 项之后，项目才真正从“实机控制 smoke”进入“可做正式 reBot 系统辨识”的状态。
