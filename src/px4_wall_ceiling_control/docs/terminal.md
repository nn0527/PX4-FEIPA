# 终端入口：吸顶 / 侧吸与模式交接

面向 PX4 多旋翼 + ROS1 Noetic + MAVROS。飞手先用遥控器在 `ALTCTL`（侧吸）或
`POSCTL`（吸顶）飞到目标附近，再用终端命令请求吸顶、侧吸或脱离。模式管理器在
`ALTCTL` 中让唯一输出节点持续发送姿态＋推力设定值，确认流连续 ≥1.5 s 后调用
`/mavros/set_mode` 请求 `OFFBOARD`；动作决策取得唯一输出 owner 后才发送控制量。
本包**不解锁、不起飞、不自动降落**（吸顶中止分支除外，见 [ceiling](ceiling.md)）。

- 生产入口：`launch/system/attachment_terminal_flight.launch`，`flight_enabled` 默认 `false`
- 操作门：`scripts/operator/terminal_operator_node.py`
- 模式交接：`scripts/operator/flight_mode_manager_node.py`

## 1. 启动

MAVROS 必须在另一个终端单独启动并保持运行：

```bash
roslaunch mavros px4.launch fcu_url:=/dev/serial/by-id/FCU_DEVICE:57600
```

先跑观察模式（不向 PX4 发设定值，也不请求 OFFBOARD）：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch px4_wall_ceiling_control attachment_terminal_flight.launch \
  ceiling_serial_port:=/dev/serial/by-id/TOP_L1_DEVICE \
  front_serial_port:=/dev/serial/by-id/FRONT_L1_DEVICE \
  flight_enabled:=false
```

没有前向 L1 时可暂不传 `front_serial_port`，侧吸请求会被阻止。设备路径必须用目标机
`ls -l /dev/serial/by-id/ /dev/serial/by-path/` 的结果。

观察模式检查项：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /sensor/health
rostopic echo -n 1 /mavros/local_position/odom
rostopic echo -n 1 /mavros/estimator_status
rostopic echo /attachment/flight_phase
rostopic hz /mavros/rc/in
rostopic echo /attachment/output_owner
```

观察模式不应出现 `/mavros/setpoint_raw/attitude` 输出。

## 2. 命令

```bash
rostopic pub -1 /attachment/terminal_command std_msgs/String "data: 'ceiling'"
rostopic pub -1 /attachment/terminal_command std_msgs/String "data: 'wall'"
rostopic pub -1 /attachment/terminal_command std_msgs/String "data: 'detach'"
rostopic pub -1 /attachment/terminal_command std_msgs/String "data: 'cancel'"
```

- `ceiling`：需从 `POSCTL` 发起；进入后前向雷达被排除在控制输入之外，直到确认回到
  `POSCTL`。
- `wall`：需从 `/flight_mode_manager_node/wall_pilot_mode`（默认 `ALTCTL`）发起。
- `detach`：只在 `SURFACE_HOLD` / `WALL_HOLD` 且姿态门槛满足时被接受；侧吸退回姿态
  不稳定时拒绝，需调整后重发。
- `cancel`：释放行为请求，决策节点执行自己的恢复路径，不会强制提前切模式。
- 同一次动作期间重复启动会被拒绝；节点安全条件变化时请求可能被拒，需检查急停与飞控反馈。

`/attachment/flight_phase` 应依次出现 `PRESTREAM`、`ENTERING`、`ACTIVE`；
用 `rostopic hz /mavros/setpoint_raw/attitude` 确认设定值流。

## 3. 模式交接契约

| 条件 | 系统动作 | 验证点 |
|---|---|---|
| ALTCTL/POSCTL、已解锁、所需传感器与 odom 新鲜 | 50 Hz 姿态＋推力预发送 | `/attachment/flight_phase=PRESTREAM`；`/mavros/setpoint_raw/attitude` 有流 |
| 预发送连续 ≥1.5 s | 请求 `OFFBOARD` | `/mavros/state.mode=OFFBOARD` |
| 行为进入执行态 | owner = 1（吸顶）或 2（侧吸），只有 mux 向 MAVROS 发控制量 | `/attachment/output_owner` 与行为状态 |
| 吸顶 `DETACH → RECOVERY_HOVER → NORMAL_FLIGHT` | 释放 owner、姿态与竖直速度稳定、摇杆回中后请求 `POSCTL` | 状态、owner=0、飞控 mode |
| 侧吸 `DETACH_ROTATE → RECOVER → EXIT/IDLE` | 同样确认交接条件后请求目标模式 | 同上 |
| 故障、候选无效、PX4 外部切模式 | 抑制继续启动；飞手用遥控器接管 | PX4 模式与失联保护行为 |

`mode_sent` 只表示服务请求已发出；必须收到新的实际模式反馈才算完成，服务返回成功
不等于已完成交接。急停、失锁、飞手主动切换模式时立即停止请求。

## 4. 遥控与回中

- `flight_enabled:=true` 时默认启用遥控姿态修正与中位交接门槛。必须先在观察模式
  逐根拨杆确认**从零开始**的横滚、俯仰、油门通道索引、正反方向与中心 PWM。
- 实飞入口必须显式提供 `rc_roll_channel_index`、`rc_pitch_channel_index`、
  `rc_throttle_channel_index`；索引缺失或重复时节点启动失败，不能进入自动 OFFBOARD。
- 方向相反用 `rc_roll_reverse` / `rc_pitch_reverse`；中位与端点用 `rc_pwm_low`、
  `rc_pwm_center`、`rc_pwm_high` 校准。
- 观察模式可开 `rc_assist_enabled:=true` 与通道索引，用
  `rostopic echo /attachment/rc_preview_deg` 检查 `vector.x`（横滚）、`vector.y`（俯仰）
  回中是否接近 0，此时不发真实设定值。
- 修正限幅、`rc_input_age_ms` 语义与急停见 [rc](rc.md)。

实飞启动示例：

```bash
roslaunch px4_wall_ceiling_control attachment_terminal_flight.launch \
  ceiling_serial_port:=/dev/serial/by-id/TOP_L1_DEVICE \
  front_serial_port:=/dev/serial/by-id/FRONT_L1_DEVICE \
  flight_enabled:=true \
  rc_roll_channel_index:=N_ROLL \
  rc_pitch_channel_index:=N_PITCH \
  rc_throttle_channel_index:=N_THR \
  hover_thrust:=CALIBRATED_HOVER_THRUST
```

`hover_thrust` 会覆盖吸顶、侧吸与统一输出三处 YAML 的同名值。`max_thrust` 没有
launch 覆盖参数，分别来自三个 YAML（当前吸顶 0.90、侧吸 0.85、统一输出 0.85），
最终仍受统一输出上限约束；这个值限制总推力请求，不是 PX4 混控后的单电机硬上限。

## 5. 上机前校准

`config/ceiling_attachment.yaml` 与 `config/wall_perch.yaml` 仍是台架初值，必须在目标机标定：

1. **悬停推力**：在 QGroundControl 的 MAVLink Console 执行
   `listener hover_thrust_estimate 5`，确认 `valid: true` 后读取 `hover_thrust`；
   结合 ULog 归一化实际推力选取。`MPC_USE_HTE=0` 时 PX4 使用固定 `MPC_THR_HOVER`，
   为 `1` 时该参数主要是估计器初值。2026-09-22 ALTCTL ULog 的有效悬停估计约 0.448，
   包内现以 0.49 作为基准；`MPC_THR_HOVER=0.54` 未随包自动修改。
2. **接触与分离距离**：实测顶部首次接触距离、安全分离距离、前向触墙距离与雷达安装方向。
   当前 `wall_angle_deg=+90` 与早期台架 `+45°` 正俯仰试验方向一致（该台架脚本未入库），
   但只有 `+45°` 经过台架验证，完整 90° 幅度与侧吸推力仍需实测。
3. **推力上限**：实飞后依据记录更新三处 `max_thrust`。
4. **估计器**：`/mavros/estimator_status` 的 `attitude_status_flag`、
   `velocity_vert_status_flag` 必须为 `true`，里程计姿态与竖直速度连续有限。
   本入口不要求 `velocity_horiz_status_flag`，因此不会自动制止水平漂移。

## 6. 实飞操作顺序

1. 在 QGC 确认模式开关能进入 `ALTCTL`/`POSCTL`，设置并验证 `COM_OF_LOSS_T`、
   `COM_OBL_RC_ACT`；准备飞手手动切回的动作。故障与 `cancel` 不会被当作正常脱离完成。
2. 只做遥控悬停，记录悬停推力估计与实际归一化推力；着陆后标定 `max_thrust`。
3. 启动 MAVROS 与观察入口，确认两路 `valid`、`/sensor/*/range` 方向正确、odom 连续。
4. 关闭观察入口，按第 4 节参数加 `flight_enabled:=true` 重启，保持唯一 MAVROS 实例；
   用遥控器在目标模式解锁并飞到目标附近。
5. 发送 `ceiling` 或 `wall`，观察 `/attachment/flight_phase`、`/mavros/state`、
   `/attachment/output_owner` 与对应 status；只在 `SURFACE_HOLD`/`WALL_HOLD` 后发 `detach`。
6. 确认机体接近水平、速度较小、摇杆回中；确认 PX4 实际回到目标模式后由飞手接管降落。
   出现 `ABORT` 或未按预期返回时立即用模式开关接管，不要只依赖 `cancel`。

先分别验证吸顶与侧吸，不要在同一次初始实飞中连续尝试两个行为。

## 7. 代码责任划分

| 层 | 文件 | 责任 |
|---|---|---|
| 传感器 | `scripts/sensors/lidar_distance_node.py`、`sensor_range_manager_node.py` | 两路独立解码、量测有效性与超时 |
| 操作指令 | `scripts/operator/terminal_operator_node.py` | 命令→持续 `OperatorCommand`，脱离脉冲送入对应状态机 |
| 模式交接 | `scripts/operator/flight_mode_manager_node.py` | 预发送 1.5 s、请求 OFFBOARD、正常脱离后请求目标模式 |
| 吸顶决策 | `scripts/decisions/ceiling_attachment_decision_node.py` | 接近、触顶、保持、脱离与安全分离确认 |
| 侧吸决策 | `scripts/decisions/wall_perch_decision_node.py` | 前距捕获、翻转、侧贴、脱离旋转与恢复 |
| 输出仲裁 | `scripts/outputs/attachment_output_mux_node.py`、`src/.../domain/owner_arbiter.py` | 唯一 MAVROS 发布者、互斥 owner、限幅与新鲜度 |
| 协议适配 | `src/.../adapters/mavros_setpoint.py`、`odometry.py` | ROS ENU/FLU 边界、消息生成、odom 数值校验 |
| 参数/接口 | `config/*.yaml`、`msg/*.msg` | 增益、阈值与话题契约 |
| 测试 | `test/*.py` | 状态机安全门、模式交接与输出边界 |

`attachment_mechanism_output_node.py` 为独立机构保留；当前仅靠飞控推力的方案不需要
物理机构驱动，生产入口关闭其输出并取消机构 ACK。

## 8. 评估结论

软件结构合理的部分：两路传感器独立校验；两个行为共用一个输出仲裁点；OFFBOARD 前有
高于 PX4 最低要求的持续 setpoint；切回目标模式以正常脱离完成与 owner 释放为前提；
输出与模式切换默认关闭；自动化测试覆盖 owner 互斥、新鲜度、预发送与返回条件。

尚不能仅凭代码证明的部分：

1. `odometry_is_finite()` 只验证新鲜且数值有限，不能自动制止水平漂移；实飞前仍需在
   PX4/QGC 验证姿态与竖直速度估计。
2. 吸顶接触距离、压缩量与脱离推力必须用真实机架确认；无独立吸附机构时，阈值不正确
   会让状态机永远无法确认贴顶。
3. 侧吸 `wall_angle_deg=+90`、翻转时间与距离阈值仍需实测；电机转速响应不等于侧吸
   姿态与推力已能安全支撑机体，正 `+45°` 台架结果不能外推为 90° 可飞。
4. 前向雷达必须通过 `front_serial_port` 传给入口；只改 `config/sensors.yaml` 不足以
   启动前向节点，必须看到 `/sensor/health.front_valid=true`。
5. 正常脱离失败时管理节点不会把故障当成成功自动切回；飞手必须有已验证的遥控接管
   路径，并在 PX4 配置 `COM_OF_LOSS_T` 与 `COM_OBL_RC_ACT`。

**当前判定**：代码可以构建、启动并在台架验证输出链；不能据此判定可直接进行接触式
实飞。放行还取决于悬停推力、传感器外参、机架接触几何、竖直速度估计、PX4 失联动作
与分阶段飞行记录。

## 9. 构建与测试

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg px4_wall_ceiling_control
source devel/setup.bash
catkin_make run_tests_px4_wall_ceiling_control
catkin_test_results
```

本工作区要求所有程序改动在 `ros1_noetic` 容器内编译：

```bash
docker exec ros1_noetic bash -lc 'source /opt/ros/noetic/setup.bash && cd /root/ros_ws && catkin_make'
```

全量测试并非全绿，历史失败项需与基线逐项对照，不能把"没有新增失败"当作全部通过。
