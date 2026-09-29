# 侧吸决策与控制逻辑

`scripts/decisions/wall_perch_decision_node.py` 是外部参考 PX4 `wall_perch`
模块的 ROS/MAVROS 决策移植。状态顺序保持一致：

```text
IDLE
  -> FRONT_WALL_DETECT
  -> SLOW_APPROACH
  -> FLIP_TO_WALL
  -> WALL_CAPTURE
  -> WALL_HOLD
  -> DETACH_ROTATE
  -> RECOVER
  -> EXIT
  -> IDLE
```

任何执行状态释放侧吸开关、收到 `/wall_perch/cancel`，或触发安全检查，都会进入
`ABORT -> EXIT -> IDLE`。完成一次动作后必须先释放侧吸开关才能再次启动，避免
保持高电平时自动重复翻转。

## 输入接口

| Topic | 类型 | 用途 |
|---|---|---|
| `/sensor/front/range` | `sensor_msgs/Range` | 校验后的前向距离，仅翻转前参与判定 |
| `/sensor/ceiling/range` | `sensor_msgs/Range` | 校验后的顶部接触距离，从翻转阶段起必须有效 |
| `/sensor/health` | `SensorHealth` | 两路传感器有效性、年龄、序号及无效帧计数 |
| `/operator/command` | `OperatorCommand` | 独立侧吸 RC 开关 |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | 姿态、高度、速度和角速度 |
| `/mavros/state` | `mavros_msgs/State` | 连接、解锁和 OFFBOARD 状态 |
| `/wall_perch/detach` | `std_msgs/Bool` | 从 WALL_HOLD 正常脱离 |
| `/wall_perch/cancel` | `std_msgs/Bool` | 进入 ABORT 恢复 |

取得侧吸输出 owner 前默认要求 `/sensor/health` 中顶部传感器有效；可通过
`require_top_sensor_on_acquire` 配置，但进入翻转前仍必须获得新鲜顶部距离。

决策输出 `/wall_perch/status` 使用 `WallPerchStatus`，包含状态、两路距离健康度、
触发条件、四元数姿态候选、归一化推力和故障原因。
飞控候选另行发布到 `/wall_perch/control_candidate`，类型为通用
`AttachmentControlCandidate`；输出 mux 不读取侧吸状态枚举。

## 与 PX4 原模块的差异

- PX4 的 `WALL_PIN` 会关闭 control allocation 并直接发布裸电机输出。ROS/MAVROS
  链路不复刻该高风险旁路；`pin_enabled` 默认关闭。即使显式启用，也只生成受
  `max_thrust` 限制的姿态/推力候选。
- PX4 内部 NED 状态改为读取 MAVROS/ROS ENU odometry；向下速度判断已相应换符号。
- 完成或中止后要求 RC 开关经历一次释放，防止电平触发导致动作自动重复。
- 前向、翻转和顶部接触保持计时分别从对应状态开始，并要求不同真实测量样本跨越保持时间；控制循环或健康状态发布不会替代新样本。
- 翻转前必须确认顶部传感器新鲜；恢复退出要求顶部距离越过独立解除阈值并持续保持。
- 进入 `FLIP_TO_WALL` 后不再要求前向雷达新鲜，也不再用其距离做状态或故障判定；
  顶部雷达及飞行状态的安全检查保持有效。
- 前距不大于0.50 m并由不同的新测量样本连续确认0.4 s后，直接进入
  `SLOW_APPROACH`，不经过翻转前的 `STABILIZE_HOVER`；慢速接近从一开始就以悬停
  姿态为基准向墙倾斜3°。
- 台架初值已放慢：接近最多 20 s、翻转 2 s、捕获最多 8 s、接触确认 1 s。
  传感器、里程计、飞控和操作指令的失联超时没有延长。这些时间值不是有桨飞行标定值，
  必须先在 SITL/HITL 验证姿态轨迹、推力和失败恢复。
- `SLOW_APPROACH` 使用0.47归一化推力；`FLIP_TO_WALL` 以悬停推力0.45乘
  `flip_thrust_multiplier=1.35`，得到约0.608。进入 `WALL_CAPTURE` 后最多等待8 s。
- `WALL_CAPTURE/WALL_HOLD` 使用顶部滤波距离调节压墙推力。目标0.05 m、容差
  0.01 m；距离偏大时按0.03/s增加，最高0.75；恢复接触或压得过紧时按0.05/s
  回到名义保持值0.675。顶部距离超过独立解除阈值0.10 m才判定接触丢失，
  0.05--0.10 m之间先尝试增加压墙推力。传感器或飞行状态安全故障仍立即中止。
- 根据2026-09-22侧吸日志中单电机达到1.0的情况，侧吸决策和统一输出的总推力
  上限先降为0.85，`WALL_PIN` 也使用0.85。该上限作用于MAVROS姿态目标中的总推力，
  不能保证PX4混控后的每个电机都不超过0.85；真正的单电机上限必须放在PX4控制
  分配器或ESC中。
- 顶部滤波距离不大于0.05 m并由新样本连续确认1 s后才进入 `WALL_HOLD`。
- 翻转、捕获和恢复阶段使用各自的角速度上限，不再在动态阶段完全关闭速率保护。
- `DETACH_ROTATE` 开始时锁定里程计高度，目标姿态按当前台架初值在 4 s 内平滑转回水平。接近侧立时保持压墙推力，让前轮有机会沿墙滚动；随实测推力轴获得竖直分量，逐渐切换为高度误差、竖直速度和实测倾角计算的推力，上限为 `max_thrust`。`RECOVER` 继续保持该高度，只有高度误差和竖直速度都进入容差、姿态稳定且顶部测距确认离墙后才退出。
- 脱离时维持墙面保持推力，允许前轮沿墙滚动，随后随竖直升力增加平滑转入高度控制。当前系统没有前轮接触、制动或轮速反馈，也没有前轮相对重心及雷达的完整外参；顶部 L1 的距离不能单独证明前轮始终抵墙。固定高度目标适用于前轮可沿墙滚动的假设，不能当作固定支点轨迹。
- 默认 `hold_time_s=0`，贴墙保持没有自动计时脱离；操作员停轮并发送脱离命令后才开始恢复。
- `WALL_HOLD` 保持初次接触时的机体顶部推力轴朝墙，同时跟随机体绕该轴的转动。
  因此轮子可沿墙面带动机体转向；控制器仍会纠正使推力偏离墙面的倾斜。
  轮子造成的姿态变化仅从飞控实测姿态获取，没有轮子停转反馈。
- 进入 `WALL_HOLD` 时记录实测姿态，并在 `hold_entry_blend_s` 内从接触阶段的姿态目标
  平滑过渡到该参考姿态，避免接触阶段与轮子阶段的指令跳变。
- 仅当实测姿态回到初次贴墙姿态附近，且角速度、竖直速度连续稳定时，才接受脱离命令。
  门槛由 `detach_reference_angle_deg`、`detach_axis_error_deg`、
  `detach_max_rate_rps`、`detach_max_abs_vertical_speed_mps` 和
  `detach_stable_hold_s` 控制。不满足门槛时拒绝本次命令，回正稳定后需重新发送。
  脱离旋转从触发时的实测姿态开始。
- 机体仅靠螺旋桨推力压墙。接近 90° 时竖直推力分量趋近于零，高度闭环无法克服这一物理限制；墙面接触必须在机体具有足够竖直推力前提供支撑。高度反馈增益、悬停推力、姿态响应及推力上限均为台架初值，尚未验证脱墙后能维持高度。
- 最终 MAVROS 消息由 `attachment_output_mux_node.py` 统一发布，侧吸决策本身不
  直接连接飞控。

## 输出唯一性

`attachment_output_mux_node` 在吸顶和侧吸之间锁定当前 owner，并发布：

```text
/attachment/output_owner  0=无，1=吸顶，2=侧吸
/attachment/output_mode   0=无，2=姿态+推力（统一飞行入口）
```

进入 DETACH/RECOVER/ABORT 后，即使 RC 开关释放，owner 仍保持到决策安全退出；
退出后必须先进入 owner=0，并等待两个开关关闭达到 `handoff_dwell_s`，才能授权
下一模式。
不要同时运行旧 `ceiling_attachment_output_node`、`setpoint_mux_node` 或其他 MAVROS
setpoint 发布器。
