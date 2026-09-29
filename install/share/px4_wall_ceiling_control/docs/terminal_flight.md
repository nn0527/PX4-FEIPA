# 终端触发的吸顶、侧吸与模式交接

本入口面向 **PX4 多旋翼 + ROS1 Noetic + MAVROS**。操作者先用遥控器在
`ALTCTL` 定高模式飞到目标附近；终端命令请求吸顶或侧吸。模式管理节点在
`ALTCTL` 中让统一输出节点持续发送姿态＋推力 setpoint，确认流已连续至少 1.5 秒，
然后调用 `/mavros/set_mode` 请求 `OFFBOARD`。动作决策取得唯一输出 owner 后才
发送姿态＋推力。原来要求局部水平速度的竖直速度候选由输出节点根据里程计的
竖直速度转换为姿态＋推力，不再把局部速度 setpoint 发给 PX4。脱离流程完成、
owner 释放后，确认机体接近水平、竖直速度较小以及三根摇杆回中，再请求回
`ALTCTL`，操作者继续用
遥控器降落。代码从不解锁或自动起飞。

## 硬件输入

- 顶部 L1：用于吸顶接近、接触及脱离安全距离确认。
- 前向 L1：侧吸入口必需；没有它时不会请求侧吸 OFFBOARD。
- MAVROS：必须由单独的 `roslaunch mavros px4.launch fcu_url:=...` 启动。
- 当前实现无独立吸附机构，吸顶接触、保持和脱离仅使用飞控推力。终端飞行入口
  设置 `require_mechanism_ack=false`，但仍要求真实顶距越过安全分离阈值。

使用 USB 串口时，设备路径必须用目标机器
`ls -l /dev/serial/by-id/ /dev/serial/by-path/` 的结果，不要复制示例设备名。
如果两只 USB 串口雷达没有唯一序列号，可用固定 USB 插口的 `by-path` 路径
分别指定它们。使用树莓派 GPIO UART 时，应改用实际映射的 `/dev/ttyAMA*`
设备或自行建立的稳定别名，不能沿用 USB 设备路径。
雷达节点启动时发送 `iACM`；关闭 launch 时默认只关闭串口，不发送 `iHALT`，
因此雷达在持续供电时仍可保持测距，但 ROS 不再接收或发布数据。需要退出时
停止测距，可把相应节点的 `halt_on_shutdown` 参数设为 `true`。
如果没有前向雷达，前向节点不会启动，吸顶仍可用。

## 构建与启动

以下命令在搭载 ROS 的目标机执行，以 USB 串口为例。先从本机设备列表确认飞控、
顶部 L1 和前向 L1 各自的路径；`FCU_DEVICE`、`TOP_L1_DEVICE`、
`FRONT_L1_DEVICE` 是待替换的设备名，`57600` 也须与飞控串口波特率一致。

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg px4_wall_ceiling_control
source devel/setup.bash
ls -l /dev/serial/by-id/ /dev/serial/by-path/
```

终端 A 启动唯一 MAVROS 实例，并保持运行：

```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch \
  fcu_url:=/dev/serial/by-id/FCU_DEVICE:57600
```

终端 B 先启动统一链路的观察模式。这里的 `flight_enabled:=false` 不会向 PX4
发送实际姿态设定值，也不会请求 OFFBOARD；确认传感器和遥控通道后，先关闭这个
launch，再用下文的实飞命令重启。不要同时启动 `ceiling_motor_bench.launch` 或
旧的输出节点：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch px4_wall_ceiling_control attachment_terminal_flight.launch \
  ceiling_serial_port:=/dev/serial/by-id/TOP_L1_DEVICE \
  front_serial_port:=/dev/serial/by-id/FRONT_L1_DEVICE \
  flight_enabled:=false
```

没有前向 L1 时，可暂不传 `front_serial_port`；侧吸请求会被阻止。
如果设备使用 `by-path` 路径，把对应的完整路径直接放入参数。

### 树莓派 5 GPIO UART 移植

本包的 L1 节点使用独立的串口设备、38400 baud、8N1，并向每只 L1 发送
`iACM`。MAVROS 使用第三路独立串口连接飞控 TELEM2。只要三路设备最终都是
Linux 串口，决策和输出代码无需因 USB 改 GPIO 而修改；在上述命令中换成
实际的 `/dev/ttyAMA*` 路径即可。先确认 L1 裸接口确实是兼容 3.3 V 的
TTL UART；若它实际输出 5 V TTL、RS-232 或 RS-485，应使用相应电平转换器
或收发器，不能直接接树莓派 GPIO。

树莓派 5 的 `/dev/serial0` 默认指向专用调试 UART，而非 40 针 GPIO 的
UART0。先按所选 GPIO 引脚启用三路不冲突的 UART overlay，关闭占用这些
串口的 Linux 控制台，再在目标机核实每个端口对应哪个设备和传感器；不要
根据 `/dev/ttyAMA*` 编号猜接线。在 Raspberry Pi OS 上可查询 overlay 选项；
其他系统也应查其 `/boot/firmware/overlays/README` 或等效配置：

```bash
dtoverlay -h uart0-pi5
dtoverlay -h uart1-pi5
dtoverlay -h uart2-pi5
ls -l /dev/ttyAMA* /dev/serial0
readlink -f /dev/serial0
```

飞控 TELEM2 的 TX 接树莓派所选 UART 的 RX，RX 接 TX，两端共地；确认
飞控型号的 TELEM2 引脚定义，勿把 TELEM2 的 5 V 供电脚直接接到 GPIO。
在 PX4 检查该口仍配置为 MAVLink，记录 `SER_TEL2_BAUD`，并把终端 A 的
`fcu_url` 改为 `实际飞控UART:相同波特率`。PX4 文档中的 TELEM2 常见默认值
是 921600，不能直接沿用上面的 57600。若只接 TX/RX/GND 而没有 RTS/CTS，
还须确认对应 MAVLink 实例的硬件流控配置不会阻塞通信。

本项目确定继续使用 ROS1 Noetic 和 MAVROS1，不迁移 ROS2，现有节点、消息、
launch 与 MAVROS 话题接口保持不变。树莓派 5 若使用 Ubuntu 24.04 或
Raspberry Pi OS Bookworm，推荐在 ARM64 Noetic 容器内运行 ROS1、MAVROS 和
本工作空间；容器使用 host 网络，并显式映射飞控及两只 L1 的三个
`/dev/ttyAMA*` 设备。GPIO UART overlay、Linux 串口控制台和设备权限仍在宿主机
配置。若目标机已有经过验证的 Ubuntu 20.04 ARM64/Noetic 原生环境，也可继续
原生部署。不要直接假设 Ubuntu 24.04 自带 `/opt/ros/noetic`。

迁移后先用 `flight_enabled:=false` 检查三路串口、两路 `/sensor/health`、
`/mavros/state` 和 `/mavros/local_position/odom`，再测试真实输出。最终容器或
原生系统都必须使用 MAVROS1 的 `roslaunch mavros px4.launch`，不能混用
ROS2 的 `mavros` 或 uXRCE-DDS 启动方式。

硬件与平台依据：[树莓派 UART 配置](https://www.raspberrypi.com/documentation/computers/configuration.html)、
[PX4 TELEM2 接线](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi)、
[PX4 MAVLink 串口参数](https://docs.px4.io/main/en/peripherals/mavlink_peripherals)、
[ROS Noetic 生命周期](https://www.ros.org/blog/noetic-eol/)。

在另一个终端检查连接、测距、里程计和遥控输入：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /sensor/health
rostopic echo -n 1 /mavros/local_position/odom
rostopic echo -n 1 /mavros/estimator_status
rostopic echo /attachment/flight_phase
rostopic hz /mavros/rc/in
rostopic echo /attachment/output_owner
```

观察模式不应出现 `/mavros/setpoint_raw/attitude` 输出。以下动作命令只在
**实飞命令启动后**、飞控已解锁且处于 `ALTCTL`、顶距/前距以及 odom 有效时输入。
发送动作命令后，用 `rostopic hz /mavros/setpoint_raw/attitude` 检查设定值流：

```bash
rostopic pub -1 /attachment/terminal_command std_msgs/String "data: 'ceiling'"
# 或：
rostopic pub -1 /attachment/terminal_command std_msgs/String "data: 'wall'"
```

`/attachment/flight_phase` 应依次出现 `PRESTREAM`、`ENTERING`、`ACTIVE`。
贴顶或贴墙保持后，用下面的命令触发完整脱离：

```bash
rostopic pub -1 /attachment/terminal_command std_msgs/String "data: 'detach'"
```

侧吸脱离命令只触发一次，须在 `WALL_HOLD` 状态发送。若 `/wall_perch/status.reason` 显示姿态尚未回到贴墙参考值或
机体未稳定，程序拒绝本次命令；调整姿态并稳定后重新输入 `detach`。

吸顶必须经过 `DETACH -> RECOVERY_HOVER -> NORMAL_FLIGHT`；侧吸必须经过
`DETACH_ROTATE -> RECOVER -> EXIT/IDLE`。只有完成这些状态并释放 owner，管理节点
才请求 `ALTCTL`。`wall_hold_time_s=0`，所以侧吸保持阶段不会因计时自动脱离。

`cancel` 用于中止请求；故障或中止不会被当作正常脱离完成，飞手须监控 PX4 模式
并用遥控器接管。飞控因失去 OFFBOARD 流的处理由 PX4 的失联参数决定。

## 上机前校准

`config/ceiling_attachment.yaml` 与 `config/wall_perch.yaml` 目前均是台架初值。
必须在目标机标定悬停推力、接触距离、雷达安装方向、姿态/推力限值及脱离距离，
并在无桨台架和 SITL/HITL 验证模式交接、速度符号和脱离完成条件。尤其需要
确认 PX4 的 `ALTCTL` 可由当前遥控器接管，且本机有有效本地 odom；只有气压高度
而没有本地里程计时，本入口不会切入 OFFBOARD。

MAVROS 已连接 PX4 后可以读取配置值：

```bash
rosrun mavros mavparam get MPC_THR_HOVER
rosrun mavros mavparam get MPC_USE_HTE
```

`MPC_USE_HTE=0` 时 PX4 使用固定的 `MPC_THR_HOVER`；设为 `1` 时，这个参数
主要是悬停推力估计器的初值。需要当前估计值时，在稳定悬停后从
QGroundControl 的 MAVLink Console 执行 `listener hover_thrust_estimate 5`，
确认 `valid: true` 再读取 `hover_thrust`。以多次稳定读数作为本包两个
`hover_thrust` 参数的标定起点，随后验证实际高度响应。仅在台架看到电机转速
变化，无法测出承载机体重量所需的悬停推力。
2026-09-22 ALTCTL ULog 的有效悬停估计约为0.448，包内统一入口现以0.45作为
悬停基准。PX4 日志中的 `MPC_THR_HOVER=0.54` 尚未随包配置自动修改；接触和
脱离推力仍需继续用真实飞行数据复核。

## 遥控摇杆与延迟

终端飞行入口在 `flight_enabled:=true` 时默认启用遥控姿态修正和中位交接门槛。
必须先在观察模式运行 `rostopic echo /mavros/rc/in`，逐根移动横滚、俯仰、油门杆，
确认三个**从零开始**的通道索引、正反方向、中心 PWM。实飞入口必须显式提供
`rc_roll_channel_index`、`rc_pitch_channel_index`、`rc_throttle_channel_index`；
索引缺失或重复时节点启动失败，不能进入自动 OFFBOARD。方向相反时使用
`rc_roll_reverse:=true` 或 `rc_pitch_reverse:=true`。PWM 中位和端点可在
`rc_pwm_low`、`rc_pwm_center`、`rc_pwm_high` 启动参数中按实际遥控器校准。

先关闭终端 B 的初始观察 launch，再保持 `flight_enabled:=false`，加上
`rc_assist_enabled:=true` 和已查出的横滚、俯仰通道索引重新启动：

```bash
roslaunch px4_wall_ceiling_control attachment_terminal_flight.launch \
  ceiling_serial_port:=/dev/serial/by-id/TOP_L1_DEVICE \
  front_serial_port:=/dev/serial/by-id/FRONT_L1_DEVICE \
  flight_enabled:=false \
  rc_assist_enabled:=true \
  rc_roll_channel_index:=N_ROLL \
  rc_pitch_channel_index:=N_PITCH
```

在另一个终端执行 `rostopic echo /attachment/rc_preview_deg`：
逐根拨杆时，`vector.x` 是计划的横滚角度、`vector.y` 是计划的俯仰角度，
回中都应接近 0。这个观察模式不会发布真实 MAVROS 设定值。

完成传感器、方向、遥控通道、悬停推力及上限标定后，关闭终端 B 的观察 launch，
保持终端 A 的 MAVROS 运行，在终端 B 启动实际输出。`N_ROLL`、`N_PITCH`、
`N_THR` 必须替换为从零开始且互不相同的实测索引；`CALIBRATED_HOVER_THRUST`
必须替换为归一化悬停推力实测值。若遥控方向或 PWM 中位不同，还需在命令中
补上已校准的 `rc_roll_reverse`、`rc_pitch_reverse`、`rc_pwm_center` 等参数。

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch px4_wall_ceiling_control attachment_terminal_flight.launch \
  ceiling_serial_port:=/dev/serial/by-id/TOP_L1_DEVICE \
  front_serial_port:=/dev/serial/by-id/FRONT_L1_DEVICE \
  flight_enabled:=true \
  rc_roll_channel_index:=N_ROLL \
  rc_pitch_channel_index:=N_PITCH \
  rc_throttle_channel_index:=N_THR \
  hover_thrust:=CALIBRATED_HOVER_THRUST
```

这个启动参数会覆盖吸顶决策、侧吸决策和统一输出节点 YAML 中的
`hover_thrust`。吸顶、侧吸及统一输出节点的 `max_thrust` 没有对应的启动覆盖
参数，分别从 `config/ceiling_attachment.yaml`、`config/wall_perch.yaml` 和
`config/attachment_output.yaml` 读取；当前侧吸和统一输出上限均为0.85，最终输出
仍受统一输出节点上限限制。这个值限制总推力请求，不是PX4混控后的单电机硬上限。
手动飞行测得上限后，着陆并更新这三个 YAML，再用实飞命令启动。

唯一输出节点直接订阅 `/mavros/rc/in`，不经过 10 Hz 操作开关节点；以 50 Hz
发出姿态目标。摇杆修正先限幅、再限变化率：自由飞行阶段默认最多 8°，
接触阶段默认最多 2°；侧吸脱离旋转默认仅允许横滚 2°，俯仰 0°。
这些是保守的软件初值，不代表机体已经验证可在这些角度保持接触或高度。
RC 输入过期时修正渐退到零，模式交接被阻止；飞手仍可通过遥控器模式开关直接
请求 `ALTCTL`。`/attachment/rc_input_age_ms` 只表示 ROS 收到遥控输入至发布
设定值的年龄，不包括飞控至 MAVROS 和 PX4 实际姿态响应延迟；`-1` 表示 RC
数据无效或超时。
确认 PX4 的 `COM_RC_OVERRIDE` 设置不会在修正摇杆时意外退出 OFFBOARD；
用于接管的独立模式开关仍需实际验证。
`rostopic hz` 只能检查频率；实际延迟须同步记录遥控输入、姿态设定值与实测姿态。

## 实飞前后的操作顺序

1. 在 QGroundControl 确认遥控器模式开关能进入 `ALTCTL`，并设置、验证
   `COM_OF_LOSS_T`、`COM_OBL_RC_ACT` 的 OFFBOARD 失联处理。准备好飞手手动切回
   `ALTCTL` 的动作；故障和 `cancel` 不会被当作正常脱离完成。
2. 先只进行遥控器 `ALTCTL` 悬停，读取上面的悬停推力估计值和飞行日志中的
   归一化实际推力；后续手动试飞后，着陆并根据记录选取吸顶、侧吸及统一输出
   节点各自的 `max_thrust` 安全上限。
   遥控器油门杆位置不是归一化实际推力。实飞入口的 `hover_thrust` 用上文启动
   参数传入；这个参数同时覆盖三个节点 YAML 中的同名值。
   同时按真实机架测量顶部首次接触距离、安全分离距离、前向触墙距离和雷达安装
   方向。当前 `wall_angle_deg=+90` 的方向与已验证的 `pitch_45_test.py`
   正俯仰方向一致，但只有 `+45°` 已经过台架验证；目标幅度与侧吸推力仍需验证。
   翻转期间若顶部雷达提前检测到接触，状态机会保持当时的目标角度进入接触、
   保持和脱离阶段，不再立即跳到 `+90°`。该分支仍须在 SITL 和固定机架验证。
3. 启动 MAVROS 和本包默认观察入口，确认 `/sensor/health` 的两路 `valid` 均为
   `true`，两个 `/sensor/*/range` 有连续且方向正确的读数，
   `/mavros/local_position/odom` 连续更新，且 `/mavros/estimator_status` 的
   `attitude_status_flag` 和 `velocity_vert_status_flag` 为 `true`，且里程计中的
   姿态与竖直速度连续、有限。此入口不再要求 `velocity_horiz_status_flag`，
   因此也不会自动制止水平漂移。
4. 关闭观察入口，再按上文给出三根 RC 通道索引和实测悬停推力，
   加 `flight_enabled:=true` 启动。保持
   唯一 MAVROS 实例，且不要同时运行电机台架或旧输出节点。此时还不会主动切换
   模式；飞手用遥控器在 `ALTCTL` 解锁并飞到目标附近。
5. 终端发送 `ceiling` 或 `wall`。观察 `/attachment/flight_phase`、
   `/mavros/state`、`/attachment/output_owner` 和相应的
   `/ceiling_attachment/status` 或 `/wall_perch/status`。只在状态达到
   `SURFACE_HOLD` 或 `WALL_HOLD` 后发送 `detach`。
6. 确认机体接近水平、下降/上升速度较小，把横滚、俯仰、油门杆回到校准中位；
   模式管理节点确认后才请求 `ALTCTL`。确认 PX4 实际回到 `ALTCTL` 后，
   飞手接管并降落。若状态机出现 `FAULT`/`ABORT`
   或没有按预期返回，立即按事先验证的遥控器模式开关接管；不要仅依靠终端
   `cancel` 命令完成模式交接。

先分别验证吸顶和侧吸，不要在同一次初始实飞中连续尝试两个行为。侧吸的翻转
姿态和推力须在仿真、固定机架及受控条件下逐步验证。
姿态＋推力链路消除了无效水平速度设定值。侧吸脱离现以开始脱离时的里程计高度为目标，
在机体回平过程中用竖直速度和实测倾角修正推力；接近侧立时可用的竖直推力仍可能
不足，推力还受 `max_thrust` 限制。水平漂移、悬停推力和高度反馈增益均未经实飞验证；
代码测试通过不等于可以直接实飞脱离。
