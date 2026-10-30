# 传感器与自动记录

本文覆盖两路 L1 测距、距离校验与健康话题、失效原因，以及启动即生效的自动飞行记录。

- 驱动：`scripts/sensors/lidar_distance_node.py`（两个实例）
- 校验与管理：`scripts/sensors/sensor_range_manager_node.py`
- 配置：`config/sensors.yaml`

## 1. 两路 L1

| 节点 | 默认串口 | 原始话题 | frame_id |
|---|---|---|---|
| `ceiling_lidar_node` | `/dev/serial/by-path/wall-top` | `/sensor/ceiling/raw` | `ceiling_lidar_link` |
| `front_lidar_node` | `/dev/serial/by-path/wall-front` | `/sensor/front/raw` | `front_lidar_link` |

- 串口参数：38400、8N1、`l1_ascii`，启动时发送 `iACM`。
- `halt_on_shutdown=false`：关闭 ROS 节点只停止接收，持续供电的 L1 仍继续测距。
- 软件量程默认 0.03–40 m；这是配置边界，不等于硬件在所有环境下的可靠量程。
- 两路互不依赖：没有前向雷达时吸顶仍可用，但侧吸请求会被阻止。

设备路径必须用目标机的 `ls -l /dev/serial/by-id/ /dev/serial/by-path/` 结果，
不要复制示例名。两只 USB 串口雷达若没有唯一序列号，可用固定 USB 插口的 `by-path`。
GPIO UART 场景改用实际映射的 `/dev/ttyAMA*` 或自建稳定别名；L1 裸接口须为 3.3 V TTL，
5 V TTL / RS-232 / RS-485 需要电平转换。

## 2. 校验与管理

`sensor_range_manager_node` 独立校验两路输入，不做模式选择：

| 话题 | 方向 | 说明 |
|---|---|---|
| `/sensor/ceiling/raw`、`/sensor/front/raw` | 订阅 | 驱动原始距离 |
| `/sensor/ceiling/diagnostic`、`/sensor/front/diagnostic` | 订阅 | 驱动诊断 JSON |
| `/sensor/ceiling/range`、`/sensor/front/range` | 发布 | 校验后的距离 |
| `/sensor/health` | 发布 | 两路有效性、年龄、序号与错误码 |

- 输入超时 `input_timeout_s=0.60`，健康发布 `10 Hz`。
- 顶距在校验后同时供吸顶决策与侧吸决策（顶部接触/压墙反馈）使用。
- 侧吸在翻转后不再依赖前向雷达；翻转前必须有效。

## 3. 失效原因与错误码

`/sensor/health`（`SensorHealth`）中顶部仍使用 `ceiling` 命名，两路独立处理；
有效数据恢复时错误码清零、原因清空。

```yaml
front_valid: false
front_error_code: 255
front_error_type: sensor
front_error_reason: "L1 error 255 (weak reflection or calculation failure)"
```

| 错误码 | 类型 | 含义 |
|---|---|---|
| 0 | 空 | 有效 |
| 正数（如 255、258） | `sensor` | 传感器原始 L1 错误码，未知码也保留 |
| -1 | `timeout` | 数据超时 |
| -2 | `no_data` | 节点启动后尚未收到数据 |
| -3 | `nonfinite` | 距离为 NaN 或无穷 |
| -4 | `out_of_range` | 超出配置量程 |
| -5 | `parse` | 串口帧无法解析 |
| -6 | `configuration` | 串口未配置 |
| -7 | `driver` | 缺少串口驱动依赖 |
| -8 | `serial` | 串口打开或读写异常 |
| -9 | `diagnostic` | 诊断消息格式错误 |

超时不伪装成传感器原生错误码。驱动诊断话题
`/sensor/{front,ceiling}/diagnostic` 为 `std_msgs/String`，JSON 含 `valid`、
`error_code`、`error_type`、`error`、`stamp`、`raw_frame`（例如 `E=255`）。

`SensorHealth` 字段变化会改变消息 MD5：编译后必须在停桨状态重启整套基础 ROS 程序，
让所有发布者与订阅者加载同一版本；只重启单个节点可能无法连接。

常用查看命令（容器内）：

```bash
source /root/ros_ws/devel/setup.bash
rostopic echo /sensor/health
rostopic echo /sensor/front/diagnostic
rostopic echo /sensor/ceiling/diagnostic
rostopic hz /sensor/front/range
```

## 4. 自动飞行记录

记录范围仅限本项目 ROS 程序及其可接收的信号；不收集机身参数，不要求测试模板。

### 操作者只做三件事

1. 更新代码后按原方式构建、加载环境。
2. 照常启动 `attachment_terminal_flight.launch` 或 `attachment_system_mavros.launch`；
   记录随系统启动，无需另开终端。
3. 测试后正常 `Ctrl+C` 停止原 launch，等待退出，把本次
   `~/wall_test_records/<UTC时间_编号>/` 整个目录交给分析者，并口述当时现象。

每次启动自动生成独立目录，终端提示 `Automatic flight logs: ...`。记录节点
（`scripts/test_support/attachment_flight_logger.py`）是非 required 旁路，异常不会
连带关闭控制节点；其他节点 stdout/stderr 由 roslaunch 保存为文件以保留堆栈。

### 目录内容

| 文件 | 作用 |
|---|---|
| `events.log` | 状态/故障原因、PX4 模式、owner、飞行阶段、ROS 警告与错误 |
| `signals.csv` | 约 10 Hz 快照：实测/目标姿态、角速度、位置与竖直速度、推力、测距、数据年龄、估计器状态 |
| `flight_*.bag` | 原始话题，保留原发布频率，用于重建消息顺序与波形 |
| `ros_logs/` | 每 2 秒复制的 roslaunch 日志，含节点 stdout/stderr 与堆栈 |
| `ros_parameters.json` | 启动时的实际 ROS 参数 |
| `package_snapshot.zip`、`source_sha256.json` | 运行版本与校验值 |
| `session.json`、`recorder.log` | 会话结果、原日志位置、录制链路诊断 |

覆盖状态/候选/最终姿态推力、FCU 目标回传、odom/IMU/估计器、雷达原始与处理值、
操作与脱离指令、RC、电池、GPS/高度、ROS/PX4 状态文本。不存在的话题不会产生数据。

bag 按 30 秒 ROS 记录时长分段；磁盘不足 1 GB 时停止记录并报错。强制杀进程或断电
可能留下 `.bag.active`，需要恢复索引，末尾缓冲可能丢失。

### 分析解释规则

- CSV 是定期读取最近收到的数据，不是硬件同步采样；必须结合 UTC、ROS 时间、
  源 header 时间与接收年龄，不要忽略陈旧数据。
- 目标与实测误差用四元数最小旋转夹角；目标超过 0.25 s、odom 超过 0.5 s 或目标掩码
  忽略姿态时不输出误差，空值不代表 0。
- 三条目标路径必须区分：决策候选、ROS 最终发送、FCU 目标回传（后者仍不等于实测姿态）。
- CSV 竖直速度按 `odom_twist_in_child_frame` 解释；原始 odom 仍在 bag 中。
- 状态机退出不等于物理脱墙成功；估计位置变化也不必然是真实运动。
- ROS 侧 RC 输出不是实际 RPM 或电机饱和诊断；若问题发生在 PX4 内部，需补 ULog。

### 可选参数

```bash
# 关闭记录
roslaunch px4_wall_ceiling_control attachment_terminal_flight.launch record_logs:=false

# 改保存位置
roslaunch px4_wall_ceiling_control attachment_terminal_flight.launch \
  test_log_directory:=/home/你的用户/flight_records
```

统一系统入口支持同样参数。订阅与录制启动不构成飞行准入检查；首次部署仍应在
ROS 机器做一次短时启停，确认生成 CSV、事件日志与 bag。

## 5. 光流与融合

光流质量、积分时间、`OpticalFlowRad.distance` 与 EKF 参数切换用于侧吸水平环与吸顶
顶距估算，见 [wall](wall.md) 第 7 节和 [ceiling](ceiling.md) 第 3 节。ROS 侧看到
原始光流有数据、质量正常或 `estimator_status` 正常，都不等于 PX4 正在融合，
需要 ULog 核对 aid source 与样本时间。
