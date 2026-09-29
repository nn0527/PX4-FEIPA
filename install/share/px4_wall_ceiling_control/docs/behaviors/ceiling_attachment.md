# 吸顶决策与控制逻辑

## 参考程序的状态链

原始外部参考程序 `ceiling_controller.cpp` 实现的是接触式吸顶，不是简单定距悬停：

```text
NORMAL_FLIGHT
  -> CEILING_ARMED
  -> APPROACH
  -> ATTACH_CONTROL
  -> SURFACE_HOLD
  -> DETACH
  -> RECOVERY_HOVER
  -> NORMAL_FLIGHT
```

- `CEILING_ARMED`：等待顶距进入捕获范围且垂直速度足够低。
- `APPROACH`：先短暂悬停稳定，再以固定速度向上接近。
- `ATTACH_CONTROL`：用悬停推力的倍率生成接触锁定推力。
- `SURFACE_HOLD`：用另一个推力倍率保持贴顶。
- `DETACH`：目标距离逐渐退回接触距离，并计算距离 PID 候选推力。
- `RECOVERY_HOVER`：安全分离后等待固定时间，再回正常飞行。

## ROS 移植版

实现位于 `scripts/decisions/ceiling_attachment_decision_node.py`。当前直接输入：

```text
/sensor/ceiling/range
/operator/command
/mock_mavros/local_position/odom（台架测试）
/mavros/state
/attachment/mechanism_status
```

决策输出为：

```text
/ceiling_attachment/status             诊断状态
/ceiling_attachment/control_candidate  通用飞控候选
```

状态消息保留接近速度、固定姿态和归一化 body-z 推力，兼容现有监控；统一输出层只
订阅 `AttachmentControlCandidate`，不再理解吸顶状态枚举。两路传感器由上游
`sensor_range_manager_node` 独立校验和转发；吸顶决策只订阅顶部输出。

机构动作先发布到 `/attachment/mechanism_candidate`，必须经过独立的机构输出门后
才能到达 `/attachment/mechanism_command`。真机机构输出默认关闭；统一链路还要求
当前 owner 为 ceiling。

## 决策下游输出

`scripts/standalone/ceiling_attachment_output_node.py` 是独立吸顶链路的 MAVROS
适配出口；统一链路使用 `scripts/outputs/attachment_output_mux_node.py`：

| 决策状态 | 输出接口 | 含义 |
|---|---|---|
| `CEILING_ARMED` | `PositionTarget` | 零速度等待 |
| `APPROACH` | `PositionTarget` | ENU z 正方向向上接近 |
| `ATTACH_CONTROL` / `SURFACE_HOLD` | `AttitudeTarget` | 保持进入动作时捕获的姿态并输出归一化推力 |
| `DETACH` | `AttitudeTarget` | 机构释放确认后，输出低于悬停值的距离 PID 脱离推力 |
| `RECOVERY_HOVER` | `PositionTarget` | 零速度恢复悬停 |

节点在决策、MAVROS state 超时，以及未连接、未解锁、非 OFFBOARD 时禁止开始
动作；运行中故障进入锁存的恢复/FAULT，输入恢复后也不会从中间状态继续。操作
开关在贴顶时释放后，控制权会保留到 `DETACH` 和
`RECOVERY_HOVER` 完成，避免直接遗留上一条贴顶推力。

台架入口将结果隔离在 `/mock_mavros/setpoint_raw/*`；真机入口
`launch/standalone/ceiling_attachment_mavros.launch` 默认保持
`output_enabled=false`。

## 对参考程序的修正

1. 所有时间参数统一使用秒，避免原程序中 `1000`、微秒和秒的混用。
2. 脱离目标距离每周期只累加一次，避免参考程序在状态机和 PID 中重复累加。
3. 使用接收时间进行 range/odom/operator 超时判断，而非只看单次订阅是否更新。
4. `ATTACH_CONTROL -> SURFACE_HOLD` 默认要求不同真实测量样本在确认时间内持续达到压缩阈值；参考程序虽然计算了 `attach_confirmed`，但没有用于状态跳转。
5. L1 错误帧不会刷新有效顶距；超时后贴顶状态进入脱离，接近阶段回到正常状态。
6. 脱离使用独立的安全分离距离和持续确认，推力限制在标定的脱离最小值与悬停值之间；超时进入 FAULT。
7. 完成或故障后必须先释放吸顶开关才能再次启动。

## 上机前必须标定

- `contact_distance_m`：机构首次触顶时雷达读数；
- `target_compression_m`：机构允许压缩量；
- 两个推力倍率及真实悬停推力；
- 接近速度、姿态限制和接近超时；
- 雷达安装外参、倾角补偿和天花板材质下的有效更新率。
