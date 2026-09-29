# 真 PX4 + 真串口测距的无桨观察

此入口只观察吸顶决策。它读取真实 PX4 的 `/mavros/state`、
`/mavros/rc/in`、`/mavros/local_position/odom` 和顶部 L1 串口传感器；
所有程序输出都在 `/bench/` 下。启动文件不含飞控设定值或机构命令输出节点，
并且仅在这个入口中放宽决策节点的已解锁和 OFFBOARD 要求。
观察入口将 PX4 状态超时设为 1.5 秒，以匹配当前约 1 Hz 的 MAVROS 状态更新。

1. 给机架断电，拆下全部桨叶，固定机架。接好飞控、遥控器和顶部 L1 后，
   确认飞控与 L1 使用两个不同的串口设备。保持 PX4 未解锁，
   不切入 OFFBOARD；只给飞控与传感器供电。
2. 每个终端执行：

   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/catkin_ws/devel/setup.bash
   ```

3. 找到实际设备路径：

   ```bash
   ls -l /dev/serial/by-id/ /dev/serial/by-path/
   ```

   当前连接中，顶部 L1 是 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
   （`ttyUSB0`），PX4 是
   `/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00`（`ttyACM0`）。
   顶部 L1 使用 38400 baud、`l1_ascii` 格式。飞控串口速率与 PX4
   MAVLink 端口配置保持一致；设备重插后仍应先核对路径。

4. 在终端 A 启动 MAVROS：

   ```bash
   roslaunch mavros px4.launch \
     fcu_url:=/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00:57600
   ```

5. 在终端 B 启动观察入口：

   ```bash
   roslaunch px4_wall_ceiling_control ceiling_attachment_px4_observe.launch \
     ceiling_serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
   ```

6. 在终端 C 检查输入；`connected: True`、持续更新的测距和里程计、
   以及有效遥控通道缺一不可：

   ```bash
   rostopic echo -n 1 /mavros/state
   rostopic echo -n 1 /mavros/rc/in
   rostopic echo -n 1 /mavros/local_position/odom
   rostopic hz /bench/sensor/ceiling/range
   rostopic echo /bench/ceiling_attachment/status
   ```

   用 `Ctrl+C` 依次结束持续输出的命令。观察入口的终端也会打印状态变化。

7. 顶部传感器对准可移动的平面板。默认参数在
   `config/ceiling_attachment.yaml`：顶部读数小于 **0.50 m** 且竖直速度
   小于 **0.20 m/s** 时，可从 `CEILING_ARMED` 进入 `APPROACH`；
   滤波读数降至 **0.12 m** 以下，在 0.50 s 稳定等待后进入
   `ATTACH_CONTROL`；读数保持在约 **0.085 m** 以下并满足持续确认后，
   进入 `SURFACE_HOLD`。这些是台架初值，不能当作实机接触标定。
   缓慢移动板并以 `/bench/ceiling_attachment/status` 里的
   `ceiling_distance_filtered_m` 和 `reason` 为准。

遥控器吸顶开关位于 `RCIn.channels[4]`（第五个通道），PWM 至少 1700；
侧吸开关 `channels[5]` 应保持关闭。先关闭两个开关，再打开吸顶开关。
如果传感器日志出现 `E=255` 且测距话题没有数据，请调整平面板位置和朝向，
直到 `/bench/sensor/ceiling/range` 持续更新。
如果 `/mavros/local_position/odom` 不更新，这个真实 PX4 观察入口不会触发；
可先用 `ceiling_attachment_sensor_test.launch` 的模拟飞行反馈检查传感器和状态机。
测试结束后关闭吸顶开关并停止两个 launch。
若已到 `SURFACE_HOLD`，关闭开关后会进入 `DETACH`；本入口没有机构反馈模拟器，
因此随后出现机构释放超时 `FAULT` 是预期的观察结果。
