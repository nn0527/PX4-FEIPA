# PX4 桥梁吸附巡检

本仓库基于 PX4-Autopilot，增加了桥底吸顶（Ceiling）和桥侧吸附（Wall Perch）控制。当前版本的核心修改是：

- ESP32 串口数据帧由 **7 字节单测距**升级为 **9 字节双测距**；
- 一帧同时传输上方（UP）和前方（FRONT）距离，并增加 CRC-8/ATM 校验；
- 两路距离被发布为两个独立的 `distance_sensor` 实例；
- 吸顶控制只选择 UP，侧吸控制按阶段选择 FRONT 和 UP；
- 吸顶和侧吸控制器可以独立编译、独立启动和独立测试，避免同时抢占 AUX 开关或控制设定值。

> 本项目包含真实飞行控制代码。首次检查协议和传感器链路时应拆除螺旋桨；实际吸附测试必须设置安全区域、保护措施和人工急停。

## 1. 9 字节数据帧

### 1.1 新旧格式

旧格式只有一路距离，也没有 CRC：

```text
AA 55 D_H D_L RSV 0D 0A
```

新格式固定为 9 字节：

```text
AA 55 UP_H UP_L FWD_H FWD_L CRC 0D 0A
```

| 字节索引 | 字段 | 说明 |
| --- | --- | --- |
| 0 | `0xAA` | 帧头第 1 字节 |
| 1 | `0x55` | 帧头第 2 字节 |
| 2 | `UP_H` | 上方距离高字节 |
| 3 | `UP_L` | 上方距离低字节 |
| 4 | `FWD_H` | 前方距离高字节 |
| 5 | `FWD_L` | 前方距离低字节 |
| 6 | `CRC` | 字节 0～5 的 CRC-8/ATM |
| 7 | `0x0D` | 帧尾 CR |
| 8 | `0x0A` | 帧尾 LF |

两路距离均为 **uint16 大端序**，单位为 mm：

```text
up_mm    = (UP_H  << 8) | UP_L
front_mm = (FWD_H << 8) | FWD_L
```

`0` 和 `0xFFFF` 表示该方向无有效测距。UP 和 FRONT 独立判定：其中一路无效时，只停止发布该方向，另一路仍可正常使用。因此最大有效距离为 `65534 mm`。

### 1.2 CRC 参数

CRC 使用以下固定参数：

- 算法：CRC-8/ATM；
- 多项式：`0x07`；
- 初始值：`0x00`；
- `RefIn=false`、`RefOut=false`；
- `XorOut=0x00`；
- 计算范围：帧的字节 0～5，**不包含 CRC 自身和 `0D 0A` 帧尾**。

ESP32 端可按下面的方式组帧：

```cpp
uint8_t crc8_atm(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
                               : (uint8_t)(crc << 1);
        }
    }

    return crc;
}

uint8_t frame[9] = {
    0xAA, 0x55,
    (uint8_t)(up_mm >> 8),    (uint8_t)up_mm,
    (uint8_t)(front_mm >> 8), (uint8_t)front_mm,
    0x00,
    0x0D, 0x0A
};

frame[6] = crc8_atm(frame, 6);
Serial.write(frame, sizeof(frame));
```

例如 UP=`100 mm`、FRONT=`200 mm` 时，完整帧为：

```text
AA 55 00 64 00 C8 C5 0D 0A
```

旧的 7 字节 ESP32 固件与当前接收端不兼容，必须同步升级发送格式。

## 2. 实现方法

完整数据链如下：

```text
ESP32 / Gazebo 双测距
        │  AA 55 UP_H UP_L FWD_H FWD_L CRC 0D 0A
        ▼
uart_rx（TELEM2，115200，8N1）
        │  校验帧头、帧尾、CRC，失步后重新寻找 AA 55
        ▼
esp32_uart_frame（uint8[9] uORB 消息）
        ▼
CeilingReader（二次校验、解码、mm → m）
        ├── UP    → distance_sensor，orientation=24（UPWARD）
        │             └── ceiling_controller
        └── FRONT → distance_sensor，orientation=0（FORWARD）
                      └── wall_perch（同时在翻转后使用 UP 判断接触）
```

### 2.1 串口接收与重新同步

[`src/modules/uart_rx/UartRx.cpp`](src/modules/uart_rx/UartRx.cpp) 默认独占读取 FMUv6X 的 TELEM2（`/dev/ttyS4`），配置为 115200 baud、8N1、无软硬件流控。接收状态机逐字节寻找 `AA 55`，收满 9 字节后同时检查 CRC 和 `0D 0A`。

坏帧不会发布。发生 CRC 错误、插字节或丢字节时，接收器会在已有缓冲区中继续查找内嵌的 `AA 55`；若最后一个字节是 `AA`，则保留它作为下一帧起点，避免连续丢失后续正确帧。合法原始帧通过 `esp32_uart_frame` 发布，消息数组已在 [`msg/Esp32UartFrame.msg`](msg/Esp32UartFrame.msg) 中改为 `uint8[9]`。

### 2.2 双路距离发布

[`src/modules/ceiling_reader/CeilingReader.cpp`](src/modules/ceiling_reader/CeilingReader.cpp) 对原始帧再次校验，然后分别解析 UP 和 FRONT。模块使用两个 `PublicationMulti<distance_sensor_s>` 发布标准 PX4 测距消息：

- UP：`ROTATION_UPWARD_FACING`，orientation=`24`；
- FRONT：`ROTATION_FORWARD_FACING`，orientation=`0`；
- 类型为 Laser，单位由 mm 转为 m；
- `0` 或 `0xFFFF` 只使对应方向失效；
- 两个方向沿用同一原始帧时间戳。

消费者按 `orientation` 选择方向，不应假定 UP 永远是实例 0、FRONT 永远是实例 1，因为系统中可能还有其他测距传感器。

### 2.3 吸顶控制

[`src/modules/ceiling_controller`](src/modules/ceiling_controller) 只接受朝上的有效 `distance_sensor`。控制状态依次完成待机、接近、接触压缩、表面保持、卸载脱离和恢复：

```text
NORMAL → ARM → APPROACH → ATTACH → SURFACE → DETACH → RECOVERY → NORMAL
```

接近阶段覆盖 Z 轴速度，接触后使用距离 PID 计算机体系 Z 轴推力；XY 和航向仍由 PX4 正常位置控制链负责。`mc_pos_control`、`FlightModeManager`、悬停推力估计器和降落检测器增加了吸顶状态的交接、积分复位及接触保护逻辑。

默认使用 AUX1 开始、AUX2 脱离。重新测试前需将 AUX1/AUX2 都拨回低位，以清除重新触发锁。控制器要求传感器和状态数据有效，并要求已解锁的多旋翼处于 ALTCTL；条件不满足时不会进入吸顶控制或会执行保护退出。

### 2.4 侧吸控制

[`src/modules/wall_perch`](src/modules/wall_perch) 在靠墙阶段使用 FRONT，在机体向墙面翻转后使用 UP 判断顶部机构与墙面接触：

```text
IDLE → FRONT_DETECT → STABILIZE → SLOW_APPROACH → FLIP
     → CAPTURE → HOLD/PIN → DETACH → RECOVER → EXIT → IDLE
```

模块在有效阶段发布姿态和推力设定值；进入可选的 `WALL_PIN` 直接电机阶段时，`ControlAllocator` 根据 `wall_perch_status.direct_motor_control` 暂停标准电机输出，避免两个发布者同时控制执行器。`mc_pos_control` 在 `wall_perch_status.active` 时也停止发布标准姿态设定值，并持续同步高度目标以便平滑交还控制权。

侧吸固定使用 AUX1 开始、AUX2 脱离。启动、失败或完成后需先将 AUX1/AUX2 都拨回低位，再通过 AUX1 上升沿触发；动作开始后 AUX1 降低不会脱离，脱离由 AUX2 触发。开始前 FRONT 和 UP 都必须新鲜有效，飞行器需已解锁并处于多旋翼 ALTCTL。

## 3. 分开测试吸顶和侧吸

两个控制器共用 AUX1/AUX2，也都会接入 PX4 控制设定值链路，因此测试时必须互斥。可以采用下面两种方式。

### 3.1 FMUv6X 固件独立编译（真实飞行推荐）

公共串口和双测距模块在两种固件中都应保留。在 [`boards/px4/fmu-v6x/default.px4board`](boards/px4/fmu-v6x/default.px4board) 中保持：

```text
CONFIG_MODULES_UART_RX=y
CONFIG_MODULES_CEILING_READER=y
```

当前工作区已经配置为下面的“吸顶单测固件”。

吸顶单测固件：

```text
CONFIG_MODULES_CEILING_CONTROLLER=y
CONFIG_MODULES_WALL_PERCH=n
```

同时在 [`ROMFS/px4fmu_common/init.d/rc.mc_apps`](ROMFS/px4fmu_common/init.d/rc.mc_apps) 中只启动：

```sh
ceiling_controller start
#wall_perch start
```

侧吸单测固件：

```text
CONFIG_MODULES_CEILING_CONTROLLER=n
CONFIG_MODULES_WALL_PERCH=y
```

并只启动：

```sh
#ceiling_controller start
wall_perch start
```

编译和通过 USB 上传：

```bash
make px4_fmu-v6x_default
make px4_fmu-v6x_default upload
```

切换配置后必须确认编译日志中包含目标模块。启动后再用 `status` 确认只有一个控制器运行。

### 3.2 SITL 运行时互斥测试

SITL 固件可以同时编译两个模块，但运行时只启动一个。完整 Gazebo 双测距场景的详细说明见 [`docs/gazebo_sitl/README.md`](docs/gazebo_sitl/README.md)。基本流程为：

终端 1：

```bash
make px4_sitl gz_x500_dual_range_bridge
```

PX4 和 Gazebo 就绪后，在终端 2 启动交互工具：

```bash
python3 Tools/bridge_interactive_sitl.py
```

只测试吸顶：

```text
bridge> mode ceiling
bridge> prepare
bridge> status
bridge> aux1 on
bridge> aux2 on
bridge> aux2 off
bridge> aux1 off
bridge> land
```

只测试侧吸：

```text
bridge> mode wall
bridge> prepare
bridge> status
bridge> aux1 on
bridge> aux2 on
bridge> aux2 off
bridge> aux1 off
bridge> land
```

`mode ceiling` 会先停止 `wall_perch`，`mode wall` 会先停止 `ceiling_controller`。这里的 `mode` 只表示选择哪个自定义控制器，不等于 PX4 飞行模式。

> **当前 SITL 工具限制：** `prepare` 使用 Offboard 完成预定位，但脚本不会自动切回 ALTCTL；两个控制器源码都明确拒绝 Offboard。完成预定位后，必须通过 QGC/遥控器或在 `pxh>` 中执行 `commander mode altctl`，确认已进入 ALTCTL，再拨高 AUX1。否则 `input_valid/control_mode_valid` 不会通过，吸附动作不会开始。
>
> 交互工具的 AUX1/AUX2 只负责开始和脱离，不能替代独立人工急停与安全措施。

## 4. 协议和数据链检查

不启动飞行控制即可验证 9 字节组帧和黄金样例：

```bash
python3 Tools/esp32_dual_range_sitl.py --self-test
python3 Tools/bridge_interactive_sitl.py --self-test
```

预期输出包含：

```text
CRC/frame self-test passed: AA 55 00 64 00 C8 C5 0D 0A
```

在 PX4 `pxh>` 中检查真实串口数据链：

```text
uart_rx status
CeilingReader status
listener esp32_uart_frame -n 1
listener distance_sensor -n 1
```

根据固件类型继续检查：

```text
ceiling_controller status
listener ceiling_contact_status -n 1
```

或：

```text
wall_perch status
listener wall_perch_status -n 1
```

`uart_rx status` 会显示合法帧、非法帧、CRC 错误以及最近的 UP/FRONT 距离。`CeilingReader status` 会分别显示两路发布次数和最后更新时间。

若只验证一路数据，可让 ESP32 将未测试方向置为 `0xFFFF`：

- UP 链路单测：UP 为有效距离，FRONT=`0xFFFF`；
- FRONT 链路单测：UP=`0xFFFF`，FRONT 为有效距离。

这只能验证两路解析和发布是否独立。完整的 `wall_perch` 起始安全检查要求 FRONT 与 UP 都有效，因此侧吸动作测试仍需同时提供两路真实测距。

## 5. 关键文件

| 文件 | 作用 |
| --- | --- |
| [`msg/Esp32UartFrame.msg`](msg/Esp32UartFrame.msg) | 9 字节原始帧 uORB 消息 |
| [`src/modules/uart_rx`](src/modules/uart_rx) | UART 配置、组帧、CRC、重同步和原始帧发布 |
| [`src/modules/ceiling_reader`](src/modules/ceiling_reader) | 双路解码并发布 UP/FRONT `distance_sensor` |
| [`src/modules/ceiling_controller`](src/modules/ceiling_controller) | 吸顶状态机和 Z 轴控制 |
| [`src/modules/wall_perch`](src/modules/wall_perch) | 侧吸接近、翻转、吸附、脱离和恢复状态机 |
| [`Tools/esp32_dual_range_sitl.py`](Tools/esp32_dual_range_sitl.py) | 9 字节编码、PTY 串口注入和 CRC 自测 |
| [`Tools/bridge_interactive_sitl.py`](Tools/bridge_interactive_sitl.py) | Gazebo 双测距转 UART，并互斥选择吸顶/侧吸 |
| [`ROMFS/px4fmu_common/init.d/rc.mc_apps`](ROMFS/px4fmu_common/init.d/rc.mc_apps) | 真机控制器启动选择 |
| [`boards/px4/fmu-v6x/default.px4board`](boards/px4/fmu-v6x/default.px4board) | FMUv6X 模块编译选择 |

## 6. 常见问题

- **一直没有合法帧**：检查 ESP32 是否仍发送旧 7 字节格式，以及 CRC 是否覆盖了字节 0～5。
- **距离数值颠倒或异常大**：两路距离均为高字节在前的大端序，单位为 mm。
- **只有一个 `distance_sensor`**：检查另一方向是否发送了 `0`/`0xFFFF`，并通过 `orientation` 而非实例编号识别方向。
- **无法打开 TELEM2**：`uart_rx` 需要独占 `/dev/ttyS4`；不要再把该串口分配给 MAVLink、GPS 或另一个串口读取模块。
- **AUX1 无法触发**：先检查飞行模式、解锁状态、`RC_MAP_AUX1/2` 映射和传感器新鲜度，并确认触发前已将 AUX1/AUX2 都拨回低位。
- **两个控制器都有输出**：立即将 AUX1/AUX2 置低并停止其中一个模块；真机测试建议使用独立编译固件。

PX4 原项目的构建、硬件和开发文档见 [PX4 User Guide](https://docs.px4.io/main/) 与 [PX4 Developer Guide](https://docs.px4.io/main/en/development/development.html)。本仓库沿用上游 BSD 3-Clause 许可证，详见 [`LICENSE`](LICENSE)。
