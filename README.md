# PX4 Drone Autopilot

[![Releases](https://img.shields.io/github/release/PX4/PX4-Autopilot.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![DOI](https://zenodo.org/badge/22634/PX4/PX4-Autopilot.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/PX4-Autopilot)

[![Build Targets](https://github.com/PX4/PX4-Autopilot/actions/workflows/build_all_targets.yml/badge.svg?branch=main)](https://github.com/PX4/PX4-Autopilot/actions/workflows/build_all_targets.yml) [![SITL Tests](https://github.com/PX4/PX4-Autopilot/workflows/SITL%20Tests/badge.svg?branch=master)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22SITL+Tests%22)

[![Discord Shield](https://discordapp.com/api/guilds/1022170275984457759/widget.png?style=shield)](https://discord.gg/dronecode)

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

PX4 is highly portable, OS-independent and supports Linux, NuttX and MacOS out of the box.

## 本仓库二次开发说明：ESP32 上视距离与 ceiling_controller

本仓库在 PX4 基础上增加了面向 Pixhawk 6X 的 ESP32 UART 上视距离接入链路，用于先验证“飞控可以读取外部 ESP32 距离数据，并由正式 `ceiling_controller` 模块消费该数据”。

当前数据链路如下：

```text
ESP32
  ↓ TELEM2 UART, 115200, 8N1
uart_rx
  ↓ uORB: esp32_uart_frame
ceiling_controller
  ↓ uORB: ceiling_contact_status
```

### 1. UART 接收模块 `uart_rx`

`uart_rx` 位于：

```text
src/examples/uart_rx/
```

它负责独占读取 Pixhawk 6X 的 TELEM2 串口：

```text
TELEM2 = /dev/ttyS4
baudrate = 115200
format = 8N1
```

ESP32 发送固定 7 字节二进制帧：

```text
AA 55 distance_high distance_low reserved 0D 0A
```

距离单位为 mm，大端格式，例如 200 mm：

```text
AA 55 00 C8 00 0D 0A
```

`uart_rx` 验证帧头 `AA 55` 和帧尾 `0D 0A` 后，发布原始帧到：

```text
esp32_uart_frame
```

同时在 dmesg 中打印解析出的距离：

```text
INFO  [uart_rx] distance: 200 mm
```

### 2. 正式模块 `ceiling_controller`

`ceiling_controller` 位于：

```text
src/modules/ceiling_controller/
```

它不直接读取 UART，也不打开 `/dev/ttyS4`。距离数据来源固定为 `uart_rx` 发布的 uORB 话题：

```cpp
ORB_ID(esp32_uart_frame)
```

模块内部将距离从 mm 转为 m：

```cpp
distance_mm = (uint16_t(frame[2]) << 8) | frame[3];
ceiling_distance_m = distance_mm * 0.001f;
```

然后继续运行贴顶状态机，并发布：

```text
ceiling_contact_status
```

当前阶段 `ceiling_controller` 只完成距离接入、状态判断和状态发布；尚未接入 `mc_pos_control` 或 `mc_rate_control`，因此还不会真正覆盖多旋翼 z 向推力。

### 3. 自动启动与端口占用

FMUv6X 默认配置已启用：

```text
CONFIG_EXAMPLES_UART_RX=y
CONFIG_MODULES_CEILING_CONTROLLER=y
```

板级启动脚本中启动顺序为：

```sh
uart_rx start
ceiling_controller start
```

使用前需要确保 TELEM2 没有被 MAVLink 占用。若在 QGroundControl 中使用 TELEM2，请将对应 `MAV_x_CONFIG` 设置为 Disabled，否则 `uart_rx` 无法独占 `/dev/ttyS4`。

### 4. 编译与测试

编译 Pixhawk 6X 固件：

```sh
make px4_fmu-v6x_default
```

或在已有 Ninja 构建目录中：

```sh
env CCACHE_DISABLE=1 ninja -C build/px4_fmu-v6x_default -j2 --quiet
```

上板后在 NSH 中检查：

```sh
uart_rx status
ceiling_controller status
listener esp32_uart_frame -n 1
listener ceiling_contact_status -n 1
```

ESP32 发送：

```text
AA 55 00 C8 00 0D 0A
```

预期结果：

```text
esp32_uart_frame.frame: [170, 85, 0, 200, 0, 13, 10]
ceiling_contact_status.ceiling_distance: 0.200
```

### 5. 重要边界

- `uart_rx` 是唯一 UART 读取模块。
- `ceiling_controller` 只订阅 `esp32_uart_frame`，不直接访问串口。
- 当前没有修改 `mc_pos_control`、`mc_rate_control` 或电机分配逻辑。
- 后续如果要实现贴顶后覆盖 z 向推力，建议让 `mc_rate_control` 订阅 `ceiling_contact_status`，并在发布 `vehicle_thrust_setpoint` 前应用受限的 z 推力覆盖。

* Official Website: http://px4.io (License: BSD 3-clause, [LICENSE](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE))
* [Supported airframes](https://docs.px4.io/main/en/airframes/airframe_reference.html) ([portfolio](https://px4.io/ecosystem/commercial-systems/)):
  * [Multicopters](https://docs.px4.io/main/en/frames_multicopter/)
  * [Fixed wing](https://docs.px4.io/main/en/frames_plane/)
  * [VTOL](https://docs.px4.io/main/en/frames_vtol/)
  * [Autogyro](https://docs.px4.io/main/en/frames_autogyro/)
  * [Rover](https://docs.px4.io/main/en/frames_rover/)
  * many more experimental types (Blimps, Boats, Submarines, High Altitude Balloons, Spacecraft, etc)
* Releases: [Downloads](https://github.com/PX4/PX4-Autopilot/releases)

## Releases

Release notes and supporting information for PX4 releases can be found on the [Developer Guide](https://docs.px4.io/main/en/releases/).

## Building a PX4 based drone, rover, boat or robot

The [PX4 User Guide](https://docs.px4.io/main/en/) explains how to assemble [supported vehicles](https://docs.px4.io/main/en/airframes/airframe_reference.html) and fly drones with PX4. See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## Changing Code and Contributing

This [Developer Guide](https://docs.px4.io/main/en/development/development.html) is for software developers who want to modify the flight stack and middleware (e.g. to add new flight modes), hardware integrators who want to support new flight controller boards and peripherals, and anyone who wants to get PX4 working on a new (unsupported) airframe/vehicle.

Developers should read the [Guide for Contributions](https://docs.px4.io/main/en/contribute/).
See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## Weekly Dev Call

The PX4 Dev Team syncs up on a [weekly dev call](https://docs.px4.io/main/en/contribute/).

> **Note** The dev call is open to all interested developers (not just the core dev team). This is a great opportunity to meet the team and contribute to the ongoing development of the platform. It includes a QA session for newcomers. All regular calls are listed in the [Dronecode calendar](https://www.dronecode.org/calendar/).


## Maintenance Team

See the latest list of maintainers on [MAINTAINERS](MAINTAINERS.md) file at the root of the project.

For the latest stats on contributors please see the latest stats for the Dronecode ecosystem in our project dashboard under [LFX Insights](https://insights.lfx.linuxfoundation.org/foundation/dronecode). For information on how to update your profile and affiliations please see the following support link on how to [Complete Your LFX Profile](https://docs.linuxfoundation.org/lfx/my-profile/complete-your-lfx-profile). Dronecode publishes a yearly snapshot of contributions and achievements on its [website under the Reports section](https://dronecode.org).

## Supported Hardware

For the most up to date information, please visit [PX4 User Guide > Autopilot Hardware](https://docs.px4.io/main/en/flight_controller/).

## Project Governance

The PX4 Autopilot project including all of its trademarks is hosted under [Dronecode](https://www.dronecode.org/), part of the Linux Foundation.

<a href="https://www.dronecode.org/" style="padding:20px" ><img src="https://dronecode.org/wp-content/uploads/sites/24/2020/08/dronecode_logo_default-1.png" alt="Dronecode Logo" width="110px"/></a>
<div style="padding:10px">&nbsp;</div>
