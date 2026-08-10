# PX4 Gazebo x500 仿真指南

本文档记录本仓库中 PX4 SITL、Gazebo Harmonic 和 `gz_x500` 的常用启动与故障恢复命令。

## 1. 首次配置

```bash
cd /home/fn/PX4-bridge-inspection
git submodule update --init --recursive
```

确认子模块完整；正常情况下以下命令不应输出内容：

```bash
git submodule status | awk '$1 ~ /^[-+U]/ {print}'
```

## 2. 仿真场景

### 默认场景

默认 world 只有地面：

```bash
make px4_sitl gz_x500
```

### bridge 场景

`bridge.sdf` 包含：

- 两面长 20 m、高 7 m、厚 0.2 m 的侧墙；
- 10 m 净宽；
- 下表面高 7 m 的天花板；
- 开放的前后两端，无桥墩和巡检通道。

普通 `x500` 只用于查看场景，没有本项目的双测距仪：

```bash
PX4_GZ_WORLD=bridge make px4_sitl gz_x500
```

执行 Ceiling/Wall 完整仿真时必须使用带真实 UP/FRONT ray sensor 的专用目标：

```bash
make px4_sitl gz_x500_dual_range_bridge
```

该目标不会改变 `bridge.sdf`。天花板下表面仍为约 7 m，两侧墙面仍位于
Gazebo Y 轴约 ±5.1 m。

## 3. VMware 中启动 GUI

专用机型固定使用Ogre1。原因是VMware虚拟显卡运行Ogre2/OgreNext时可能直接
`SIGSEGV`，Ubuntu会显示“应用程序ruby3.2意外停止”。`/usr/bin/gz`本身是
Ruby启动脚本；实际崩溃位置是`libOgreNextMain`和
`libgz-rendering8-ogre2`，不是PX4控制代码、UART或双测距数据链崩溃。

模型另外包含一层无碰撞体、动力学影响可忽略的SDF基础几何外观，用于解决Ogre1
下原始x500 COLLADA机身偶尔透明、只剩旋翼和选中框的问题。正常启动只需：

```bash
make px4_sitl gz_x500_dual_range_bridge
```

若刚刚出现过`ruby3.2`崩溃框，点击“不发送”关闭即可。先结束该次启动可能残留的
PX4/Gazebo进程，再重新运行上面的`make`命令：

```bash
pkill -TERM -x px4
pkill -TERM -f "gz sim"
```

不需要删除`build`，渲染器崩溃与编译缓存无关。如果窗口仍然灰屏，改用两终端
headless server + Ogre1 GUI方式。

启动时PX4终端必须出现下面这一行，才能确认实际加载的是本仓库的`bridge.sdf`：

```text
INFO  [init] Starting gazebo with world: .../Tools/simulation/gz/worlds/bridge.sdf
```

如果已有`default`或其他world还在运行，启动程序现在会明确报错并停止，不再静默
把专用机型生成到旧world中。先执行上面的两个`pkill`命令，确认下面两条命令均无
输出，再重新启动：

```bash
pgrep -a px4
gz topic -l | grep '^/world/'
```

终端 1 启动 PX4 和 Gazebo server：

```bash
cd /home/fn/PX4-bridge-inspection
HEADLESS=1 CCACHE_DISABLE=1 make px4_sitl gz_x500_dual_range_bridge
```

专用机型使用Gazebo物理引擎的body-frame raycast，并发布标准LaserScan话题，
不依赖GPU lidar或GUI渲染。因此即使GUI暂时没有打开，headless server中的
UP/FRONT数据仍可正常更新。该目标默认使用DART+Bullet碰撞检测；专用模型的
基础几何机身在Ogre1下可见，不依赖原始COLLADA/PBR机身是否能渲染。

出现 `Ready for takeoff!` 后，终端 2 启动 GUI：

```bash
cd /home/fn/PX4-bridge-inspection
source build/px4_sitl_default/rootfs/gz_env.sh
GZ_IP=127.0.0.1 gz sim -g --render-engine-gui ogre
```

两个终端都需保持运行。

## 4. Ceiling/Wall 交互仿真

PX4和Gazebo正常启动并出现`Ready for takeoff!`后，在另一个终端运行：

```bash
cd /home/fn/PX4-bridge-inspection
python3 Tools/bridge_interactive_sitl.py
```

脚本读取Gazebo的真实UP和FRONT测距话题，再模拟ESP32，通过以下完整链路送入控制器：

```text
Gazebo LaserScan
  -> AA 55 UP_H UP_L FWD_H FWD_L CRC 0D 0A
  -> PTY -> uart_rx -> esp32_uart_frame
  -> CeilingReader -> distance_sensor多实例
  -> ceiling_controller / wall_perch
```

脚本不会直接发布`distance_sensor`，也不会覆盖QGC参数。它会发送GCS心跳，
所以不打开QGC也可以通过PX4的正常健康检查并解锁。

### Ceiling流程

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

`prepare`使用PX4本地高度上升到相对地面`6.0 ± 0.05 m`并悬停。UP只用于
显示真实天花板距离和后续吸顶决策，不作为这次起飞的停止条件。AUX1开始吸顶，
AUX2执行脱离。输入`aux2 on`后，DETACH仍由UP距离PID直接减推；交互工具同时将
DETACH结束后的Offboard恢复高度设置为当前高度下方1 m，避免恢复标准位置控制时
重新执行原来的6 m目标并撞回天花板。脱离后必须先将AUX1关闭，才能再次开启吸顶。

### Wall流程

```text
bridge> mode wall
bridge> prepare
bridge> status
bridge> aux1 on
bridge> aux2 on
bridge> aux2 off
bridge> abort
bridge> land
```

Wall的`prepare`先飞到1.5 m，再朝左墙转向，并根据真实FRONT距离慢速移动到
约0.45 m处。UP从不冻结或覆盖：水平飞行时测量天花板，翻转后随机体转向墙面，
供`wall_perch`判断顶部接触。AUX1开始、AUX2脱离、`abort`发送AUX3中止脉冲。

Ceiling和Wall模式互斥；`mode ceiling`会停止`wall_perch`，`mode wall`会停止
`ceiling_controller`，避免两个模块同时使用AUX1或发布控制设定值。

### QGC参数

控制参数仍在QGC的Parameters页面修改和保存。重点参数包括：

- Ceiling：`CEIL_APPR_START`、`CEIL_DIST_THR`、`CEIL_D0`、`CEIL_APPR_VZ`、
  `CEIL_DET_THR_MIN`和`CEIL_DET_TO`。
- Wall：`WP_FRN_RD_DIST`、`WP_FLP_TRD_DIST`、`WP_TOP_CT_DIST`、
  `WP_THR_APPROACH`、`WP_SENS_TIMEOUT`和各阶段时间参数。

交互脚本启动时只读取参数并报告不兼容项。若QGC保存值与专用airframe默认值不同，
QGC保存值优先；修改参数后可输入`params`重新检查。专用airframe为终端AUX设置
`COM_RC_IN_MODE=2`，并按GZ x500约0.72的悬停推力设置
`WP_THR_APPROACH=0.74`；如果QGC中曾保存其他值，应以实际飞行表现重新调整。

## 5. 停止与重新启动

优先在 PX4 控制台输入：

```text
shutdown
```

也可以先在 GUI 终端按 `Ctrl+C`，再在 PX4 终端按 `Ctrl+C`，等待出现 `PX4 Exiting...`。

若原终端已经丢失，清理残留进程：

```bash
pkill -TERM -x px4
pkill -TERM -f "gz sim"
```

确认进程已经退出：

```bash
pgrep -a px4
pgrep -af "gz sim"
```

没有输出后再运行启动命令。若看到下面的信息，说明 instance 0 仍被旧 PX4 占用：

```text
PX4 server already running for instance 0
```

不要只删除 `/tmp/px4-sock-0`，应先正常结束旧进程。切换 `default` 和 `bridge` world 前也必须完整停止旧仿真。

## 6. 修改代码后重新运行

修改 C/C++、uORB 消息、参数 YAML、CMake、Kconfig、airframe或Gazebo模型后，
先在交互工具输入`land`和`quit`，再结束PX4/Gazebo，最后重新执行：

```bash
make px4_sitl gz_x500_dual_range_bridge
```

Ninja会自动增量编译，通常无需删除`build`。

只修改 Python 测试脚本或在 PX4 控制台调整参数时，不需要重新编译 PX4。

如果出现`unknown target 'gz_x500_dual_range_bridge'`，先原地重新生成构建配置：

```bash
CCACHE_DISABLE=1 cmake \
  -S . \
  -B build/px4_sitl_default \
  -G Ninja \
  -DCONFIG=px4_sitl_default
```

确认目标恢复：

```bash
ninja -C build/px4_sitl_default -t targets | grep '^gz_x500_dual_range_bridge:'
```

只有在CMake缓存损坏、消息代码未重新生成或模型资源持续使用旧版本时，才删除
SITL的单个构建目录：

```bash
rm -rf /home/fn/PX4-bridge-inspection/build/px4_sitl_default
make px4_sitl gz_x500_dual_range_bridge
```

不要删除整个仓库、`Tools/simulation/gz`或用户参数文件；也不要在仿真进程仍运行时
删除构建目录。

## 7. 常用检查

检查 PX4、Gazebo 和当前 world：

```bash
pgrep -af 'build/px4_sitl_default/bin/px4|gz sim'
gz topic -l | grep '^/world/'
```

检查`bridge`中的专用x500是否已生成：

```bash
gz service -s /world/bridge/scene/info \
  --reqtype gz.msgs.Empty \
  --reptype gz.msgs.Scene \
  --timeout 3000 \
  --req '' | grep 'name: "x500_dual_range_0"'
```

检查两个Gazebo测距话题：

```bash
gz topic -l | grep '^/bridge/x500_dual_range/'
gz topic -e -t /bridge/x500_dual_range/up -n 1 --json-output
gz topic -e -t /bridge/x500_dual_range/front -n 1 --json-output
```

在 PX4 `pxh>` 中检查本项目模块：

```text
ceiling_controller status
wall_perch status
uart_rx status
listener ceiling_contact_status -n 1
listener distance_sensor -i 0 -n 1
listener distance_sensor -i 1 -n 1
```

正常运行至少应满足：PX4显示`Ready for takeoff!`、Gazebo Entity Tree中存在
`x500_dual_range_0`、两个LaserScan话题持续更新，并且`uart_rx`和`CeilingReader`
均为`running`。无人机初始朝向桥梁开放端时，FRONT帧会是`65535`且FRONT实例不
发布，这是正确的单路无效行为；执行Wall `prepare`转向墙面后会出现
`orientation=0`的FRONT实例。
