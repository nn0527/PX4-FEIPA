# 测试台：ceiling_wall_console

统一维护树莓派 8790 端口的多设备测试台，页面与后端都在本 ROS 包内，不再携带另一份
飞行程序。真实链路复用 `wall_console.py`、`wall_console_server.py` 与 `shared_bridge.py`。

- 页面与登录：`web/ceiling_wall_console/`
- 启动入口：`scripts/operator/ceiling_wall_console.py`
- HTTP/会话/连接/模拟：`src/px4_wall_ceiling_control/ceiling_wall_console/`
- 系统服务：`config/systemd/ceiling_wall_console.service`
- 测试：`test/test_ceiling_wall_console.py`、`test/ceiling_wall_console/*.cjs`

## 1. 本机离线预览

在仓库根目录双击 `Start-CeilingWallConsole.command`（Mac）或
`Start-CeilingWallConsole.cmd`（Windows，需 Python 3.8+），或运行：

```sh
python3 -B src/px4_wall_ceiling_control/scripts/operator/ceiling_wall_console.py \
  --bridge mock --port 8791 --open-browser
```

浏览器打开 `http://127.0.0.1:8791/login`，离线访问码为 `offline`。流程是：
填写设备名称 → 登录 → 申请控制权 → 使用黄色“离线模拟”区的场景选择器。

离线模式只绑定本机回环地址，使用内存配置与模拟后端，不读取或覆盖真实访问码/配置，
不调用 ROS、Docker 或 SSH。模拟按钮只改变页面场景，不能验证控制器动力学或实飞；
不会把一次按钮回执自动解释为飞行完成。真机模式没有 `/api/mock-scenario` 路由，
也不会导入模拟后端。

可查看的顶吸场景：入口、接近、保持、气压/融合下降、模式交回；侧吸场景：保持、
测距三阶段、测距失效、融合恢复、人工定高、异常后恢复、急停、反馈过期与断线。

## 2. 树莓派真实运行

网页服务运行在树莓派宿主机，ROS 运行在 `ros1_noetic` 容器内。每个登录浏览器有独立
会话，通过共享桥接服务使用同一个 ROS 运行时；登录只观察，不自动获得控制权，也不
发送解锁、起飞或飞行动作。

```sh
python3 -B /home/pi/ros_ws/src/px4_wall_ceiling_control/scripts/operator/ceiling_wall_console.py \
  --bridge local --port 8790 --public-host 192.168.2.20
```

- 访问码、配置、会话与运行日志不复制到 Git；配置目录 `~/.config/ceiling_wall_console/`，
  缺失时自动复用旧 `~/.config/ceiling-launchpad/access-code` 与 `settings.json`。
- 服务更新后浏览器需重新登录。IP 选择器保留原地址并增加 `192.168.2.20`。
- 保存的启动配置包含飞控/顶部/前向串口、230400 波特、悬停推力与试验模式；
  “一键配置并启动”调用 `wall_console_flight.launch`（见 [wall](wall.md) 与 [ceiling](ceiling.md)）。

## 3. 保留与已删除

保留：登录访问码、主机/来源/CSRF 检查、登录限速、独立设备会话、单控制端与观察端、
控制权交接、并发请求保护、断线不重放动作、配置启动、MAVROS 地面恢复、
顶吸/离顶、贴墙/离墙、取消贴墙等待与停止基础链路。

已删除（不要按旧说明操作）：

- 旧 8787 `wall_console` 页面、专用 HTTP 入口、Windows GUI 与打包启动脚本、
  旧 `ceiling-launchpad.service` 与 `/home/pi/ceiling-launchpad-app/`。
- 在线与离线测试台中的独立“一键遥控接管”按钮与 `/api/mock-scenario` 之外的接管接口；
  正常离墙/离顶完成后的自动交回流程与状态反馈继续保留。
- 侧吸“一键定高”的页面、接口、服务、消息字段与目标参数。

`wall_console_server.py` 不是独立测试台入口，直接运行只会提示旧控制台已删除；
它只接受 `--bridge` / `--shared-service`，作为本页后端 worker 使用。

## 4. 后端模块

| 模块 | 作用 |
|---|---|
| `wall_console.py` | 与控制台状态、端口/配置校验、启动参数生成、状态快照与事件；不依赖 ROS 与浏览器 |
| `wall_console_server.py` | ROS 节点 `wall_console_bridge`：订阅状态、发布 `/attachment/terminal_command` 与 `/wall_perch/cancel_wait`、节点/串口检查与启停 |
| `shared_bridge.py` | 宿主机进程与容器内 worker 的 Unix socket 桥接、会话与控制权仲裁 |
| `ceiling_wall_console/` | FastAPI 风格 HTTP 服务、登录、会话、静态页面与离线模拟后端 |

状态新鲜度与显示限制统一定义在 `wall_console.py` 的 `LIMITS`；页面只显示未过期数据。

## 5. MAVROS 地面恢复

页面“重启 MAVROS（地面恢复）”取代原“修复 MAVROS 连接”。确认飞机已落地、上锁且电机
停止后，服务先停止并核验旧控制、MAVROS、雷达与日志，再重启唯一 MAVROS 实例，
等待新的未解锁飞控心跳。恢复成功后需手动点击“一键配置并启动”，不会恢复旧动作请求。

- 基础链路仍标记运行、控制阶段未结束、过期 `armed=true` 缓存或其他请求等待回执时，
  不再要求先操作另一个灰色按钮。
- 新鲜反馈显示飞控仍解锁时仍拒绝恢复。
- 无控制权时，只在飞控反馈失效、无控制端或原控制端已断线时允许经人工落地确认执行；
  其他在线控制端且飞控正常时仍由原控制端操作。
- 停止按钮只用新鲜飞控状态判断解锁，过期状态改由落地确认处理；按钮下方显示禁用原因。
- 反馈服务不可达时不能发出恢复指令，页面显示连接原因。

## 6. 文件位置

| 内容 | 包内路径 |
|---|---|
| 页面、登录与 IP 选择器 | `web/ceiling_wall_console/` |
| 启动入口 | `scripts/operator/ceiling_wall_console.py` |
| HTTP、会话、连接与模拟 | `src/px4_wall_ceiling_control/ceiling_wall_console/` |
| 用户服务 | `config/systemd/ceiling_wall_console.service` |
| HTTP 与离线回归 | `test/test_ceiling_wall_console.py`、`test/ceiling_wall_console/*.cjs` |

修改 HTML/CSS/JS 后刷新页面即可；修改 Python 后重启本地预览。代码在 Mac/Windows
可直接运行，不需要安装 ROS 或连接树莓派。

## 7. 验证

```sh
python3 -B -m unittest discover -s src/px4_wall_ceiling_control/test -p test_ceiling_wall_console.py -v
node src/px4_wall_ceiling_control/test/ceiling_wall_console/test_ui.cjs
node src/px4_wall_ceiling_control/test/ceiling_wall_console/test_wall_ui.cjs
node src/px4_wall_ceiling_control/test/ceiling_wall_console/test_range_ui.cjs
node src/px4_wall_ceiling_control/test/ceiling_wall_console/test_connect.cjs
```

修改 ROS 工作区程序后仍须在树莓派容器内编译（命令见 [terminal](terminal.md) 第 9 节）。

## 8. 历史迁移摘要

2026-10-21 迁移以树莓派 `ceiling-launchpad-app/current` 实际源码为基准：

- 8 项旧 HTTP 测试中 1 项把 IP 选择页误当登录页，按实际“连接→登录”流程修正；
  与本地权限/接管用例合并后扩展到 26 项，全部通过。
- 相关 ROS/测距/控制台回归合计 214 项通过；页面与连接状态 144 项断言通过。
- 全量 ROS 单元测试 865 项中 13 项为既有问题（6 断言失败、7 旧夹具错误），
  与修改前基线逐项一致。
- 旧部署目录、发布包与用户服务已清理，不能直接重新启用；回退需从历史备份恢复。

现场与详细记录见 [records](records/)。
