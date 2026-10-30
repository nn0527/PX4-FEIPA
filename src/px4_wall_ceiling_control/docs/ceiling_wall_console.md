> 后续授权更新：已补入源版safe_transfer、共享桥接自动加载rc_startup、动态本机地址白名单，以及正常停止后保留遥控通信的适配。旧文中“未引入rc_startup/保持原safe_transfer/不动态发现地址”是首轮历史方案，已由最新实现取代。启动配置悬停推力保持0.49。详见[test1019back遥控与失联保护说明](test1019back_rc_control_and_failsafe_20261028.md)。当前仍未编译或部署。

> 最新调整：用户已要求删除侧吸“一键定高”的页面、接口、服务、消息字段和目标参数，并采用源版日志合并。原文中“保留定高/双记录/不调复制频率”是首轮方案，已由本说明取代。正常脱离的自动高度采集、恢复入口重锚和高度闭环仍保留，侧吸仍两次CH12后自动交回。日志仅保留一套实体记录，ceiling_test为同一会话的目录链接；文本日志增量快照30秒检查，session.json约2秒，CSV约10Hz，rosbag不主动降采样，正常停止补最终快照。

> 本分支更新：吸顶同步自 `6a8e0e3`，采用POSCTL、融合高度下降0.40m和恢复后CH12确认；侧吸保持 `124495a` 的两次CH12、稳定自动交回及一键定高。旧8787前端与Windows旧入口已删除，保留本页面和共用桥接。下文旧行为说明若涉及ALTCTL或气压主闭环，以本段及吸顶专用说明为准。

# ceiling_wall_console · 顶吸与侧吸测试台

统一维护树莓派原 8790 端口的多设备测试台，源码位于 `ros_ws` 的同一个 ROS 包中。
页面、登录和连接模块从树莓派当前部署版本迁入；不再携带另一份飞行程序或控制台后端。
真实链路复用 `wall_console.py`、`wall_console_server.py` 和 `shared_bridge.py`。

## 本机离线修改与预览

在仓库根目录双击 `Start-CeilingWallConsole.command`（Mac）或
`Start-CeilingWallConsole.cmd`（Windows，需要 Python 3.8+）。也可运行：

```sh
python3 -B src/px4_wall_ceiling_control/scripts/operator/ceiling_wall_console.py --bridge mock --port 8791 --open-browser
```

浏览器打开 `http://127.0.0.1:8791/login`，离线访问码是 `offline`。
先填写设备名称、登录、申请控制权，再使用黄色“离线模拟”区的场景选择器。
可以查看顶吸入口/接近/保持/气压下降/ALTCTL交回，以及侧吸保持、测距三个阶段、
测距失效、融合恢复、人工定高、异常后恢复、急停、反馈过期和断线。

离线模式只绑定本机回环地址，使用内存中的配置和模拟后端；不读取或覆盖真实访问码/配置，
不调用 ROS、Docker 或 SSH。模拟按钮只改变页面场景，不能用于验证控制器动力学或实飞。
可以手动切换下一阶段；不会将一次按钮回执自动解释为飞行完成。
真机模式没有 `/api/mock-scenario` 路由，也不会导入模拟后端。

## 文件位置

| 内容 | 包内路径 |
| --- | --- |
| 页面、登录及 IP 选择器 | `web/ceiling_wall_console/` |
| 启动入口 | `scripts/operator/ceiling_wall_console.py` |
| HTTP、会话、连接和模拟模块 | `src/px4_wall_ceiling_control/ceiling_wall_console/` |
| 树莓派用户服务 | `config/systemd/ceiling_wall_console.service` |
| HTTP 与离线回归测试 | `test/test_ceiling_wall_console.py` |
| 页面状态与连接测试 | `test/ceiling_wall_console/*.cjs` |

修改 HTML / CSS / JavaScript 后刷新页面即可；修改 Python 后重新启动本地预览。
代码在 Mac/Windows 上直接运行，不需要安装 ROS 或连接树莓派。

## 后端模块与旧控制台

旧 8787 `wall_console` 页面、专用 HTTP 入口、Windows GUI 及打包启动脚本已在当前分支移除，仅保留本页（`ceiling_wall_console`，树莓派 8790 / 本机预览 8791）这一套测试台。

`wall_console_server.py`、`wall_console.py` 不是独立测试台，而是本页复用的后端：前者是控制台 worker（HTTP 桥接、节点与串口检查、配置与启停），后者是 ROS 桥接与状态检查逻辑。
直接运行 `wall_console_server.py` 会提示“旧控制台已删除”，它只接受 `--bridge` / `--shared-service`。

## 树莓派真实运行

网页服务仍运行在树莓派宿主机；ROS 仍在 `ros1_noetic` 容器内。
每个登录浏览器有独立会话，借助原共享桥接服务使用同一个 ROS 运行时；
登录仅观察，不自动获得控制权，也不发送解锁、起飞或飞行动作。

```sh
python3 -B /home/pi/ros_ws/src/px4_wall_ceiling_control/scripts/operator/ceiling_wall_console.py --bridge local --port 8790 --public-host 192.168.2.20
```

用户服务改名为 `ceiling_wall_console.service`，服务文件指向上述工作区入口。
访问地址及原 8790 端口保持兼容，IP 选择器保留原地址并增加 `192.168.2.20`。
新的配置目录为 `~/.config/ceiling_wall_console/`。
若其中尚无相应文件，自动复用原 `~/.config/ceiling-launchpad/access-code` 和 `settings.json`；
因此原访问码、串口、波特率、悬停推力和试验模式继续有效。
2026-10-21 清理旧部署时，现用访问码和配置已迁入新目录，旧配置目录已删除。
访问码、配置、会话及运行日志不复制到 Git。服务更新后浏览器重新登录。

旧 `ceiling-launchpad.service`、服务覆盖配置、`/home/pi/ceiling-launchpad-app/` 和
`/home/pi/ceiling-launchpad-lan-release.tar.gz` 已于 2026-10-21 按用户要求删除。
当前仅运行工作空间中的新服务；独立的 `codex_backups` 历史备份保留。

## 保留功能与新适配

- 保留登录访问码、主机/来源/CSRF 检查、登录限速、独立设备会话、单控制端、观察端、
  控制权交接、单设备暂停反馈、并发请求保护及断线不重放动作。
- 保留配置启动、MAVROS 修复、顶吸/离顶、贴墙/离墙、取消贴墙等待及停止基础链路。
  按钮继续调用原桥接动作和原飞行服务，原安全条件由控制节点判断。
- 在线和离线测试台均移除独立“一键遥控接管”按钮、点击逻辑、HTTP 接口和模拟场景。
  正常离墙/离顶完成后的自动交回流程及状态反馈继续保留。
- 顶吸继续使用气压下降量及 ALTCTL 交回；侧吸新增测距任务阶段、原始距离、几何高度、
  固定目标、测距速度、首帧证据、有效性、连续稳定与阻挡原因。
- 固定目标默认 1.5 m，启动前设置并随本轮冻结。定高按钮使用节点的可用标志和权限检查；
  每轮一次、默认 15 秒预算，保留原故障及首帧，异常后的人工恢复与正常离墙完成分别显示。
- 光流面板显示原值/目标/确认值和恢复原因，沿用实际倾角≤80°请求恢复的现行逻辑。
  参数确认与飞控实际融合效果分开显示，不修改飞行控制算法。

## 验证

```sh
python3 -B -m unittest discover -s src/px4_wall_ceiling_control/test -p test_ceiling_wall_console.py -v
node src/px4_wall_ceiling_control/test/ceiling_wall_console/test_ui.cjs
node src/px4_wall_ceiling_control/test/ceiling_wall_console/test_wall_ui.cjs
node src/px4_wall_ceiling_control/test/ceiling_wall_console/test_range_ui.cjs
node src/px4_wall_ceiling_control/test/ceiling_wall_console/test_connect.cjs
```

修改 ROS 工作区程序后，仍须在树莓派 Docker 内编译：

```sh
docker exec ros1_noetic bash -lc 'source /opt/ros/noetic/setup.bash && cd /root/ros_ws && catkin_make'
```

## 2026-10-21 迁移验证结果

- 迁移以树莓派 `/home/pi/ceiling-launchpad-app/current` 的实际源码作为基准；原目录当时保留，随后按用户要求清理。
- 现场原有 8 项 HTTP 测试中，1 项旧断言误将当前 IP 选择页当作登录页；迁移按实际连接→登录流程修正该断言。
  原 HTTP 回归用例与本机已有的权限/接管用例合并，扩展为 26 项测试，全部通过。
- 相关 ROS / 测距 / 控制台回归合计 214 项通过；页面及连接状态共 144 项断言通过。
- 全量 ROS 单元测试 865 项中有 13 项已有问题（6 项断言失败、7 项旧夹具错误），
  与修改前独立基线逐项对照一致，没有新增失败；详细日志在本机 `output/20261021-ceiling-wall-console/`。
- 树莓派 `ros1_noetic` 内按工作区规定执行 `catkin_make` 成功。
- `ceiling_wall_console.service` 已启用并运行，旧服务停用；8790 端口与原访问码继续使用。
- 两个真实 HTTP 观察端已验证独立登录、独立令牌、实时桥接、新配置兼容及静态页面可用。
  未申请控制权，未提交任何飞行动作，也没有启动或重启飞行链路。
- 本机浏览器验证了离线登录、控制权、场景切换和定高：目标从 1.24 变为 1.50 m，
  当前测距高度 1.10 m、首帧 1.24 m 均保留，重复按钮禁用。
- 原服务及工作区文件备份在 `/home/pi/codex_backups/ceiling_wall_console-20261021/`。
  旧服务及旧部署现已清理，不能直接重新启用；如需回退，须先从相应历史备份恢复所需文件。

## 2026-10-21 旧部署清理

- 删除旧程序目录、旧发布包、旧用户服务及其覆盖目录；系统服务重新加载后，旧服务为 `not-found`。
- 将现用访问码和配置迁入 `~/.config/ceiling_wall_console/`，逐字节核对一致，文件权限为 `0600`；删除迁空的旧配置目录。
- 仅重启网页服务，以原访问码登录观察端，确认保存参数一致、页面可用；新服务为 `active` / `enabled`。
- 未修改飞行程序，未启动飞行链路，也未发送飞行动作。


## MAVROS 地面恢复（2026-10-21）

页面“重启 MAVROS（地面恢复）”取代原“修复 MAVROS 连接”。确认飞机已落地、上锁且电机停止后，服务先停止并核验旧控制、MAVROS、雷达和日志，再重启唯一 MAVROS 实例，等待新的未解锁飞控心跳。恢复成功后手动点击“一键配置并启动”，不会恢复旧的飞行动作请求。

基础链路仍标记运行、控制阶段未结束、过期 armed=true 缓存，以及本页面其他请求等待回执，不再要求用户先操作另一个灰色按钮。新鲜反馈显示飞控仍解锁时仍拒绝恢复。无控制权时，仅在飞控反馈失效、无控制端或原控制端已断线时允许经人工落地确认执行恢复，不授予飞行动作控制权；其他在线控制端且飞控连接正常时仍由原控制端操作。暂停推送不阻止地面恢复。

停止按钮仅使用新鲜飞控状态判断解锁，过期状态改由落地确认处理。按钮下方显示禁用原因。地面恢复使用独立的页面/HTTP请求状态；页面渲染异常也会释放请求忙标记。反馈服务本身不可达时不能发出恢复指令，页面会明确显示连接原因。
