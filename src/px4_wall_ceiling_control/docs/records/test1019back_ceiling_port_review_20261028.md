> 后续授权更新：已补入源版safe_transfer、共享桥接自动加载rc_startup、动态本机地址白名单，以及正常停止后保留遥控通信的适配。旧文中“未引入rc_startup/保持原safe_transfer/不动态发现地址”是首轮历史方案，已由最新实现取代。启动配置悬停推力保持0.49。详见[test1019back遥控与失联保护说明](test1019back_rc_control_and_failsafe_20261028.md)。当前仍未编译或部署。

> 最新调整：用户已要求删除侧吸“一键定高”的页面、接口、服务、消息字段和目标参数，并采用源版日志合并。原文中“保留定高/双记录/不调复制频率”是首轮方案，已由本说明取代。正常脱离的自动高度采集、恢复入口重锚和高度闭环仍保留，侧吸仍两次CH12后自动交回。日志仅保留一套实体记录，ceiling_test为同一会话的目录链接；文本日志增量快照30秒检查，session.json约2秒，CSV约10Hz，rosbag不主动降采样，正常停止补最终快照。

# test1019back 吸顶移植：共享差异确认与实施记录

目标基线：`f8187cb19d862380bd32133bd970cc2d83412ba3`；侧吸程序来源：`124495ae80464e902e2c0ba7a7966731adb1058c`。
吸顶来源：`6a8e0e38ffc866f20b3685883115ddbdd16ccd7f`。

用户已批准 CH8 选择＋CH12 触发、全局失联参数自动写入、删除旧控制台，其他项目按下表方案只适配吸顶。侧吸第三次确认、斜面与定高删除相关断言不导入。
吸顶与共享代码已完成本地适配，保留原侧吸定高和自动交回。暂不编译、不连接树莓派、不推送；ROS 构建与运行验证仍未完成。

## 只读核验与告警的含义

源版保护节点会读取三个 PX4 全局参数；数值不匹配时发送参数写入，再重新读回确认。只读方案保留读取和状态展示，取消所有写入：例如当前 COM_OF_LOSS_T 是其他值，页面显示实际值及期望2.0，不会改成2.0。
只读告警方案仍允许其他吸顶条件满足后申请接管，但必须明确标为未核验；只读阻止方案则阻止新的吸顶接管，直到现有值满足要求。二者都不会改变侧吸已有的飞控全局配置；也不保证现有失联动作就是源版要求的动作。

## 逐项差异与拟定处理

### 1. 共用控制候选消息

涉及：`msg/AttachmentControlCandidate.msg`

源版差异：源版新增 ceiling_yaw_reference_rad、ceiling_abort_land_requested、ceiling_rc_assist_blocked，并另外新增 incline_active。

实施边界：只增加三个 ceiling_* 字段；不加入 incline_active，不改旧字段、侧吸取值或判据。

影响或边界：消息结构和 ROS MD5 会变化，后续需统一 Docker 编译并重启相关节点；不能与旧消息进程混用。

### 2. 共用操作消息

涉及：`msg/OperatorCommand.msg`

源版差异：源版新增 MODE_INCLINE=5 与 pilot_handover_request_time。

实施边界：只添加交回事件时间，用于顶吸恢复 CH12 确认；不添加斜面模式，不给侧吸增加第三次确认。

影响或边界：同样涉及消息 MD5；侧吸意图保持原 mode/enable/source/reason，确认时间为零。

### 3. 构建注册

涉及：`CMakeLists.txt`

源版差异：源版注册 CeilingRcAssist、吸顶保护与测试，同时删除 SetRangeHeight 服务、旧吸顶入口并增加斜面/侧吸第三次确认测试。

实施边界：只注册吸顶新增消息、吸顶组件及适配后的吸顶测试；保留 SetRangeHeight、原侧吸测试和注册。不整文件覆盖。

影响或边界：不修改侧吸构建入口；当前依用户要求不执行编译。

### 4. 遥控模式选择

涉及：`src/px4_wall_ceiling_control/domain/momentary_switch.py；scripts/operator/terminal_operator_node.py；三个 system launch`

源版差异：新增 CH8 选择，源版默认 CH8 上拨低值选顶吸、下拨高值选侧吸；CH12 触发当前所选任务。

实施边界：用户已选择采用源版 CH8＋CH12；只新增模式选择器，不改现有 MomentarySwitch 去抖/回弹算法。

影响或边界：侧吸启动由 CH12 直接请求改为先 CH8 选侧吸，这是已明确选择的入口变化。

### 5. CH8 入口就绪互锁

涉及：`scripts/operator/terminal_operator_node.py::_on_attachment_toggle`

源版差异：源版启动任何任务都要求顶吸与侧吸两个节点同时反馈新鲜且空闲。

实施边界：建议侧吸沿用原 wall/FCU/RC/kill/odom 检查，不额外等待顶吸节点；顶吸使用新顶吸检查和既有操作意图互斥。

影响或边界：已按批准的表中方案保留原侧吸检查，不添加顶吸节点就绪门槛。

### 6. 吸顶遥控与任务意图

涉及：`scripts/operator/terminal_operator_node.py::_on_command/_apply_command/_on_state/_on_wall_toggle/_on_timer/_publish_intent`

源版差异：源版吸顶入口和交回改 POSCTL；订阅顶吸状态；顶吸 HOLD 的 CH12 请求脱离，恢复后再 CH12 确认；另有侧吸第三次确认、斜面、异步参数镜像。

实施边界：移植顶吸专用状态、POSCTL、HOLD 脱离和恢复确认；只在当前任务为顶吸时执行新分支。侧吸保持原两次 CH12 和意图语义，不复制侧吸确认、斜面或异步镜像重构。

影响或边界：CH8 在动作期间只选择下一轮，不能把本轮 detach 发给另一任务。

### 7. 模式管理与交回

涉及：`scripts/operator/flight_mode_manager_node.py`

源版差异：源版增加吸顶位置/光流/基准准入、所有权握手、融合高度恢复、第三次 CH12、新鲜 POSCTL 确认、吸顶 ABORT 降落；还加入侧吸确认和斜面映射。

实施边界：仅为 selected_mode=MODE_CEILING 接入新增路径；保留原 _captured_recovery_handover_ready、_wall_takeover_ready、_tick_wall_abort_land 和侧吸回正/交回判据。

影响或边界：共用 _entry_stream_failed、_send_mode_request、_on_timer 必须做吸顶条件隔离；源版顶吸握手可加，侧吸仍调用原检查。共用锁定/急停提示与行为也不整体覆盖。

### 8. 输出仲裁与连续性

涉及：`scripts/outputs/attachment_output_mux_node.py`

源版差异：源版新增吸顶候选 ID/序号/阶段时效校验、预发送姿态推力衔接、最终吸顶姿态包络、遥控辅助反馈；另有侧吸固定姿态禁用摇杆及斜面输出限制。

实施边界：仅 SOURCE_CEILING 或当前吸顶任务启用新增分支；侧吸 _wall_candidate_problem、_limit_wall_publication、辅助和连续性保持原版。

影响或边界：源版 _on_state/_on_fcu_attitude_target 的基准冻结会经过共用回调，须增加当前吸顶任务限定，避免吸顶残留快照影响下一轮侧吸。

### 9. 启动和参数覆盖

涉及：`launch/system/attachment_system_mavros.launch；attachment_terminal_flight.launch；wall_console_flight.launch`

源版差异：吸顶改融合高度控制、下降0.40m、HOLD推力0.55、HOLD不限时且显式detach、恢复0.50s；新增CH8和保护节点。源版还删除侧吸定高目标、合并日志和加载斜面。

实施边界：只同步吸顶命名空间的值和必要吸顶节点、CH8入口；保留每条 /wall_perch*、/wall_sensor_fusion* 参数与既有共享 hover_thrust 等值；不加载斜面，不删除定高，不合并日志。

影响或边界：共享 hover_thrust/max_thrust/急停/RC标定等不随源版改值；如后续发现吸顶要求改共享值，重新列出请用户决定。

### 10. 飞控全局失联保护

涉及：`新增 scripts/operator/ceiling_failsafe_guard_node.py 和 adapters/ceiling_failsafe.py`

源版差异：源版地面读回不匹配后会写 COM_OBL_RC_ACT=4、COM_OF_LOSS_T=2.0、COM_OBL_ACT=0，核验后公布状态。

实施边界：用户已明确选择保留源版自动写入与新鲜读回核验；保护节点仅在飞控已连接且解除武装时写入，不改侧吸ROS参数。

影响或边界：这些值是飞控全局配置，并非顶吸私有。只读不会修改现有失联处理，但也不保证实际配置已具备源版要求的降落保护。已获该项授权并迁入保护节点；当前未连接或写入真实飞控。

### 11. 控制台 ROS 桥接与运行时

涉及：`scripts/operator/wall_console_server.py；src/px4_wall_ceiling_control/wall_console.py`

源版差异：源版透传吸顶运动/遥控协调/融合高度/保护核验/确认状态，吸顶网页请求改 POSCTL；同时删定高、加斜面、改进程/日志管理。

实施边界：只增加顶吸字段和顶吸按钮准入/回执，保留侧吸序列化、set_range_height、配置/服务、原启动停止链路和权限判据。

影响或边界：顶吸屏蔽前雷达后的恢复由原 ALTCTL 确认改为 POSCTL 确认，影响后续侧吸何时可重新开始；这是适配新版顶吸交回所必需的衔接，需要确认。

### 12. 浏览器传输、会话与权限

涉及：`src/px4_wall_ceiling_control/shared_bridge.py；ceiling_wall_console/server.py；ceiling_wall_console/transport.py`

源版差异：源版改变 safe_transfer 支持模式、删除定高路由/白名单，自动安装 rc_startup，并增加动态本机地址白名单。

实施边界：保留所有原会话/控制权转移、定高路由和网络白名单；仅为顶吸新增反馈接入。若顶吸 POSCTL 下原安全移交门槛不允许，可独立增加顶吸场景，不扩大侧吸权限。

影响或边界：不照搬 rc_startup 遥控基础服务启停或共享权限改变；基线已包含 10.52.56.66 地址修复，保留。

### 13. 新版统一测试台页面

涉及：`web/ceiling_wall_console/index.html；app.js；style.css`

源版差异：源版顶吸显示光流基准接近、融合高度下降/PI控制、无限HOLD、CH12恢复确认与保护状态；也改侧吸布局说明、去定高、隐藏斜面。

实施边界：仅更新顶吸 DOM、顶吸渲染函数和必要的顶吸样式；保留侧吸按钮、定高输入/按钮/事件、侧吸状态函数和参数表。

影响或边界：页面不得把新顶吸规则套到侧吸；按任务归属隔离共享阶段提示。

### 14. 旧控制台与离线模拟

涉及：`web/wall_console/index.html；app.js；ceiling_wall_console/mock.py`

源版差异：源版旧页面吸顶状态/文字和模拟场景随新顶吸更新，同时包含侧吸功能删除。

实施边界：只更新顶吸文字/字段/模拟场景，侧吸模拟、定高与原控制按钮保留。

影响或边界：模拟仅用于页面检查，不证明飞行控制通过。

### 15. 共用日志

涉及：`scripts/test_support/attachment_flight_logger.py`

源版差异：源版增加顶吸融合高度与速度/推力字段，同时删除侧吸定高字段，合并双记录、增量复制、30秒复制间隔、重复 rosout 去重。

实施边界：建议只追加顶吸数据列；保留原侧吸记录字段、频率、目录、双记录启动和日志复制方式，不做共用性能重构。

影响或边界：新增顶吸列不改变控制命令；整文件覆盖会改变侧吸数据记录，因此不采用。

### 16. 共用输出配置和旧入口

涉及：`config/attachment_output.yaml 及旧吸顶 bench/standalone 入口`

源版差异：源版从输出 YAML 删除旧独立吸顶 adapter，并删除旧 bench/standalone 程序与 launch。

实施边界：只需新版统一链路替换吸顶；建议保留旧入口文件并标明未迁移/非本次入口，不复制配置清理。

影响或边界：不触碰共用输出数值；如要求同时删除旧入口，需另行明确范围。

### 17. 共享测试和验证

涉及：`test/test_flight_handover.py；test_attachment_safety.py；test_rc_wall_toggle.py；test_wall_console.py；test_shared_bridge.py；test_ceiling_wall_console.py 等`

源版差异：源版既有新增吸顶断言，也包含第三次侧吸 CH12、斜面和删除定高的断言。

实施边界：保留原侧吸测试断言并新增 CH8 已授权入口的覆盖；吸顶使用来源专用测试及必要适配，不用源版整套覆盖。

影响或边界：后续必须区分本地静态/页面测试与真实 ROS Docker 编译。当前保持暂不编译、不部署。

## 明确保留的侧吸主体

`config/wall_perch.yaml`、`config/wall_sensor_fusion.yaml`、`msg/WallPerchStatus.msg`、`srv/SetRangeHeight.srv`、侧吸决策节点、`adapters/wall_*`、侧吸相关 domain 控制器和原参数值保持目标基线。
不导入源版侧吸第三次 CH12、固定姿态等待、删除定高、斜面控制或侧吸诊断缓存优化。

## 当前状态

已完成本地适配。侧吸控制/config/domain/service 共17个受保护文件逐字一致；三个 system launch 中原有非吸顶参数值及条件一致。共享模式管理器和输出仲裁保留原侧吸专用方法，新增路径限定顶吸。已批准的 CH8 选择改变遥控入口；全局失联参数写入改变飞控全局失联配置，这两项不属于侧吸控制算法改值。

旧 `web/wall_console` 页面及其 HTTP 服务入口已删除，共用 bridge 后端保留。新版侧吸定高按钮、路由/白名单、服务和字段保留。未引入 rc_startup、斜面或侧吸第三次确认代码/断言。

吸顶源版本采用第三次 CH12 恢复交回；侧吸仍两次 CH12 后稳定自动交回。后续实际部署前必须按规定完成统一 Docker 编译与相关节点重启，不能混用旧消息。
