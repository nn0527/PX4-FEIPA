> 后续授权更新：已补入源版safe_transfer、共享桥接自动加载rc_startup、动态本机地址白名单，以及正常停止后保留遥控通信的适配。旧文中“未引入rc_startup/保持原safe_transfer/不动态发现地址”是首轮历史方案，已由最新实现取代。启动配置悬停推力保持0.49。详见[test1019back遥控与失联保护说明](test1019back_rc_control_and_failsafe_20261028.md)。当前仍未编译或部署。

# test1019back 吸顶移植与定高移除：本地验证

侧吸基线：`f8187cb`（程序主体来自 `124495a`）；吸顶来源：`6a8e0e3`。
本轮按用户要求移除人工一键定高，并采用 `24b1dc6` 中的专用移除补丁，未导入后续侧吸缓存/第三次CH12/斜面变化。

- 已删除定高按钮、输入、HTTP路由、动作白名单项、ROS服务及目标参数；旧保存参数仅在迁移时丢弃，不进入运行配置。
- `wall_detach_vertical.py`、`range_height_feedback.py` 与历史专用移除版本一致。侧吸决策节点仅去掉人工定高完成标记；正常恢复、水平控制和融合参数保持原版。
- 34个共享文件内侧吸专用方法仍与基线一致。侧吸普通自动测距高度闭环保留。
- 日志采用 `6a8e0e3`：一套实体记录＋ceiling_test目录链接，文本快照由2秒全量复制改为30秒增量检查，停止补最终快照；session.json约2秒、CSV约10Hz、rosbag原始话题采集及30秒分包保持原设置。
- 本地测试通过：控制台状态76项、HTTP/会话27项、共享桥接9项、记录器12项、测距高度26项，共150项；7组网页测试通过；20个离线场景可生成反馈。
- Python AST、launch XML、git diff空白检查通过；已确认启动链路仅一个记录节点。

按用户要求未编译、未部署、未提交或推送。当前缺少rospy，ROS控制节点回归未运行；上述不等于Docker构建或实飞验证。全局失联保护只是接入源码，未对真实飞控写参数。

## 已核对的侧吸方法

- `scripts/operator/flight_mode_manager_node.py::_wall_fusion_required`
- `scripts/operator/flight_mode_manager_node.py::_wall_fusion_ready`
- `scripts/operator/flight_mode_manager_node.py::_wall_fusion_handover_ready`
- `scripts/operator/flight_mode_manager_node.py::_wall_horizontal_ready`
- `scripts/operator/flight_mode_manager_node.py::_auto_wall_selected`
- `scripts/operator/flight_mode_manager_node.py::_wall_takeover_ready`
- `scripts/operator/flight_mode_manager_node.py::_manual_wall_selected`
- `scripts/operator/flight_mode_manager_node.py::_manual_wall_ready`
- `scripts/operator/flight_mode_manager_node.py::_wall_approach_timeout_requested`
- `scripts/operator/flight_mode_manager_node.py::_wall_timeout_handover_failure`
- `scripts/operator/flight_mode_manager_node.py::_wall_timeout_handover_ready`
- `scripts/operator/flight_mode_manager_node.py::_wall_return_requires_exit`
- `scripts/operator/flight_mode_manager_node.py::_wall_range_mode`
- `scripts/operator/flight_mode_manager_node.py::_wall_range_identity`
- `scripts/operator/flight_mode_manager_node.py::_captured_recovery`
- `scripts/operator/flight_mode_manager_node.py::_wall_range_feedback_ready`
- `scripts/operator/flight_mode_manager_node.py::_wall_return_exit_ready`
- `scripts/operator/flight_mode_manager_node.py::_captured_recovery_handover_ready`
- `scripts/operator/flight_mode_manager_node.py::_wall_return_reason`
- `scripts/operator/flight_mode_manager_node.py::_wall_approach_aborted`
- `scripts/operator/flight_mode_manager_node.py::_wall_flip_started`
- `scripts/operator/flight_mode_manager_node.py::_wall_land_status_identity`
- `scripts/operator/flight_mode_manager_node.py::_wall_abort_land_live`
- `scripts/operator/flight_mode_manager_node.py::_tick_wall_abort_land`
- `scripts/outputs/attachment_output_mux_node.py::_wall_candidate_problem`
- `scripts/outputs/attachment_output_mux_node.py::_wall_control`
- `scripts/outputs/attachment_output_mux_node.py::_roll_yaw_limits`
- `scripts/outputs/attachment_output_mux_node.py::_wall_snapshot_fresh`
- `scripts/outputs/attachment_output_mux_node.py::_wall_fresh`
- `scripts/outputs/attachment_output_mux_node.py::_preflip_bias_attitude`
- `scripts/outputs/attachment_output_mux_node.py::_limit_final_approach_target`
- `scripts/outputs/attachment_output_mux_node.py::_limit_final_recovery_target`
- `scripts/outputs/attachment_output_mux_node.py::_wall_publication_seed`
- `scripts/outputs/attachment_output_mux_node.py::_limit_wall_publication`
