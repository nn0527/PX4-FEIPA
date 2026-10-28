# 顶吸异常 ABORT / AUTO.LAND 修改与验证

接触安全异常优先于脱离请求，直接进入 ABORT。实际配置的全部顶吸 ABORT（含脱离下降、恢复悬停、接管上升的断连/EKF 磁异常）锁存降落意图，不再等待正常三次 CH12 交接。地面失联参数守护仅在连接且未解锁时设置并读回确认，未核验时阻止顶吸接管。通信断开后不把飞控状态默认的 armed=false 当作已落地解除武装，不丢弃降落意图；实际新鲜 AUTO.LAND 模式反馈才确认降落。

## 验证

- 树莓派现有 `ros1_noetic` 容器内运行规定 `catkin_make`，成功；未在树莓派宿主机编译。
- 顶吸阶段故障、断连/重连、磁创新/姿态故障、地面参数设置保护、现有 HOLD 中止、运动闭环、融合高度与第三次 CH12 等 88 项测试通过。
- Node 控制台 5 个测试文件全部通过，增加断连时不能显示“已确认 AUTO.LAND”的断言。
- 全量 unittest 1413 项：5 failures、8 errors。与 `/tmp/test1019-fused-height-all.log` 基线比较，13 个失败名称完全一致，没有新增失败；全量未全部通过。
- 单独启动守护节点进行未连接状态检查：正常发布 ready=false 和“未核验，禁止新的顶吸接管”；未发送解锁或飞行模式命令。
- 20:57 后现场检查：`/dev/serial/by-path/wall-fcu` 指向 `ttyUSB0`，飞控通过 CH340 串口连接；MAVROS 仍为 connected=false，vehicle_info_get 无飞控条目。真实飞控参数写入和 AUTO.LAND 行为尚未核验，不能把软件测试当作实飞通过。

规则和参数详见 `docs/behaviors/ceiling_attachment.md`。
