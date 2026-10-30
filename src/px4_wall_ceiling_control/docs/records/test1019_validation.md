# test1019 验证记录

日期：2026-10-19。分支 `test1019`，基线 `ai_correction` 的 `4ce1b4833d3056720fe16c9cb9d97453957ccf67`。功能说明及台架验收步骤见 [侧吸行为说明](../wall.md)。

## 编译与执行环境

在树莓派 `pi@10.187.58.66` 的现有 `ros1_noetic` 容器中执行 `catkin_make`，编译成功，包括新增 `WallSensorFusionStatus` 消息和管理节点。部署环境 MAVROS 包版本为 `1.20.1-1focal.20250520.052056`。

树莓派现有 `/home/pi/ros_ws` 位于另一分支并含本地修改，因此验证使用容器内独立源码副本 `/tmp/feipa-test1019.gEZwdd`，未覆盖原源码、运行程序或飞控参数。编译命令：

```bash
docker exec ros1_noetic bash -lc '
  source /opt/ros/noetic/setup.bash && cd /root/ros_ws &&
  catkin_make --source /tmp/feipa-test1019.gEZwdd/src \
    --build /tmp/feipa-test1019.gEZwdd/build \
    -DCATKIN_DEVEL_PREFIX=/tmp/feipa-test1019.gEZwdd/devel \
    -DCMAKE_INSTALL_PREFIX=/tmp/feipa-test1019.gEZwdd/install
'
```

## 测试结果

按 CMake 登记的 38 个测试模块，在上述容器中从各模块所在 `test` 目录逐个执行 `python3 -m unittest -v <模块名>`：共执行 639 项，629 项通过，10 项失败。每个模块使用独立进程，单模块超时限制 30 秒。

新增参数管理测试 34 项、融合恢复与交接测试 21 项，全部通过。覆盖首次 HOLD、80°及复合倾角、向水平运动、低角度起点、过期/重复/异常姿态、参数部分失败、错误确认、迟到响应、单工作线程、持久化失败、重启隔离、重复重启、异常回正、断连/姿态断流后的恢复，以及新的入口、水平制动和自动交接门槛。

脱离保高、回正状态机、水平退墙、正常交接及超时交接等已登记回归测试通过。异步日志的 5 项测试在独立 unittest 进程中通过。界面 JavaScript 语法检查、14 项显示逻辑检查、Python AST、launch XML 和 `git diff --check` 通过。

`catkin_make run_tests` 的原 nose 运行方式存在同目录测试模块导入问题，并在 `test_async_logging` 挂起；加入测试目录搜索路径后仍有日志运行器挂起，因此完整结果以上述独立 unittest 运行为准，不宣称原始 nose 全套通过。

## 已在基线复现的 10 项失败

将基线提交单独解包、在同一 Docker 容器编译，再执行相关测试。以下失败的名称、FAIL/ERROR 类型及最终异常文本与 `test1019` 一致：

| 模块 | 数量 | 类型 |
| --- | ---: | --- |
| `test_attachment_safety` | 5 | 2 FAIL、3 ERROR |
| `test_attachment_range_loss` | 2 | 1 FAIL、1 ERROR |
| `test_ceiling_approach_timeout` | 3 | 3 FAIL |

这些旧用例涉及侧吸旧测试夹具缺少属性、旧距离/脱离断言和吸顶 ALTCTL 请求断言；本次未修改这些行为来迎合旧断言。基线对照包含上述三组和异步日志，共 74 项测试，仍为同样的 6 FAIL、4 ERROR。

## 尚待台架验证

未运行实机飞行动作，也未宣称达到实际融合时序目标。`80°` 为标定初值；需对齐新版本 ROS 请求/确认、实测总倾角及 ULog，分别确认测距高度、测距地形和光流实际融合是否在回到 `10°` 前恢复。参数确认、应用缓冲和单元测试均不能替代该验证。
