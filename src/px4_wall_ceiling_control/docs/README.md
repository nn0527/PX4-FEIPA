# 文档索引

本目录按主题组织：1 篇索引（本文件）、1 篇架构说明、7 篇主题文档和 `records/` 历史记录。
参数数值以 `config/*.yaml`、launch 覆盖与运行时配置为准，文档中的数值是编写时的快照。

| 文档 | 内容 |
|---|---|
| [architecture.md](architecture.md) | 分层职责、话题契约、安全不变量、坐标边界 |
| [ceiling.md](ceiling.md) | 吸顶：入口、状态阶段、顶距估算、保持/脱离/恢复、CH12 交回、ceiling + behaviors 合并逻辑流程图 |
| [wall.md](wall.md) | 侧吸：接近与翻转门限、压墙保持、水平速度环、融合参数切换、wall + behaviors 合并逻辑流程图 |
| [behaviors.md](behaviors.md) | 分支行为：两段垂直控制、离墙 demo、动态制动、异常续接、手动回正、自动降落分支 |
| [sensors.md](sensors.md) | 两路 L1、校验与健康话题、错误码、自动飞行记录与解释规则 |
| [rc.md](rc.md) | 通道分配、CH8/CH12、急停、遥控修正、未解锁启动手势、一键接管 |
| [terminal.md](terminal.md) | 终端入口、模式交接契约、上机前校准、实飞顺序、代码责任划分与评估 |
| [console.md](console.md) | 测试台：页面/端口/登录、离线模拟、真机部署、地面恢复 |
| [records/](records/) | 日期型实飞/台架验证记录与未实现的设计稿，只作历史依据 |

## 阅读顺序

1. 先读 [architecture.md](architecture.md) 了解唯一输出节点与 owner 约束。
2. 按任务读 [ceiling.md](ceiling.md) 或 [wall.md](wall.md)。
3. 涉及脱离、恢复、异常处理时读 [behaviors.md](behaviors.md)。
4. 上机前读 [sensors.md](sensors.md)、[rc.md](rc.md) 与 [terminal.md](terminal.md)。
5. 使用网页测试台时读 [console.md](console.md)。

## 运行入口

| 入口 | 用途 | 特点 |
|---|---|---|
| `launch/system/attachment_terminal_flight.launch` | 终端/CH12 实飞 | `flight_enabled` 默认 `false`；吸顶 `POSCTL`、侧吸 `ALTCTL` |
| `launch/system/wall_console_flight.launch` | 网页测试台启动 | 两行为均 `POSCTL`；启用测距离墙、水平速度环、融合切换 |
| `launch/system/attachment_system_mavros.launch` | 被上面两者包含 | 统一链路与参数；单独使用时不切模式 |
| `launch/sensors/lidar_dual.launch` | 仅传感器 | 无飞控输出 |

## 安全边界（所有文档通用）

- 统一运行时只有一个 MAVROS 设定值发布者：`attachment_output_mux_node`。
- owner 只能从 NONE 获取，结束或超时后需经过中性切换驻留才能被另一行为获取。
- 飞行输出与物理机构输出各自有默认关闭的总开关；本包不解锁、不起飞。
- 只有新的实际模式反馈才算完成模式交接；服务返回成功不等于已切换。
- 故障、`cancel` 与状态机退出都不等于物理脱离成功，飞手必须保留已验证的遥控接管路径。

## 维护约定

- 新结论直接改对应主题文档，不再新增日期型说明；验证过程与一次性记录放 `records/`。
- `records/` 中的文件是历史快照，不随代码更新，其中的旧接口与旧路径不代表现状。
- 引用代码时使用仓库相对路径；引用外部脚本或厂家数据时注明来源，避免出现仓库中不存在的程序名。
