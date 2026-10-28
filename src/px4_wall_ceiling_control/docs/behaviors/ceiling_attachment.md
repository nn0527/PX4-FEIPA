## 当前顶吸 ABORT 自动降落规则（2026-10-26）

实际顶吸配置启用 `abort_land_on_fault` 和 `ekf_abort_guard_enabled`。

- 接触安全异常（接触丢失、姿态超限、里程计失效等）优先进入 ABORT，优先级高于同一周期的正常脱离请求；接触确认超时也直接 ABORT。
- 脱离下降、恢复悬停以及接管上升中进入 ABORT 后锁存 `abort_land_requested` 和本轮唯一 ID，立即请求 `AUTO.LAND`，不等待下降达标、恢复稳定或第三次 CH12，不再转为正常 POSCTL 交接。
- 从 `/mavlink/from` 读取目标飞控的 MAVLink `ESTIMATOR_STATUS` 原始磁创新比；首个有效报告 `mag_ratio > 1`、非有限磁创新比、姿态估计无效或加速度异常即锁存故障。错误发送端、过期/未来时间、重复源时间不刷新监视器。磁安全反馈缺失超过 2 秒会阻止准入或使运行中的动作 ABORT。只能在飞控报告后检测，不能保证在物理磁干扰发生的同一瞬间识别。
- FCU `connected=false` 或状态源/接收时间超时后立即 ABORT。USB 拔出后 ROS 无法通过断开的链路发送降落命令：降落意图保持锁存，重连后继续处理；仅新的实际 `AUTO.LAND` 模式反馈才确认飞控进入降落。
- `ceiling_failsafe_guard_node` 仅在飞控连接且未解锁时，通过定向 MAVLink 请求读回、设置并独立读回确认 `COM_OBL_RC_ACT=4`、`COM_OF_LOSS_T=0.2`；旧固件存在 `COM_OBL_ACT` 时要求其为 `0`。只有核实为 PX4 1.14+ 时才允许缺少旧参数；未知/旧固件不能用参数超时当作“不存在”。失联后清除核验，下一次地面连接重新核验。未核验或守护节点心跳过期会阻止新的顶吸接管。
- 0.2 秒是 Offboard 失联动作的参数值，并非 USB 拔出到落地或切换模式的保证总延迟。最终行为由 PX4 内部检测、状态估计有效性和其他 failsafe 决策决定。
- 急停、确认解除武装和飞手明确改变模式保留原有优先级。未收到新的实际模式反馈前，不把模式服务调用成功显示为“已自动降落”。

依据：[PX4 Offboard](https://docs.px4.io/main/en/flight_modes_mc/offboard)、[PX4 参数](https://docs.px4.io/main/en/advanced_config/parameter_reference#COM_OBL_RC_ACT)、[MAVLink ESTIMATOR_STATUS](https://mavlink.io/en/messages/common.html#ESTIMATOR_STATUS)。

# 吸顶 POSCTL 流程与参数（2026-10-25）

> 2026-10-28 更新：**“普通模式”（真实顶部雷达 + 低通 + 距离 PID 脱离）已删除**。现在吸顶只有一条链路：顶距由“接管时锁定的顶部基准 + 朝下光流升高量”推算，接触确认走 ATTACH_CONTROL 的稳定判据，脱离走融合高度/气压下降量。`console_contact_profile`、`closed_loop_approach_enabled`、`approach_flow_height_enabled`、`activation_distance_m` 以及 `attachment_terminal_flight.launch` 之外的台架吸顶入口均已移除；`attachment_terminal_flight.launch` 保留但仍使用同一套光流顶距和 YAML 距离参数。

> 2026-10-26 更新：当前测试台的正常高度控制和下降判断使用 PX4 融合高度/竖直速度，气压仅作辅助对照。下文旧版气压专用描述属于历史实现，以 `ceiling_fused_height_20261026.md` 和现行源码为准；下降、恢复稳定、锁定姿态及第三次 CH12 确认的阶段顺序保留。

本文对应 test1019 的统一控制台入口 `launch/system/wall_console_flight.launch`。
当前流程包含恢复姿态锁定及第三次 CH12 确认，吸顶入口和交回模式均为 POSCTL。
参数以 YAML、launch 覆盖及运行时配置为准；保持推力取 `surface_hold_thrust`（控制台入口 0.55）。

## HOLD超时自动降落（树莓派test1024）

`hold_timeout_s=5.0`：进入SURFACE_HOLD后超过5秒，若仍未启动DETACH，则进入锁存ABORT并发布机构释放请求。模式管理器进入CEILING_LANDING，立即请求PX4 AUTO.LAND，不等待正常脱离的0.50 m气压下降或恢复稳定条件。

- 同一周期已有脱离或取消请求时优先正常DETACH；进入DETACH后不再使用HOLD计时器。
- 顶部测距失效不暂停HOLD超时。ABORT清除水平积分，等待模式交接期间保持受限姿态和0.50支撑推力。
- 降落请求要求本轮新鲜ABORT状态、连接且解锁的OFFBOARD飞控反馈，以及急停未生效；异步服务调用前再次检查本轮身份和实际模式。拒绝后按现有重试间隔重试；服务成功不视为实际降落已开始。
- 只有新的飞控AUTO.LAND反馈才确认降落模式。飞手切换POSCTL等其他模式、遥控器接管、急停或解除武装时停止自动请求；已结束的本轮ABORT不得自动重入。
- 新增状态消息字段，编译后需在落地上锁时重启整套控制链路，使所有节点使用同一消息版本。本次修改及测试不发送真实飞行降落指令。

## 状态与前置条件

| 阶段 | 进入条件 | 行为及主要参数 |
|---|---|---|
| POSCTL 待命 / 等待请求 | 飞手已起飞、已解锁并稳定在 POSCTL；提交吸顶请求 | 单帧有效顶距≤1.00 m；位置/水平及竖直速度/姿态估计有效，光流连续有效0.30 s；水平速度≤0.20 m/s、角速度≤20°/s时采样平衡姿态。至少3个FCU目标样本跨越0.50 s，总倾角≤5°、轴向分散≤1.5°；等待期间继续由飞手控制POSCTL。 |
| CEILING_ARMED 接管准备 | 满足入口检查，完成预发送；飞控实际反馈本次 OFFBOARD | 请求OFFBOARD前冻结平衡姿态及FCU推力，最多保留2 s，防止接管前后反馈回显污染基准；OFFBOARD确认时捕获本地x/y和航向。零竖直速度等待；顶距准入、反馈有效且未暂停后进入接近。 |
| APPROACH 上升接近 | 顶距准入、运动反馈有效、未暂停 | 向上目标0.15 m/s，竖直目标加速度0.15 m/s²；冻结航向，弱位置环加水平速度PI纠偏。单帧有效原始顶距≤0.067 m直接进入SURFACE_HOLD，不经过接触渐增推力阶段。控制台无额外起步等待；有效接近超时12 s进入ABORT。顶部测距失效时暂停上升并暂停接近计时。 |
| ATTACH_CONTROL 接触确认 | 原始顶距≤0.067 m、竖直速度≤0.05 m/s、总倾角≤5° | 继承接管时支撑推力，按0.02/s在[保持推力, attach_max_thrust]之间调整；原始/滤波距离在目标±`stable_tolerance_m`内、速度/姿态/角速度稳定且推力回到保持值持续0.50 s后进入保持。5 s内未达成则ABORT。 |
| SURFACE_HOLD 吸顶保持 | ATTACH_CONTROL 稳定判据全部满足 | 推力目标固定 `surface_hold_thrust`（控制台入口0.55），保持捕获航向及≤5°受限姿态；不运行水平位置积分。记录进入保持时的融合高度/气压作为离顶基准；`hold_timeout_s=0` 时无HOLD超时（须显式脱离）。顶部测距失效仅告警并维持保持推力；恢复后重新评估。 |
| DETACH 受控脱离 | 离顶命令、取消吸顶、接触失败或接触安全条件失效 | 先请求机构释放；若启用机构回执则必须同序号确认。以保持气压基准下降0.50 m，最大向下目标0.15 m/s，临近阈值至少0.05 m/s以确保有限时间到达。顶距≥0.10 m持续0.20 s后允许水平纠偏；确认下降0.50 m进入恢复。超时20 s进入ABORT。接触失败未进入保持时，单独捕获当前气压作为中止脱离基准。 |
| RECOVERY_HOVER 稳定恢复 | 气压确认已下降≥0.50 m | 对下降目标高度制动并稳定；高度误差≤0.05 m（下降量0.45～0.55 m）、竖直速度绝对值≤0.15 m/s、水平速度≤0.15 m/s、水平漂移≤0.30 m、总倾角≤10°、角速度模≤20°/s、航向误差≤10°，全部在不同新反馈样本上连续0.50 s后捕获实测姿态并锁定，关闭水平姿态修正和摇杆微调，高度闭环继续；第三次有效CH12确认后才允许请求POSCTL。锁定前15 s未稳定进入ABORT；锁定后等待确认不使用该超时，反馈和运动故障保护继续生效。 |
| POSCTL 交回 / NORMAL_FLIGHT | 恢复姿态已锁定，收到晚于锁定时刻的第三次CH12确认，且恢复条件、位置估计、RC、连续输出等交回检查有效 | 模式管理器请求POSCTL并继续保留本轮输出；服务成功不视为完成。确认新鲜且已连接的飞控实际反馈POSCTL后释放控制归属、恢复前部雷达输入并清理基准。重新启动前须释放旧请求。 |

此入口默认机构硬件输出关闭、`require_mechanism_ack=false`，所以“释放”是现有控制协议动作，不能据此认定真实机构已经释放。真实机构启用回执时按同序号确认执行。

单帧门槛使用有效原始顶距，不受低通或中位数阻挡，不要求持续时间和多帧确认。同一决策周期内的合格帧保留到处理或测距超时，后续偏大帧不会吞掉该触发；无效帧、健康状态失效或新请求会清理旧触发。距离满足只解除距离门槛，光流、位置、平衡基准和飞控等安全准入仍保留。

下降≥0.50 m为本轮一次性达标条件：未达到时，即使下降0.49 m且稳定也不能交回；达到后可回弹到0.45～0.55 m范围内稳定，再按原速度、姿态及0.50 s连续新样本条件申请POSCTL。

## 水平与姿态初始限制

以下参数已经生成并启用，采用本轮约定的较宽初值。

| 参数 | 初值与作用 |
|---|---|
| `horizontal_position_deadband_m` / `horizontal_position_kp` | 0.10 m / 0.30 s⁻¹；死区内不追点，但保留速度制动 |
| `horizontal_max_speed_mps` | 纠偏目标速度模上限0.20 m/s |
| `horizontal_velocity_kp` / `horizontal_velocity_ki` | 1.50 / 0.30；使用飞控融合ENU水平速度，光流质量及估计器状态共同准入 |
| `horizontal_max_acceleration_mps2` / `horizontal_integral_limit_mps2` | 0.40 / 0.20 m/s² |
| `horizontal_target_acceleration_mps2` | 目标速度变化加速度0.10 m/s² |
| `horizontal_drift_warn_m` / `horizontal_drift_pause_m` / `horizontal_drift_resume_m` | 0.30 / 0.50 / 0.30 m |
| `attitude_max_tilt_deg` / `contact_max_tilt_deg` | 最终目标总倾角10° / 接触保持5°；接触RC微调另受原±2°限制 |
| `yaw_max_offset_deg` | 最终目标航向相对接管航向±10°；默认保持接管航向 |
| `attitude_tilt_slew_deg_s` / `attitude_yaw_slew_deg_s` | 最终目标推力轴和航向各15°/s；所有RC修正之后统一限幅 |
| `actual_tilt_pause_deg` / `actual_tilt_resume_deg` | 实测总倾角15° / 12° |
| `actual_body_rate_pause_deg_s` / `actual_body_rate_resume_deg_s` | 实测角速度模30°/s / 20°/s |
| `motion_limit_confirm_s` / `motion_resume_hold_s` | 超限确认0.20 s；降回恢复阈值连续0.50 s才恢复运动 |
| `vertical_speed_kp` / `vertical_target_acceleration_mps2` | 竖直速度反馈增益0.30，目标速度加速度0.15 m/s² |

自由飞行竖直支撑来自接管前实际FCU推力及平衡推力轴，并加倾角补偿和速度反馈；接触/保持仍使用上表固定或渐增推力。水平积分仅在确认最终输出属于吸顶且有新鲜实际输出反馈时累积，暂停、失联或接触时清除。

这些值约束控制目标及触发程序响应，不能保证真实漂移、姿态角或角速度始终不超限。尤其硬倾角包络优先时，入口姿态投影可能优先于目标变化率限制。实际稳定程度仍取决于光流/EKF、飞控姿态环、推力标定、气压扰动及机构特性，需仿真和受控实测验证。

## 暂停与故障

- 自由飞行时反馈无效立即暂停竖直运动、清除水平积分；有效新样本连续0.30 s才重新准入。光流接收和源时间戳均检查，超时0.60 s、质量阈值1，拒绝重复源样本；估计器超时1.50 s。
- 漂移、实测倾角或角速度超限确认0.20 s后暂停竖直运动，保留反馈可信时的有限水平纠偏；恢复阈值连续0.50 s后解除暂停。反馈持续失效2 s或运动暂停持续8 s进入锁存ABORT。
- 接触/保持时不做水平追点；确认姿态或角速度危险时受控脱离。
- 位置相邻样本跳变超过0.75 m加当前速度可解释的位移，或航向跳变超过20°，本轮锁存ABORT，停止追赶旧位置参考。源时间异常不能用接收时间刷新掩盖。
- 接近超时及运动故障不自动重入；模式管理器在位置估计、遥控输入和连续输出等交回条件有效时请求POSCTL。条件无效时维持故障及受限候选，不能宣称已经安全交接。失去实际姿态/高度反馈后的稳定控制无法由本逻辑保证。
- 气压无效时不确认下降完成；恢复确认必须有不同的新压力、里程计和光流样本，定时器重复运行不能凑足稳定时间。

## 实现与验证入口

- 决策：`scripts/decisions/ceiling_attachment_decision_node.py`。
- 运动反馈适配：`src/px4_wall_ceiling_control/adapters/ceiling_motion.py`。
- 运动控制与包络：`src/px4_wall_ceiling_control/domain/ceiling_motion.py`。
- 接管交回：`scripts/operator/flight_mode_manager_node.py`。
- 最终输出：`scripts/outputs/attachment_output_mux_node.py`。启用运动控制时各自由飞行阶段也输出姿态/推力候选。
- 参数：`config/ceiling_attachment.yaml`；上述流程数值包含 `wall_console_flight.launch` 的覆盖，单独加载YAML不是同一配置。
- `CeilingAttachmentStatus` 新增运动、暂停、恢复确认和实测限制字段；候选消息增加航向基准。ROS端必须重新生成消息并让所有节点使用同一消息版本。
- 原始备份：`test1024/backups/ceiling-posctl-stability-20261025/px4_wall_ceiling_control`；离线验证说明位于 `test1024/validation/README.md`。

统一中止阶段使用 `ABORT=7`（原 FAULT 编号），不再有独立 FAULT 或编号8的ABORT。普通异常保留原FAULT的受限水平/竖直支撑、禁止继续动作及POSCTL接管逻辑，不发自动降落或新增机构释放请求；仅HOLD超时的ABORT设置 `abort_land_requested=true`，锁存并请求AUTO.LAND。候选控制消息携带同一帧降落标记，输出端据此选择原故障包络或接触支撑包络。


## 遥控横滚、俯仰与自动水平稳定

控制台入口已启用标定后的横滚/俯仰修正：自由飞行每轴±5°、变化率10°/s，最终总倾角≤10°；接触保持每轴±2°、总倾角≤5°。偏航仍保持捕获参考。侧吸保持原横滚/偏航逻辑。

`rc_manual_coordination_enabled=true` 时，输出端仅在实际发布顶吸候选后发送 `/attachment/ceiling_rc_assist`，携带本轮动作标识、候选序号、阶段及源时间。决策端拒绝过期、未来、错轮次、错阶段和倒序反馈。

- APPROACH、确认机构释放且顶距≥0.10m持续0.20s后的DETACH、RECOVERY_HOVER允许协调。人工拨杆或残余修正超过0.05°时清除并暂停水平积分，位置回拉权重平滑降至0.25；速度阻尼保留。
- 回中后权重平滑回到1，恢复期间不积累水平积分。`rc_manual_blend_s=0.50`表示0.25与1之间的完整切换约需0.50秒（控制周期异常时不追赶跳变）。原x/y位置参考始终不变，不能靠连续拨杆移动允许范围。
- 漂移≥0.30m或运动暂停时保持完整位置回拉，原0.50m暂停阈值和姿态/角速度保护继续生效。
- 接管准备、ABORT、反馈异常或尚未确认离顶时关闭附加遥控修正。HOLD维持原有小幅修正及5秒超时降落逻辑。
- 修正反馈接收与源时间均要求≤0.20s。反馈缺失时恢复完整位置回拉、暂停积分并禁止恢复完成确认，避免用未知人工输入宣称稳定。
- 恢复阶段只有本轮修正反馈有效、摇杆回中、残余修正消退且位置回拉权重恢复到1后，才开始累计原有0.50s稳定时间。

更改消息后需要重新生成ROS消息并协调重启相关节点。首次存在最多一个反馈周期的人工介入识别延迟，最终输出角度与速率限制始终生效。

## 2026-10-26 恢复姿态锁定与 CH12 确认

正常脱离下降达标并恢复稳定后捕获实测四元数作为固定姿态目标，保留恢复高度闭环、推力连续性和最终输出包络。第三次 CH12 必须在锁定后、节点仍报告恢复就绪且反馈新鲜时触发；提前拨动不排队，CH8 只影响下一轮任务。没有第三次确认不会正常请求 POSCTL。异常、急停、主动切换飞控模式仍按原保护路径执行。
