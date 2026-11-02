# 侧吸行为（贴墙）

本文是侧吸行为的唯一说明，覆盖前距触发、翻转门限、压墙保持、水平速度环、融合参数
切换与脱离恢复入口。脱离后的分支策略见 [behaviors](behaviors.md)；两篇合并后的完整
流程图见第 8 节。

- 决策：`scripts/decisions/wall_perch_decision_node.py`
- 融合参数管理：`scripts/sensors/wall_sensor_fusion_manager_node.py`
- 水平速度适配：`src/px4_wall_ceiling_control/adapters/wall_horizontal_velocity.py`
- 脱离垂直控制：`src/px4_wall_ceiling_control/adapters/wall_detach_vertical.py`

## 1. 状态顺序

```text
IDLE -> FRONT_WALL_DETECT -> SLOW_APPROACH -> FLIP_TO_WALL
     -> WALL_CAPTURE -> WALL_HOLD -> DETACH_ROTATE -> RECOVER -> EXIT -> IDLE
```

任何执行状态释放侧吸请求、收到 `/wall_perch/cancel` 或触发安全检查，都会进入
`ABORT -> EXIT -> IDLE`。完成一次动作后必须先释放请求才能再次启动，避免高电平
自动重复翻转。`/wall_perch/cancel` 在本包内已无发布者，取消由
`/attachment/terminal_command "cancel"` 经操作门转换成 `OperatorCommand` 实现。

## 2. 入口差异

| 入口 | 侧吸交回模式 | 关键覆盖 |
|---|---|---|
| `launch/system/attachment_terminal_flight.launch` | `ALTCTL`（`wall_pilot_mode` 默认值） | 正常脱离完成、owner 释放后请求 `ALTCTL` |
| `launch/system/wall_console_flight.launch` | `POSCTL` | 启用测距离墙高度环、水平速度环、融合参数切换、动态制动 |

两份 YAML 默认关闭的功能（`detach_range_height_enabled`、`horizontal_velocity_enabled`、
`wall_sensor_fusion_enabled`）只由测试台入口打开，终端入口保持简化链路。

## 3. 接近与翻转门限

| 条件 | 当前值 | 说明 |
|---|---:|---|
| `front_ready_distance_m` | 0.50 | 前向原始距离不大于此值 |
| `front_ready_hold_s` | 0.40 | 需不同新测量样本连续确认 |
| `flip_trigger_distance_m` | 0.20 | 单帧新鲜有效原始前距即可翻转 |
| `flip_trigger_hold_s` | 0.0 | 不等待滤波或连续确认 |
| `approach_abort_front_distance_m` | 1.8 | 前距超此值中止翻转前接近 |
| `approach_timeout_s` | 20.0 | 接近总时限，含等待 |
| `flip_time_s` | 2.00 | 翻转轨迹累计时间 |
| `flip_duration_timeout_s` | 6.00 | 翻转阶段独立墙钟超时 |
| `capture_timeout_s` | 8.00 | 捕获等待上限 |
| `hold_entry_min_pitch_deg` | 80.0 | 实测俯仰角准入 |

- 前距 ≤0.50 m 并由新样本连续确认 0.40 s 后直接进入 `SLOW_APPROACH`，不经
  `STABILIZE_HOVER`（`stabilize_time_s=0`）。慢速接近从一开始就以悬停姿态为基准向墙倾斜 3°。
- 进入 `FLIP_TO_WALL` 后不再要求前向雷达新鲜，也不再用其距离做状态或故障判定；
  顶部雷达与飞行状态的安全检查保持有效。
- 翻转完成必须以累计轨迹时间 + 新鲜里程计实测俯仰角共同判断，不能只看命令角或墙钟。
  实测俯仰角 ≥80° 直接进入 `WALL_HOLD`，否则进入 `WALL_CAPTURE` 等待，
  逾期先中止，即使当周期恰好达标也不再准入。
- 直接 `FLIP_TO_WALL -> WALL_HOLD` 时保留该时刻姿态目标，避免跳到完整翻转终点。
- 顶部测距不参与 HOLD 准入；`hold_entry_top_distance_check_enabled` 默认关闭，
  `top_contact_distance_m=0.056` 与 `flip_top_start_fraction=0.90` 仅保留为诊断。

## 4. 压墙保持

- 目标 `wall_pressure_target_distance_m=0.05`，容差 `0.01`；距离偏大时按 0.03/s
  增加压墙推力，上限 `wall_pressure_max_thrust=0.75`；恢复接触或过紧时按 0.05/s
  回到名义保持值 `hover_thrust × hold_thrust_multiplier`（0.49×1.50≈0.735，受 0.75 上限约束）。
- 顶部距离越过 `top_clear_distance_m=0.10` 并持续 `0.50 s` 才判定接触丢失；
  0.05–0.10 m 之间先尝试增加压墙推力。
- `hold_time_s=0.00`：保持阶段没有自动计时脱离，操作员停轮并发起脱离后才进入恢复。
- `WALL_HOLD` 保持初次接触时的推力轴朝墙，同时跟随机体绕该轴的转动，因此轮子可沿
  墙面带动机体转向；控制器仍纠正使推力偏离墙面的倾斜。机体转动只取实测姿态，
  没有轮子停转反馈。
- 进入 HOLD 时记录实测姿态，并在 `hold_entry_blend_s=0.50` 内从接触阶段姿态目标平滑过渡。
- `manual_wall_recovery_enabled=true`（测试台与 YAML 默认）：满足翻转完成与 80° 俯仰
  准入后，进入/保持 HOLD 只要求运动反馈有限，不因角速度、下降速度幅值或顶部距离
  偏大而拒绝；飞控、里程计、急停保护仍生效。详见 [behaviors](behaviors.md) 的手动回正章节。

## 5. 推力与输出上限

| 阶段 | 推力 |
|---|---|
| `SLOW_APPROACH` | 0.47 归一化（悬停基准 + 前倾补偿） |
| `FLIP_TO_WALL` | `hover_thrust × flip_thrust_multiplier`（0.49 × 1.35 ≈ 0.66） |
| `WALL_CAPTURE` / `WALL_HOLD` | 由顶部距离调节，上限 0.75 |
| 脱离支撑 | `detach_support_max_thrust=0.85` |
| 总上限 | `max_thrust=0.85` |

该上限作用于 MAVROS 姿态目标里的总推力，**不能保证 PX4 混控后每个电机都不超限**；
真正的单电机上限必须放在 PX4 控制分配器或 ESC。`pin_enabled` 默认 `false`：不复刻
PX4 `WALL_PIN` 的裸电机旁路，即使启用也只生成受 `max_thrust` 限制的姿态/推力候选。

## 6. 水平速度环（测试台入口启用）

- 速度取 `/mavros/local_position/odom` 中 PX4 融合速度，按 `child_frame_id` 旋转到
  ENU，只转换一次；接近方向沿本次接管保存的航向，不随偏航修正改变墙面方向。
- 使用 `/mavros/estimator_status` 的姿态、水平速度有效标志，拒绝恒位置推算模式；
  同时检查 `/mavros/px4flow/raw/optical_flow_rad` 的新鲜度、质量、积分时间、有限数值
  与正距离。这些组合不等于证明 EKF 正在融合光流。
- `SLOW_APPROACH`：前向目标 0.05 m/s（测试台入口覆盖为 0.10 m/s）、横向 0；目标速度
  变化 ≤0.10 m/s²，速度误差增益 1.5 /s，水平加速度需求上限 0.30 m/s²；最终仍受总倾角
  5°、推力轴 10°/s、航向 20°/s 限制。
- 雷达失效时目标前进速度退回 0 并用有效反馈制动；光流/水平估计失效时取消自主水平
  倾角，保持原水平姿态目标并报告等待，不声称保持住位置。
- 回正末段先等待两项融合参数恢复并完成 1.2 s 应用缓冲，再按倾角、角速度、新样本
  连续 0.30 s 恢复水平环，目标水平速度 0。不会用墙面阶段缓存数据立即刹车。
- 恢复阶段：倾角 5° 起线性降权、8° 归零；角速度 30–45°/s 线性降权。降权期间冻结积分，
  单次停顿只消费 ≤50 ms，不追补。
- `horizontal_velocity_ready` 为真而 `active` 为假，表示反馈健康但当前不按旧速度纠偏，
  不代表满足交回条件。

## 7. 光流与对地测距融合参数切换

`wall_sensor_fusion_manager_node` 只处理本次运行内的参数切换，不读取断电恢复记录。

- 在 `IDLE` 阶段通过 MAVROS 原始 MAVLink 链路按名 `PARAM_REQUEST_READ`
  `EKF2_OF_CTRL`、`EKF2_RNG_CTRL` 原值（不拉整张参数表）。
- 首次进入 `WALL_HOLD` 时在内存保存原值，再异步把两项设 `0`；翻转开始、捕获阶段
  不提前关闭。关闭顺序：光流 → 测距；恢复顺序：测距 → 光流。
- 进入 `DETACH_ROTATE` 后按新鲜姿态计算实测总倾角
  `acos(clamp(1 - 2(x²+y²), -1, 1))`，不超过 `restore_tilt_deg=80°` 时请求恢复原值；
  进入脱离时已低于阈值则立即请求。姿态在阈值附近波动不会重复开关。
- 参数读写在工作线程串行执行，`PARAM_SET` 由匹配的 `PARAM_VALUE` 确认；恢复后经过
  `application_settle_s=1.2` 应用缓冲，再按名读回校验才放行。
- 超时 5 s、单次响应等待 0.60 s、重试 2 次、退避 1–8 s。部分成功、迟到响应、断连
  保留待恢复义务；原值未知时不猜测或改写参数。
- **参数确认与应用缓冲都不能证明实际融合已经开始**，必须用 ULog 核对光流 aid source、
  测距高度/地形状态与样本时间。

## 8. 逻辑流程图

下图把本文的接近、翻转、压墙与融合切换，与 [behaviors](behaviors.md) 的脱离、恢复、
异常续接、手动回正和自动降落分支合并成一条完整链路。图中数值取自
`config/wall_perch.yaml` 当前快照；`detach_range_height_enabled=true` 的测距离墙路径
与终端入口的姿态支撑路径共用同一状态序列，差异只在 8.2 的竖直控制来源。

### 8.1 状态机主流程

```mermaid
flowchart TD
    REQ(["CH8 选择侧吸<br/>+ 操作请求 enable_control"])
    ADM{"进入准入<br/>高度高于 min_altitude_m<br/>飞控/里程计/急停输入新鲜<br/>融合参数就绪（仅测试台）<br/>本次 OFFBOARD 接管已确认"}
    IDLE["IDLE 贴墙待命<br/>不持有 owner"]
    FD["FRONT_WALL_DETECT<br/>等待前距条件"]
    AP["SLOW_APPROACH<br/>悬停基准前倾 3°<br/>测试台：前向 0.05／0.10 m/s 速度环，横向 0"]
    FL["FLIP_TO_WALL<br/>2.00 s 五次曲线，推力 ≈0.66"]
    CA["WALL_CAPTURE<br/>等待实测倾角达标，最多 8 s"]
    HL["WALL_HOLD<br/>压墙保持，hold_time_s=0，不会自动脱离"]
    AB["ABORT<br/>记录原因，停止新指令，保留恢复责任"]
    DR["DETACH_ROTATE<br/>离墙回平（内部时序见 8.2）"]
    RC["RECOVER<br/>水平修正与稳定判定"]
    EX["EXIT<br/>保持水平目标，等待模式交接"]
    LAND["AUTO.LAND 分支<br/>锁存 abort_land_requested"]
    STOP["停止 ROS 姿态推力输出"]

    REQ --> ADM
    ADM -->|"不满足"| IDLE
    IDLE -.->|"每周期重新判定"| ADM
    ADM -->|"自动接近入口"| AP
    ADM -->|"手动接近入口"| FD

    FD -->|"前距 ≤0.50 m，新样本连续 0.40 s"| AP
    FD -->|"安全检查失效"| AB
    AP -->|"单帧新鲜有效前距 ≤0.20 m"| FL
    AP -->|"接近阶段前距 ＞1.8 m（有效帧）<br/>20 s 接近总限时<br/>安全检查失效"| AB
    FL -->|"轨迹完成 + 实测俯仰 ≥80°"| HL
    FL -->|"轨迹完成 + 实测俯仰 ＜80°"| CA
    FL -->|"墙钟 6 s 超时 / 时钟倒退 / 安全检查失效"| AB
    CA -->|"实测俯仰 ≥80°"| HL
    CA -->|"8 s 超时 / 安全检查失效"| AB

    HL -.->|"50 Hz 压力环"| HP["顶部距离 ＞0.05±0.01 m → 推力 +0.03/s，上限 0.75<br/>恢复接触或过紧 → −0.05/s 回 0.735<br/>首次进入：保存并异步置 0：EKF2_OF_CTRL、EKF2_RNG_CTRL"]
    HL -->|"操作员停轮 + 脱离请求"| DG{"脱离准入"}
    DG -->|"测距离墙：姿态/估计器/OFFBOARD/最终输出有效<br/>非测距路径：轴偏差 ≤15°、各轴 ≤25°/s、稳定 ≥0.5 s"| DR
    DG -->|"不满足"| WT["保持 HOLD 等待<br/>请求 8 s 未准入则取消，可重试"]
    WT --> HL
    HL -->|"顶部距离 ＞0.10 m 持续 0.50 s（手动回正关闭时）<br/>收到 cancel / 释放侧吸 / 安全检查失效"| AB

    DR --> DN{"轨迹完成<br/>目标为零且整段结束<br/>实测有符号 pitch 判定"}
    DN -->|"总倾角 ＜10° 且 pitch ≥0°"| RC
    DN -->|"总倾角 ≥10°（10 s 内）"| WTL["继续等待实测回平<br/>不重发指令"]
    WTL --> DN
    DN -->|"pitch ＜0°"| LAND
    DN -->|"10 s 总超时 / 制动或垂直控制故障 / 安全输入失效"| AB

    RC -->|"倾角 ≤10°、角速度 ≤20°/s 连续 0.50 s<br/>≥2 个新源样本、高度就位、水平降权解锁"| EX
    RC -->|"15 s 恢复超时 / 传感器或飞行故障"| AB
    EX -->|"实际 ALTCTL／POSCTL 反馈确认"| IDLE
    EX -->|"仍在 OFFBOARD 且高度或离墙门限被撤销"| RC
    AB -->|"姿态、高度、角速度判据满足"| EX
    LAND -->|"AUTO.LAND 反馈确认 / 已解除武装"| IDLE
    LAND -->|"外部模式接管"| STOP
```

### 8.2 DETACH_ROTATE 内部时序

```mermaid
flowchart TD
    S["进入 DETACH_ROTATE<br/>继承最后实际发布的姿态/推力与压墙推力"]
    RW["REQUEST_WAIT<br/>等待姿态、估计器、OFFBOARD 与最终输出"]
    RO["ROTATE 回平<br/>3.0 s 五次曲线，跟踪误差 30° 内推进，单步 ≤0.05 s"]
    BK["动态制动重规划<br/>实测倾角 ≤30° 首次锁存后才接入"]
    WL["WAIT_LEVEL<br/>禁止水平速度环覆盖制动姿态"]
    LS["LEVEL_STABLE<br/>曲线完成且倾角 ≤5°、角速度 ≤10°/s 持续 0.30 s"]
    SUP["SUPPORT 仅姿态推力支撑<br/>无高度/速度闭环，无积分"]
    WO["WAIT_OUTPUT<br/>速度未建立时沿用受限支撑"]
    RP["RANGE_P<br/>测距高度比例段"]
    RPD["RANGE_PD<br/>clamp 0.15·Δh − 0.10·v，±0.12"]
    RL["RANGE_LOSS_SUPPORT<br/>受限保持 ≤2 s，禁止退回局部 Z"]
    PR["恢复融合参数：EKF2_RNG_CTRL → EKF2_OF_CTRL<br/>1.2 s 应用缓冲 + 按名读回校验"]
    HZ{"高度残差 ＞0.25 m"}
    HC["有界续接：仅姿态支撑<br/>单次 ≤1.2 s、累计 ≤1.5 s"]
    HB["0.30 s 平滑恢复闭环<br/>同步更新高度基准"]
    FAIL(["进入 ABORT"])
    COMP(["目标为零且整段结束：轨迹完成<br/>按 8.1 的有符号 pitch 分支判定"])

    S --> RW
    RW -->|"准入通过"| RO
    RO -->|"实测倾角 ≤30°"| BK
    BK --> WL
    WL --> LS
    LS --> COMP
    RO -.->|"测距锁定前"| SUP
    SUP -->|"首个倾角 ≤10° 的合格测距"| RP
    RP --> RPD
    SUP -.->|"快速速度反馈未建立"| WO
    RPD -.->|"测距短时失效"| RL
    RL --> RPD
    S -.->|"进入脱离时实测倾角 ≤80°"| PR
    RO -.->|"制动或支撑期间残差越限"| HZ
    HZ -->|"倾角 ≤30° 且输入有效"| HC
    HC -->|"确认：0.30 s、≥4 个新样本、单步残差 ≤0.05 m、竖直速度 ≤0.20 m/s、倾角 ≤10°"| HB
    HB --> RO
    HC -->|"预算耗尽或确认失败"| FAIL
```

### 8.3 中止与交回约定

- `ABORT` 只表示本条链路退回恢复路径，不代表物理脱离已完成。`abort_land_requested`
  一旦锁存就跳过正常 `ABORT → EXIT` 交回，只等实际 `AUTO.LAND` 反馈或解除武装。
- `EXIT` 期间仍持续发布水平姿态与推力目标，直到新鲜飞控反馈确认模式已离开
  `OFFBOARD`；owner 释放、服务返回成功、`mode_sent` 都不能代替模式反馈。
- 参数切换只在本次运行内有效：关闭顺序光流 → 测距，恢复顺序测距 → 光流；恢复后需
  1.2 s 应用缓冲并按名读回校验，参数确认不等于 PX4 已开始融合。
- 防重复翻转：一次动作结束后必须先释放请求才能重新进入 `FRONT_WALL_DETECT`；
  执行中被释放、收到 `cancel` 或安全检查失效一律走 `ABORT`。

## 9. 接口与状态

| 话题 | 方向 | 说明 |
|---|---|---|
| `/sensor/front/range` | 订阅 | 前向校验距离，仅翻转前参与判定 |
| `/sensor/ceiling/range` | 订阅 | 顶部接触/压墙反馈（不参与 HOLD 准入） |
| `/sensor/health` | 订阅 | 两路有效性与年龄 |
| `/operator/command` | 订阅 | 侧吸请求与待命 |
| `/wall_perch/detach`、`/wall_perch/cancel` | 订阅 | 脱离与取消 |
| `/mavros/state`、`/mavros/local_position/odom` | 订阅 | 飞控状态、姿态与速度 |
| `/wall_perch/status` | 发布 | `WallPerchStatus`，含阶段、门限、推力、脱离子阶段 |
| `/wall_perch/control_candidate` | 发布 | 通用 `AttachmentControlCandidate`，mux 不读侧吸状态枚举 |
| `/wall_perch/departure_trace` | 发布 | 离墙轨迹诊断 JSON |
| `/wall_perch/handover_readiness` | 订阅 | 模式管理器的交接就绪快照 |

`/wall_perch/sensor_fusion_status`、`/wall_perch/sensor_fusion_operation` 由融合管理节点发布。

## 10. 尚未验证

- `wall_angle_deg=+90` 与正俯仰方向一致，但只有 +45° 经台架确认；完整 90° 幅度、
  翻转时间与推力仍需实测。
- 接近 90° 时竖直推力分量趋近于零，高度闭环无法克服这一物理限制；墙面接触必须在
  机体具有足够竖直推力前提供支撑。
- 没有前轮接触、制动或轮速反馈，也没有前轮相对重心与雷达的完整外参；顶部 L1 的
  距离不能单独证明前轮始终抵墙，固定高度目标只适用于前轮可沿墙滚动的假设。
- 电机转速响应不等于侧吸姿态与推力已能安全支撑机体，侧吸必须单独逐步标定。
