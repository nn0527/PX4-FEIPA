# px4_wall_ceiling_control

ROS1 Noetic package for two independent L1 range sensors, ceiling attachment,
wall perching, owner arbitration and one MAVROS command outlet.

> Safety: controller and physical mechanism outputs default to disabled. This
> package never arms PX4. The terminal flight entry can switch between ALTCTL
> and OFFBOARD when `flight_enabled:=true`. Validate frames, signs, limits and
> failsafes in isolated bench tests and SITL/HITL before powered use.

## Active runtime chain

```text
ceiling L1 -> /sensor/ceiling/raw --┐
                                    ├-> sensor_range_manager
front L1   -> /sensor/front/raw ----┘       ├-> ceiling_attachment_decision --┐
                                            └-> wall_perch_decision ---------┤
RC -> PX4 -> MAVROS -> operator_gate ----------------------------------------┤
PX4 state/odom ---------------------------------------------------------------┤
                                                                              v
                                                        attachment_output_mux
                                                              ├-> MAVROS/PX4
                                                              └-> output owner

ceiling decision -> mechanism candidate -> mechanism output gate -> mechanism driver
```

Both decisions publish the same `AttachmentControlCandidate` contract to the
output mux. Their behavior-specific Status messages remain diagnostic outputs;
the mux does not depend on either state-machine enum.

The unified runtime has exactly one MAVROS setpoint publisher:
`attachment_output_mux_node`.

## Source layout

```text
px4_wall_ceiling_control/
├── config/                  shared and behavior-specific YAML
├── docs/
│   ├── behaviors/           ceiling and wall state-machine notes
│   └── archive/             historical, non-executable instructions
├── launch/
│   ├── system/              primary combined runtime
│   ├── sensors/             sensor-only utilities
│   ├── bench/               isolated observation and fixed-frame motor tests
│   ├── standalone/          ceiling-only compatibility runtime
│   └── legacy/              old generic distance-control chain
├── msg/                     ROS interface contracts
├── scripts/
│   ├── sensors/             hardware adapters and range validation
│   ├── bench/               bounded motor bench setpoint output
│   ├── operator/            RC command gate
│   ├── decisions/           ceiling and wall behavior decisions
│   ├── outputs/             owner mux and mechanism safety gate
│   ├── standalone/          ceiling-only MAVROS adapter
│   ├── test_support/        mock flight and mechanism inputs
│   └── legacy/              old generic FSM/controller/mux entry points
├── src/px4_wall_ceiling_control/
│   ├── adapters/            odometry and MAVROS message boundaries
│   ├── domain/              ROS-independent geometry, hold and owner logic
│   ├── freshness.py         shared validation helpers
│   └── legacy/              old generic controller implementation
└── test/                    automated safety tests
```

Source files are grouped by responsibility. Installed ROS node names remain
unchanged, so existing `type="..._node.py"` launch entries still work.

## Build and tests

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg px4_wall_ceiling_control
source devel/setup.bash
catkin_make run_tests_px4_wall_ceiling_control
catkin_test_results
```

## Launch entry points

| Purpose | Launch file | Real MAVROS output |
|---|---|---|
| Unified ceiling + wall runtime | `launch/system/attachment_system_mavros.launch` | Disabled by default |
| Terminal triggered ALTCTL/OFFBOARD flight | `launch/system/attachment_terminal_flight.launch` | Enabled with `flight_enabled:=true` |
| Dual sensor input only | `launch/sensors/lidar_dual.launch` | None |
| Wall bench test | `launch/bench/wall_perch_sensor_test.launch` | Mock namespace only |
| Ceiling bench test | `launch/bench/ceiling_attachment_sensor_test.launch` | Mock namespace only |
| Disarmed PX4 + real L1 observation | `launch/bench/ceiling_attachment_px4_observe.launch` | No MAVROS output node |
| Fixed, propeller-free PX4 motor bench | `launch/bench/ceiling_motor_bench.launch` | Bounded attitude setpoints; disabled by default |
| Standalone ceiling runtime | `launch/standalone/ceiling_attachment_mavros.launch` | Disabled by default |
| Old generic chain | `launch/legacy/system.launch` | Legacy; do not combine with unified runtime |

Typical sensor-only test:

```bash
roslaunch px4_wall_ceiling_control lidar_dual.launch \
  ceiling_serial_port:=/dev/serial/by-id/CEILING_DEVICE \
  front_serial_port:=/dev/serial/by-id/FRONT_DEVICE
```

Unified runtime with real inputs but inhibited outputs:

```bash
roslaunch px4_wall_ceiling_control attachment_system_mavros.launch \
  ceiling_serial_port:=/dev/serial/by-id/CEILING_DEVICE \
  front_serial_port:=/dev/serial/by-id/FRONT_DEVICE \
  ceiling_enabled:=true wall_enabled:=true \
  output_enabled:=false mechanism_output_enabled:=false
```

## Configuration ownership

| File | Owns |
|---|---|
| `config/sensors.yaml` | Sensor drivers and two-channel range manager |
| `config/operator.yaml` | RC operator gate |
| `config/ceiling_attachment.yaml` | Ceiling decision and controller parameters |
| `config/wall_perch.yaml` | Wall decision and attitude planner parameters |
| `config/attachment_output.yaml` | Owner mux, MAVROS adapters and mechanism output gate |
| `config/legacy.yaml` | Old generic supervisor/FSM/controller/mux chain |

`config/common.yaml` is only a deprecated marker; current launches do not load
it. Each active node now has one configuration owner.

Keep topic names in these YAML files and launch overrides. Algorithm code
should not acquire additional deployment-specific topic names.

## Documentation

- [System architecture](docs/architecture.md)
- [Ceiling behavior](docs/behaviors/ceiling_attachment.md)
- [Wall behavior](docs/behaviors/wall_perch.md)
- [Disarmed PX4 observation](docs/px4_disarmed_observation.md)
- [Real PX4 motor bench procedure](../../bench_test_procedure.txt)
- [Terminal flight handover](docs/terminal_flight.md)
- [Code framework and readiness review](docs/code_framework_review.md)

Files under `docs/archive/` describe older interfaces and must not be used as
current test instructions.
