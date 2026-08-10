# Wall-perch dual range sensors

PX4 uses the body-fixed FRD frame: `+X` is forward, `+Y` is right, and `+Z`
is down. The suction/contact sensor is mounted on the upper suction face, so
its optical axis is body `-Z` (`ROTATION_UPWARD_FACING`, value 24). The approach
sensor points horizontally forward along body `+X`
(`ROTATION_FORWARD_FACING`, value 0).

## Signal path

| Sensor | ESP32 frame ID | `distance_sensor.orientation` | Controller use |
| --- | ---: | ---: | --- |
| Suction/contact (body -Z) | 0 | 24 | Wall contact, capture, hold, detach confirmation |
| Forward approach (body +X) | 1 | 0 | Wall-ready distance and flip trigger |

The ESP32 frame is `AA 55 distance_high distance_low sensor_id 0D 0A`.
`sensor_id=0` is backward compatible with the former single-sensor frame,
whose fifth byte was reserved. `CeilingReader` publishes the two signals as
separate instances of PX4's standard `distance_sensor` topic. A separately
connected PX4 rangefinder can be used instead, provided its driver publishes
the orientation shown above.

`wall_perch` scans every `distance_sensor` instance and selects samples by
orientation; it does not depend on startup-dependent instance numbers. Both
signals must be fresh before the maneuver starts. The forward signal is
required through approach; the body -Z signal is required from rotation onward.
Signal age is checked against `WP_SENS_TIMEOUT` using the publisher's timestamp.

The maneuver logic is:

1. In ALTCTL, the rising edge of the side-perch switch immediately transfers
   attitude/thrust ownership to `wall_perch`. There is no intermediate hover
   stabilization state. The current altitude and yaw are latched.
2. The aircraft commands a small nose-down pitch toward the wall. The forward
   body +X sensor reduces the pitch inside `WP_FRN_RD_DIST`; remaining below
   `WP_FLP_TRD_DIST` for `WP_FLP_TRD_HLD` starts the 90-degree rotation. A
   local altitude PID (`WP_ALT_KP/KI/KD`) holds the switch-on altitude during
   this horizontal approach.
3. Rotation continues directly from the measured approach attitude to
   `WP_WALL_ANGLE` (default -90 degrees), so no level-attitude recovery command
   is inserted. Normal attitude control remains active only to track this
   commanded rotation.
4. After the measured pitch reaches `WP_PIN_PITCH`, the body -Z sensor becomes
   the motor-sufficiency feedback. While its distance is above
   `WP_TOP_CT_DIST`, thrust ramps from the recorded hover value toward
   `WP_PIN_THR`. Reaching and holding the distance for `WP_TOP_CT_HLD` means
   the current RPM is sufficient, so the ramp stops at that achieved value.
5. With `WP_PIN_ENABLE=1`, sufficient RPM enters `WALL_PIN`: normal control
   allocation is suppressed and all four motors hold the same achieved command,
   so no attitude-recovery torque is generated. If distance later grows, the
   sensor causes the command to increase again. With pinning disabled, the same
   distance feedback continues through the attitude-controlled hold path.
6. Stale required signals, user cancel, or remaining above the target for
   `WP_CONTACT_TMO` after reaching maximum thrust enters safety recovery.

`wall_perch_status` is the reserved status/diagnostic topic. It contains both
filtered distances, validity flags, selected `distance_sensor` instance
numbers, altitude-loop data, wall/contact/boost gates, state, and failsafe
state.

On FMUv6X, set `WP_ENABLE=1` and reboot (or run `wall_perch start`) after both
sensor publishers are running. `wall_perch status`, `CeilingReader status`,
and `listener distance_sensor -n 10` can be used to verify the two directions
before arming.
