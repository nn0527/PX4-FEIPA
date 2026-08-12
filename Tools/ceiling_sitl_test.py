#!/usr/bin/env python3
"""
Ceiling Contact Controller SITL Test Script

Test sequence:
  1. Takeoff to 1m (Offboard)
  2. Feed ESP32 dual-range frames through a pseudo UART
  3. AUX1=ON -> trigger CEILING_ARM -> APPROACH -> ATTACH_CONTROL
  4. Hold attach for a few seconds
  5. Keep AUX1 on, set AUX2 on -> DETACH -> RECOVERY -> NORMAL
  6. Verify the AUX1 re-arm interlock
  7. Stop sensor injection, land & disarm

Prerequisites (run in PX4 nsh before this script):
    ceiling_controller start
    param set RC_CHAN_CNT 8
    param set RC_MAP_ROLL 1
    param set RC_MAP_THROTTLE 4
    param set RC_MAP_AUX1 5
    param set RC_MAP_AUX2 6
    rc_update stop
    rc_update start

Run the default dedicated-switch test with ``--detach-case aux2``. Additional
runs with ``aux1``, ``sensor-timeout`` and ``distance-timeout`` cover the
legacy switch path and both detach failure exits.
"""

import argparse
import math
import re
import threading
import time
from collections import deque

from esp32_dual_range_sitl import DualRangePty, px4_command

# ============== Config ==============
AUX_OFF = 1000
AUX_ON  = 2000
TARGET_Z = -1.0          # 1m altitude (NED)
# target_dist = D0 - comp_tgt = 0.22 - 0.02 = 0.20m = 20cm
# Drone hovers at 1m, ceiling at 1.5m → initial distance 50cm
# Controller drives to target_dist=20cm → final height ~1.3m
CEILING_HEIGHT_M = 1.5
ATTACH_DIST_CM = 20      # fallback fixed distance (cm)
DETACH_DIST_CM = 70      # final injected ceiling distance after detach (cm)
OFFBOARD_RATE = 50       # Hz
#
# IMPORTANT: run these in PX4 nsh before test:
#   param set CEIL_D0 0.22
#   param set CEIL_DIST_THR 0.60
#   param set CEIL_COMP_TGT 0.02

# ============== Globals ==============
running = True            # pos_thread flag (runs until script exit)
inject_running = False    # sensor_inject_thread flag (started in phase 5, stopped in phase 9)
inject_dist_cm = DETACH_DIST_CM
inject_front_mm = 2000
inject_up_valid = True
pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
log_buffer = deque(maxlen=500)
lock = threading.Lock()
range_link = None
port = None
mavutil = None


# ============== Threads ==============
def pos_thread():
    while running:
        m = port.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if m:
            with lock:
                pos['x'], pos['y'], pos['z'] = m.x, m.y, m.z
                pos['vx'], pos['vy'], pos['vz'] = m.vx, m.vy, m.vz
        time.sleep(0.05)


def sensor_inject_thread():
    """Write dual-range ESP32 frames at 10 Hz through the pseudo UART."""
    while inject_running:
        with lock:
            z = pos['z']
        # Compute distance to virtual ceiling; NED z is negative upward
        actual_height = -z
        dist = int(max(5.0, (CEILING_HEIGHT_M - actual_height) * 100.0))
        # Allow fixed override when explicitly set to non-zero
        if inject_dist_cm > 0:
            dist = inject_dist_cm
        up_mm = dist * 10 if inject_up_valid else 0xFFFF
        range_link.write_frame(up_mm, inject_front_mm)
        time.sleep(0.1)


# ============== Helpers ==============
def send_offboard_sp(x, y, z):
    port.mav.set_position_target_local_ned_send(
        0, port.target_system, port.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000,
        x, y, z, 0, 0, 0, 0, 0, 0, 0, 0)


def send_offboard_vel(vx, vy, vz):
    """Send velocity-only setpoint (position/accel/yaw ignored)"""
    # Ignore: position(1+2+4), acceleration(64+128+256), yaw(1024), yaw_rate(2048)
    # Valid: velocity only
    type_mask = (1 | 2 | 4 | 64 | 128 | 256 | 1024 | 2048)
    port.mav.set_position_target_local_ned_send(
        0, port.target_system, port.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)


def set_rc(ch5, ch6):
    port.mav.rc_channels_override_send(
        port.target_system, port.target_component,
        65535, 65535, 65535, 65535, ch5, ch6,
        65535, 65535, 65535, 65535, 65535, 65535,
        65535, 65535, 65535, 65535, 65535)


def listener_output(topic):
    """Read one fresh uORB sample through the PX4 daemon socket."""
    _, output = px4_command(
        f"listener {topic} -n 1", range_link.socket_path, timeout=2.0, check=False)
    return output


def scalar_field(output, field, default=None):
    match = re.search(rf"^\s*{re.escape(field)}:\s*([^\s]+)", output, re.MULTILINE)
    if not match:
        return default
    value = match.group(1).rstrip(',')
    if value.lower() in ("true", "false"):
        return value.lower() == "true"
    try:
        return float(value) if any(char in value.lower() for char in ('.', 'e', 'n')) else int(value)
    except ValueError:
        return default


def ceiling_status():
    output = listener_output("ceiling_contact_status")
    return {
        "state": scalar_field(output, "state", -1),
        "distance": scalar_field(output, "ceiling_distance", float("nan")),
        "target": scalar_field(output, "target_distance", float("nan")),
        "thrust_z": scalar_field(output, "thrust_body_z_sp", float("nan")),
        "z_control_mode": scalar_field(output, "z_control_mode", 0),
        "detach_phase": scalar_field(output, "detach_phase", 0),
        "sensor_valid": scalar_field(output, "distance_sensor_valid", False),
        "input_valid": scalar_field(output, "input_valid", False),
        "rearm_required": scalar_field(output, "rearm_required", False),
        "fault_detected": scalar_field(output, "fault_detected", False),
        "fault_reason": scalar_field(output, "fault_reason", 0),
    }


def attitude_thrust_z():
    output = listener_output("vehicle_attitude_setpoint")
    match = re.search(r"^\s*thrust_body:\s*\[[^,]+,[^,]+,\s*([^\]]+)\]", output, re.MULTILINE)
    return float(match.group(1)) if match else float("nan")


def hover_thrust():
    return scalar_field(listener_output("hover_thrust_estimate"), "hover_thrust", float("nan"))


def wait_ceiling_state(expected_states, timeout, label):
    deadline = time.time() + timeout
    while time.time() < deadline:
        send_offboard_sp(0, 0, TARGET_Z)
        status = ceiling_status()
        if status["state"] in expected_states:
            print(f"    [{label}] state={status['state']} dist={status['distance']:.3f}m", flush=True)
            return status
        time.sleep(0.05)
    print(f"    [{label}] TIMEOUT waiting for {sorted(expected_states)}", flush=True)
    return None


def get_pos():
    with lock:
        return dict(pos)


def log_state(label):
    p = get_pos()
    line = (f"[{label}] t={time.time()-t_start:.1f}s  "
            f"pos=({p['x']:.2f}, {p['y']:.2f}, {p['z']:.2f})  "
            f"vel=({p['vx']:.2f}, {p['vy']:.2f}, {p['vz']:.2f})")
    print(f"    {line}", flush=True)
    log_buffer.append(line)


def wait_stable(target_z, tol=0.3, timeout=15, label="stable"):
    t0 = time.time()
    while time.time() - t0 < timeout:
        send_offboard_sp(0, 0, target_z)
        p = get_pos()
        if abs(p['z'] - target_z) < tol:
            print(f"    [{label}] reached in {time.time()-t0:.1f}s", flush=True)
            return True
        time.sleep(0.05)
    print(f"    [{label}] TIMEOUT after {timeout}s", flush=True)
    return False


def stream_sp(x, y, z, duration_sec):
    n = int(duration_sec * OFFBOARD_RATE)
    for _ in range(n):
        send_offboard_sp(x, y, z)
        time.sleep(1.0 / OFFBOARD_RATE)


def set_mode(mode_num):
    port.mav.command_long_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_num, 0, 0, 0, 0, 0)


def wait_for_preflight(timeout=30):
    print("    waiting for EKF/preflight ...", flush=True)
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = port.recv_match(type='SYS_STATUS', blocking=False)
        if s:
            healthy = (s.onboard_control_sensors_health & 0b11111111) == 0b11111111
            if healthy:
                return True
        time.sleep(0.5)
    return False


def phase_delay(seconds, label="delay", x=0, y=0, z=None):
    """Print position every second during delay, keep streaming offboard setpoint at 20Hz"""
    if z is None:
        z = TARGET_Z
    for i in range(seconds):
        for _ in range(20):  # 20Hz setpoint stream
            send_offboard_sp(x, y, z)
            time.sleep(0.05)
        log_state(f"{label} {i+1}s/{seconds}s")


def phase_delay_no_sp(seconds, label="delay"):
    """Print position every second without sending setpoints (caller handles streaming)"""
    for i in range(seconds):
        time.sleep(1.0)
        log_state(f"{label} {i+1}s/{seconds}s")


def save_log(path="/tmp/ceiling_sitl_log.txt"):
    with open(path, 'w') as f:
        f.write("=== Ceiling SITL Flight Log ===\n\n")
        for line in log_buffer:
            f.write(line + "\n")
    print(f"\nLog saved to {path}", flush=True)


# ============== Main ==============
t_start = time.time()


def main():
    global running, inject_running, inject_dist_cm, inject_front_mm, inject_up_valid, mavutil, port, range_link

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--detach-case",
        choices=("aux2", "aux1", "sensor-timeout", "distance-timeout"),
        default="aux2",
        help="detach path to validate (default: dedicated AUX2 switch)")
    args = parser.parse_args()

    from pymavlink import mavutil as pymavlink_mavutil
    mavutil = pymavlink_mavutil

    print("Connecting to PX4 SITL (udp:127.0.0.1:14540) ...")
    port = mavutil.mavlink_connection('udp:127.0.0.1:14540')
    port.wait_heartbeat()
    print(f"  Connected (sysid={port.target_system}, compid={port.target_component})")
    port.mav.request_data_stream_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 4, 1)
    port.mav.request_data_stream_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
    threading.Thread(target=pos_thread, daemon=True).start()

    print("\n" + "=" * 60)
    print(f"  Ceiling Contact Controller SITL Test ({args.detach_case})")
    print("=" * 60)

    # --- Phase 0: Wait EKF ---
    print("\n[PHASE 0] Waiting for EKF / preflight ...", flush=True)
    if not wait_for_preflight(timeout=30):
        print("    WARNING: preflight timeout, proceeding anyway", flush=True)
    phase_delay(3, "preflight")

    # --- Phase 1: Stream offboard setpoint ---
    print("\n[PHASE 1] Warming up offboard stream (2s) ...", flush=True)
    stream_sp(0, 0, TARGET_Z, 2.0)
    log_state("after stream")

    # --- Phase 2: Switch to Offboard ---
    print("\n[PHASE 2] Switching to Offboard mode ...", flush=True)
    set_mode(6)
    phase_delay(3, "offboard")

    # --- Phase 3: Arm ---
    print("\n[PHASE 3] Arming ...", flush=True)
    port.mav.command_long_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    phase_delay(3, "armed")

    # --- Phase 4: Takeoff ---
    print(f"\n[PHASE 4] Takeoff to {abs(TARGET_Z):.0f}m ...", flush=True)
    stream_sp(0, 0, TARGET_Z, 6.0)
    if wait_stable(TARGET_Z, tol=0.3, timeout=10, label="takeoff"):
        log_state("takeoff OK")
    else:
        log_state("takeoff FAIL")
    phase_delay(3, "hover")

    # --- Phase 5: Start distance sensor injection ---
    print("\n[PHASE 5] Starting distance sensor injection (far) ...", flush=True)
    print(f"    inject_dist = {DETACH_DIST_CM}cm (no ceiling)", flush=True)
    range_link = DualRangePty()
    print(f"    pseudo UART = {range_link.slave_path}", flush=True)
    range_link.start_px4_modules()

    for name, value in (
            ("CEIL_D0", 0.22),
            ("CEIL_DIST_THR", 0.60),
            ("CEIL_COMP_TGT", 0.02),
            ("CEIL_RAMP_DIST", 0.10),
            ("CEIL_ENTRY_T", 0.10),
            ("CEIL_CONT_T", 0.10),
            ("CEIL_APPR_HOVER", 0.20),
            ("CEIL_ATTACH_TO", 10000),
            ("CEIL_DETACH_VZ", 0.20),
            ("CEIL_DET_THR", 0.25),
            ("CEIL_DET_DIST", 0.50),
            ("CEIL_UNLOAD_TO", 500),
            ("CEIL_DETACH_TO", 1000 if args.detach_case == "distance-timeout" else 8000),
            ("CEIL_RECOV_T", 1.0),
            ("CEIL_SENSOR_TO", 300),
            ("CEIL_RAMP_T", 0.30)):
        px4_command(f"param set {name} {value}", range_link.socket_path)

    inject_dist_cm = DETACH_DIST_CM
    inject_front_mm = 2000
    inject_up_valid = True
    inject_running = True
    sensor_t = threading.Thread(target=sensor_inject_thread, daemon=True)
    sensor_t.start()
    phase_delay(3, "sensor_far")

    # --- Phase 6: AUX1=ON + dynamic close distance (simulate ceiling) ---
    print("\n[PHASE 6] AUX1=ON + dynamic close distance ...", flush=True)
    print("    inject_dist = dynamic (based on actual altitude)", flush=True)
    print("    Expected state transition: NORMAL(0) -> ARM(1) -> APPROACH(2)", flush=True)
    set_rc(AUX_ON, AUX_OFF)
    inject_dist_cm = 0   # enable dynamic injection immediately
    phase_delay(3, "inject_close")

    print("    Expected: ARM(1) -> APPROACH(2) -> ATTACH(3)", flush=True)
    # Stream position setpoint continuously so offboard doesn't timeout.
    # mc_pos_control's APPROACH_MODE will override vz_sp=-0.08 upward.
    # ATTACH mode will override thrust_body_z directly.
    # We keep sending so offboard stays active and horizontal position is held.
    t0 = time.time()
    attached = False
    while time.time() - t0 < 20.0:
        send_offboard_sp(0, 0, TARGET_Z)
        status = ceiling_status()
        if 3 <= status["state"] <= 4:
            attached = True
            print(f"    [ATTACH DETECTED] state={status['state']} dist={status['distance']:.2f}m "
                  f"thrust_z={status['thrust_z']:.3f}", flush=True)
            break
        time.sleep(0.05)
    if attached:
        log_state("attach_confirmed")
    else:
        log_state("attach_timeout")

    # --- Phase 7: Hold attach ---
    print("\n[PHASE 7] Holding attach (6s) ...", flush=True)
    for i in range(6):
        for _ in range(20):  # 20Hz setpoint to keep offboard alive
            send_offboard_sp(0, 0, TARGET_Z)
            time.sleep(0.05)
        log_state(f"hold_attach {i+1}s/6s")
    log_state("after hold")

    # --- Phase 8: controlled detach and selected failure/switch path ---
    print(f"\n[PHASE 8] Starting controlled detach case: {args.detach_case} ...", flush=True)
    print("    Expected: DETACH(5) -> RECOVERY(6) -> NORMAL(0)", flush=True)
    inject_dist_cm = ATTACH_DIST_CM
    inject_front_mm = 2000

    if args.detach_case == "aux1":
        # Legacy path: lowering AUX1 while attached must also detach.
        set_rc(AUX_OFF, AUX_OFF)
    else:
        # Dedicated path: AUX1 deliberately remains high while AUX2 is raised.
        set_rc(AUX_ON, AUX_ON)

    detach_trigger_time = time.monotonic()
    detach_status = wait_ceiling_state({5}, 0.5, "DETACH entered")
    detach_latency = time.monotonic() - detach_trigger_time
    detach_entered_fast = detach_status is not None and detach_latency <= 0.20

    # Hold the measured distance briefly while the target ramps away. This must
    # produce a command below hover thrust rather than remaining pinned.
    minimum_thrust_seen = False
    bridge_match_seen = False
    hover = hover_thrust()
    if not math.isfinite(hover):
        hover = 0.72
    for sample in range(3):
        # Vary FRONT aggressively. It must not alter ceiling DETACH decisions.
        inject_front_mm = 300 if sample % 2 == 0 else 4000
        send_offboard_sp(0, 0, TARGET_Z)
        status = ceiling_status()
        actual_thrust = attitude_thrust_z()
        if status["state"] == 5:
            minimum_thrust_seen |= status["thrust_z"] > -hover
            bridge_match_seen |= abs(actual_thrust - status["thrust_z"]) < 0.02
        time.sleep(0.1)

    recovery_seen = False
    fault_case_passed = True

    if args.detach_case == "sensor-timeout":
        # Keep FRONT valid while invalidating UP only. DETACH must relinquish
        # direct thrust after the 500 ms freshness limit.
        inject_up_valid = False
        recovery_status = wait_ceiling_state({6}, 1.5, "UP timeout recovery")
        recovery_seen = recovery_status is not None
        fault_case_passed = (recovery_seen
                             and not recovery_status["sensor_valid"]
                             and recovery_status["fault_detected"]
                             and (int(recovery_status["fault_reason"]) & 2) != 0)
        inject_up_valid = True

    elif args.detach_case == "distance-timeout":
        # Keep UP close so the separation threshold is never reached.
        recovery_status = wait_ceiling_state({6}, 1.5, "detach timeout recovery")
        recovery_seen = recovery_status is not None
        fault_case_passed = (recovery_seen
                             and recovery_status["fault_detected"]
                             and (int(recovery_status["fault_reason"]) & 128) != 0)

    else:
        # Emulate physical separation using UP while continuing to vary FRONT.
        for distance_cm in range(ATTACH_DIST_CM, DETACH_DIST_CM + 1):
            inject_dist_cm = distance_cm
            inject_front_mm = 300 if distance_cm % 2 == 0 else 4000
            send_offboard_sp(0, 0, TARGET_Z)
            status = ceiling_status()
            if status["state"] == 6:
                recovery_seen = True
                break
            time.sleep(0.1)

    if not detach_entered_fast:
        print(f"    FAIL: {args.detach_case} did not enter DETACH within 200ms", flush=True)
    if not minimum_thrust_seen:
        print("    FAIL: no below-hover detach thrust observed", flush=True)
    if not bridge_match_seen:
        print("    FAIL: mc_pos_control did not apply detach thrust", flush=True)
    if not recovery_seen:
        print("    FAIL: DETACH did not reach RECOVERY", flush=True)
    if not fault_case_passed:
        print(f"    FAIL: {args.detach_case} fault flags are incorrect", flush=True)

    inject_dist_cm = DETACH_DIST_CM
    inject_front_mm = 2000

    if args.detach_case == "aux2":
        # Release AUX2 but keep AUX1 high. The latch must prevent re-engagement.
        set_rc(AUX_ON, AUX_OFF)
        normal_status = wait_ceiling_state({0}, 4.0, "RECOVERY complete")
        phase_delay(1, "rearm_latched")
        latched_status = ceiling_status()
        rearm_latched = (normal_status is not None
                         and latched_status["state"] == 0
                         and latched_status["rearm_required"])

        # Only an AUX1 low-to-high cycle may start a fresh engagement.
        set_rc(AUX_OFF, AUX_OFF)
        phase_delay(1, "AUX1 reset")
        set_rc(AUX_ON, AUX_OFF)
        rearmed_status = wait_ceiling_state({1}, 1.0, "AUX1 re-armed")

    else:
        # These paths already leave AUX1 low, so recovery can clear the latch.
        set_rc(AUX_OFF, AUX_OFF)
        normal_status = wait_ceiling_state({0}, 4.0, "RECOVERY complete")
        stream_sp(0, 0, TARGET_Z, 0.3)
        reset_status = ceiling_status()
        set_rc(AUX_ON, AUX_OFF)
        rearmed_status = wait_ceiling_state({1}, 1.0, "AUX1 re-armed")
        rearm_latched = normal_status is not None and not reset_status["rearm_required"]

    set_rc(AUX_OFF, AUX_OFF)

    detached = detach_entered_fast and minimum_thrust_seen and bridge_match_seen \
        and recovery_seen and fault_case_passed and rearm_latched and rearmed_status is not None
    log_state("detach_confirmed" if detached else "detach_failed")

    # --- Phase 9: Stop injection ---
    print("\n[PHASE 9] Stopping sensor injection ...", flush=True)
    inject_running = False
    sensor_t.join(timeout=1)
    range_link.stop_px4_uart()
    range_link.close()
    range_link = None
    phase_delay(3, "post_inject", z=TARGET_Z)

    # --- Phase 10: Land ---
    print("\n[PHASE 10] Landing ...", flush=True)
    # Send LAND command to trigger auto-landing
    port.mav.command_long_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0)
    # Stream z=0 setpoints to descend (keep offboard alive if still in offboard)
    stream_sp(0, 0, 0, 3.0)

    # Wait for actual touchdown
    t0 = time.time()
    landed_detected = False
    while time.time() - t0 < 15.0:
        send_offboard_sp(0, 0, 0)
        m = port.recv_match(type='EXTENDED_SYS_STATE', blocking=False)
        if m and m.landed_state == 1:  # MAV_LANDED_STATE_ON_GROUND
            landed_detected = True
            break
        time.sleep(0.1)
    if landed_detected:
        log_state("landed_confirmed")
    else:
        log_state("land_timeout")

    # --- Phase 11: Disarm ---
    print("\n[PHASE 11] Disarming ...", flush=True)
    port.mav.command_long_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    phase_delay(3, "disarmed", z=0)

    # Stop pos thread
    running = False

    # --- Summary ---
    print("\n" + "=" * 60)
    print("  Test Complete")
    print("=" * 60)
    save_log("/tmp/ceiling_sitl_log.txt")
    print("\nCheck PX4 console for [CeilCtrl] state transitions", flush=True)
    if not detached:
        raise RuntimeError(f"ceiling detach case failed: {args.detach_case}")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        running = False
        inject_running = False
        if range_link is not None:
            range_link.stop_px4_uart()
            range_link.close()
        save_log("/tmp/ceiling_sitl_log.txt")
        print("\nInterrupted by user. Log saved.", flush=True)
    except Exception:
        running = False
        inject_running = False
        if range_link is not None:
            range_link.stop_px4_uart()
            range_link.close()
        raise
