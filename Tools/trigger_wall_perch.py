#!/usr/bin/env python3
#
# trigger_wall_perch.py
#
# SITL script that triggers the wall_perch 90-degree flip maneuver.
#
# Strategy:
#   1. Start uart_rx/CeilingReader/wall_perch and feed the ESP32 frame via PTY.
#   2. Set wall_perch + RC params via MAVLink param_set.
#   3. Send RC override IMMEDIATELY (ch1-4=1500 neutral, ch5=AUX_OFF) so
#      rc_update (which is callback-driven on input_rc) actually runs,
#      processes the param_update, and calibrates (_rc_calibrated=true,
#      RC_MAP_AUX1→channel 5 mapped).
#   4. Take off with OFFBOARD + MAVLink arm, then switch to ALTCTL.
#   5. Raise ch5 to 2000 (AUX_ON) → rc_update publishes
#      manual_control_input.aux1=1.0 → manual_control_setpoint.aux1=1.0
#      → wall_perch sees start_req=true and triggers.
#   6. Watch for |pitch| > 60 deg, then reduce UP range to confirm contact.
#
# Usage:
#   Terminal 1: make px4_sitl gz_x500
#   Terminal 2: python3 Tools/trigger_wall_perch.py

import sys
import time
import threading

from pymavlink import mavutil

from esp32_dual_range_sitl import DualRangePty, px4_command

PORT = sys.argv[1] if len(sys.argv) > 1 else "14540"
CONN = f"udp:127.0.0.1:{PORT}"

PX4_MODE_OFFBOARD = 6
PX4_MODE_ALTCTL = 2

TARGET_Z = -1.5
OFFBOARD_RATE = 50
AUX_OFF = 1000             # low → aux1 = -1.0; required to clear the re-arm latch
AUX_ON  = 2000             # high → aux1 ≈ 1.0

# ---- Shared state ----
lock = threading.Lock()
state = {
    "z": 0.0, "vz": 0.0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "armed": False,
    "have_lpos": False,
    "health_bad": None,
    "main_mode": None,
}

running = True
sp_z = TARGET_Z
aux5 = AUX_OFF  # RC channel 5 value (shared between main and rc_thread)
sensor_up_mm = 1000
range_link = None


def main():
    global running, sp_z, aux5, sensor_up_mm, range_link

    m = mavutil.mavlink_connection(CONN)
    m.wait_heartbeat(timeout=30)
    sysid, compid = m.target_system, m.target_component
    print(f"[*] heartbeat from sys={sysid} comp={compid}")

    range_link = DualRangePty()
    print(f"[*] pseudo UART: {range_link.slave_path}")
    range_link.start_px4_modules(start_wall_perch=True)

    # Request telemetry streams
    m.mav.request_data_stream_send(
        sysid, compid, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 4, 1)
    m.mav.request_data_stream_send(
        sysid, compid, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 20, 1)
    m.mav.request_data_stream_send(
        sysid, compid, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20, 1)

    # ---- Parameter helpers ----
    def pid(msg):
        p = msg.param_id
        if isinstance(p, bytes):
            p = p.decode(errors="ignore")
        return p.rstrip("\0")

    def get_param_type(name):
        req = name.encode() + b"\0" * (16 - len(name))
        m.mav.param_request_read_send(sysid, compid, req, -1)
        deadline = time.time() + 3
        while time.time() < deadline:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg and pid(msg) == name:
                return msg.param_type
        return None

    def set_param(name, value):
        ptype = get_param_type(name)
        if ptype is None:
            print(f"[!] could not fetch type for {name}; defaulting REAL32")
            ptype = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        req = name.encode() + b"\0" * (16 - len(name))
        m.mav.param_set_send(sysid, compid, req, float(value), ptype)
        deadline = time.time() + 3
        while time.time() < deadline:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg and pid(msg) == name:
                print(f"[*] {name} = {msg.param_value} (type={ptype})")
                return
        print(f"[!] no ack for {name}")

    # ---- RC override helper ----
    def set_rc(ch1, ch2, ch3, ch4, ch5, ch6):
        """Full 18-channel override. ch5/6 map to start/detach."""
        m.mav.rc_channels_override_send(
            sysid, compid,
            ch1, ch2, ch3, ch4, ch5, ch6,
            65535, 65535, 65535, 65535, 65535, 65535,
            65535, 65535, 65535, 65535, 65535, 65535)

    # ---- Configure params ----
    set_param("WP_PIN_ENABLE", 0)       # SITL: use the standard attitude-control chain
    set_param("WP_FLP_TRD_DIST", 0.5)    # front 0.3 ≤ 0.5 → flip_ready passes
    set_param("WP_TOP_CT_DIST", 0.08)    # UP changes to 0.04m after flip

    # RC mapping params — rc_update must be calibrated for
    # manual_control_input.valid to be true AND for aux1 to be mapped.
    set_param("COM_RC_IN_MODE", 2)  # accept MAVLink RC_CHANNELS_OVERRIDE
    set_param("RC_CHAN_CNT", 8)
    set_param("RC_MAP_ROLL", 1)
    set_param("RC_MAP_PITCH", 2)
    set_param("RC_MAP_YAW", 3)
    set_param("RC_MAP_THROTTLE", 4)
    set_param("RC_MAP_AUX1", 5)
    set_param("RC_MAP_AUX2", 6)

    # RC channel calibration: without these, rc_update's interpolateNXY
    # sees min=trim=max=0 for aux channels → output always 0 → aux1=0.
    set_param("RC5_MIN", 1000)
    set_param("RC5_TRIM", 1500)
    set_param("RC5_MAX", 2000)
    set_param("RC6_MIN", 1000)
    set_param("RC6_TRIM", 1500)
    set_param("RC6_MAX", 2000)

    # ---- RC override sender (runs continuously at ~10 Hz) ----
    # rc_update is callback-driven on input_rc.  We must send override
    # *before* the trigger so rc_update processes param_update and
    # stabilises (channel_count_stable, _rc_calibrated).
    def rc_thread():
        while running:
            set_rc(1500, 1500, 1500, 1500, aux5, 1500)
            time.sleep(0.1)
    threading.Thread(target=rc_thread, daemon=True).start()

    # ---- RX thread ----
    def rx_thread():
        while running:
            msg = m.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "LOCAL_POSITION_NED":
                with lock:
                    state["z"] = msg.z
                    state["vz"] = msg.vz
                    state["have_lpos"] = True
            elif t == "ATTITUDE":
                with lock:
                    state["roll"] = msg.roll
                    state["pitch"] = msg.pitch
                    state["yaw"] = msg.yaw
            elif t == "HEARTBEAT":
                with lock:
                    state["armed"] = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    state["main_mode"] = (msg.custom_mode >> 16) & 0xff
            elif t == "SYS_STATUS":
                bad = msg.onboard_control_sensors_enabled & ~msg.onboard_control_sensors_health
                with lock:
                    state["health_bad"] = bad
            elif t == "STATUSTEXT":
                text = msg.text
                if isinstance(text, bytes):
                    text = text.decode(errors="ignore")
                if text.strip():
                    print(f"    [PX4] {text.strip()}")
    threading.Thread(target=rx_thread, daemon=True).start()

    # ---- Offboard setpoint sender ----
    def offboard_thread():
        mask = 0b110111111000
        while running:
            m.mav.set_position_target_local_ned_send(
                0, sysid, compid,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                mask,
                0, 0, sp_z, 0, 0, 0, 0, 0, 0, 0, 0)
            time.sleep(1.0 / OFFBOARD_RATE)
    threading.Thread(target=offboard_thread, daemon=True).start()

    def get(k):
        with lock:
            return state[k]

    def set_main_mode(mode, name):
        print(f"[*] set mode {name}")
        m.mav.command_long_send(
            sysid, compid, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode, 0, 0, 0, 0, 0)
        deadline = time.time() + 5.0

        while time.time() < deadline:
            if get("main_mode") == mode:
                return

            time.sleep(0.1)

        raise RuntimeError(f"PX4 did not enter {name}")

    # --- Wait for rc_update to stabilise (channel_count_stable) ---
    print("[*] waiting for rc_update to stabilise (2 s) ...")
    time.sleep(2.0)

    # --- Warm up offboard stream ---
    print("[*] warming up offboard stream (2 s) ...")
    time.sleep(2.0)

    # --- Switch to OFFBOARD ---
    set_main_mode(PX4_MODE_OFFBOARD, "OFFBOARD")

    # --- Arm ---
    armed = False
    for attempt in range(6):
        print(f"[*] arm attempt {attempt + 1}")
        m.mav.command_long_send(
            sysid, compid, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        deadline = time.time() + 4
        while time.time() < deadline:
            if get("armed"):
                armed = True
                break
            time.sleep(0.1)
        if armed:
            print("[*] ARMED")
            break
        bad = get("health_bad")
        if bad is not None:
            print(f"[!] arm failed (health missing bits: 0x{bad:06x}); retrying")
        else:
            print("[!] arm failed; retrying")
        time.sleep(2)
    if not armed:
        print("[!] could not arm.")
        running = False
        range_link.stop_px4_uart()
        range_link.close()
        range_link = None
        return

    # --- Takeoff ---
    print(f"[*] takeoff to {abs(TARGET_Z):.1f} m ...")
    t0 = time.time()
    while time.time() - t0 < 15:
        z = get("z")
        if get("have_lpos") and -z > abs(TARGET_Z) - 0.2:
            break
        time.sleep(0.1)
    print(f"[*] altitude reached: {-get('z'):.2f} m; hovering 3 s")
    time.sleep(3.0)

    # --- Feed the dual-range ESP32 frame AFTER takeoff + hover ---
    # FRONT=0.3m triggers approach. UP stays far until the flip is observed.
    print("[*] feeding ESP32 frames: FRONT=0.3m, UP=1.0m ...")
    def inject_thread():
        while running:
            range_link.write_frame(sensor_up_mm, 300)
            time.sleep(0.03)
    threading.Thread(target=inject_thread, daemon=True).start()
    time.sleep(0.5)  # let a few samples arrive

    # WALL deliberately accepts starts only in ALTCTL. Neutral throttle keeps
    # altitude while the fixed AUX1 channel is used for the rising-edge trigger.
    set_main_mode(PX4_MODE_ALTCTL, "ALTCTL")

    # --- Trigger wall_perch through its fixed AUX1 channel ---
    print("[*] TRIGGER wall_perch")
    aux5 = AUX_ON

    # --- Watch for the 90-degree flip ---
    print("[*] watching for flip (|pitch| > 60 deg) ...")
    t0 = time.time()
    flipped = False
    while time.time() - t0 < 30:
        deg = get("pitch") * 57.2958
        if abs(deg) > 60:
            print(f"[+] FLIP DETECTED: pitch = {deg:.1f} deg "
                  f"(roll={get('roll')*57.2958:.1f})")
            flipped = True
            break
        time.sleep(0.05)
    if not flipped:
        print("[!] flip not detected within timeout")

    else:
        sensor_up_mm = 40
        print("[*] UP reduced to 0.04m; waiting for WALL_CAPTURE/WALL_HOLD ...")
        time.sleep(3.0)
        _, status_output = px4_command(
            "listener wall_perch_status -n 1", range_link.socket_path, check=False)
        if status_output:
            print(status_output)

    print("[*] done")
    running = False
    range_link.stop_px4_uart()
    range_link.close()
    range_link = None


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        running = False
        if range_link is not None:
            range_link.stop_px4_uart()
            range_link.close()
        print("\n[!] interrupted by user")
    except Exception:
        running = False
        if range_link is not None:
            range_link.stop_px4_uart()
            range_link.close()
        raise
