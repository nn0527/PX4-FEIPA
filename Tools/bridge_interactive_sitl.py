#!/usr/bin/env python3
"""Interactive bridge-inspection SITL driver.

Gazebo supplies real UP and FRONT LaserScan measurements.  This program reads
those Gazebo topics, encodes the measurements with the ESP32 nine-byte UART
protocol, and feeds PX4 through a pseudo-TTY.  It also maintains Offboard
position setpoints and RC overrides so AUX1/AUX2/AUX3 can be controlled from
this terminal.

Start PX4 first with::

    make px4_sitl gz_x500_dual_range_bridge

Then run this program and use ``help`` at the interactive prompt.
"""

import argparse
import json
import math
import os
import re
import subprocess
import threading
import time
from typing import Dict, Optional, Tuple

from esp32_dual_range_sitl import DualRangePty, INVALID_DISTANCE, px4_command, self_test as frame_self_test


UP_TOPIC = "/bridge/x500_dual_range/up"
FRONT_TOPIC = "/bridge/x500_dual_range/front"
FRAME_RATE_HZ = 30.0
SENSOR_TIMEOUT_S = 0.5
CEILING_ALTITUDE_M = 6.0
CEILING_ALTITUDE_TOLERANCE_M = 0.05
CEILING_DETACH_RETREAT_M = 1.0
CEILING_MIN_RETREAT_ALTITUDE_M = 0.5
WALL_ALTITUDE_M = 1.5
WALL_STANDOFF_M = 0.45
PX4_OFFBOARD_MODE = 6
AUX_OFF = 1000
AUX_ON = 2000


def laser_range_from_json(message: Dict) -> Optional[float]:
    """Extract the median finite range from a narrow Gazebo LaserScan."""
    ranges = message.get("ranges")
    if not isinstance(ranges, list) or not ranges:
        return None

    range_min = message.get("rangeMin", message.get("range_min", 0.0))
    range_max = message.get("rangeMax", message.get("range_max", float("inf")))

    try:
        minimum = float(range_min)
        maximum = float(range_max)
    except (TypeError, ValueError):
        return None

    valid = []

    for value in ranges:
        try:
            distance = float(value)
        except (TypeError, ValueError):
            continue

        if math.isfinite(distance) and minimum <= distance <= maximum and distance > 0.0:
            valid.append(distance)

    if not valid:
        return None

    valid.sort()
    return valid[len(valid) // 2]


def metres_to_uart_mm(distance_m: Optional[float]) -> int:
    """Convert a Gazebo range to the ESP32 invalid marker or uint16 mm."""
    if distance_m is None or not math.isfinite(distance_m) or distance_m <= 0.0:
        return INVALID_DISTANCE

    distance_mm = int(round(distance_m * 1000.0))
    return distance_mm if 0 < distance_mm < INVALID_DISTANCE else INVALID_DISTANCE


class GazeboLaserReader:
    """Read a Gazebo topic through the installed CLI without Python bindings."""

    def __init__(self, topic: str) -> None:
        self.topic = topic
        self._lock = threading.Lock()
        self._distance_m: Optional[float] = None
        self._timestamp = 0.0
        self._messages = 0
        self._parse_errors = 0
        self._running = False
        self._process: Optional[subprocess.Popen] = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        environment = os.environ.copy()
        environment.setdefault("GZ_IP", "127.0.0.1")
        self._process = subprocess.Popen(
            ["gz", "topic", "-e", "-t", self.topic, "--json-output"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            env=environment,
        )
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self) -> None:
        assert self._process is not None and self._process.stdout is not None
        decoder = json.JSONDecoder()
        buffer = ""

        while self._running:
            try:
                chunk = os.read(self._process.stdout.fileno(), 4096)
            except OSError:
                break

            if not chunk:
                break

            buffer += chunk.decode("utf-8", errors="ignore")

            while buffer:
                buffer = buffer.lstrip()

                if not buffer:
                    break

                if not buffer.startswith("{"):
                    object_start = buffer.find("{")

                    if object_start < 0:
                        buffer = ""
                        break

                    buffer = buffer[object_start:]

                try:
                    message, end = decoder.raw_decode(buffer)
                except json.JSONDecodeError:
                    if len(buffer) > 1024 * 1024:
                        self._parse_errors += 1
                        buffer = buffer[-65536:]

                    break

                buffer = buffer[end:]
                distance = laser_range_from_json(message)

                with self._lock:
                    self._distance_m = distance
                    self._timestamp = time.monotonic()
                    self._messages += 1

    def latest(self, timeout_s: float = SENSOR_TIMEOUT_S) -> Tuple[Optional[float], float]:
        with self._lock:
            distance = self._distance_m
            timestamp = self._timestamp

        age = time.monotonic() - timestamp if timestamp else float("inf")
        return (distance if age <= timeout_s else None), age

    def counters(self) -> Tuple[int, int]:
        with self._lock:
            return self._messages, self._parse_errors

    def stop(self) -> None:
        self._running = False

        if self._process is not None and self._process.poll() is None:
            self._process.terminate()

            try:
                self._process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=1.0)

        if self._thread is not None:
            self._thread.join(timeout=1.0)


class GazeboUartBridge:
    """Merge two Gazebo sensors into the physical ESP32 UART wire format."""

    def __init__(self, socket_path: str, rate_hz: float, timeout_s: float) -> None:
        self.socket_path = socket_path
        self.rate_hz = rate_hz
        self.timeout_s = timeout_s
        self.up = GazeboLaserReader(UP_TOPIC)
        self.front = GazeboLaserReader(FRONT_TOPIC)
        self.pty = DualRangePty(socket_path)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.frames_sent = 0

    @staticmethod
    def verify_topics() -> None:
        environment = os.environ.copy()
        environment.setdefault("GZ_IP", "127.0.0.1")
        result = subprocess.run(
            ["gz", "topic", "-l"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
            env=environment,
        )
        advertised = set(result.stdout.splitlines())
        missing = [topic for topic in (UP_TOPIC, FRONT_TOPIC) if topic not in advertised]

        if missing:
            raise RuntimeError(
                "Gazebo双测距话题未出现：" + ", ".join(missing)
                + "\n请确认使用 make px4_sitl gz_x500_dual_range_bridge 启动。"
            )

    def start(self) -> None:
        try:
            self.verify_topics()
            self.up.start()
            self.front.start()
            self.pty.start_px4_modules(start_wall_perch=False, start_ceiling_controller=False)
            self._running = True
            self._thread = threading.Thread(target=self._frame_loop, daemon=True)
            self._thread.start()

            deadline = time.monotonic() + 5.0

            while time.monotonic() < deadline:
                _, up_age = self.up.latest(self.timeout_s)
                _, front_age = self.front.latest(self.timeout_s)

                if math.isfinite(up_age) and math.isfinite(front_age):
                    return

                time.sleep(0.05)

            raise RuntimeError("Gazebo测距话题存在，但5秒内没有收到LaserScan数据；请确认仿真未暂停。")
        except Exception:
            self.stop()
            raise

    def _frame_loop(self) -> None:
        period = 1.0 / self.rate_hz
        next_send = time.monotonic()

        while self._running:
            up_m, _ = self.up.latest(self.timeout_s)
            front_m, _ = self.front.latest(self.timeout_s)
            self.pty.write_frame(metres_to_uart_mm(up_m), metres_to_uart_mm(front_m))
            self.frames_sent += 1
            next_send += period
            time.sleep(max(0.0, next_send - time.monotonic()))

    def ranges(self) -> Dict[str, Optional[float]]:
        up_m, up_age = self.up.latest(self.timeout_s)
        front_m, front_age = self.front.latest(self.timeout_s)
        return {
            "up": up_m,
            "front": front_m,
            "up_age": up_age,
            "front_age": front_age,
        }

    def stop(self) -> None:
        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=1.0)

        self.up.stop()
        self.front.stop()

        try:
            self.pty.stop_px4_uart()
        except (OSError, RuntimeError):
            pass

        self.pty.close()


class MavlinkFlight:
    """MAVLink telemetry, Offboard setpoints and terminal-controlled AUX input."""

    def __init__(self, connection: str) -> None:
        from pymavlink import mavutil

        self.mavutil = mavutil
        print(f"连接PX4 MAVLink：{connection} ...", flush=True)
        self.port = mavutil.mavlink_connection(connection)
        self.port.wait_heartbeat(timeout=30)
        self.system_id = self.port.target_system
        self.component_id = self.port.target_component
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._running = True
        self._setpoint_enabled = True
        self._state = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "have_position": False,
            "armed": False,
            "main_mode": -1,
        }
        self._target = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        self._aux = {1: AUX_OFF, 2: AUX_OFF, 3: AUX_OFF}

        self.port.mav.request_data_stream_send(
            self.system_id, self.component_id,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 20, 1)
        self.port.mav.request_data_stream_send(
            self.system_id, self.component_id,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20, 1)
        self.port.mav.request_data_stream_send(
            self.system_id, self.component_id,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 4, 1)

        self._rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._setpoint_thread = threading.Thread(target=self._setpoint_loop, daemon=True)
        self._rc_thread = threading.Thread(target=self._rc_loop, daemon=True)
        self._rx_thread.start()
        self._setpoint_thread.start()
        self._rc_thread.start()

    def _receive_loop(self) -> None:
        while self._running:
            message = self.port.recv_match(blocking=True, timeout=1.0)

            if message is None:
                continue

            message_type = message.get_type()

            with self._lock:
                if message_type == "LOCAL_POSITION_NED":
                    self._state.update({
                        "x": message.x,
                        "y": message.y,
                        "z": message.z,
                        "vx": message.vx,
                        "vy": message.vy,
                        "vz": message.vz,
                        "have_position": True,
                    })
                elif message_type == "ATTITUDE":
                    self._state.update({"roll": message.roll, "pitch": message.pitch, "yaw": message.yaw})
                elif message_type == "HEARTBEAT":
                    self._state["armed"] = bool(
                        message.base_mode & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self._state["main_mode"] = (message.custom_mode >> 16) & 0xFF

            if message_type == "STATUSTEXT":
                status_text = message.text
                if isinstance(status_text, bytes):
                    status_text = status_text.decode(errors="ignore")
                if status_text.strip():
                    print(f"\n[PX4] {status_text.strip()}", flush=True)

    def _setpoint_loop(self) -> None:
        # Position and yaw enabled; velocity, acceleration and yaw-rate ignored.
        type_mask = 0x9F8

        while self._running:
            with self._lock:
                target = dict(self._target)
                enabled = self._setpoint_enabled

            if enabled:
                with self._send_lock:
                    self.port.mav.set_position_target_local_ned_send(
                        0, self.system_id, self.component_id,
                        self.mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        type_mask,
                        target["x"], target["y"], target["z"],
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        target["yaw"], 0.0,
                    )

            time.sleep(0.05)

    def _rc_loop(self) -> None:
        next_heartbeat = 0.0

        while self._running:
            with self._lock:
                aux1, aux2, aux3 = self._aux[1], self._aux[2], self._aux[3]

            with self._send_lock:
                now = time.monotonic()

                # This terminal tool is the GCS for an interactive, QGC-free
                # run.  Sending a GCS heartbeat satisfies PX4's data-link
                # arming check without changing or bypassing any parameters.
                if now >= next_heartbeat:
                    self.port.mav.heartbeat_send(
                        self.mavutil.mavlink.MAV_TYPE_GCS,
                        self.mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0,
                    )
                    next_heartbeat = now + 1.0

                self.port.mav.rc_channels_override_send(
                    self.system_id, self.component_id,
                    1500, 1500, 1500, 1500,
                    aux1, aux2, aux3,
                    65535, 65535, 65535, 65535, 65535,
                    65535, 65535, 65535, 65535, 65535, 65535,
                )

            time.sleep(0.1)

    def snapshot(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._state)

    def set_target(self, x: float, y: float, z: float, yaw: float) -> None:
        with self._lock:
            self._target.update({"x": x, "y": y, "z": z, "yaw": yaw})
            self._setpoint_enabled = True

    def set_aux(self, index: int, enabled: bool) -> None:
        with self._lock:
            self._aux[index] = AUX_ON if enabled else AUX_OFF

    def aux_snapshot(self) -> Dict[int, int]:
        with self._lock:
            return dict(self._aux)

    def wait_for_position(self, timeout_s: float = 15.0) -> Dict[str, float]:
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            state = self.snapshot()
            if state["have_position"]:
                return state
            time.sleep(0.1)

        raise RuntimeError("没有收到LOCAL_POSITION_NED；请等待PX4 EKF完成初始化。")

    def _command_long(self, command: int, param1: float, param2: float = 0.0) -> None:
        with self._send_lock:
            self.port.mav.command_long_send(
                self.system_id, self.component_id, command, 0,
                param1, param2, 0.0, 0.0, 0.0, 0.0, 0.0)

    def ensure_offboard_and_armed(self) -> None:
        state = self.wait_for_position()
        self.set_target(state["x"], state["y"], state["z"], state["yaw"])
        time.sleep(1.2)

        if self.snapshot()["main_mode"] != PX4_OFFBOARD_MODE:
            for _ in range(3):
                self._command_long(
                    self.mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    self.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    PX4_OFFBOARD_MODE,
                )
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    if self.snapshot()["main_mode"] == PX4_OFFBOARD_MODE:
                        break
                    time.sleep(0.1)
                if self.snapshot()["main_mode"] == PX4_OFFBOARD_MODE:
                    break

        if self.snapshot()["main_mode"] != PX4_OFFBOARD_MODE:
            raise RuntimeError("无法进入OFFBOARD；请查看PX4控制台中的拒绝原因。")

        if not self.snapshot()["armed"]:
            for _ in range(5):
                self._command_long(self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    if self.snapshot()["armed"]:
                        return
                    time.sleep(0.1)

            raise RuntimeError("无法解锁；请根据PX4/QGC健康检查信息排除preflight错误。")

    def wait_at_altitude(self, altitude_m: float, tolerance_m: float,
                         stable_s: float, timeout_s: float) -> None:
        stable_since = 0.0
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            state = self.snapshot()
            error = abs((-state["z"]) - altitude_m)

            if error <= tolerance_m and abs(state["vz"]) <= 0.15:
                if stable_since == 0.0:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= stable_s:
                    return
            else:
                stable_since = 0.0

            time.sleep(0.1)

        state = self.snapshot()
        raise RuntimeError(
            f"高度等待超时：当前={-state['z']:.2f} m，目标={altitude_m:.2f} m")

    def land(self) -> None:
        if not self.snapshot()["armed"]:
            print("飞行器已经上锁。")
            return

        print("下降并降落 ...", flush=True)
        with self._lock:
            self._setpoint_enabled = False

        # NAV_LAND switches PX4 to its standard landing controller.  This is
        # important because an Offboard z=0 setpoint can keep producing thrust
        # at ground contact and prevent the land detector from asserting.
        self._command_long(self.mavutil.mavlink.MAV_CMD_NAV_LAND, 0.0)
        deadline = time.monotonic() + 45.0

        while time.monotonic() < deadline:
            if not self.snapshot()["armed"]:
                return
            time.sleep(0.2)

        raise RuntimeError("自动降落超时；请查看PX4的NAV_LAND和land_detector状态。")

    def stop(self) -> None:
        self._running = False
        for thread in (self._rx_thread, self._setpoint_thread, self._rc_thread):
            thread.join(timeout=1.0)
        self.port.close()


def parse_param_value(output: str, name: str) -> Optional[float]:
    for line in output.splitlines():
        if name not in line:
            continue
        match = re.search(r":\s*(-?(?:\d+(?:\.\d*)?|\.\d+))", line)
        if match:
            return float(match.group(1))
    return None


class BridgeInteractive:
    CEILING_STATES = {
        0: "NORMAL", 1: "ARM", 2: "APPROACH", 3: "ATTACH",
        4: "SURFACE", 5: "DETACH", 6: "RECOVERY",
    }
    WALL_STATES = {
        0: "IDLE", 1: "FRONT_WALL_DETECT", 2: "STABILIZE", 3: "SLOW_APPROACH",
        4: "FLIP", 5: "WALL_CAPTURE", 6: "WALL_HOLD", 7: "WALL_PIN",
        8: "DETACH_ROTATE", 9: "RECOVER", 10: "EXIT", 11: "ABORT",
    }

    def __init__(self, args: argparse.Namespace) -> None:
        self.socket_path = args.socket
        self.mode: Optional[str] = None
        self.range_bridge = GazeboUartBridge(args.socket, args.frame_rate, args.sensor_timeout)
        self.flight: Optional[MavlinkFlight] = None

        print("检查Gazebo双测距并启动UART数据链 ...", flush=True)
        try:
            self.range_bridge.start()
            self.flight = MavlinkFlight(args.mavlink)
            self.flight.wait_for_position()
            self._px4("ceiling_controller stop", check=False)
            self._px4("wall_perch stop", check=False)
            self.check_parameters()
        except Exception:
            if self.flight is not None:
                self.flight.stop()
            self.range_bridge.stop()
            raise

    def _px4(self, command: str, check: bool = True) -> str:
        _, output = px4_command(command, self.socket_path, check=check)
        return output

    def _param(self, name: str) -> Optional[float]:
        return parse_param_value(self._px4(f"param show {name}", check=False), name)

    def check_parameters(self) -> None:
        required = {
            "COM_RC_IN_MODE": 2,
            "RC_MAP_AUX1": 5,
            "RC_MAP_AUX2": 6,
            "RC_MAP_AUX3": 7,
            "WP_ENABLE": 1,
            "WP_AUX_CH": 1,
            "WP_DETACH_AUX_CH": 2,
            "WP_CANCEL_AUX_CH": 3,
            "WP_PIN_ENABLE": 0,
        }
        warnings = []

        for name, expected in required.items():
            actual = self._param(name)
            if actual is None or int(round(actual)) != expected:
                warnings.append(f"{name}={actual!r}（建议{expected}）")

        approach_start = self._param("CEIL_APPR_START")
        detach_threshold = self._param("CEIL_DIST_THR")
        if approach_start is None:
            warnings.append("CEIL_APPR_START不存在；需要重新编译PX4")
        elif detach_threshold is not None and approach_start <= detach_threshold:
            warnings.append(
                f"CEIL_APPR_START={approach_start:g}应大于CEIL_DIST_THR={detach_threshold:g}")

        if warnings:
            print("\n参数检查警告（工具不会自动改参，请在QGC中修改）：")
            for warning in warnings:
                print(f"  - {warning}")
        else:
            print("参数检查通过；参数来源为PX4/QGC，交互工具未覆盖参数。")

    def set_mode(self, requested: str) -> None:
        assert self.flight is not None
        if requested not in ("ceiling", "wall"):
            raise ValueError("mode只支持ceiling或wall")

        self.flight.set_aux(1, False)
        self.flight.set_aux(2, False)
        self.flight.set_aux(3, False)
        self._px4("ceiling_controller stop", check=False)
        self._px4("wall_perch stop", check=False)
        time.sleep(0.25)

        if requested == "ceiling":
            output = self._px4("ceiling_controller start")
        else:
            output = self._px4("wall_perch start")

        if output:
            print(output)
        self.mode = requested
        print(f"已进入{requested}模式；另一控制器已停止。")

    def prepare_ceiling(self) -> None:
        assert self.flight is not None
        if self.mode != "ceiling":
            raise RuntimeError("请先输入 mode ceiling")

        self.flight.set_aux(1, False)
        self.flight.set_aux(2, False)
        state = self.flight.wait_for_position()
        self.flight.set_target(state["x"], state["y"], -CEILING_ALTITUDE_M, state["yaw"])
        self.flight.ensure_offboard_and_armed()
        # ensure_offboard_and_armed initially holds the current point, so apply the climb target again
        state = self.flight.snapshot()
        self.flight.set_target(state["x"], state["y"], -CEILING_ALTITUDE_M, state["yaw"])
        print(f"上升到{CEILING_ALTITUDE_M:.2f} ± {CEILING_ALTITUDE_TOLERANCE_M:.2f} m ...")
        self.flight.wait_at_altitude(
            CEILING_ALTITUDE_M, CEILING_ALTITUDE_TOLERANCE_M, stable_s=2.0, timeout_s=45.0)
        ranges = self.range_bridge.ranges()
        print(
            f"Ceiling准备完成：高度={-self.flight.snapshot()['z']:.2f} m，"
            f"真实UP={self._format_range(ranges['up'])}。现在可输入 aux1 on。")

    def prepare_wall(self) -> None:
        assert self.flight is not None
        if self.mode != "wall":
            raise RuntimeError("请先输入 mode wall")

        self.flight.set_aux(1, False)
        self.flight.set_aux(2, False)
        self.flight.set_aux(3, False)
        state = self.flight.wait_for_position()
        hold_x, hold_y = state["x"], state["y"]
        # PX4 yaw=0 is NED north, which maps to Gazebo +Y and the left wall.
        self.flight.set_target(hold_x, hold_y, -WALL_ALTITUDE_M, 0.0)
        self.flight.ensure_offboard_and_armed()
        self.flight.set_target(hold_x, hold_y, -WALL_ALTITUDE_M, 0.0)
        print(f"上升到{WALL_ALTITUDE_M:.2f} m并朝向左墙 ...")
        self.flight.wait_at_altitude(WALL_ALTITUDE_M, 0.08, stable_s=1.0, timeout_s=30.0)
        time.sleep(1.0)

        print(f"根据真实FRONT距离慢速接近，在{WALL_STANDOFF_M:.2f} m处悬停 ...")
        deadline = time.monotonic() + 35.0
        target_x = self.flight.snapshot()["x"]
        stable_since = 0.0

        while time.monotonic() < deadline:
            state = self.flight.snapshot()
            ranges = self.range_bridge.ranges()
            front = ranges["front"]

            if front is None:
                stable_since = 0.0
                time.sleep(0.1)
                continue

            error = front - WALL_STANDOFF_M

            if error > 0.03:
                # Accumulate the forward target to obtain a bounded approach
                # speed instead of presenting the position controller with a
                # permanently tiny error.
                target_x += min(0.04, max(0.005, error * 0.08))
                self.flight.set_target(target_x, hold_y, -WALL_ALTITUDE_M, 0.0)
                stable_since = 0.0
            elif error < -0.03:
                # If inertia carries the aircraft inside the stand-off, drop
                # the accumulated target and command a small reverse step.
                target_x = state["x"] + max(-0.04, error * 0.25)
                self.flight.set_target(target_x, hold_y, -WALL_ALTITUDE_M, 0.0)
                stable_since = 0.0
            else:
                target_x = state["x"]
                self.flight.set_target(state["x"], hold_y, -WALL_ALTITUDE_M, 0.0)
                horizontal_speed = math.hypot(state["vx"], state["vy"])
                if abs(error) <= 0.05 and horizontal_speed <= 0.15:
                    if stable_since == 0.0:
                        stable_since = time.monotonic()
                    elif time.monotonic() - stable_since >= 1.0:
                        print(
                            f"Wall准备完成：高度={-state['z']:.2f} m，"
                            f"FRONT={front:.3f} m，真实UP={self._format_range(ranges['up'])}。"
                            "现在可输入 aux1 on。")
                        return
                else:
                    stable_since = 0.0

            time.sleep(0.1)

        raise RuntimeError(
            "Wall预定位超时；检查FRONT话题、左墙方向和QGC中的速度/位置控制参数。")

    def prepare(self) -> None:
        if self.mode == "ceiling":
            self.prepare_ceiling()
        elif self.mode == "wall":
            self.prepare_wall()
        else:
            raise RuntimeError("请先输入 mode ceiling 或 mode wall")

    def set_aux(self, index: int, enabled: bool) -> None:
        assert self.flight is not None
        previous = self.flight.aux_snapshot()[index]
        self.flight.set_aux(index, enabled)

        if (self.mode == "ceiling" and index == 2 and enabled
                and previous != AUX_ON):
            # DETACH owns body-Z thrust while state 5 is active. Once it has
            # opened the measured gap, mc_pos_control returns to the Offboard
            # position target. Keeping the prepare target at 6 m would command
            # the vehicle straight back into the ceiling during RECOVERY.
            state = self.flight.snapshot()
            current_altitude = max(0.0, -state["z"])
            retreat_altitude = max(
                CEILING_MIN_RETREAT_ALTITUDE_M,
                current_altitude - CEILING_DETACH_RETREAT_M)
            self.flight.set_target(
                state["x"], state["y"], -retreat_altitude, state["yaw"])
            print(
                f"AUX2=ON；DETACH后Offboard恢复目标设为{retreat_altitude:.2f} m "
                f"（当前{current_altitude:.2f} m，下移{current_altitude-retreat_altitude:.2f} m）")

            # Give the 10 Hz RC override and 100 Hz controller enough time to
            # consume the command, then report whether AUX2 reached the state
            # machine. This distinguishes an RC mapping problem from a thrust
            # or distance-control problem immediately at the terminal.
            time.sleep(0.25)
            output = self._px4("listener ceiling_contact_status -n 1", check=False)
            detach_seen = self._field(output, "detach_switch_on") == "true"
            state_value = self._field(output, "state")

            if not detach_seen:
                print(
                    "警告：控制器未收到AUX2；检查COM_RC_IN_MODE=2和RC_MAP_AUX2=6，"
                    "然后输入params/status。")
            elif state_value == str(5):
                print("Ceiling控制器已确认进入DETACH(5)。")
            else:
                state_name = self.CEILING_STATES.get(
                    int(state_value), "UNKNOWN") if state_value is not None else "NO_STATUS"
                print(f"AUX2已到达控制器；当前状态={state_value} ({state_name})。")
        else:
            print(f"AUX{index}={'ON' if enabled else 'OFF'}")

    def abort(self) -> None:
        assert self.flight is not None
        if self.mode != "wall":
            print("abort是Wall模式的AUX3中止命令；当前未处于Wall模式。")
            return
        self.flight.set_aux(3, True)
        print("AUX3中止脉冲已发送。")
        time.sleep(1.0)
        self.flight.set_aux(3, False)

    @staticmethod
    def _format_range(value: Optional[float]) -> str:
        return "INVALID" if value is None else f"{value:.3f} m"

    @staticmethod
    def _field(output: str, name: str) -> Optional[str]:
        match = re.search(rf"^\s*{re.escape(name)}:\s*([^\s]+)", output, re.MULTILINE)
        return match.group(1).rstrip(",") if match else None

    def status(self) -> None:
        assert self.flight is not None
        state = self.flight.snapshot()
        ranges = self.range_bridge.ranges()
        aux = self.flight.aux_snapshot()
        up_messages, up_errors = self.range_bridge.up.counters()
        front_messages, front_errors = self.range_bridge.front.counters()
        print(
            f"mode={self.mode or 'none'} armed={state['armed']} "
            f"pos=({state['x']:.2f},{state['y']:.2f},{-state['z']:.2f}m) "
            f"yaw={math.degrees(state['yaw']):.1f}deg")
        print(
            f"Gazebo UP={self._format_range(ranges['up'])} age={ranges['up_age']:.3f}s，"
            f"FRONT={self._format_range(ranges['front'])} age={ranges['front_age']:.3f}s")
        print(
            f"UART sent frames={self.range_bridge.frames_sent}，"
            f"LaserScan UP={up_messages}/{up_errors}err FRONT={front_messages}/{front_errors}err，"
            f"AUX1/2/3={aux[1]}/{aux[2]}/{aux[3]}")

        topic = "ceiling_contact_status" if self.mode == "ceiling" else "wall_perch_status"
        output = self._px4(f"listener {topic} -n 1", check=False) if self.mode else ""
        state_value = self._field(output, "state")
        if state_value is not None:
            numeric_state = int(state_value)
            names = self.CEILING_STATES if self.mode == "ceiling" else self.WALL_STATES
            print(f"{topic}: state={numeric_state} ({names.get(numeric_state, 'UNKNOWN')})")

            if self.mode == "ceiling":
                print(
                    "  "
                    f"distance={self._field(output, 'ceiling_distance')} m，"
                    f"target={self._field(output, 'target_distance')} m，"
                    f"thrust_z={self._field(output, 'thrust_body_z_sp')}，"
                    f"detach={self._field(output, 'detach_switch_on')}，"
                    f"sensor_valid={self._field(output, 'distance_sensor_valid')}，"
                    f"rearm={self._field(output, 'rearm_required')}，"
                    f"timeout={self._field(output, 'detach_timed_out')}")
        elif self.mode:
            print(f"{topic}: 暂无状态消息")

        uart_status = self._px4("uart_rx status", check=False)
        last_ranges = next((line.strip() for line in uart_status.splitlines() if "last ranges:" in line), None)
        if last_ranges:
            print(f"uart_rx: {last_ranges}")

    def land(self) -> None:
        assert self.flight is not None
        self.flight.set_aux(1, False)
        self.flight.set_aux(2, False)
        self.flight.set_aux(3, False)
        self.flight.land()

    @staticmethod
    def help() -> None:
        print(
            """命令：
  mode ceiling  只运行ceiling_controller
  mode wall     只运行wall_perch
  prepare       Ceiling升到6.0±0.05m；Wall升到1.5m并自动靠墙至0.45m
  aux1 on|off   吸顶/贴墙开始开关
  aux2 on|off   脱离开关
  abort         Wall AUX3中止脉冲
  status        显示高度、真实测距、UART和控制状态
  params        重新检查QGC/PX4参数（不会改参）
  land          关闭AUX并降落、上锁
  quit          若已解锁则先降落，然后退出本工具
  help          显示本帮助""")

    def command_loop(self) -> None:
        self.help()
        while True:
            try:
                command = input("bridge> ").strip().lower()
            except EOFError:
                command = "quit"

            if not command:
                continue

            try:
                if command in ("quit", "exit"):
                    if self.flight is not None and self.flight.snapshot()["armed"]:
                        self.land()
                    return
                if command == "help":
                    self.help()
                elif command == "status":
                    self.status()
                elif command == "params":
                    self.check_parameters()
                elif command == "prepare":
                    self.prepare()
                elif command == "land":
                    self.land()
                elif command == "abort":
                    self.abort()
                elif command.startswith("mode "):
                    self.set_mode(command.split(maxsplit=1)[1])
                elif re.fullmatch(r"aux[12] (on|off)", command):
                    aux_name, switch = command.split()
                    self.set_aux(int(aux_name[-1]), switch == "on")
                else:
                    print("未知命令；输入 help 查看命令。")
            except (OSError, RuntimeError, ValueError) as error:
                print(f"错误：{error}")

    def close(self) -> None:
        if self.flight is not None:
            self.flight.set_aux(1, False)
            self.flight.set_aux(2, False)
            self.flight.set_aux(3, False)

            for command in ("ceiling_controller stop", "wall_perch stop"):
                try:
                    self._px4(command, check=False)
                except (OSError, RuntimeError, TimeoutError):
                    pass

            self.flight.stop()
        self.range_bridge.stop()


def self_test() -> None:
    frame_self_test()
    sample = {
        "rangeMin": 0.02,
        "rangeMax": 12.0,
        "ranges": [1.234],
    }
    assert laser_range_from_json(sample) == 1.234
    assert metres_to_uart_mm(laser_range_from_json(sample)) == 1234
    assert laser_range_from_json({"ranges": ["Infinity"]}) is None
    assert metres_to_uart_mm(None) == INVALID_DISTANCE
    print("Gazebo LaserScan JSON self-test passed")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mavlink", default="udp:127.0.0.1:14540")
    parser.add_argument("--socket", default="/tmp/px4-sock-0")
    parser.add_argument("--frame-rate", type=float, default=FRAME_RATE_HZ)
    parser.add_argument("--sensor-timeout", type=float, default=SENSOR_TIMEOUT_S)
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    self_test()
    if args.self_test:
        return
    if args.frame_rate <= 0.0 or args.sensor_timeout <= 0.0:
        raise ValueError("frame-rate和sensor-timeout必须大于0")

    application: Optional[BridgeInteractive] = None
    try:
        application = BridgeInteractive(args)
        application.command_loop()
    except KeyboardInterrupt:
        print("\n收到Ctrl+C，关闭交互桥。")
    finally:
        if application is not None:
            application.close()


if __name__ == "__main__":
    main()
