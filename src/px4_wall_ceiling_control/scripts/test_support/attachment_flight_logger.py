#!/usr/bin/env python3
"""Automatic passive capture. Never publishes or calls a flight-control service."""

import hashlib
import csv
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid
from threading import Lock
from datetime import datetime, timezone
import zipfile


TOPICS = [
    "/wall_perch/sensor_fusion_status", "/wall_perch/handover_readiness",
    "/wall_perch/status", "/wall_perch/departure_trace", "/wall_perch/control_candidate", "/wall_perch/detach", "/wall_perch/cancel",
    "/attachment/ceiling_rc_assist", "/attachment/ceiling_failsafe_status",
    "/ceiling_attachment/status", "/ceiling_attachment/control_candidate", "/ceiling_attachment/detach",
    "/mavros/setpoint_raw/attitude", "/mavros/setpoint_raw/target_attitude",
    "/mavros/state", "/mavros/local_position/odom", "/mavros/local_position/velocity_local",
    "/mavros/imu/data", "/mavros/imu/static_pressure", "/mavros/estimator_status", "/mavros/rc/in", "/mavros/rc/out",
    "/mavros/px4flow/raw/optical_flow_rad",
    "/attachment/kill_switch",
    "/mavros/battery", "/mavros/altitude", "/mavros/global_position/raw/fix",
    "/mavros/global_position/raw/gps_vel", "/mavros/statustext/recv",
    "/sensor/front/raw", "/sensor/front/range", "/sensor/ceiling/raw", "/sensor/ceiling/range",
    "/attachment/terminal_command", "/attachment/terminal_ack",
    "/sensor/health", "/operator/command", "/attachment/output_owner", "/attachment/output_mode",
    "/attachment/output_health",
    "/attachment/flight_phase", "/attachment/flight_phase_reason", "/attachment/prestream",
    "/attachment/rc_preview_deg",
    "/attachment/recovery_target_deg",
    "/attachment/preflip_roll_bias_deg",
    "/attachment/approach_target_deg",
    "/attachment/recovery_horizontal_command", "/attachment/rc_input_age_ms",
    "/rosout", "/rosout_agg", "/clock",
]


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def save_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(str(temporary), str(path))


def copy_run_logs(source, destination, state=None):
    """Copy snapshots, never modify the roslaunch log directory."""
    errors = []
    if source is None or not source.is_dir():
        return ["roslaunch log directory not available"]
    destination.mkdir(exist_ok=True)
    for path in source.rglob("*.log"):
        if path.is_symlink() or not path.is_file():
            continue
        target = destination / path.relative_to(source)
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_name(target.name + ".tmp")
            with path.open("rb") as stream:
                stat = os.fstat(stream.fileno())
                identity = (stat.st_dev, stat.st_ino)
                previous = state.get(str(path)) if state is not None else None
                append = (previous is not None and previous[:2] == identity
                          and target.exists() and target.stat().st_size == previous[2]
                          and stat.st_size > previous[2])
                unchanged = (previous == identity + (stat.st_size, stat.st_mtime_ns)
                             and target.exists() and target.stat().st_size == stat.st_size)
                if unchanged:
                    continue
                offset = previous[2] if append else 0
                stream.seek(offset)
                with (target if append else temporary).open("ab" if append else "wb") as output:
                    remaining = stat.st_size - offset
                    while remaining:
                        data = stream.read(min(1024 * 1024, remaining))
                        if not data:
                            raise OSError("log changed while copying")
                        output.write(data)
                        remaining -= len(data)
                if not append:
                    os.replace(str(temporary), str(target))
                if state is not None:
                    state[str(path)] = identity + (stat.st_size, stat.st_mtime_ns)
        except OSError as exc:
            errors.append("{}: {}".format(path.name, exc))
    return errors


def snapshot_package(package, session):
    hashes = {}
    # Capture executable code, messages, configuration and launch files, not accumulated logs.
    with zipfile.ZipFile(str(session / "package_snapshot.zip"), "w", zipfile.ZIP_DEFLATED) as archive:
        for folder in ("scripts", "src", "config", "launch", "msg"):
            for path in sorted((package / folder).rglob("*")):
                if (not path.is_file() or path.is_symlink() or "__pycache__" in path.parts
                        or session in path.parents or path.suffix not in (".py", ".yaml", ".json", ".launch", ".msg")):
                    continue
                data = path.read_bytes()
                name = path.relative_to(package).as_posix()
                archive.writestr(name, data)
                hashes[name] = hashlib.sha256(data).hexdigest()
        for name in ("CMakeLists.txt", "package.xml"):
            path = package / name
            if path.is_file():
                data = path.read_bytes()
                archive.writestr(name, data)
                hashes[name] = hashlib.sha256(data).hexdigest()
    save_json(session / "source_sha256.json", hashes)


def valid_quaternion(q):
    values = tuple(getattr(q, key) for key in ("x", "y", "z", "w"))
    if not all(math.isfinite(v) for v in values):
        return None
    norm = math.sqrt(sum(v * v for v in values))
    return tuple(v / norm for v in values) if norm > 1e-6 else None


def attitude_error_deg(a, b):
    qa, qb = valid_quaternion(a), valid_quaternion(b)
    if qa is None or qb is None:
        return None
    return math.degrees(2 * math.acos(min(1., abs(sum(x * y for x, y in zip(qa, qb))))))


def world_vertical_speed(odom, child_frame):
    v = odom.twist.twist.linear
    if not child_frame:
        return v.z
    q = valid_quaternion(odom.pose.pose.orientation)
    if q is None:
        return None
    x, y, z, w = q
    return 2 * (x * z - w * y) * v.x + 2 * (y * z + w * x) * v.y + (1 - 2 * (x * x + y * y)) * v.z


class SignalSummary:
    """Small readable summary; full-rate original signals stay in the bag."""
    def __init__(self, session, ros, child_frame):
        self.ros, self.child_frame = ros, child_frame
        self.lock = Lock()
        self.latest, self.counts, self.event_keys = {}, {}, {}
        self.pending = []
        self.subscribers = []
        self.type_cache = {}
        self.csv_file = (session / "signals.csv").open("w", encoding="utf-8", newline="")
        self.event_file = (session / "events.log").open("w", encoding="utf-8")
        self.writer = None

    def subscribe(self):
        for topic in TOPICS:
            # rosout_agg repeats rosout. Keep both in the raw bag, summarize once.
            if topic == "/rosout_agg":
                continue
            self.subscribers.append(self.ros.Subscriber(topic, self.ros.AnyMsg,
                self.receive, callback_args=topic, queue_size=30))

    def receive(self, raw, topic):
        try:
            import roslib.message
            message_type = raw._connection_header["type"]
            cls = self.type_cache.get(message_type)
            if cls is None:
                cls = roslib.message.get_message_class(message_type)
                if cls is None:
                    raise ValueError("message class unavailable: " + message_type)
                self.type_cache[message_type] = cls
            message = cls().deserialize(raw._buff)
            self.remember(topic, message)
        except Exception as exc:
            self.ros.logwarn_throttle(5, "Readable signal summary failed: %s (raw bag remains available)", exc)

    def remember(self, topic, message):
        event = None
        if topic == "/wall_perch/status":
            event = "wall state={} reason={} faults={} candidate_valid={}".format(
                message.state_name, message.reason, message.fault_count, message.candidate_valid)
            if getattr(message, "detach_range_height_enabled", False):
                event += " task={} episode={} locked={} first_raw={} first_h={} first_tilt={} first_source={} first_received={}".format(
                    message.detach_task_phase_name, message.detach_range_episode, message.detach_range_target_locked,
                    message.detach_range_lock_raw_m, message.detach_range_lock_height_m, message.detach_range_lock_tilt_deg,
                    message.detach_range_lock_source_time.to_sec(), message.detach_range_lock_received_time.to_sec())
                event += ' target={} source={} revision={}'.format(
                    message.detach_range_target_m, message.detach_range_target_source, message.detach_range_target_revision)
        elif topic == "/wall_perch/sensor_fusion_status":
            # Deliberately exclude tilt and publishing time: they change every
            # loop, while the event stream records only transaction changes.
            event = ("wall sensor parameter switch phase={} episode={} original={}/{} "
                     "target={}/{} confirmed={}/{} enabled={} snapshot={} restore_required={} restored={} control_ready={} "
                     "request_s={:.6f} ack_s={:.6f} retries={} reason={}").format(
                message.phase, message.episode_id, message.original_flow, message.original_range,
                message.target_flow, message.target_range, message.confirmed_flow, message.confirmed_range,
                message.enabled, message.snapshot_valid, message.restore_required,
                message.parameters_restored, message.control_ready,
                message.request_stamp.to_sec(), message.confirmed_stamp.to_sec(),
                message.retry_count, message.reason)
        elif topic == "/ceiling_attachment/status":
            event = "ceiling state={} reason={} faults={} candidate_valid={}".format(
                message.state_name, message.reason, message.fault_count, message.candidate_valid)
        elif topic == "/operator/command":
            event="operator mode={} enabled={} reason={}".format(message.mode,message.enable_control,message.reason)
        elif topic == "/mavros/state":
            event = "PX4 connected={} armed={} mode={}".format(message.connected, message.armed, message.mode)
        elif topic in ("/attachment/flight_phase", "/attachment/flight_phase_reason",
                       "/attachment/output_owner", "/attachment/output_mode", "/wall_perch/detach", "/wall_perch/cancel", "/ceiling_attachment/detach", "/attachment/terminal_command", "/attachment/terminal_ack"):
            event = "{} {}".format(topic, message.data)
        elif topic in ("/rosout", "/rosout_agg") and message.level >= 4:
            event = "ROS level={} node={} {}".format(message.level, message.name, message.msg)
        elif topic == "/attachment/output_health":
            health = json.loads(message.data)
            event = "output health mode={} owner={} pause={}".format(
                health.get("output_mode"), health.get("owner"), health.get("pause_reason"))
        elif topic == "/mavros/statustext/recv":
            event = "PX4 text severity={} {}".format(message.severity, message.text)
        with self.lock:
            self.latest[topic] = (message, time.monotonic())
            self.counts[topic] = self.counts.get(topic, 0) + 1
            if event is not None and event != self.event_keys.get(topic):
                if len(self.pending) >= 2000:
                    self.pending.pop(0)
                self.pending.append((utc_now(), self.ros.Time.now().to_sec(), event))
                self.event_keys[topic] = event

    def write(self):
        with self.lock:
            latest, pending = dict(self.latest), self.pending
            self.pending = []
        now = time.monotonic()
        row = {"utc": utc_now(), "ros_time_s": self.ros.Time.now().to_sec()}
        def sample(topic, name):
            item = latest.get(topic)
            row[name + "_age_s"] = now - item[1] if item else None
            message = item[0] if item else None
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            row[name + "_header_s"] = stamp.to_sec() if stamp else None
            return message
        health = sample("/attachment/output_health", "output_health")
        row["output_health_json"] = health.data if health is not None else None
        trace = sample("/wall_perch/departure_trace", "departure_trace")
        row["departure_trace_json"] = trace.data if trace is not None else None
        wall = sample("/wall_perch/status", "wall")
        for key in ("state_name", "reason", "progress", "candidate_valid", "fault_count", "thrust_normalized",
                    "front_distance_raw_m", "front_distance_filtered_m", "top_distance_raw_m",
                    "top_distance_filtered_m", "top_range_fresh", "top_preflight_ready", "odom_fresh", "horizontal_velocity_enabled", "horizontal_velocity_ready", "horizontal_velocity_active", "horizontal_velocity_stable", "horizontal_velocity_reason", "horizontal_forward_target_mps", "horizontal_forward_measured_mps", "horizontal_integral_x_enu_mps2", "horizontal_integral_y_enu_mps2", "horizontal_integral_active", "horizontal_integral_reason", "horizontal_control_saturated",  "horizontal_velocity_x_enu_mps", "horizontal_velocity_y_enu_mps", "horizontal_target_x_enu_mps", "horizontal_target_y_enu_mps", "horizontal_accel_x_enu_mps2", "horizontal_accel_y_enu_mps2", "optical_flow_quality", "vertical_control_active", "vertical_reference_valid", "vertical_reference_m", "vertical_reference_source", "vertical_target_enu_mps", "vertical_thrust_correction", "approach_takeover_thrust", "approach_base_vertical_thrust", "flip_vertical_support_enabled", "flip_vertical_support_weight", "flip_entry_thrust", "flip_base_vertical_thrust", "flip_support_thrust", "flip_pressure_thrust", "horizontal_equilibrium_enabled", "horizontal_equilibrium_valid", "horizontal_equilibrium_frozen", "horizontal_equilibrium_reason", "horizontal_prestream_valid", "fault_recovery_completed", "recover_attitude_locked", "recover_stable", "detach_final_pitch_deg", "abort_land_requested", "recovery_support_valid", "recovery_thrust_limit", "detach_vertical_active", "detach_vertical_reason", "detach_vertical_base_source", "detach_vertical_base_thrust", "detach_vertical_integral", "detach_vertical_integral_active", "detach_vertical_saturated", "detach_vertical_slew_limited", "detach_vertical_requested_thrust", "detach_vertical_support_weight", "detach_height_error_m", "departure_clearance_ready", "departure_clearance_distance_m", "departure_clearance_reason", "recovery_horizontal_integral_x", "recovery_horizontal_integral_y", "recovery_horizontal_integral_active",
                    "detach_task_phase", "detach_task_phase_name", "detach_range_height_enabled", "detach_range_episode", "detach_range_maneuver_id", "detach_request_time", "detach_rotation_start_time", "detach_range_target_locked", "detach_range_source", "detach_range_target_m", "detach_range_height_m", "detach_range_raw_m", "detach_range_projected_m", "detach_range_velocity_mps", "detach_range_velocity_valid", "detach_range_valid", "detach_range_height_invalid_since_s", "detach_range_velocity_age_s", "detach_range_velocity_invalid_since_s", "detach_range_velocity_reason", "detach_range_fast_velocity_mps", "detach_range_fast_velocity_valid", "detach_range_fast_velocity_age_s", "detach_range_fast_velocity_reason", "detach_range_current_tilt_deg", "detach_range_sample_tilt_deg", "detach_range_pose_match_dt_s", "detach_range_pose_before_dt_s", "detach_range_pose_after_dt_s", "detach_range_pose_span_s", "detach_range_pose_wait_s", "detach_range_entry_tilt_deg", "detach_range_source_time", "detach_range_received_time", "detach_range_lock_source_time", "detach_range_lock_received_time", "detach_range_lock_raw_m", "detach_range_lock_height_m", "detach_range_lock_tilt_deg", "detach_range_velocity_samples", "detach_range_age_s", "detach_range_stable_elapsed_s", "detach_range_stable_samples", "detach_range_handover_ready", "detach_range_block_reason", "detach_range_config_json", "detach_range_rejections_json", "detach_posctl_request_time", "detach_posctl_confirmed_time", "detach_range_target_revision", "detach_range_target_source", "detach_range_action_started_time"):
            value = getattr(wall, key, None)
            row["wall_" + key] = value.to_sec() if hasattr(value, "to_sec") else value
        for name in ("horizontal_equilibrium_attitude", "horizontal_prestream_attitude"):
            value=getattr(wall,name,None)
            for key in "xyzw":row["wall_"+name+"_"+key]=getattr(value,key,None)
        fusion = sample("/wall_perch/sensor_fusion_status", "wall_sensor_fusion")
        for key in ("enabled", "phase", "episode_id", "snapshot_valid", "restore_required",
                    "parameters_restored", "control_ready", "original_flow", "original_range",
                    "target_flow", "target_range", "confirmed_flow", "confirmed_range",
                    "measured_tilt_deg", "retry_count", "reason"):
            row["wall_sensor_fusion_" + key] = getattr(fusion, key, None)
        for key in ("request_stamp", "confirmed_stamp", "ready_after"):
            stamp = getattr(fusion, key, None)
            row["wall_sensor_fusion_" + key + "_s"] = stamp.to_sec() if stamp is not None else None
        ceiling = sample("/ceiling_attachment/status", "ceiling")
        for key in ("state_name", "reason", "candidate_valid", "fault_count", "range_fresh", "odom_fresh",
                    "ceiling_distance_raw_m", "ceiling_distance_filtered_m", "activation_ready",
                    "attach_confirmed", "detach_start_height_m", "descended_height_m", "detach_height_ready",
                    "thrust_body_z_normalized", "approach_velocity_z_enu_mps", "detach_failed",
                    "height_source", "baro_fresh", "baro_reference_ready", "baro_pressure_pa", "hold_pressure_pa",
                    "target_vertical_speed_mps", "fused_vertical_speed_mps", "height_feedback_valid",
                    "detach_target_drop_m", "detach_max_drop_m", "detach_target_height_m",
                    "vertical_integral_thrust", "vertical_support_thrust", "vertical_proportional_thrust"):
            row["ceiling_" + key] = getattr(ceiling, key, None)
        odom = sample("/mavros/local_position/odom", "odom")
        for key in ("x", "y", "z"):
            row["position_" + key + "_m"] = getattr(odom.pose.pose.position, key) if odom else None
            row["angular_" + key + "_rad_s"] = getattr(odom.twist.twist.angular, key) if odom else None
        row["vertical_speed_world_mps"] = world_vertical_speed(odom, self.child_frame) if odom else None
        row["odom_frame"] = odom.header.frame_id if odom else None
        row["odom_child_frame"] = odom.child_frame_id if odom else None
        for topic, name in (("/wall_perch/control_candidate", "candidate"),
                            ("/mavros/setpoint_raw/attitude", "sent"),
                            ("/mavros/setpoint_raw/target_attitude", "fcu_target")):
            message = sample(topic, name)
            row[name + "_thrust"] = getattr(message, "thrust_normalized" if name == "candidate" else "thrust", None)
            target = getattr(message, "attitude_setpoint" if name == "candidate" else "orientation", None)
            valid = bool(message is not None and (message.valid and message.kind == 2 if name == "candidate" else not (message.type_mask & 128)))
            row[name + "_valid"] = valid
            row[name + "_type_mask"] = getattr(message, "type_mask", None)
            for key in ("x", "y", "z", "w"):
                row[name + "_q_" + key] = getattr(target, key, None)
            fresh = row[name + "_age_s"] is not None and row[name + "_age_s"] <= .25
            row[name + "_error_deg"] = attitude_error_deg(odom.pose.pose.orientation, target) if (
                valid and target is not None and odom is not None and fresh and row["odom_age_s"] <= .5) else None
        # Keep legacy candidate_* above bound to wall attitude candidates.
        # Independent fields retain raw validity for BOTH candidate kinds.
        for topic, name in (("/wall_perch/control_candidate", "wall_candidate"),
                            ("/ceiling_attachment/control_candidate", "ceiling_candidate")):
            message = sample(topic, name)
            for key in ("source", "kind", "acquire_request", "keep_ownership", "reason"):
                row[name + "_" + key] = getattr(message, key, None)
            row[name + "_thrust"] = getattr(message, "thrust_normalized", None)
            # *_candidate_valid already belongs to the behavior status summary.
            row[name + "_raw_valid"] = getattr(message, "valid", None)
            row[name + "_velocity_z_enu_mps"] = getattr(message, "velocity_z_enu_mps", None)
            target = getattr(message, "attitude_setpoint", None)
            for key in ("x", "y", "z", "w"):
                row[name + "_q_" + key] = getattr(target, key, None)
            fresh = row[name + "_age_s"] is not None and row[name + "_age_s"] <= .25
            attitude_valid = bool(message is not None and message.valid and message.kind == 2)
            row[name + "_fresh"] = fresh
            row[name + "_attitude_valid"] = attitude_valid
            row[name + "_error_deg"] = attitude_error_deg(odom.pose.pose.orientation, target) if (
                attitude_valid and target is not None and odom is not None and fresh
                and row["odom_age_s"] <= .5) else None
        # Ownership is sampled independently; it does not prove a candidate was sent.
        for topic, name in (("/attachment/output_owner", "output_owner"),
                            ("/attachment/output_mode", "output_mode")):
            message = sample(topic, name)
            row[name] = getattr(message, "data", None)
        for key in ("x", "y", "z", "w"):
            row["measured_q_" + key] = getattr(odom.pose.pose.orientation, key) if odom else None
        pressure = sample("/mavros/imu/static_pressure", "pressure")
        raw_pressure = getattr(pressure, "fluid_pressure", None)
        row["pressure_reported"] = raw_pressure
        row["pressure_pa"] = (raw_pressure*100.0 if 600.0 <= raw_pressure <= 1200.0 else raw_pressure) if raw_pressure is not None else None
        state = sample("/mavros/state", "px4")
        for key in ("connected", "armed", "mode"):
            row["px4_" + key] = getattr(state, key, None)
        estimator = sample("/mavros/estimator_status", "estimator")
        for key in ("attitude_status_flag", "velocity_horiz_status_flag", "velocity_vert_status_flag",
                    "pos_horiz_rel_status_flag", "pos_vert_abs_status_flag", "gps_glitch_status_flag",
                    "vel_ratio", "pos_horiz_ratio", "pos_vert_ratio"):
            row["estimator_" + key] = getattr(estimator, key, None)
        if self.writer is None:
            self.writer = csv.DictWriter(self.csv_file, fieldnames=list(row))
            self.writer.writeheader()
        self.writer.writerow({key: None if isinstance(value, float) and not math.isfinite(value) else value for key, value in row.items()})
        for utc, ros_time, event in pending:
            self.event_file.write("{} ROS={:.6f} {}\n".format(utc, ros_time, event.replace("\n", "\\n")))
        self.csv_file.flush()
        self.event_file.flush()

    def close(self):
        for subscriber in self.subscribers:
            subscriber.unregister()
        self.write()
        self.csv_file.close()
        self.event_file.close()


class FlightLogger:
    def __init__(self, rospy):
        self.ros = rospy
        self.process = None
        self.console = None
        self.stopped = False
        self.run_logs = None
        self.session = None
        self.info = {"schema_version": 1, "started_utc": utc_now(), "status": "starting"}
        self.summary = None
        self.shutdown_requested = False
        self.log_copy_state = {}

    def request_stop(self):
        # Signal callback only requests shutdown; file I/O finishes in the main loop.
        self.shutdown_requested = True

    def start(self):
        root = Path(self.ros.get_param("~directory", "~/wall_test_records")).expanduser().resolve()
        run_id = self.ros.get_param("/run_id", "")
        if isinstance(run_id, str) and run_id and Path(run_id).name == run_id:
            log_root = Path(os.environ.get("ROS_LOG_DIR", str(Path(
                os.environ.get("ROS_HOME", "~/.ros")).expanduser() / "log"))).expanduser().resolve()
            self.run_logs = log_root / run_id
            if root == self.run_logs or self.run_logs in root.parents:
                raise ValueError("test log directory must be outside the roslaunch run directory")
        root.mkdir(parents=True, exist_ok=True)
        if shutil.disk_usage(str(root)).free < 1024 ** 3:
            raise RuntimeError("less than 1 GB free; automatic capture not started")
        self.session = root / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:6])
        self.session.mkdir()
        # One physical recording, with the existing ceiling discovery path retained.
        alias_directory = self.ros.get_param("~alias_directory", "")
        if alias_directory:
            try:
                alias_root = Path(alias_directory).expanduser().resolve()
                alias_root.mkdir(parents=True, exist_ok=True)
                alias = alias_root / self.session.name
                if alias != self.session:
                    alias.symlink_to(self.session, target_is_directory=True)
                self.info["alias_directory"] = str(alias)
            except OSError as exc:
                self.info["alias_error"] = str(exc)
        topics = self.ros.get_param("~topics", TOPICS)
        if not isinstance(topics, list) or not topics or any(not isinstance(t, str) or not t.startswith("/") for t in topics):
            raise ValueError("logger topics must be a nonempty list of absolute topic names")
        self.info.update({"topics": topics, "run_id": run_id,
                          "roslaunch_log_directory": str(self.run_logs) if self.run_logs else None,
                          "ros_distro": os.environ.get("ROS_DISTRO"),
                          "note": "bag and log capture is best effort; does not replace PX4 ULog"})
        save_json(self.session / "session.json", self.info)
        self.console = (self.session / "recorder.log").open("w", encoding="utf-8")
        command = ["rosbag", "record", "--split", "--duration=30", "--buffsize=128",
                   "-O", str(self.session / "flight")] + topics
        self.info["recorder_command"] = command
        self.process = subprocess.Popen(command, stdout=self.console, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        self.ros.on_shutdown(self.request_stop)
        child_frame = self.ros.get_param("/wall_perch_decision_node/odom_twist_in_child_frame", True)
        self.info["odom_twist_in_child_frame"] = child_frame
        self.summary = SignalSummary(self.session, self.ros, child_frame)
        self.summary.subscribe()
        self.info["status"] = "recording"
        save_json(self.session / "session.json", self.info)
        self.ros.loginfo("Automatic flight logs: %s", self.session)
        # Recorder starts first. Snapshot failures do not stop the existing control nodes.
        try:
            save_json(self.session / "ros_parameters.json", self.ros.get_param("/"))
        except Exception as exc:
            self.info["parameter_snapshot_error"] = str(exc)
        try:
            import rospkg
            package = Path(rospkg.RosPack().get_path("px4_wall_ceiling_control"))
            snapshot_package(package, self.session)
            self.info["package_snapshot_path"] = str(package)
        except Exception as exc:
            self.info["code_snapshot_error"] = str(exc)
        save_json(self.session / "session.json", self.info)

    def run(self):
        last_copy = 0.
        last_metadata = 0.
        while not self.ros.is_shutdown() and not self.stopped and not self.shutdown_requested:
            exit_code = self.process.poll()
            if exit_code is not None:
                self.info.update({"status": "failed", "recorder_exit_code": exit_code,
                                  "error": "rosbag exited unexpectedly"})
                self.ros.logerr("Flight recorder stopped unexpectedly; existing flight control is unaffected")
                break
            if shutil.disk_usage(str(self.session)).free < 1024 ** 3:
                self.info.update({"status": "failed", "error": "less than 1 GB free disk space"})
                self.ros.logerr("Low disk space; stopping only flight recording")
                break
            self.summary.write()
            if time.monotonic() - last_copy >= 30:
                self.info["last_log_copy_errors"] = copy_run_logs(
                    self.run_logs, self.session / "ros_logs", self.log_copy_state)
                last_copy = time.monotonic()
            if time.monotonic() - last_metadata >= 2:
                self.info["last_capture_check_utc"] = utc_now()
                self.info["received_messages"] = dict(self.summary.counts)
                save_json(self.session / "session.json", self.info)
                last_metadata = time.monotonic()
            time.sleep(.1)
        self.stop()

    def stop(self):
        if self.stopped:
            return
        self.stopped = True
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
                self.info["recorder_exit_code"] = self.process.wait(timeout=8)
            except (OSError, subprocess.TimeoutExpired) as exc:
                self.info["close_error"] = str(exc)
                try:
                    self.process.terminate()
                    self.process.wait(timeout=2)
                except (OSError, subprocess.TimeoutExpired):
                    self.process.kill()
                    self.process.wait(timeout=2)
        if self.console is not None:
            self.console.close()
        if self.summary is not None:
            try:
                self.summary.close()
            except Exception as exc:
                self.info["summary_close_error"] = str(exc)
            self.info["received_messages"] = dict(self.summary.counts)
        if self.session is not None:
            self.info["last_log_copy_errors"] = copy_run_logs(
                self.run_logs, self.session / "ros_logs", self.log_copy_state)
            self.info["stopped_utc"] = utc_now()
            self.info["files"] = [{"name": path.name, "bytes": path.stat().st_size}
                                  for path in sorted(self.session.glob("*.bag*")) if path.is_file()]
            self.info["unclosed_bags"] = [path.name for path in self.session.glob("*.active")]
            if self.info["status"] != "failed":
                self.info["status"] = "stopped" if (self.info.get("recorder_exit_code") == 0
                                                     and not self.info["unclosed_bags"]
                                                     and list(self.session.glob("*.bag"))) else "incomplete"
            save_json(self.session / "session.json", self.info)


def main():
    import rospy
    rospy.init_node("attachment_flight_logger")
    logger = FlightLogger(rospy)
    try:
        logger.start()
        logger.run()
    except Exception as exc:
        logger.info.update({"status": "failed", "error": str(exc)})
        rospy.logerr("Automatic flight logging failed: %s; existing flight control is unaffected", exc)
        logger.stop()


if __name__ == "__main__":
    main()
