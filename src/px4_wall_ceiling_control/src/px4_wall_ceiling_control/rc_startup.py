"""Disarmed, physical RC startup gesture for the existing shared console runtime.

No arming, mode, RC override, setpoint, or flight-task commands are sent here.
The opt-in config is local to the Pi; the shared broker serializes startup with
web requests. A console business stop keeps telemetry alive, so the same
physical gesture can restart the saved configuration without opening a page.
"""
import json
import math
from pathlib import Path
import threading
import time

from px4_wall_ceiling_control.adapters.kill_switch import KillSwitchConfig
from px4_wall_ceiling_control.domain.momentary_switch import MomentarySwitch
from px4_wall_ceiling_control.wall_console import validate_config


class StickGesture:
    """Require neutral before each continuous hold, including after signal loss."""
    def __init__(self, hold=2.0):
        self.hold = hold
        self.ready = False
        self.since = None
        self.last = None

    @staticmethod
    def valid(channels):
        return (len(channels) >= 7 and
                all(isinstance(v, (int, float)) and math.isfinite(v)
                    and 800 <= v <= 2200 for v in channels[:7]))

    @staticmethod
    def matches(channels):
        return (StickGesture.valid(channels) and channels[0] < 1200
                and channels[1] > 1800 and channels[2] < 1200
                and channels[3] > 1800)

    def update(self, channels, allowed, now):
        if not allowed or not self.valid(channels):
            self.ready, self.since, self.last = False, None, None
            return False
        if self.last is not None and now - self.last > .25:
            self.ready, self.since = False, None
        self.last = now
        neutral = channels[2] < 1200 and all(abs(channels[i] - 1500) <= 150 for i in (0, 1, 3))
        if neutral:
            self.ready, self.since = True, None
            return False
        if not self.ready or not self.matches(channels):
            self.since = None
            return False
        if self.since is None:
            self.since = now
        if now - self.since < self.hold:
            return False
        self.ready, self.since = False, None
        return True


SWITCH_MAPS = (
    "RC_MAP_MODE_SW", "RC_MAP_OFFB_SW", "RC_MAP_RETURN_SW", "RC_MAP_LOITER_SW",
    "RC_MAP_ACRO_SW", "RC_MAP_STAB_SW", "RC_MAP_MAN_SW", "RC_MAP_POSCTL_SW",
    "RC_MAP_GEAR_SW", "RC_MAP_TRANS_SW", "RC_MAP_FLAPS", "RC_MAP_AUX1",
    "RC_MAP_AUX2", "RC_MAP_AUX3", "RC_MAP_AUX4", "RC_MAP_AUX5", "RC_MAP_AUX6",
    "RC_MAP_PARAM1", "RC_MAP_PARAM2", "RC_MAP_PARAM3", "RC_MAP_FLTM_BTN",
    "RC_MAP_ENG_MOT", "RC_MAP_PAY_SW", "RC_MAP_GIMB_SW",
)


def audit_parameters(read):
    """Read confirmed parameters; never change FCU settings to make binding fit."""
    expected = {"MAN_ARM_GESTURE": 0, "RC_MAP_ROLL": 1, "RC_MAP_PITCH": 2,
                "RC_MAP_THROTTLE": 3, "RC_MAP_YAW": 4, "RC_MAP_ARM_SW": 6,
                "RC_MAP_KILL_SW": 7, "RC_MAP_FLTMODE": 5, "COM_RC_IN_MODE": 0}
    for name, value in expected.items():
        if read(name) != value:
            raise ValueError("FCU parameter conflict: %s must be %s" % (name, value))
    for name in SWITCH_MAPS:
        value = read(name, optional=True)
        if value in (1, 2, 3, 4):
            raise ValueError("FCU function shares a gesture channel: " + name)
    configs = []
    for channel, threshold in ((6, "RC_ARMSWITCH_TH"), (7, "RC_KILLSWITCH_TH")):
        configs.append(KillSwitchConfig(channel, read(threshold),
                       read("RC%d_MIN" % channel), read("RC%d_MAX" % channel),
                       read("RC%d_REV" % channel)))
    return configs


def safe_snapshot(snapshot, arm, kill):
    flight = snapshot["status"]["state"]
    rc = snapshot["status"]["rc"]
    value = flight.get("value") or {}
    channels = (rc.get("value") or {}).get("channels", [])
    if (not flight["fresh"] or not rc["fresh"] or not value.get("connected")
            or value.get("armed") is not False or value.get("mode") not in ("POSCTL", "ALTCTL", "STABILIZED", "MANUAL")
            or not StickGesture.valid(channels)
            or any(channels[i] >= 1200 for i in (5, 6))):
        return False
    for config in (arm, kill):
        report = config.report(channels, True)
        if not report["valid"] or report["active"]:
            return False
    return True


def safe_stop_snapshot(snapshot):
    """A physical stop cannot substitute for missing disarmed FCU feedback."""
    flight, rc = snapshot["status"]["state"], snapshot["status"]["rc"]
    value = flight.get("value") or {}
    channels = (rc.get("value") or {}).get("channels", [])
    return bool(flight["fresh"] and rc["fresh"] and value.get("connected")
                and value.get("armed") is False and len(channels) >= 12
                and isinstance(channels[11], (int, float))
                and math.isfinite(channels[11]) and 800 <= channels[11] <= 2200)


class RcStartup:
    def __init__(self, broker, config_path, clock=time.monotonic):
        self.broker, self.runtime, self.clock = broker, broker.runtime, clock
        self.path = Path(config_path)
        self.settings = json.loads(self.path.read_text(encoding="utf-8"))
        if self.settings.get("enabled") is not True:
            raise ValueError("RC startup is not enabled")
        self.config = validate_config(self.settings["startup_config"])
        self.gesture = StickGesture()
        self.stop_ch12_enabled = self.settings.get("stop_ch12_enabled") is True
        self.stop_switch = MomentarySwitch()
        self.arm = self.kill = None
        self.audited_at = -math.inf
        self.last_bootstrap = -math.inf
        self.last_audit_attempt = -math.inf
        self.last_status = None
        self.pending = False
        self.pending_at = 0
        self.tune_pub = None
        self.last_connection = False
        self.absent_optional = set()
        self.audit_lock = threading.Lock()
        with self.runtime.state.lock:
            self.runtime.state.config = dict(self.config)
        self.runtime.rc_startup_enabled = True
        self.runtime.rc_stop_ch12_enabled = self.stop_ch12_enabled

    def status(self, state, reason):
        item = {"state": state, "reason": reason}
        if item == self.last_status:
            return
        self.last_status = item
        self.runtime.state.event("RC startup: " + state + " / " + reason,
                                 "warning" if state in ("BLOCKED", "ERROR") else "info")
        try:
            path = self.runtime.log_dir / "rc-startup-status.json"
            temporary = path.with_suffix(".tmp")
            temporary.write_text(json.dumps(dict(item, time=time.time()), ensure_ascii=False))
            temporary.replace(path)
        except OSError:
            pass

    def audit(self):
        with self.audit_lock:
            self._audit()

    def _audit(self):
        from mavros_msgs.srv import ParamGet
        ros = self.runtime.ros
        ros.wait_for_service("/mavros/param/get", timeout=2)
        get = ros.ServiceProxy("/mavros/param/get", ParamGet)
        def read(name, optional=False):
            if optional and name in self.absent_optional:
                return None
            reply = get(param_id=name)
            if not reply.success:
                if optional:
                    self.absent_optional.add(name)
                    return None
                raise ValueError("Cannot confirm FCU parameter: " + name)
            return reply.value.integer if name.startswith("RC_MAP_") or name in ("MAN_ARM_GESTURE", "COM_RC_IN_MODE") else reply.value.real
        arm, kill = audit_parameters(read)
        # CH12 is the existing wall-action switch. Reject any software remap
        # onto the four sticks; never repurpose that switch for startup.
        if (ros.get_param("/terminal_operator_node/rc_wall_toggle_enabled", False)
                and ros.get_param("/terminal_operator_node/rc_wall_channel_index", 11) != 11):
            raise ValueError("Wall-action switch no longer uses the confirmed CH12")
        self.arm, self.kill, self.audited_at = arm, kill, self.clock()

    def bootstrap(self):
        """Only telemetry is automatic; full controls still require the gesture."""
        runtime = self.runtime
        with self.broker.lock:
            if runtime.stop_requested or runtime.stopping or runtime.state.starting or getattr(runtime, "repairing", False):
                return
            runtime.ensure_ros_master()
            runtime.init_ros()
            nodes = runtime.nodes()
            if "/mavros" in nodes:
                if runtime.mavros_alive():
                    if not runtime.fcu_url_matches(self.config["fcu_port"] + ":" + str(self.config["baud"])):
                        raise ValueError("Existing MAVROS uses a different serial configuration")
                    return
                # A dead node can leave a master registration. Only recover
                # transport after all output processes and serial owners are gone.
                if runtime.mavros_process_conflicts() or runtime.port_owners(self.config["fcu_port"]):
                    raise ValueError("Existing MAVROS is unresponsive or controls remain; use ground recovery")
                runtime.cleanup_stale_mavros()
            if runtime.mavros_process_conflicts() or runtime.port_owners(self.config["fcu_port"]):
                raise ValueError("FCU serial port or existing controls are occupied; no duplicate MAVROS")
            if not Path(self.config["fcu_port"]).is_char_device():
                raise ValueError("FCU serial device is missing")
            runtime.spawn("mavros", ["roslaunch", "px4_wall_ceiling_control",
                          "wall_console_mavros.launch", "fcu_url:=" + self.config["fcu_port"] + ":" + str(self.config["baud"])])
            self.status("WAITING", "Telemetry started; waiting for disarmed FCU and neutral sticks")

    def sound(self, name):
        from mavros_msgs.msg import PlayTuneV2
        flight = self.runtime.state.snapshot()["status"]["state"]
        if not flight["fresh"] or (flight.get("value") or {}).get("armed") is not False:
            return
        if self.tune_pub is None:
            self.tune_pub = self.runtime.ros.Publisher("/mavros/play_tune", PlayTuneV2, queue_size=3)
        deadline = self.clock() + 1
        while self.tune_pub.get_num_connections() == 0 and self.clock() < deadline:
            time.sleep(.05)
        if self.tune_pub.get_num_connections() == 0:
            self.runtime.state.event("RC startup: buzzer transport unavailable", "warning")
            return
        tunes = {"accepted": "MFT180L16O5C", "ready": "MFT180L16O5CP16C", "error": "MFT100L4O3C",
                 "stopped": "MFT180L16O4GP16GP16G"}
        flight = self.runtime.state.snapshot()["status"]["state"]
        if flight["fresh"] and (flight.get("value") or {}).get("armed") is False:
            self.tune_pub.publish(PlayTuneV2(format=1, tune=tunes[name]))

    def trigger(self):
        # Recheck live FCU parameters immediately before dispatch, then read
        # fresh sticks again; a stale periodic audit can never authorize start.
        qualified_at = self.clock()
        self.audit()
        with self.broker.lock:
            snapshot = self.runtime.state.snapshot()
            if (self.runtime.stop_requested or self.runtime.stopping or snapshot["starting"]
                    or getattr(self.runtime, "repairing", False)
                    or self.clock() - qualified_at > 3
                    or not safe_snapshot(snapshot, self.arm, self.kill)):
                raise ValueError("Gesture or disarmed safety conditions changed before startup")
            # This physical, disarmed start does not take web ownership, and
            # cannot invoke wall, ceiling, detach, mode, or arming actions.
            config = dict(self.runtime.state.config) if (snapshot["status"]["runtime"].get("value") or {}).get("enabled") else dict(self.config)
            self.runtime.start(config)
            self.pending, self.pending_at = True, self.clock()
            self.status("STARTING", "Gesture accepted; executing the web console startup flow")
        self.sound("accepted")

    def trigger_stop(self):
        qualified_at = self.clock()
        with self.broker.lock:
            runtime = self.runtime
            snapshot = runtime.state.snapshot()
            if (runtime.stopping or getattr(runtime, "repairing", False)
                    or self.clock() - qualified_at > 1
                    or not safe_stop_snapshot(snapshot)):
                raise ValueError("CH12 stop cancelled: fresh connected disarmed FCU/RC required")
            self.pending = False
            self.gesture.update([], False, self.clock())
            self.stop_switch.reset()
            self.status("STOPPING", "Disarmed CH12 pulse accepted; stopping the base pipeline")
            # Never pass grounded_confirmed: RC cannot approve cleanup when
            # feedback is stale. stop() independently checks the current state.
            result = runtime.stop()
            self.status("STOPPED", result["message"])
        self.sound("stopped")

    def handle_stop_switch(self, snapshot, now):
        if not getattr(self, "stop_ch12_enabled", False):
            return False
        if (self.runtime.stopping or getattr(self.runtime, "repairing", False)
                or not safe_stop_snapshot(snapshot)):
            self.stop_switch.reset()
            return False
        channels = snapshot["status"]["rc"]["value"]["channels"]
        if not self.stop_switch.update(channels[11], now):
            return False
        self.trigger_stop()
        return True

    def tick(self):
        runtime, now = self.runtime, self.clock()
        # Stop remains available during startup or after a failed cleanup;
        # its admission is independent of start's parameter audit/pilot mode.
        if self.handle_stop_switch(runtime.state.snapshot(), now):
            return
        if runtime.stop_requested or runtime.stopping or getattr(runtime, "repairing", False):
            self.gesture.update([], False, now)
            self.pending = False
            self.status("INHIBITED", "Stop/recovery is in progress or cleanup failed; waiting for verified completion")
            return
        snapshot = runtime.state.snapshot()
        flight = snapshot["status"]["state"]
        value = flight.get("value") or {}
        connected = flight["fresh"] and value.get("connected", False)
        if not connected or not self.last_connection:
            self.audited_at = -math.inf
            self.absent_optional.clear()
        self.last_connection = connected
        if not connected or value.get("armed") is not False:
            self.gesture.update([], False, now)
            self.pending = False
            self.status("BLOCKED", "Waiting for connected, disarmed FCU")
            return
        now = self.clock()
        if now - self.audited_at > 15 or self.arm is None or self.kill is None:
            self.gesture.update([], False, now)
            self.status("BLOCKED", "Live FCU conflict audit is unavailable")
            return
        snapshot = runtime.state.snapshot()
        enabled = (snapshot["status"]["runtime"].get("value") or {}).get("enabled", False)
        if enabled and not snapshot["starting"]:
            current = validate_config(dict(runtime.state.config))
            if current != self.config:
                self.config = current
                self.settings["startup_config"] = current
                temp = self.path.with_suffix(".tmp")
                temp.write_text(json.dumps(self.settings, indent=2))
                temp.replace(self.path)
        if self.pending:
            if snapshot["launch_error"]:
                self.pending = False
                self.status("ERROR", snapshot["launch_error"])
                self.sound("error")
            elif enabled and snapshot["status"]["runtime"]["fresh"] and not snapshot["starting"]:
                self.pending = False
                self.status("READY", "All console startup checks passed")
                self.sound("ready")
            elif now - self.pending_at > 90:
                self.pending = False
                self.status("ERROR", "Startup confirmation timed out")
                self.sound("error")
            return
        channels = (snapshot["status"]["rc"].get("value") or {}).get("channels", [])
        allowed = not snapshot["starting"] and safe_snapshot(snapshot, self.arm, self.kill)
        if self.gesture.update(channels, allowed, now):
            self.trigger()
        elif not allowed:
            self.status("BLOCKED", "Require fresh RC, low arm switch, clear kill switch and pilot mode")
        elif self.gesture.ready and (not self.last_status or self.last_status["state"] in ("BLOCKED", "INHIBITED", "WAITING", "ERROR", "STOPPED")):
            self.status("LISTENING", "Low throttle; sticks inward/down for 2 seconds starts the saved configuration")
        elif not self.last_status:
            self.status("WAITING", "Center roll, pitch and yaw with low throttle before the first gesture")

    def maintain(self):
        """Slow ROS/XMLRPC/parameter operations never interrupt stick sampling."""
        while True:
            try:
                runtime = self.runtime
                if not (runtime.stop_requested or runtime.stopping or getattr(runtime, "repairing", False)):
                    self.bootstrap()
                    flight = runtime.state.snapshot()["status"]["state"]
                    value = flight.get("value") or {}
                    if flight["fresh"] and value.get("connected") and value.get("armed") is False:
                        self.audit()
            except Exception as error:
                self.audited_at = -math.inf
                self.status("ERROR", str(error))
            time.sleep(3)

    def run(self):
        threading.Thread(target=self.maintain, name="rc-startup-audit", daemon=True).start()
        while True:
            try:
                self.tick()
            except Exception as error:
                self.gesture.update([], False, self.clock())
                self.audited_at = -math.inf
                self.status("ERROR", str(error))
            time.sleep(.05)


def install(broker):
    path = Path(__file__).resolve().parents[2] / "config" / "rc_startup.json"
    if not path.exists():
        return None
    try:
        controller = RcStartup(broker, path)
    except Exception as error:
        broker.runtime.state.event("RC startup disabled: " + str(error), "warning")
        return None
    threading.Thread(target=controller.run, name="rc-startup", daemon=True).start()
    return controller
