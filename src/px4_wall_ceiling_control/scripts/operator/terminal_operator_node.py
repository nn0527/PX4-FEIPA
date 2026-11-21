#!/usr/bin/env python3
"""One-shot terminal commands for a persistent attachment operator intent."""

import json
from functools import wraps
import threading
import time
import rospy
from mavros_msgs.msg import State, RCIn
from mavros_msgs.srv import ParamGet
from std_msgs.msg import Bool, String

from px4_wall_ceiling_control.msg import OperatorCommand, WallPerchStatus, CeilingAttachmentStatus
from px4_wall_ceiling_control.adapters.kill_switch import KillSwitchConfig

from px4_wall_ceiling_control.domain.momentary_switch import MomentarySwitch


def serialized(callback):
    @wraps(callback)
    def wrapped(self, *args, **kwargs):
        lock = getattr(self, "intent_lock", None)
        if lock is None:  # Also support legacy isolated test fixtures.
            return callback(self, *args, **kwargs)
        with lock:
            return callback(self, *args, **kwargs)
    return wrapped


class TerminalOperatorNode:
    def __init__(self):
        self.intent_lock = threading.RLock()
        self.rc_wall_toggle_enabled = bool(rospy.get_param("~rc_wall_toggle_enabled", False))
        self.rc_wall_channel_index = int(rospy.get_param("~rc_wall_channel_index", 11))
        if self.rc_wall_channel_index < 0:
            raise ValueError("rc_wall_channel_index must be nonnegative")
        self.rc_selected_mode = None
        self.ceiling_status, self.ceiling_status_at = None, None
        self.wall_status, self.wall_status_at = None, None
        self.wall_switch = MomentarySwitch()
        self.pilot_handover_request_time = rospy.Time(0)
        self.front_sensor_masked = False
        self.front_mask_seen_offboard = False
        rospy.set_param("/attachment/front_sensor_masked", False)
        self.mode = OperatorCommand.MODE_STANDBY
        self.detaching = False
        self.detach_pulse = False
        self.was_offboard = False
        self.flight_mode = ""
        self.kill_guard_enabled = bool(rospy.get_param("~kill_guard_enabled", False))
        self.kill_config = None
        self.kill_config_loading = False
        self.kill_config_attempt_at = -float("inf")
        self.rc_channels, self.rc_at = (), None
        self.flight_at = None
        self.flight_connected = self.flight_armed = False
        self.request_acks = {}
        self.command_ack_pub = rospy.Publisher("/attachment/terminal_ack", String, queue_size=10)
        self.kill_pub = rospy.Publisher("/attachment/kill_switch", String, queue_size=1)
        self.selected_mode_pub = rospy.Publisher(
            "/attachment/rc_selected_mode", String, queue_size=1, latch=True)
        self.selected_mode_pub.publish(String(data="unavailable"))
        self.command_pub = rospy.Publisher(
            "/operator/command", OperatorCommand, queue_size=10
        )
        self.ceiling_detach_pub = rospy.Publisher(
            "/ceiling_attachment/detach", Bool, queue_size=10
        )
        self.wall_detach_pub = rospy.Publisher(
            "/wall_perch/detach", Bool, queue_size=10
        )
        rospy.Subscriber("/wall_perch/status", WallPerchStatus, self._on_wall_status, queue_size=1)
        rospy.Subscriber("/ceiling_attachment/status", CeilingAttachmentStatus,
                         self._on_ceiling_status, queue_size=1)
        rospy.Subscriber("/mavros/rc/in", RCIn, self._on_rc, queue_size=1)
        rospy.Subscriber(
            "/attachment/terminal_command", String, self._on_command, queue_size=10
        )
        rospy.Subscriber("/mavros/state", State, self._on_state, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._on_timer)

    @serialized
    def _on_command(self, message):
        if not message.data.lstrip().startswith("{"):
            return self._apply_command(message)  # Preserve existing terminal commands.
        request_id = ""
        command = "ceiling"
        try:
            request = json.loads(message.data)
            request_id = request.get("id", "")
            command = request.get("command", "")
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64 or command not in ("ceiling", "incline", "select_rc_mode"):
                raise ValueError("请求格式无效")
            if request_id in self.request_acks:
                self.command_ack_pub.publish(String(data=json.dumps(self.request_acks[request_id],ensure_ascii=False)))
                return
            if command == "select_rc_mode":
                selected = request.get("mode")
                if selected not in ("ceiling", "wall", "incline"):
                    raise ValueError("请选择顶吸、侧吸或斜面")
                if self.mode != OperatorCommand.MODE_STANDBY or self.detaching or self.flight_mode == "OFFBOARD":
                    raise ValueError("当前任务未结束，不能切换 CH12 目标")
                self.rc_selected_mode = selected
                self.wall_switch.reset()
                self.selected_mode_pub.publish(String(data=selected))
                self._ack_request({
                    "id": request_id, "command": command, "accepted": True, "selected_mode": selected,
                })
                return
            age = (rospy.Time.now()-self.flight_at).to_sec() if self.flight_at is not None else float('inf')
            safety = self._kill_report() if self.kill_guard_enabled else {"valid":True,"active":False}
            reason = ("急停开关已生效或反馈无效" if not safety['valid'] or safety['active'] else
                      "飞控未连接、心跳过期或尚未解锁" if not (self.flight_connected and self.flight_armed and 0 <= age <= 1.5) else
                      "吸顶请求需从POSCTL发起" if self.flight_mode != "POSCTL" else
                      "已有动作请求，不能重复启动" if self.mode != OperatorCommand.MODE_STANDBY or self.detaching else "")
            if reason:
                accepted = False
            else:
                self._apply_command(String(data=command))
                accepted = self.mode == (OperatorCommand.MODE_INCLINE if command == "incline" else OperatorCommand.MODE_CEILING)
                reason = "吸顶节点已接收：等待有效顶部测距、光流基准及接管条件" if accepted else "节点安全条件变化，请检查急停和飞控反馈"
                if command == "incline" and accepted:
                    reason = "斜面节点已接收：等待前距小于2m准入及接管条件"
                if accepted:
                    self._publish_intent()  # Confirmed intent is sent immediately, not on the next timer tick.
            reply = {"id":request_id,"command":command,"accepted":accepted,"reason":reason,
                     "mode":self.mode,"enable_control":accepted,
                     "ros_time":rospy.Time.now().to_sec()}
            self._ack_request(reply)
        except Exception as error:
            reply = {"id":request_id,"command":command,"accepted":False,"reason":str(error)}
            self.command_ack_pub.publish(String(data=json.dumps(reply,ensure_ascii=False)))
        rospy.logwarn("ceiling request acknowledgement: %s",json.dumps(reply,ensure_ascii=False))

    def _ack_request(self, reply):
        if len(self.request_acks) >= 32:
            self.request_acks.pop(next(iter(self.request_acks)))
        self.request_acks[reply["id"]] = reply
        self.command_ack_pub.publish(String(data=json.dumps(reply, ensure_ascii=False)))

    def _apply_command(self, message):
        command = message.data.strip().lower()
        if command in ("ceiling", "wall", "incline", "detach") and getattr(self, "kill_guard_enabled", False):
            safety = self._kill_report()
            if not safety["valid"] or safety["active"]:
                rospy.logwarn("terminal operator: kill switch active or unavailable; command rejected")
                return
        if command in ("wall", "incline") and getattr(self, "front_sensor_masked", False):
            rospy.logwarn("front sensor masked during ceiling cycle; wait for confirmed POSCTL handover before wall testing")
            return
        if command in ("ceiling", "wall", "incline"):
            if self.mode != OperatorCommand.MODE_STANDBY or self.detaching:
                rospy.logwarn("terminal operator: finish current cycle before %s", command)
                return
            pilot_mode = (str(rospy.get_param("/flight_mode_manager_node/wall_pilot_mode","ALTCTL")).upper() if command == "wall" else "POSCTL")
            if self.flight_mode != pilot_mode:
                rospy.logwarn("terminal operator: start only from PX4 %s",pilot_mode)
                return
            if command == "incline":
                from px4_wall_ceiling_control.domain.incline_attachment import InclineSettings
                settings = InclineSettings.read(lambda key, default: rospy.get_param('/incline_attachment/' + key, default))
                error = settings.error(float(rospy.get_param('/attachment_output_mux_node/max_thrust', .85)))
                age = (rospy.Time.now()-self.flight_at).to_sec() if self.flight_at else float('inf')
                if error or not (self.flight_connected and self.flight_armed and 0 <= age <= 1.5):
                    rospy.logwarn('incline start rejected: %s', error or 'flight feedback invalid')
                    return
            self.mode = {"ceiling": OperatorCommand.MODE_CEILING,
                         "wall": OperatorCommand.MODE_WALL,
                         "incline": OperatorCommand.MODE_INCLINE}[command]
            self.was_offboard = False
            self.pilot_handover_request_time = rospy.Time(0)
            if command == "ceiling":
                self.front_sensor_masked = True
                self.front_mask_seen_offboard = False
                rospy.set_param("/attachment/front_sensor_masked", True)
                rospy.logwarn("ceiling requested: front sensor excluded from control until confirmed POSCTL handover")
            rospy.loginfo("terminal operator: %s requested", command)
        elif command == "detach":
            if self.mode not in (
                OperatorCommand.MODE_CEILING,
                OperatorCommand.MODE_WALL,
                OperatorCommand.MODE_INCLINE,
            ):
                rospy.logwarn("terminal operator: no attachment to detach")
                return
            self.pilot_handover_request_time = rospy.Time(0)
            self.detaching = True
            self.detach_pulse = True
            rospy.loginfo("terminal operator: detach requested")
        elif command == "cancel":
            # Release the behavior request. Decision nodes then execute their
            # recovery path; this does not force a premature mode change.
            self.mode = OperatorCommand.MODE_STANDBY
            self.detaching = False
            self.detach_pulse = False
            self.pilot_handover_request_time = rospy.Time(0)
            rospy.logwarn("terminal operator: behavior cancel requested")
        else:
            rospy.logwarn("terminal operator: use ceiling, wall, detach or cancel")

    @serialized
    def _on_state(self, message):
        self.flight_at = rospy.Time.now()
        self.flight_connected, self.flight_armed = message.connected, message.armed
        if getattr(self, "kill_guard_enabled", False) and message.connected and not message.armed and not self.kill_config_loading and time.monotonic() - self.kill_config_attempt_at >= 10:
            self.kill_config_loading = True
            self.kill_config_attempt_at = time.monotonic()
            threading.Thread(target=self._load_kill_config, daemon=True).start()
        if not message.connected:
            self.kill_config = None
        if getattr(self, "kill_guard_enabled", False) and not message.armed:
            self._release_intent()
        self.flight_mode = message.mode.upper()
        if message.connected and getattr(self, "front_sensor_masked", False):
            if self.flight_mode == "OFFBOARD":
                self.front_mask_seen_offboard = True
            elif self.flight_mode == "POSCTL" and (getattr(self, "front_mask_seen_offboard", False) or self.mode == OperatorCommand.MODE_STANDBY):
                self.front_sensor_masked = False
                self.front_mask_seen_offboard = False
                rospy.set_param("/attachment/front_sensor_masked", False)
                rospy.loginfo("confirmed POSCTL handover: front sensor control input restored")
        if self.flight_mode == "OFFBOARD":
            self.was_offboard = True
        elif self.was_offboard and message.connected and self.flight_mode == (str(rospy.get_param("/flight_mode_manager_node/wall_pilot_mode","ALTCTL")).upper() if self.mode == OperatorCommand.MODE_WALL else "POSCTL"):
            self.mode = OperatorCommand.MODE_STANDBY
            self.detaching = False
            self.detach_pulse = False
            self.was_offboard = False
            self.pilot_handover_request_time = rospy.Time(0)

    def _release_intent(self):
        self.mode = OperatorCommand.MODE_STANDBY
        self.detaching = self.detach_pulse = self.was_offboard = False

        self.pilot_handover_request_time = rospy.Time(0)
    def _load_kill_config(self):
        try:
            get = rospy.ServiceProxy("/mavros/param/get", ParamGet)
            def read(name, integer=False):
                result = get(param_id=name)
                if not result.success:
                    raise ValueError("FCU parameter unavailable: " + name)
                return result.value.integer if integer else result.value.real
            channel = read("RC_MAP_KILL_SW", True)
            config = (KillSwitchConfig(channel, read("RC_KILLSWITCH_TH"), read("RC%d_MIN" % channel),
                read("RC%d_MAX" % channel), read("RC%d_REV" % channel)) if channel else KillSwitchConfig(0))
            self.kill_config = config
        except Exception as error:
            rospy.logwarn("kill-switch configuration read failed: %s", error)
        finally:
            self.kill_config_loading = False

    @serialized
    def _on_rc(self, message):
        self.rc_channels, self.rc_at = tuple(message.channels), rospy.Time.now()
        if getattr(self, "rc_wall_toggle_enabled", False):
            now = time.monotonic()
            pwm = (self.rc_channels[self.rc_wall_channel_index]
                   if len(self.rc_channels) > self.rc_wall_channel_index else None)
            if self.wall_switch.update(pwm, now):
                self._on_wall_toggle()
        if self.kill_guard_enabled and self._kill_report().get("active"):
            self._release_intent()

    @serialized
    def _on_wall_status(self, message):
        self.wall_status, self.wall_status_at = message, rospy.Time.now()
        # A rejected/timed-out request can be retried by either operator input.
        if (self.mode in (OperatorCommand.MODE_WALL, OperatorCommand.MODE_INCLINE)
                and message.state == WallPerchStatus.STATE_WALL_HOLD
                and not message.detach_pending and not self.detach_pulse):
            self.detaching = False

    def _on_wall_toggle(self):
        if self._request_ceiling_handover(rospy.Time.now()):
            return
        self._on_attachment_toggle()

    def _kill_report(self):
        now = rospy.Time.now()
        if self.kill_config is None:
            return {"valid": False, "active": False, "channel": None}
        fresh = self.rc_at is not None and 0 <= (now - self.rc_at).to_sec() <= 0.25
        return self.kill_config.report(self.rc_channels, fresh)

    @serialized
    def _on_timer(self, _event):
        self.selected_mode_pub.publish(String(data=self.rc_selected_mode or "unavailable"))
        if getattr(self, "kill_guard_enabled", False):
            safety = self._kill_report()
            if not safety["valid"] or safety["active"]:
                self._release_intent()
            self.kill_pub.publish(String(data=json.dumps(safety)))
        self._publish_intent()
        if self.detach_pulse:
            if self.mode == OperatorCommand.MODE_CEILING:
                self.ceiling_detach_pub.publish(Bool(data=True))
            elif self.mode in (OperatorCommand.MODE_WALL, OperatorCommand.MODE_INCLINE):
                self.wall_detach_pub.publish(Bool(data=True))
            self.detach_pulse = False

    def _publish_intent(self):
        command = OperatorCommand()
        command.header.stamp = rospy.Time.now()
        command.mode = self.mode
        command.enable_control = self.mode in (
            OperatorCommand.MODE_CEILING,
            OperatorCommand.MODE_WALL,
            OperatorCommand.MODE_INCLINE,
        )
        command.pilot_handover_request_time = getattr(self, 'pilot_handover_request_time', rospy.Time(0))
        command.source = "terminal"
        command.reason = "detach pending" if self.detaching else "terminal request"
        self.command_pub.publish(command)


    @serialized
    def _on_ceiling_status(self, message):
        self.ceiling_status, self.ceiling_status_at = message, rospy.Time.now()


    def _request_ceiling_handover(self, now):
        c = getattr(self, 'ceiling_status', None)
        fresh = lambda stamp, limit: stamp is not None and 0 <= (now-stamp).to_sec() <= limit
        if not (self.mode == OperatorCommand.MODE_CEILING and c is not None
                and c.state == CeilingAttachmentStatus.STATE_RECOVERY_HOVER
                and c.recovery_attitude_locked and c.recovery_ready and c.keep_ownership
                and c.odom_fresh and c.detach_height_ready
                and ((c.height_feedback_valid and c.height_reference_ready)
                     if c.height_source == 'px4_fused_hold_relative' else c.baro_fresh and c.baro_reference_ready)
                and not c.fault_detected and not c.detach_failed
                and self.flight_connected and self.flight_armed and self.flight_mode == 'OFFBOARD'
                and fresh(self.flight_at, 1.5) and fresh(getattr(self, 'ceiling_status_at', None), .5)
                and fresh(c.header.stamp, .5) and c.recovery_lock_time.to_sec() > 0
                and c.recovery_lock_time <= c.header.stamp <= now and c.recovery_lock_time < now):
            return False
        safety = self._kill_report() if self.kill_guard_enabled else {'valid': True, 'active': False}
        if not safety['valid'] or safety['active']:
            rospy.logwarn('CH12 ceiling handover rejected: kill switch active or unavailable')
            return True
        if getattr(self, 'pilot_handover_request_time', rospy.Time(0)).to_sec() == 0:
            self.pilot_handover_request_time = now
            self._publish_intent()
            rospy.loginfo('CH12 ceiling handover confirmed; hold attitude until live POSCTL feedback')
        return True


    def _on_wall_or_incline_toggle(self):
        now = rospy.Time.now()
        fresh = lambda stamp, limit: stamp is not None and 0 <= (now-stamp).to_sec() <= limit
        wall = self.wall_status
        if (not self.flight_connected or not self.flight_armed
                or not fresh(self.flight_at, 1.5) or wall is None
                or not fresh(self.wall_status_at, 0.5)
                or not fresh(wall.header.stamp, 0.5)):
            rospy.logwarn("CH12 ignored: FCU must be armed and flight/wall feedback fresh")
            return
        if (self.mode == OperatorCommand.MODE_STANDBY and not self.detaching
                and wall.state == WallPerchStatus.STATE_IDLE
                and not wall.keep_ownership):
            # In IDLE, fault_detected includes the expected absence of OFFBOARD.
            # Check odometry separately; fresh, armed FCU/kill/pilot-mode checks
            # here and the manager's preflight gates still guard acquisition.
            if not wall.odom_fresh:
                rospy.logwarn("CH12 wall rejected: wall odometry unavailable or stale")
                return
            command = self.rc_selected_mode
            if command == "incline" and not wall.incline_ready:
                rospy.logwarn("CH12 incline rejected: incline is not ready")
                return
        elif (self.mode in (OperatorCommand.MODE_WALL, OperatorCommand.MODE_INCLINE)
                and self.flight_mode == "OFFBOARD"
                and wall.state == WallPerchStatus.STATE_WALL_HOLD
                and not self.detaching and not self.detach_pulse
                and not wall.detach_pending and not wall.fault_detected):
            command = "detach"
        else:
            rospy.loginfo(
                "CH12 ignored: operator_mode=%s wall_state=%s detaching=%s "
                "detach_pending=%s keep_ownership=%s fault_detected=%s reason=%s",
                self.mode, wall.state, self.detaching, wall.detach_pending,
                wall.keep_ownership, wall.fault_detected, wall.reason)
            return
        rospy.loginfo("CH12 requested %s through shared terminal/web operator", command)
        self._apply_command(String(data=command))


    def _on_attachment_toggle(self):
        now = rospy.Time.now()
        fresh = lambda stamp, limit: stamp is not None and 0 <= (now-stamp).to_sec() <= limit
        if not (self.flight_connected and self.flight_armed and fresh(self.flight_at, 1.5)):
            rospy.logwarn("CH12 ignored: FCU must be connected, armed and fresh")
            return
        if self.mode == OperatorCommand.MODE_STANDBY:
            if self.detaching or self.rc_selected_mode is None:
                return
            if self.rc_selected_mode in ("wall", "incline"):
                return self._on_wall_or_incline_toggle()
        elif self.mode in (OperatorCommand.MODE_WALL, OperatorCommand.MODE_INCLINE):
            return self._on_wall_or_incline_toggle()
        ceiling = getattr(self, 'ceiling_status', None)
        ceiling_fresh = (ceiling is not None and fresh(self.ceiling_status_at, .5)
                         and fresh(ceiling.header.stamp, .5))
        if self.mode == OperatorCommand.MODE_STANDBY and not self.detaching:
            if not (ceiling_fresh and ceiling.state == CeilingAttachmentStatus.STATE_NORMAL_FLIGHT
                    and not ceiling.keep_ownership and ceiling.odom_fresh):
                rospy.logwarn('CH12 ceiling start rejected: waiting for fresh idle ceiling feedback')
                return
            command = 'ceiling'
        elif (self.mode == OperatorCommand.MODE_CEILING and not self.detaching and not self.detach_pulse
              and self.flight_mode == 'OFFBOARD' and ceiling_fresh
              and ceiling.state == CeilingAttachmentStatus.STATE_SURFACE_HOLD
              and not ceiling.fault_detected and not ceiling.detach_failed):
            command = 'detach'
        else:
            return
        self._apply_command(String(data=command))


if __name__ == "__main__":
    rospy.init_node("terminal_operator_node")
    from px4_wall_ceiling_control.adapters.async_logging import install_ros_async_logging
    install_ros_async_logging(rospy)
    TerminalOperatorNode()
    rospy.spin()
