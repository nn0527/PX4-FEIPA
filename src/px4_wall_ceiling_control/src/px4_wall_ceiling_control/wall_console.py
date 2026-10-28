"""HTTP-console state and admission checks; independent of ROS and the browser."""
import collections
import math
import re
import threading
import time


STABLE_PORTS = {
    "fcu_port": "/dev/serial/by-path/wall-fcu",
    "ceiling_port": "/dev/serial/by-path/wall-top",
    "front_port": "/dev/serial/by-path/wall-front",
}
LEGACY_PORTS = {"fcu_port": "/dev/ttyUSB0", "ceiling_port": "/dev/ttyUSB1", "front_port": "/dev/ttyUSB2"}


def migrate_legacy_ports(values):
    """Upgrade this bench's old defaults; preserve explicit alternate layouts."""
    if isinstance(values, dict):
        values = dict(values)
        values.pop("detach_range_manual_target_m", None)  # Retired saved setting.
    if isinstance(values, dict) and all(values.get(k) == v for k, v in LEGACY_PORTS.items()):
        return dict(values, **STABLE_PORTS)
    return values


DEFAULT_CONFIG = {
    **STABLE_PORTS, "baud": 230400,
    "hover_thrust": 0.49,
    "test_profile": "flight",
}
LIMITS = {"state": 1.5, "odom": 0.5, "estimator": 1.5, "rc": 0.2,
          "health": 0.3, "wall": 0.5, "phase": 0.5, "reason": 0.5, "rc_takeover": 0.5,
          "owner": 0.3, "operator": 0.5, "setpoint": 0.3,
          "battery": 5.0, "front": 0.6, "ceiling": 0.6, "runtime": 3.0}
LIMITS.update({"front_diagnostic": 3.0, "ceiling_diagnostic": 3.0})
LIMITS["kill"] = 0.35
LIMITS["ceiling_status"] = 0.5
LIMITS["rc_selected_mode"] = 0.35
LIMITS["ceiling_failsafe"] = 1.0
LIMITS["wall_sensor_fusion"] = 0.5


def validate_config(values):
    if not isinstance(values, dict) or set(values) - set(DEFAULT_CONFIG):
        raise ValueError("配置字段无效")
    result = dict(DEFAULT_CONFIG, **values)
    if result["test_profile"] not in ("flight", "restrained_bench"):
        raise ValueError("测试配置必须为普通实飞或有约束台架")
    for name in ("fcu_port", "ceiling_port", "front_port"):
        if not isinstance(result[name], str) or not re.fullmatch(
                r"/dev/(?:tty(?:USB|ACM|AMA)[0-9]+|serial/by-(?:id|path)/[\w.:+\-]+)", result[name]):
            raise ValueError("串口必须是 /dev/ttyUSB*、ttyACM*、ttyAMA* 或稳定的 serial 路径")
    if len({result[n] for n in ("fcu_port", "ceiling_port", "front_port")}) != 3:
        raise ValueError("飞控和两路雷达必须使用三个不同的串口")
    if type(result["baud"]) is not int or result["baud"] not in (57600, 115200, 230400, 460800, 921600):
        raise ValueError("飞控波特率无效")
    if type(result["hover_thrust"]) not in (int, float) or not math.isfinite(result["hover_thrust"]) or not 0 < result["hover_thrust"] <= 0.85:
        raise ValueError("悬停推力需为 0 到 0.85 之间的标定值")
    return result


def launch_arguments(config):
    c = validate_config(config)
    return ["roslaunch", "px4_wall_ceiling_control", "wall_console_flight.launch",
            "ceiling_serial_port:=" + c["ceiling_port"], "front_serial_port:=" + c["front_port"],
            "hover_thrust:=" + str(c["hover_thrust"]),
            "restrained_bench:=" + ("true" if c["test_profile"] == "restrained_bench" else "false")]


class ConsoleState:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.samples = {}
        self.events = collections.deque(maxlen=100)
        self.config = dict(DEFAULT_CONFIG)
        self.starting = False
        self.launch_error = ""
        self.pending_since = None
        self.detach_pending_since = None
        self.sequence = 0
        self.ceiling_request = {"state":"NOT_SENT","reason":"尚未发送吸顶请求"}
        self.ceiling_request_seen = False

    def event(self, text, kind="info"):
        with self.lock:
            self.sequence += 1
            self.events.append({"id": self.sequence, "time": time.time(), "text": text, "kind": kind})

    def update(self, name, value):
        with self.lock:
            previous = self.samples.get(name)
            self.samples[name] = {"at": self.clock(), "value": value}
            if name=="operator" and value.get("mode")==2 and value.get("enabled"):
                self.ceiling_request_seen = True
            if name in ("phase", "reason", "owner") and (not previous or previous["value"] != value):
                self.event({"phase": "控制阶段", "reason": "阶段反馈", "owner": "输出归属"}[name] + ": " + str(value))
            if name in ("wall", "ceiling_status") and (not previous or previous["value"]["state_name"] != value["state_name"]
                    or previous["value"].get("reason") != value.get("reason")):
                self.event(("吸顶阶段: " if name == "ceiling_status" else "贴墙阶段: ") + value["state_name"] + " · " + value["reason"])
            if name == "state" and (not previous or previous["value"] != value):
                self.event("飞控: {} / {} / {}".format("已连接" if value["connected"] else "断开",
                           "已解锁" if value["armed"] else "未解锁", value["mode"] or "未知模式"))
            if name == "kill" and (not previous or previous["value"] != value):
                self.event("急停开关: " + ("反馈无效，动作禁用" if not value.get("valid") else
                    "已生效，动作禁用" if value.get("active") else "未触发") +
                    " · 通道 " + str(value.get("channel")), "warning" if value.get("active") or not value.get("valid") else "info")

    def snapshot(self):
        with self.lock:
            now = self.clock()
            status = {}
            for name, limit in LIMITS.items():
                item = self.samples.get(name)
                age = now - item["at"] if item else None
                status[name] = {"value": item["value"] if item else None, "age_s": age,
                                "fresh": age is not None and 0 <= age <= limit, "limit_s": limit}
            def val(name, default=None):
                return status[name]["value"] if status[name]["fresh"] else default
            flight, health, estimator, runtime = (val(n, {}) for n in ("state", "health", "estimator", "runtime"))
            wall = val("wall", {})
            kill = val("kill", {})
            kill_ok = kill.get("valid") is True and kill.get("active") is False
            bench = self.config["test_profile"] == "restrained_bench"
            range_diagnostic_only = bool(runtime.get("wall_range_faults_diagnostic_only", False))
            height_reason = ("未建立起飞地面参考；请在未解锁时启动基础程序" if not wall.get("altitude_reference_valid")
                else "相对起飞高度 {:.2f} m，需大于 {:.2f} m".format(
                    wall.get("height_above_takeoff_m", 0), wall.get("min_entry_height_m", 0.3)))
            if bench:
                height_reason = "有约束台架：高度入口门槛已关闭"
            sensor_reasons = []
            for name, label in (("front", "前向"), ("ceiling", "顶部")):
                if not health.get(name + "_valid"):
                    diagnostic = val(name + "_diagnostic", {})
                    error = diagnostic.get("error")
                    explanation = ("E255：反射弱／测量失败，请检查光点和反射面" if error and "L1 error 255" in error else
                        "E258：超出量程，请检查目标距离和光路" if error and "L1 error 258" in error else
                        error or "距离数据缺失或过期")
                    sensor_reasons.append(label + "雷达：" + explanation)
            checks = [
                ("runtime", "实飞链路", bool(runtime.get("enabled")), "基础程序未启动或实飞输出未启用"),
                ("connected", "飞控连接", bool(flight.get("connected")), "飞控未连接或心跳已过期"),
                ("kill", "急停开关", kill_ok, "急停开关已生效，动作已禁用" if kill.get("active") else "急停配置或通道反馈缺失／过期"),
                ("armed", "已解锁", bool(flight.get("armed")), "等待飞手用遥控器解锁、起飞"),
                ("mode", runtime.get("wall_pilot_mode","ALTCTL") + " 模式", flight.get("mode") == runtime.get("wall_pilot_mode","ALTCTL"), "贴墙请求需从 " + runtime.get("wall_pilot_mode","ALTCTL") + " 模式发起"),
                ("odom", "里程计", bool(val("odom", {}).get("finite")), "里程计缺失、过期或数值无效"),
                ("height", "高度门槛（台架关闭）" if bench else "相对起飞高度", bench or bool(wall.get("entry_height_ready")), height_reason),
                ("estimator", "估计器", bool(estimator.get("attitude") and estimator.get("vertical_velocity")), "估计器状态缺失、过期或竖直速度无效"),
                ("health", "雷达仅告警（不阻止接管）" if range_diagnostic_only else "雷达入口门槛（台架关闭）" if bench else "两路雷达", range_diagnostic_only or bench or bool(health.get("ceiling_valid") and health.get("front_valid")), "；".join(sensor_reasons) or "顶部或前向雷达无效／过期"),
                ("idle", "待命状态", val("phase") == "IDLE" and val("owner") == 0 and
                 val("operator", {}).get("mode") == 1 and not val("operator", {}).get("enabled", True), "已有动作请求，等待当前流程结束"),
            ]
            checks.append(("front_mask", "前部雷达控制输入", not runtime.get("front_sensor_masked", False), "前部雷达已屏蔽；确认交回 POSCTL 后恢复侧吸"))
            pending = self.pending_since is not None and now - self.pending_since < 5
            blockers = [reason for _, _, ok, reason in checks if not ok]
            if wall.get("horizontal_velocity_enabled") and not (wall.get("horizontal_velocity_ready") and estimator.get("horizontal_velocity")):
                blockers.append("水平速度接管条件未就绪：" + wall.get("horizontal_velocity_reason","等待反馈"))
            if pending:
                blockers.insert(0, "贴墙请求已发送，等待节点反馈")
            if self.launch_error:
                blockers.insert(0, self.launch_error)
            success = (kill_ok and val("wall", {}).get("state_name") == "WALL_HOLD" and
                       not val("wall", {}).get("fault_detected", False) and runtime.get("enabled") and
                       status["setpoint"]["fresh"] and
                       val("phase") == "ACTIVE" and val("owner") == 2 and
                       flight.get("connected") and flight.get("armed") and flight.get("mode") == "OFFBOARD")
            detach_pending = bool(wall.get("detach_pending") or (self.detach_pending_since is not None and
                now - self.detach_pending_since < 5))
            detach_checks = [
                (bool(runtime.get("enabled")), "基础程序反馈无效"),
                (kill_ok, "急停开关已生效，动作已禁用" if kill.get("active") else "急停反馈缺失／过期"),
                (bool(flight.get("connected") and flight.get("armed") and flight.get("mode") == "OFFBOARD"), "离墙要求已连接、已解锁且处于 OFFBOARD"),
                (bool(val("odom", {}).get("finite") and estimator.get("attitude") and estimator.get("vertical_velocity")), "里程计或估计器反馈无效／过期"),
                (range_diagnostic_only or bool(health.get("ceiling_valid")), "顶部雷达反馈无效／过期"),
                (bool(wall.get("state_name") == "WALL_HOLD" and not wall.get("fault_detected")), "仅当前贴墙保持阶段允许离墙"),
                (bool(val("phase") == "ACTIVE" and val("owner") == 2 and status["setpoint"]["fresh"] and
                    val("operator", {}).get("mode") == 3 and val("operator", {}).get("enabled")), "贴墙控制链路未就绪"),
            ]
            detach_blockers = [reason for ok, reason in detach_checks if not ok]
            if detach_pending:
                detach_blockers.insert(0, "离墙请求已发送，等待节点反馈")
            if self.launch_error or self.starting:
                detach_blockers.insert(0, "基础程序未就绪")
            ceiling = val("ceiling_status", {})
            common = [reason for i, _, ok, reason in checks if i not in ("height", "health", "front_mask", "mode") and not ok]
            ceiling_checks = [
                (flight.get("mode") == "POSCTL", "吸顶请求需从 POSCTL 模式发起"),
                (runtime.get("ceiling_enabled"), "吸顶节点未启用；请在落地后重新启动基础程序"),
                (status["ceiling_status"]["fresh"] and ceiling.get("state_name") == "NORMAL_FLIGHT", "吸顶决策未待命或反馈过期"),
                (runtime.get("ceiling_distance_gate_enabled") or (health.get("ceiling_valid") and status["ceiling"]["fresh"]), "顶部雷达无效或过期"),
                ((runtime.get("ceiling_distance_gate_enabled") or (isinstance(val("ceiling", {}).get("range_m"), (int,float)) and 0 < val("ceiling", {}).get("range_m", 99) < runtime.get("ceiling_activation_distance_m", .5))), "需手动接近顶部至激活距离内"),
            ]
            ceiling_wait_conditions = []
            if runtime.get("ceiling_motion_enabled"):
                motion_checks = [
                    (ceiling.get("motion_control_enabled") and ceiling.get("motion_ready"),
                     "等待有效光流、位置与速度反馈：" + (ceiling.get("motion_reason") or "等待新鲜反馈")),
                    (ceiling.get("baseline_ready"), "等待稳定POSCTL平衡姿态和推力基准"),
                ]
                # Match CH12: gated requests may wait in POSCTL. The manager
                # independently requires every takeover gate before OFFBOARD.
                if runtime.get("ceiling_distance_gate_enabled"):
                    ceiling_wait_conditions = [reason for ok, reason in motion_checks if not ok]
                else:
                    ceiling_checks += motion_checks
            ceiling_blockers = common + [reason for ok, reason in ceiling_checks if not ok]
            ceiling_pending = getattr(self, "ceiling_pending_since", None)
            if ceiling_pending is not None and now-ceiling_pending < 5:
                ceiling_blockers.insert(0, "吸顶请求已发送，等待节点反馈")
            if self.starting or self.launch_error:
                ceiling_blockers.insert(0, self.launch_error or "基础程序正在启动")
            ceiling_detach_ok = bool(runtime.get("enabled") and runtime.get("ceiling_enabled") and kill_ok
                and flight.get("connected") and flight.get("armed") and flight.get("mode") == "OFFBOARD"
                and ceiling.get("state_name") == "SURFACE_HOLD" and not ceiling.get("fault_detected")
                and ceiling.get("odom_fresh")
                and (ceiling.get("height_source") != "barometer_hold_relative" or (ceiling.get("baro_fresh") and ceiling.get("baro_reference_ready")))
                and (ceiling.get("height_source") != "px4_fused_hold_relative" or (ceiling.get("height_feedback_valid") and ceiling.get("height_reference_ready")))
                and val("phase") == "ACTIVE" and val("owner") == 1
                and val("operator", {}).get("mode") == 2 and val("operator", {}).get("enabled")
                and status["setpoint"]["fresh"]
                and val("odom", {}).get("finite") and estimator.get("attitude") and estimator.get("vertical_velocity"))
            request = dict(self.ceiling_request)
            if request.get("state") == "ACCEPTED":
                if val("phase") in ("PRESTREAM","ENTERING"):
                    request.update(state="WAITING",reason=val("reason") or "吸顶请求已接收，等待接管条件")
                elif flight.get("mode")=="OFFBOARD" and val("operator",{}).get("mode")==2:
                    request.update(state="ACTIVE",reason="PX4已确认OFFBOARD，吸顶流程执行中")
                elif self.ceiling_request_seen and val("operator",{}).get("mode")==1 and val("phase") in ("IDLE","LOCKED"):
                    request.update(state="RELEASED",reason=val("reason") or "吸顶请求已释放，请核对急停和飞控状态")
            return {"mavros_repair": dict(getattr(self, "mavros_repair", {"state": "IDLE", "reason": "尚未执行连接修复"})), "ceiling_request":request,"can_ceiling": not ceiling_blockers, "ceiling_blockers": ceiling_blockers,
                    "ceiling_wait_conditions": ceiling_wait_conditions, "can_ceiling_detach": ceiling_detach_ok, "status": status, "checks": [{"id": i, "label": label, "ok": ok, "reason": reason}
                    for i, label, ok, reason in checks], "blockers": blockers, "can_wall": not blockers and not self.starting,
                    "success": bool(success), "starting": self.starting, "pending": pending,
                    "can_detach": not detach_blockers, "detach_blockers": detach_blockers,
                    "detach_pending": detach_pending,
                    "can_cancel_wait": bool(val("phase") == "WAITING_WALL" and val("owner") == 0
                        and flight.get("connected") and flight.get("mode") == runtime.get("wall_pilot_mode","ALTCTL")
                        and val("operator", {}).get("enabled") and val("operator", {}).get("mode") == 3),
                    "sensor_warnings": sensor_reasons,
                    "launch_error": self.launch_error, "config": dict(self.config), "events": list(self.events)}

    def send_ceiling(self, publish):
        with self.lock:
            current=self.snapshot()
            if not current["can_ceiling"]:
                self.ceiling_request={"state":"REJECTED","reason":current["ceiling_blockers"][0]}
                self.event("吸顶请求未发送："+self.ceiling_request["reason"],"warning")
                raise ValueError(current["ceiling_blockers"][0])
            self.ceiling_pending_since=self.clock()
            self.ceiling_request_seen=False
            self.ceiling_request={"state":"SENDING","reason":"正在等待吸顶控制节点回执"}
            try:
                reply=publish("ceiling")
            except Exception as error:
                self.ceiling_pending_since=None
                self.ceiling_request={"state":"REJECTED" if getattr(error,"command_rejected",False) else "UNCONFIRMED","reason":str(error)}
                self.event("吸顶请求未确认："+str(error),"warning")
                raise
            self.ceiling_request={"state":"ACCEPTED" if isinstance(reply,dict) and reply.get("accepted") else "SENT",
                                  "id":reply.get("id") if isinstance(reply,dict) else None,
                                  "reason":reply.get("reason","等待节点反馈") if isinstance(reply,dict) else "消息已发送，等待节点反馈"}
            self.event("吸顶请求："+self.ceiling_request["reason"])

    def send_ceiling_detach(self, publish):
        with self.lock:
            if not self.snapshot()["can_ceiling_detach"]:
                raise ValueError("离顶要求当前吸顶保持、OFFBOARD及新鲜有效反馈")
            publish("detach")
            self.event("已发送离顶请求；按节点反馈的本轮目标下降量闭环控制，恢复稳定后等待CH12确认交回POSCTL")

    def send_wall(self, publish):
        # Serialize the preflight check and dispatch across tabs and double clicks.
        with self.lock:
            current = self.snapshot()
            if not current["can_wall"]:
                raise ValueError(current["blockers"][0] if current["blockers"] else "程序正在启动")
            self.pending_since = self.clock()
            try:
                publish("wall")
            except Exception:
                self.pending_since = None
                raise
            self.event("已发送 wall；等待控制节点确认，尚未判定贴墙成功")

    def send_detach(self, publish):
        with self.lock:
            current = self.snapshot()
            if not current["can_detach"]:
                raise ValueError(current["detach_blockers"][0])
            self.detach_pending_since = self.clock()
            try:
                publish("detach")
            except Exception:
                self.detach_pending_since = None
                raise
            self.event("已发送 detach；等待离墙、恢复水平和遥控接管反馈")


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    return value
