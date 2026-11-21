"""One ROS runtime, with authenticated SSH clients relayed over a local socket."""
import json
import os
from pathlib import Path
import socket
import threading
import time


class SharedBridge:
    def __init__(self, runtime, clock=time.monotonic):
        self.runtime, self.clock = runtime, clock
        self.lock = threading.RLock()
        self.clients = {}
        self.owner = None
        self.owner_label = ""
        self.reserved_until = 0

    def connect(self, identifier, label):
        with self.lock:
            if identifier in self.clients:
                raise ValueError("同一客户端已经连接")
            self.clients[identifier] = {"label": str(label)[:80], "push": True}

    def disconnect(self, identifier):
        with self.lock:
            self.clients.pop(identifier, None)
            if self.owner == identifier:
                self.reserved_until = self.clock() + 30

    def metadata(self, identifier):
        with self.lock:
            if self.owner and self.owner not in self.clients and self.clock() >= self.reserved_until:
                self.owner = None
                self.owner_label = ""
            return {"protocol": 2, "push": self.clients[identifier]["push"],
                    "control": self.owner == identifier, "owner": self.owner_label,
                    "clients": len(self.clients), "owner_connected": self.owner in self.clients}

    def safe_transfer(self):
        snapshot = self.runtime.state.snapshot()
        state = snapshot["status"]["state"]
        value = state.get("value") or {}
        # Allow initial startup without MAVROS, but never transfer during an action.
        if snapshot.get("starting"):
            return False
        if not value:
            return not (snapshot["status"]["runtime"].get("value") or {}).get("enabled")
        pilot_modes = {"POSCTL"}
        runtime = snapshot["status"]["runtime"]
        if runtime["fresh"] and (runtime.get("value") or {}).get("wall_pilot_mode") == "ALTCTL":
            pilot_modes.add("ALTCTL")
        return bool(state["fresh"] and value.get("connected") and
                    (not value.get("armed") or
                     (value.get("mode") in pilot_modes and
                      snapshot["status"]["phase"]["fresh"] and
                      snapshot["status"]["phase"]["value"] in ("IDLE", "LOCKED") and
                      snapshot["status"]["owner"]["fresh"] and
                      snapshot["status"]["owner"]["value"] == 0)))

    def recovery_available(self):
        """Ground cleanup can recover lost authority, never grant flight control."""
        flight = self.runtime.state.snapshot()["status"]["state"]
        value = flight.get("value") or {}
        if flight["fresh"] and value.get("armed"):
            return False
        return (not flight["fresh"] or not value.get("connected")
                or not self.owner or self.owner not in self.clients)

    def request(self, identifier, action, config=None):
        with self.lock:
            meta = self.metadata(identifier)
            if action == "push":
                self.clients[identifier]["push"] = bool(config["enabled"])
                return {"message": "实时反馈已开启" if config["enabled"] else "实时反馈已暂停；飞行程序与记录继续运行"}
            if action == "acquire":
                if self.owner and self.owner != identifier:
                    raise ValueError("已有控制端：" + self.owner_label + "；请由原控制端释放控制权")
                if self.owner != identifier and not self.safe_transfer():
                    raise ValueError("当前状态不允许交接控制权；需有效未解锁状态或已确认交回遥控模式")
                self.owner, self.owner_label = identifier, self.clients[identifier]["label"]
                return {"message": "已取得控制权"}
            if action == "release":
                if self.owner != identifier:
                    raise ValueError("当前电脑没有控制权")
                if not self.safe_transfer():
                    raise ValueError("动作执行中或反馈过期，不能释放控制权")
                self.owner, self.owner_label = None, ""
                return {"message": "已释放控制权"}
            if action not in ("start", "select_rc_mode", "wall", "incline", "incline_detach", "detach", "ceiling", "ceiling_detach", "cancel_wait", "rc_takeover", "stop", "repair_mavros"):
                raise ValueError("未知操作")
            recovery = (action == "repair_mavros"
                        and (config or {}).get("grounded_confirmed") is True
                        and self.recovery_available())
            if not meta["control"] and not recovery:
                raise ValueError("当前为观察端；请先申请控制权")
            if not meta["push"] and action not in ("stop", "repair_mavros"):
                raise ValueError("实时反馈已暂停，动作未发送")
            # Ownership checks and dispatch are one critical section.
            return getattr(self.runtime, action)(config) if action in ("start", "select_rc_mode", "stop", "repair_mavros") else getattr(self.runtime, action)()


def serve(runtime_factory, socket_path):
    import fcntl  # Linux only; imported after selecting the Pi service entry.
    path = Path(socket_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(str(path) + ".lock", "a") as lock_file:
        try:
            fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return  # A concurrent SSH connection already started the service.
        path.unlink(missing_ok=True)
        broker = SharedBridge(runtime_factory())
        from px4_wall_ceiling_control.rc_startup import install as install_rc_startup
        rc_startup = install_rc_startup(broker)
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        listener.bind(str(path))
        os.chmod(str(path), 0o600)
        listener.listen(16)
        frame = {"encoded": "", "at": 0}

        def sample():
            while True:
                # Serialize telemetry once for all computers.
                from px4_wall_ceiling_control.wall_console import finite_json
                frame["encoded"] = json.dumps(finite_json({"type": "snapshot", "data": broker.runtime.state.snapshot()}), ensure_ascii=False, allow_nan=False) + "\n"
                frame["at"] = time.monotonic()
                time.sleep(.1)

        def client_loop(conn):
            conn.settimeout(5)
            identifier = None
            registered = False
            write_lock = threading.Lock()
            alive = threading.Event()
            alive.set()
            last_ping = [time.monotonic()]
            reader = conn.makefile("r", encoding="utf-8")
            def send(value, encoded=False):
                with write_lock:
                    conn.sendall((value if encoded else json.dumps(value, ensure_ascii=False) + "\n").encode())
            def stream():
                previous = None
                tick = 0
                try:
                    while alive.is_set():
                        if time.monotonic() - last_ping[0] > 10:
                            break
                        meta = broker.metadata(identifier)
                        if meta != previous or tick % 10 == 0:
                            send({"type": "session", "data": meta})
                            previous = meta
                        if meta["push"] and frame["encoded"] and time.monotonic() - frame["at"] < .5:
                            send(frame["encoded"], True)
                        tick += 1
                        time.sleep(.1)
                except (OSError, KeyError):
                    pass
                finally:
                    alive.clear()
                    try:
                        conn.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        pass
            try:
                hello = json.loads(reader.readline(8192))
                identifier = hello["client"]
                if not isinstance(identifier, str) or len(identifier) != 48:
                    raise ValueError("无效客户端标识")
                broker.connect(identifier, hello.get("label", "Windows") + " · " + identifier[:6])
                registered = True
                broker.clients[identifier]["push"] = bool(hello.get("push", True))
                threading.Thread(target=stream, daemon=True).start()
                while alive.is_set():
                    line = reader.readline(8192)
                    if not line:
                        break
                    request = json.loads(line)
                    last_ping[0] = time.monotonic()
                    if request.get("action") == "ping":
                        continue
                    try:
                        data = broker.request(identifier, request.get("action"), request.get("config"))
                        send({"type": "reply", "id": request["id"], "ok": True, "data": data})
                    except Exception as error:
                        # Unexpected startup failures must reply, not kill this
                        # client thread and turn into an ambiguous UI timeout.
                        send({"type": "reply", "id": request.get("id"), "ok": False, "error": str(error)})
            except (OSError, ValueError, KeyError, TypeError):
                pass
            finally:
                alive.clear()
                if registered:
                    broker.disconnect(identifier)
                conn.close()
                reader.close()

        threading.Thread(target=sample, daemon=True).start()
        while True:
            conn, _ = listener.accept()
            threading.Thread(target=client_loop, args=(conn,), daemon=True).start()


def relay(worker):
    """Thin SSH relay: no rospy, no independent subscriptions or sampling."""
    import subprocess
    import sys
    path = str(Path.home() / ".ros/wall_console/shared.sock")
    def connect():
        connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            connection.connect(path)
            return connection
        except OSError:
            connection.close()
            return None
    connection = connect()
    if connection is None:
        log_path = Path.home() / ".ros/wall_console/shared-service.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("ab") as log:
            subprocess.Popen([sys.executable, worker, "--shared-service"], stdin=subprocess.DEVNULL,
                             stdout=log, stderr=log, start_new_session=True, close_fds=True)
        deadline = time.monotonic() + 15
        while connection is None and time.monotonic() < deadline:
            time.sleep(.1)
            connection = connect()
        if connection is None:
            raise RuntimeError("共享桥接启动超时；查看 " + str(log_path))
    def upload():
        try:
            for line in sys.stdin:
                connection.sendall(line.encode())
        finally:
            connection.shutdown(socket.SHUT_RDWR)
    threading.Thread(target=upload, daemon=True).start()
    try:
        with connection.makefile("r", encoding="utf-8") as incoming:
            for line in incoming:
                sys.stdout.write(line)
                sys.stdout.flush()
    finally:
        connection.close()
