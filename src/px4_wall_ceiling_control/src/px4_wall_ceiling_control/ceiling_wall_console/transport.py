"""One independent bridge identity per authenticated browser."""
import importlib.util
import json
from pathlib import Path
import secrets
import shlex
import subprocess
import threading
import time

PACKAGE = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "ceiling_wall_console_backend", PACKAGE / "scripts/operator/wall_console_server.py")
backend = importlib.util.module_from_spec(spec)
spec.loader.exec_module(backend)


class BridgeTransport(backend.SSHTransport):
    """Use the existing Pi bridge; never start a second ROS runtime ourselves."""

    def __init__(self, args, label):
        self.args = args
        self.label = label
        self.lock = threading.RLock()
        self.snapshot = backend.ConsoleState().snapshot()
        self.connected = False
        self.error = "正在连接飞行反馈服务"
        self.arrived = self.heartbeat = 0.0
        self.push_enabled = True
        self.session_id = secrets.token_hex(24)
        self.session = {"control": False, "owner": "", "clients": 0, "push": True}
        self.pending = {}
        self.writer = self.process = None
        self.closed = threading.Event()
        threading.Thread(target=self.connect_loop, daemon=True).start()
        threading.Thread(target=self.keepalive_loop, daemon=True).start()

    def command(self):
        workspace = self.args.remote_workspace
        worker = workspace + "/src/px4_wall_ceiling_control/scripts/operator/wall_console_server.py"
        script = "source /opt/ros/noetic/setup.bash && source " + shlex.quote(workspace + "/devel/setup.bash")
        script += " && exec python3 " + shlex.quote(worker) + " --bridge"
        command = ["docker", "exec", "-i", self.args.container, "bash", "-lc", script]
        if self.args.bridge == "ssh":
            command = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
                       "-o", "ConnectTimeout=6", "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=3",
                       "-p", str(self.args.ssh_port), "-i", str(Path(self.args.ssh_key).expanduser()),
                       self.args.ssh_user + "@" + self.args.ssh_host, shlex.join(command)]
        return command

    def connect_loop(self):
        while not self.closed.is_set():
            process = None
            try:
                process = subprocess.Popen(self.command(), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                           stderr=subprocess.DEVNULL, text=True, bufsize=1)
                with self.lock:
                    self.process = process
                    if self.closed.is_set():
                        process.terminate()
                        return
                    self.writer = process.stdin
                    self.arrived = self.heartbeat = 0.0
                    self.session = {"control": False, "owner": "", "clients": 0, "push": self.push_enabled}
                    self.writer.write(json.dumps({"client": self.session_id, "label": self.label, "push": self.push_enabled}) + "\n")
                    self.writer.flush()
                for line in process.stdout:
                    message = json.loads(line)
                    with self.lock:
                        if message.get("type") == "session":
                            self.session = message["data"]
                            self.connected, self.error, self.heartbeat = True, "", time.monotonic()
                        elif message.get("type") == "snapshot" and self.push_enabled:
                            self.snapshot = message["data"]
                            self.connected, self.error, self.arrived = True, "", time.monotonic()
                            self.heartbeat = self.arrived
                        elif message.get("type") == "reply" and message.get("id") in self.pending:
                            slot = self.pending[message["id"]]
                            slot["reply"] = message
                            slot["event"].set()
                raise OSError("反馈服务连接中断，正在重连")
            except (OSError, ValueError, KeyError) as error:
                with self.lock:
                    self.connected, self.writer, self.error = False, None, str(error)
                    for slot in self.pending.values():
                        slot["reply"] = {"ok": False, "error": "连接中断，操作结果未知；请核对反馈，不会自动重发"}
                        slot["event"].set()
            finally:
                if process is not None:
                    if process.poll() is None:
                        process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
                    for stream in (process.stdin, process.stdout):
                        if stream:
                            stream.close()
            self.closed.wait(3)

    def keepalive_loop(self):
        while not self.closed.wait(2):
            with self.lock:
                if self.writer:
                    try:
                        self.writer.write('{"action":"ping"}\n')
                        self.writer.flush()
                    except (OSError, ValueError):
                        pass

    def close(self):
        self.closed.set()
        with self.lock:
            self.connected = False
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()

    def request(self, action, config=None):
        if action == "rc_takeover":
            raise ValueError("测试台已移除独立遥控接管操作")
        # Ground recovery needs the bridge connection, not fresh FCU telemetry
        # or authority cached in this browser. The broker checks it centrally.
        if action in ("ceiling", "ceiling_detach", "ceiling_handover", "incline", "incline_detach"):
            with self.lock:
                if not self.connected or not self.push_enabled or not self.session.get("control") or time.monotonic() - self.arrived >= 2:
                    raise ValueError("需要控制权和有效实时反馈，操作未发送")
        return super().request(action, config)
