#!/usr/bin/env python3
"""Authenticated LAN application. Each browser owns its own bridge identity."""
import argparse
from collections import deque
from http.cookies import SimpleCookie, CookieError
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import os
from pathlib import Path
import secrets
import socket
import subprocess
import threading
import time
from urllib.parse import urlsplit

from .transport import BridgeTransport, backend, PACKAGE

ASSETS = PACKAGE / "web/ceiling_wall_console"
ACTIONS = {"/api/acquire": "acquire", "/api/release": "release", "/api/start": "start",
           "/api/ceiling": "ceiling", "/api/ceiling-detach": "ceiling_detach", "/api/stop": "stop", "/api/repair-mavros": "repair_mavros",
           "/api/incline": "incline", "/api/incline-detach": "incline_detach",
           "/api/wall": "wall", "/api/detach": "detach", "/api/cancel-wait": "cancel_wait", "/api/push": "push"}
SESSION_IDLE = 180


def access_code(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with open(path, "x", opener=lambda p, flags: os.open(p, flags, 0o600)) as handle:
            handle.write(secrets.token_urlsafe(18) + "\n")
    except FileExistsError:
        pass
    code = path.read_text().strip()
    if len(code) < 4:
        raise ValueError("访问码至少需要 4 个字符")
    return code


class Sessions:
    def __init__(self, factory, clock=time.monotonic):
        self.factory, self.clock = factory, clock
        self.lock = threading.RLock()
        self.items = {}
        self.attempts = {}

    def allowed_attempt(self, address):
        with self.lock:
            now = self.clock()
            self.attempts = {k: v for k, v in self.attempts.items() if v and now-v[-1] < 60}
            if len(self.attempts) >= 256 and address not in self.attempts:
                return False
            attempts = self.attempts.setdefault(address, deque())
            while attempts and now-attempts[0] >= 60:
                attempts.popleft()
            if len(attempts) >= 10:
                return False
            attempts.append(now)
            return True

    def reap(self):
        with self.lock:
            now = self.clock()
            for identifier, item in list(self.items.items()):
                if now-item["seen"] > SESSION_IDLE or now-item["created"] > 12*3600:
                    item["transport"].close()
                    del self.items[identifier]

    def create(self, label):
        with self.lock:
            self.reap()
            if len(self.items) >= 8:
                raise ValueError("在线设备已达 8 台，请关闭闲置页面后稍候再登录")
            identifier = secrets.token_urlsafe(32)
            self.items[identifier] = {"transport": self.factory(label), "csrf": secrets.token_urlsafe(32),
                                      "seen": self.clock(), "created": self.clock(), "lock": threading.Lock(),
                                      "recovery_lock": threading.Lock()}
            return identifier

    def get(self, cookie):
        try:
            parsed = SimpleCookie()
            parsed.load(cookie or "")
            cookie = parsed.get('ceiling_wall_console_session') or parsed.get('ceiling_session')
            identifier = cookie.value
        except (KeyError, ValueError, CookieError, AttributeError):
            return None
        with self.lock:
            self.reap()
            item = self.items.get(identifier)
            if item:
                item["seen"] = self.clock()
            return item


class Application:
    def __init__(self, args, factory=None):
        self.args = args
        self.offline = getattr(args, 'bridge', 'local') == 'mock'
        self.code = 'offline' if self.offline else access_code(Path(args.access_code))
        self.mock_hub = None
        if self.offline and factory is None:
            from .mock import MockHub
            self.mock_hub = MockHub()
            factory = self.mock_hub.connect
        self.sessions = Sessions(factory or (lambda label: BridgeTransport(args, label)))
        self.config_lock = threading.RLock()
        self.settings = None if self.offline else Path(args.settings)
        self.config = dict(backend.DEFAULT_CONFIG)
        if self.settings is not None and self.settings.exists():
            self.config = backend.validate_config(backend.migrate_legacy_ports(json.loads(self.settings.read_text())))
        self.hosts = {"127.0.0.1", "localhost", socket.gethostname().lower(), socket.gethostname().lower()+".local"}
        self.hosts.update(args.public_host)
        self.local_hosts = set()
        self.host_refresh_at = None
        self.host_lock = threading.Lock()
        self._refresh_local_hosts()

    def _refresh_local_hosts(self):
        # DHCP/network readiness can change after the user service starts.
        # Keep discovered addresses separate so old interface IPs expire too.
        if self.offline:
            return
        with self.host_lock:
            now = time.monotonic()
            if self.host_refresh_at is not None and now-self.host_refresh_at < 2.0:
                return
            self.host_refresh_at = now
            discovered = set()
            try:
                addresses = subprocess.check_output(
                    ["hostname", "-I"], text=True, stderr=subprocess.DEVNULL, timeout=2).split()
                for address in addresses:
                    try:
                        parsed = ipaddress.ip_address(address)
                    except ValueError:
                        continue
                    if parsed.version == 4 and not parsed.is_unspecified and not parsed.is_multicast:
                        discovered.add(str(parsed))
            except (OSError, subprocess.SubprocessError):
                pass
            self.local_hosts = discovered


    def valid_host(self, header):
        try:
            value = urlsplit("http://" + (header or ""))
            if (value.port != self.args.port or value.username or value.password
                    or value.path or value.query or value.fragment):
                return False
            if value.hostname in self.hosts:
                return True
            if self.offline or ipaddress.ip_address(value.hostname).version != 4:
                return False
            self._refresh_local_hosts()
            return value.hostname in self.local_hosts
        except ValueError:
            return False

    def handler(self):
        app = self
        class Handler(BaseHTTPRequestHandler):
            def setup(self):
                super().setup()
                self.connection.settimeout(10)

            def log_message(self, *args):
                pass  # Never log cookies, request bodies or access codes.

            def respond(self, code, body, mime="application/json; charset=utf-8", cookie=None):
                if not isinstance(body, bytes):
                    body = json.dumps(backend.finite_json(body), ensure_ascii=False, allow_nan=False).encode()
                self.send_response(code)
                self.send_header("Content-Type", mime)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Referrer-Policy", "no-referrer")
                self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self'; frame-ancestors 'none'; form-action 'self'; base-uri 'none'")
                if cookie:
                    self.send_header("Set-Cookie", cookie)
                self.end_headers()
                try:
                    self.wfile.write(body)
                except OSError:
                    pass

            def session(self):
                return app.sessions.get(self.headers.get("Cookie"))

            def do_GET(self):
                if not app.valid_host(self.headers.get("Host")):
                    return self.respond(403, {"error": "访问地址无效"})
                path = urlsplit(self.path).path
                item = self.session()
                if path == '/api/info':
                    return self.respond(200, {'app_id': 'ceiling_wall_console', 'offline': app.offline,
                                              'login_hint': 'offline' if app.offline else ''})
                if path == "/api/status":
                    if not item:
                        return self.respond(401, {"error": "请先输入访问码登录"})
                    with app.config_lock:
                        config = dict(app.config)
                    try:
                        state = item["transport"].status(config)
                    except (ValueError, TypeError, KeyError, OSError) as error:
                        return self.respond(503, {'error': str(error)})
                    state.update(token=item["csrf"], console_ui="ceiling_wall_console", console_protocol=2,
                                 offline=app.offline)
                    return self.respond(200, state)
                assets = {"/": ("index.html" if item else "connect.html", "text/html; charset=utf-8"),
                          "/connect": ("connect.html", "text/html; charset=utf-8"),
                          "/connect.js": ("connect.js", "text/javascript; charset=utf-8"),
                          "/login": ("index.html" if item else "login.html", "text/html; charset=utf-8"),
                          "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                          "/login.js": ("login.js", "text/javascript; charset=utf-8"),
                          "/style.css": ("style.css", "text/css; charset=utf-8"),
                          "/manifest.webmanifest": ("manifest.webmanifest", "application/manifest+json"),
                          "/icon.svg": ("icon.svg", "image/svg+xml"),
                          "/icon-192.png": ("icon-192.png", "image/png"),
                          "/icon-512.png": ("icon-512.png", "image/png")}
                if path in assets:
                    name, mime = assets[path]
                    return self.respond(200, (ASSETS/name).read_bytes(), mime)
                return self.respond(404, {"error": "Not found"})

            def do_POST(self):
                host = self.headers.get("Host")
                if not app.valid_host(host) or self.headers.get("Origin") != "http://"+host:
                    return self.respond(403, {"error": "操作来源校验失败"})
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < length <= 8192:
                        raise ValueError("请求长度无效")
                    data = json.loads(self.rfile.read(length))
                    if not isinstance(data, dict):
                        raise ValueError("请求内容无效")
                    if self.path == "/api/login":
                        if not app.sessions.allowed_attempt(self.client_address[0]):
                            return self.respond(429, {"error": "尝试过多，请一分钟后重试"})
                        provided = str(data.get("code", "")).encode()
                        if not secrets.compare_digest(provided, app.code.encode()):
                            return self.respond(401, {"error": "访问码不正确"})
                        label = str(data.get("label", "浏览器"))[:32].strip() or "浏览器"
                        identifier = app.sessions.create(label)
                        return self.respond(200, {"message": "已连接，默认以观察端进入"}, cookie=
                                            "ceiling_wall_console_session="+identifier+"; HttpOnly; SameSite=Strict; Path=/")
                    item = self.session()
                    if not item:
                        return self.respond(401, {"error": "会话已过期，请重新登录"})
                    if not secrets.compare_digest(self.headers.get("X-Console-Token", ""), item["csrf"]):
                        return self.respond(403, {"error": "操作令牌无效，请刷新页面"})
                    action = ACTIONS.get(self.path)
                    if self.path == '/api/mock-scenario' and app.offline:
                        action = 'mock_scenario'
                    if not action:
                        return self.respond(404, {"error": "未知操作"})
                    if action == "repair_mavros" and data.get("grounded_confirmed") is not True:
                        raise ValueError("请确认飞机已落地、未解锁且电机停止")
                    if action == "push" and type(data.get("enabled")) is not bool:
                        raise ValueError("enabled 必须为布尔值")
                    action_lock = item["recovery_lock"] if action == "repair_mavros" else item["lock"]
                    if not action_lock.acquire(blocking=False):
                        return self.respond(409, {"error": "已有操作等待回执，请勿重复提交"})
                    try:
                        if action == "start":
                            config = backend.validate_config(backend.migrate_legacy_ports(data))
                            result = item["transport"].request(action, config)
                            with app.config_lock:
                                if app.settings is not None:
                                    app.settings.parent.mkdir(parents=True, exist_ok=True)
                                    temporary = app.settings.with_suffix(".tmp")
                                    temporary.write_text(json.dumps(config, ensure_ascii=False, indent=2))
                                    temporary.replace(app.settings)
                                app.config = config
                        else:
                            result = item["transport"].request(action, data if action in ("stop", "push", "repair_mavros", 'mock_scenario') else None)
                    finally:
                        action_lock.release()
                    return self.respond(200, result)
                except (ValueError, TypeError, KeyError, OSError) as error:
                    return self.respond(409, {"error": str(error)})
        return Handler


def configuration_file(name):
    current = Path.home()/'.config/ceiling_wall_console'/name
    legacy = Path.home()/'.config/ceiling-launchpad'/name
    return str(legacy if not current.exists() and legacy.exists() else current)


def open_existing_preview(args):
    """Double-clicking the offline launcher reopens its existing local preview."""
    if args.bridge != 'mock' or not args.open_browser:
        return False
    import urllib.request
    import urllib.error
    import webbrowser
    url = 'http://{}:{}'.format(args.bind, args.port)
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(url+'/api/info', timeout=1) as reply:
            info = json.load(reply)
    except (OSError, ValueError, urllib.error.URLError):
        return False
    if info.get('app_id') != 'ceiling_wall_console' or info.get('offline') is not True:
        raise ValueError('该端口已被其他程序或实机模式占用，请用 --port 选择其他端口')
    webbrowser.open(url+'/login')
    return True


def main():
    parser = argparse.ArgumentParser(description="ceiling_wall_console · 顶吸与侧吸测试台")
    parser.add_argument("--bind", default=None)
    parser.add_argument("--port", type=int, default=8790)
    parser.add_argument("--public-host", action="append", default=[])
    parser.add_argument("--bridge", choices=("local", "ssh", "mock"), default="local")
    parser.add_argument('--open-browser', action='store_true')
    parser.add_argument("--container", default="ros1_noetic")
    parser.add_argument("--remote-workspace", default="/root/ros_ws")
    parser.add_argument("--ssh-host", default="127.0.0.1")
    parser.add_argument("--ssh-user", default="pi")
    parser.add_argument("--ssh-key", default=str(Path.home()/".ssh/id_ed25519"))
    parser.add_argument("--ssh-port", type=int, default=22)
    parser.add_argument("--access-code", default=configuration_file('access-code'))
    parser.add_argument("--settings", default=configuration_file('settings.json'))
    args = parser.parse_args()
    args.bind = args.bind or ('127.0.0.1' if args.bridge == 'mock' else '0.0.0.0')
    if args.bridge == 'mock' and args.bind not in ('127.0.0.1', 'localhost'):
        parser.error('离线模拟仅允许监听本机回环地址')
    try:
        if open_existing_preview(args):
            return
    except ValueError as error:
        parser.error(str(error))
    if args.bridge == "local":
        args.ssh_host = socket.gethostname()
    app = Application(args)
    def cleanup():
        while True:
            time.sleep(20)
            app.sessions.reap()
    threading.Thread(target=cleanup, daemon=True).start()
    print("ceiling_wall_console: http://{}:{} · {}".format(args.bind,args.port,
          '离线模拟，访问码 offline；无 ROS / Docker / SSH 连接' if app.offline else '访问码文件 '+args.access_code), flush=True)
    http = ThreadingHTTPServer((args.bind,args.port),app.handler())
    if args.open_browser:
        import webbrowser
        host = args.bind if args.bind != '0.0.0.0' else '127.0.0.1'
        threading.Thread(target=webbrowser.open, args=('http://{}:{}/login'.format(host, args.port),), daemon=True).start()
    try:
        http.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        http.server_close()
        with app.sessions.lock:
            for item in app.sessions.items.values():
                item['transport'].close()


if __name__ == "__main__":
    main()
