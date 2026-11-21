"""Exercise real HTTP requests against a fake flight runtime; never send flight commands."""
import http.client
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock

# Keep Catkin's relay package ahead of the source-only fallback: it exposes msg/ and srv/.
sys.path.append(str(Path(__file__).resolve().parents[1]/"src"))
from px4_wall_ceiling_control.ceiling_wall_console.server import Application, Sessions, ThreadingHTTPServer
from px4_wall_ceiling_control.ceiling_wall_console.transport import backend
from px4_wall_ceiling_control.shared_bridge import SharedBridge


class FakeRuntime:
    def __init__(self):
        self.state = backend.ConsoleState()
        self.commands = []
        self.gate = None
        self.entered = threading.Event()
    def ceiling(self):
        self.commands.append("ceiling")
        self.entered.set()
        if self.gate:
            self.gate.wait(3)
        return {"message":"fake accepted"}
    def start(self, config):
        self.commands.append("start")
        self.state.config=config
        return {"message":"fake start"}
    def wall(self):
        self.commands.append('wall')
        return {"message":"fake wall"}
    def select_rc_mode(self, config):
        self.commands.append(("select_rc_mode", config["mode"]))
        return {"message": "fake selection"}
    def detach(self):
        self.commands.append('detach')
        return {"message":"fake detach"}
    def cancel_wait(self):
        self.commands.append('cancel_wait')
        return {"message":"fake cancel"}
    def ceiling_detach(self):
        self.commands.append('ceiling_detach')
        return {'message': 'fake ceiling detach'}
    def stop(self, config):
        self.commands.append('stop')
        return {'message': 'fake stop'}
    def repair_mavros(self,config):
        if config.get('grounded_confirmed') is not True:
            raise ValueError('Missing ground confirmation')
        self.commands.append('repair_mavros')
        return {"message":"fake repair"}


class HTTPTests(unittest.TestCase):
    def test_ch12_selection_reaches_controller_without_starting_an_action(self):
        cookie, token = self.login("pilot")
        data = {"mode": "incline"}
        self.assertEqual(self.request("/api/select-rc-mode", data, cookie, token)[0], 409)
        self.assertEqual(self.request("/api/acquire", {}, cookie, token)[0], 200)
        self.assertEqual(self.request("/api/select-rc-mode", data, cookie, token)[0], 200)
        self.assertEqual(self.runtime.commands, [("select_rc_mode", "incline")])

    def test_retired_fixed_height_endpoint_cannot_dispatch(self):
        cookie, token = self.login('pilot')
        self.request('/api/acquire', {}, cookie, token)
        self.assertEqual(self.request('/api/set-range-height', {}, cookie, token)[0], 404)
        self.assertEqual(self.runtime.commands, [])
        self.assertFalse(hasattr(backend.RemoteRuntime, 'set_range_height'))


    def test_real_mode_has_no_mock_scenario_endpoint(self):
        cookie, token = self.login()
        self.request('/api/acquire', {}, cookie, token)
        self.assertEqual(self.request('/api/mock-scenario', {'scenario': 'range_control'}, cookie, token)[0], 404)
        info = self.request('/api/info')[1]
        self.assertEqual(info['app_id'], 'ceiling_wall_console')
        self.assertFalse(info['offline'])
        self.assertEqual(info['login_hint'], '')
        self.assertEqual(self.runtime.commands, [])

    def test_ceiling_detach_stop_and_new_configuration_are_forwarded_unchanged(self):
        cookie, token = self.login()
        self.request('/api/acquire', {}, cookie, token)
        config = dict(backend.DEFAULT_CONFIG, hover_thrust=.48)
        self.assertEqual(self.request('/api/start', config, cookie, token)[0], 200)
        self.assertEqual(self.runtime.state.config, config)
        self.assertEqual(self.request('/api/ceiling-detach', {}, cookie, token)[0], 200)
        self.assertEqual(self.request('/api/stop', {'grounded_confirmed': True}, cookie, token)[0], 200)
        self.assertEqual(self.runtime.commands, ['start', 'ceiling_detach', 'stop'])

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.runtime=FakeRuntime()
        broker=SharedBridge(self.runtime)
        self.transports=[]
        runtime=self.runtime
        class FakeTransport:
            def __init__(inner,label):
                inner.id=str(len(self.transports))
                broker.connect(inner.id,label)
                self.transports.append(inner)
                inner.closed=False
            def status(inner,config):
                state=runtime.state.snapshot()
                state.update(session=dict(broker.metadata(inner.id),feedback_live=True),ssh={"connected":True})
                return state
            def request(inner,action,config=None):
                return broker.request(inner.id,action,config)
            def close(inner):
                inner.closed=True
                broker.disconnect(inner.id)
        args=SimpleNamespace(access_code=self.temp.name+'/code',settings=self.temp.name+'/settings.json',public_host=[],port=0)
        self.app=Application(args,FakeTransport)
        self.http=ThreadingHTTPServer(('127.0.0.1',0),self.app.handler())
        args.port=self.http.server_port
        self.host='127.0.0.1:'+str(args.port)
        self.thread=threading.Thread(target=self.http.serve_forever,daemon=True)
        self.thread.start()
        self.addCleanup(self.http.server_close)
        self.addCleanup(self.http.shutdown)

    def request(self,path='/',data=None,cookie=None,token=None,origin=True,host=None):
        connection=http.client.HTTPConnection('127.0.0.1',self.http.server_port,timeout=5)
        headers={'Host':host or self.host}
        if cookie:headers['Cookie']=cookie
        if token:headers['X-Console-Token']=token
        if data is not None:
            headers['Content-Type']='application/json'
            if origin:headers['Origin']='http://'+self.host if origin is True else origin
        connection.request('POST' if data is not None else 'GET',path,json.dumps(data) if data is not None else None,headers)
        result=connection.getresponse();body=result.read();status=result.status;response_headers=dict(result.getheaders());connection.close()
        if response_headers.get('Content-Type','').startswith('application/json'):
            body=json.loads(body)
        return status,body,response_headers

    def login(self,label='test'):
        status,body,headers=self.request('/api/login',{'code':self.app.code,'label':label})
        self.assertEqual(status,200,body)
        cookie=headers['Set-Cookie'].split(';')[0]
        self.assertIn('HttpOnly',headers['Set-Cookie']);self.assertIn('SameSite=Strict',headers['Set-Cookie'])
        status,body,_=self.request('/api/status',cookie=cookie)
        return cookie,body['token']

    def test_public_page_is_login_and_telemetry_requires_login(self):
        status,body,_=self.request();self.assertEqual(status,200);self.assertIn('选择控制台地址'.encode(),body)
        self.assertIn('输入访问码'.encode(),self.request('/login')[1])
        self.assertEqual(self.request('/api/status')[0],401)
        self.assertEqual(self.request('/api/acquire',{})[0],401)
        self.assertEqual(self.transports,[])
        cookie,token=self.login()
        self.assertIn('一键启动顶吸'.encode(),self.request(cookie=cookie)[1])

    def test_bad_code_origin_host_and_csrf_rejected(self):
        self.assertEqual(self.request('/api/login',{'code':'wrong'})[0],401)
        self.assertEqual(self.request('/api/login',{'code':self.app.code},origin='http://evil.test')[0],403)
        self.assertEqual(self.request('/api/login',{'code':self.app.code},origin=False)[0],403)
        self.assertEqual(self.request(host='evil.test:'+str(self.http.server_port))[0],403)
        cookie,token=self.login()
        self.assertEqual(self.request('/api/acquire',{},cookie,'bad')[0],403)
        self.assertEqual(self.runtime.commands,[])

    def test_two_devices_do_not_share_control_or_token(self):
        a,at=self.login('phone');b,bt=self.login('computer')
        self.assertNotEqual(at,bt)
        self.assertEqual(self.request('/api/acquire',{},a,at)[0],200)
        self.assertTrue(self.request('/api/status',cookie=a)[1]['session']['control'])
        self.assertFalse(self.request('/api/status',cookie=b)[1]['session']['control'])
        self.assertEqual(self.request('/api/acquire',{},b,bt)[0],409)
        self.assertEqual(self.request('/api/ceiling',{},b,bt)[0],409)
        self.assertEqual(self.request('/api/ceiling',{},b,at)[0],403)
        self.assertEqual(self.runtime.commands,[])
        self.assertEqual(self.request('/api/ceiling',{},a,at)[0],200)
        self.assertEqual(self.runtime.commands,['ceiling'])
        self.assertEqual(self.request('/api/release',{},a,at)[0],200)
        self.assertEqual(self.request('/api/acquire',{},b,bt)[0],200)

    def test_concurrent_commands_are_not_dispatched_twice(self):
        cookie,token=self.login();self.request('/api/acquire',{},cookie,token)
        self.runtime.gate=threading.Event()
        thread=threading.Thread(target=lambda:self.request('/api/ceiling',{},cookie,token))
        thread.start()
        self.assertTrue(self.runtime.entered.wait(1))
        try:self.assertEqual(self.request('/api/ceiling',{},cookie,token)[0],409)
        finally:self.runtime.gate.set();thread.join(3)
        self.assertEqual(self.runtime.commands,['ceiling'])

    def test_config_is_validated_and_saved_after_dispatch(self):
        cookie,token=self.login();self.request('/api/acquire',{},cookie,token)
        self.assertEqual(self.request('/api/start',{'hover_thrust':99},cookie,token)[0],409)
        self.assertEqual(self.runtime.commands,[])
        self.assertEqual(self.request('/api/start',dict(backend.DEFAULT_CONFIG),cookie,token)[0],200)
        self.assertEqual(json.loads(self.app.settings.read_text()),backend.DEFAULT_CONFIG)

    def test_login_rate_limit_and_access_code_permissions(self):
        for _ in range(10):self.assertEqual(self.request('/api/login',{'code':'wrong'})[0],401)
        self.assertEqual(self.request('/api/login',{'code':self.app.code})[0],429)
        self.assertEqual(Path(self.app.args.access_code).stat().st_mode & 0o777,0o600)

    def test_icons_manifest_and_unknown_actions(self):
        for path in ['/connect','/connect.js','/manifest.webmanifest','/icon.svg','/icon-192.png','/icon-512.png','/login.js','/style.css','/app.js']:
            self.assertEqual(self.request(path)[0],200,path)
        cookie,token=self.login()
        self.assertEqual(self.request('/api/arbitrary',{},cookie,token)[0],404)

    def test_both_console_addresses_are_accepted_and_login_keeps_existing_identity(self):
        self.app.hosts.update(['172.20.10.2','10.187.58.66'])
        for host in ['172.20.10.2','10.187.58.66']:
            self.assertEqual(self.request('/connect',host=host+':'+str(self.http.server_port))[0],200)
        cookie,token=self.login()
        self.assertIn('一键启动顶吸'.encode(),self.request('/login',cookie=cookie)[1])
        self.assertEqual(len(self.transports),1)

    def test_wall_actions_require_control_and_pause_is_per_device(self):
        a,at=self.login('phone');b,bt=self.login('computer')
        for path in ['/api/wall','/api/detach','/api/cancel-wait']:
            self.assertEqual(self.request(path,{},b,bt)[0],409)
        self.assertEqual(self.runtime.commands,[])
        self.assertEqual(self.request('/api/acquire',{},a,at)[0],200)
        self.assertEqual(self.request('/api/push',{'enabled':False},a,at)[0],200)
        self.assertFalse(self.request('/api/status',cookie=a)[1]['session']['push'])
        self.assertTrue(self.request('/api/status',cookie=b)[1]['session']['push'])
        self.assertEqual(self.request('/api/wall',{},a,at)[0],409)
        self.assertEqual(self.request('/api/push',{'enabled':True},a,at)[0],200)
        for path in ['/api/wall','/api/detach','/api/cancel-wait']:
            self.assertEqual(self.request(path,{},a,at)[0],200)
        self.assertEqual(self.runtime.commands,['wall','detach','cancel_wait'])

    def test_removed_takeover_endpoint_and_ui_never_dispatch(self):
        cookie, token = self.login('pilot')
        self.assertEqual(self.request('/api/rc-takeover', {}, cookie, token)[0], 404)
        self.request('/api/acquire', {}, cookie, token)
        self.assertEqual(self.request('/api/rc-takeover', {}, cookie, token)[0], 404)
        html = self.request(cookie=cookie)[1]
        script = self.request('/app.js')[1]
        self.assertNotIn(b'id="rcTakeover"', html)
        self.assertNotIn('一键遥控接管'.encode(), html)
        self.assertNotIn(b'/api/rc-takeover', script)
        self.assertNotIn(b'takeoverView', script)
        self.assertEqual(self.runtime.commands, [])

    def test_pause_payload_requires_boolean(self):
        cookie,token=self.login()
        for bad in [{},{'enabled':'false'},{'enabled':0}]:
            self.assertEqual(self.request('/api/push',bad,cookie,token)[0],409)
        self.assertTrue(self.request('/api/status',cookie=cookie)[1]['session']['push'])

    def test_ground_recovery_has_its_own_http_request_lock(self):
        cookie, token = self.login()
        self.request('/api/acquire', {}, cookie, token)
        item = self.app.sessions.get(cookie)
        item['lock'].acquire()
        try:
            self.assertEqual(self.request('/api/stop', {'grounded_confirmed':True}, cookie, token)[0], 409)
            self.assertEqual(self.request('/api/repair-mavros', {'grounded_confirmed':True}, cookie, token)[0], 200)
        finally:
            item['lock'].release()
        self.assertEqual(self.runtime.commands, ['repair_mavros'])

    def test_mavros_repair_requires_confirmation_and_preserves_healthy_owner(self):
        cookie,token=self.login()
        self.assertEqual(self.request('/api/repair-mavros',{},cookie,token)[0],409)
        self.assertEqual(self.request('/api/repair-mavros',{'grounded_confirmed':True},cookie,token)[0],200)
        self.assertEqual(self.runtime.commands,['repair_mavros'])
        self.runtime.commands.clear()
        self.assertEqual(self.request('/api/acquire',{},cookie,token)[0],200)
        self.assertEqual(self.request('/api/repair-mavros',{'grounded_confirmed':True},cookie,token)[0],200)
        self.assertEqual(self.runtime.commands,['repair_mavros'])


class ConsoleTransportTests(unittest.TestCase):

    def test_removed_takeover_action_never_reaches_shared_backend(self):
        from px4_wall_ceiling_control.ceiling_wall_console.transport import BridgeTransport
        transport = BridgeTransport.__new__(BridgeTransport)
        with mock.patch.object(backend.SSHTransport, 'request') as dispatch:
            with self.assertRaises(ValueError):
                transport.request('rc_takeover')
            dispatch.assert_not_called()


class SessionTests(unittest.TestCase):
    def test_idle_session_closes_its_bridge_and_requires_login(self):
        now=[0.0]
        class Fake:
            def __init__(self,label):self.closed=False
            def close(self):self.closed=True
        sessions=Sessions(Fake,clock=lambda:now[0])
        identifier=sessions.create('phone');item=sessions.get('ceiling_wall_console_session='+identifier)
        now[0]=181
        self.assertIsNone(sessions.get('ceiling_wall_console_session='+identifier))
        self.assertTrue(item['transport'].closed)


class MockHTTPTests(unittest.TestCase):
    request = HTTPTests.request
    login = HTTPTests.login

    def setUp(self):
        from px4_wall_ceiling_control.ceiling_wall_console import server
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        args = SimpleNamespace(access_code=self.temp.name+'/code', settings=self.temp.name+'/settings.json',
                               public_host=[], port=0, bridge='mock')
        with mock.patch.object(server, 'BridgeTransport', side_effect=AssertionError('real transport in mock mode')), \
             mock.patch.object(server.subprocess, 'Popen', side_effect=AssertionError('process in mock mode')), \
             mock.patch.object(server.subprocess, 'check_output', side_effect=AssertionError('process in mock mode')):
            self.app = Application(args)
        self.http = ThreadingHTTPServer(('127.0.0.1', 0), self.app.handler())
        args.port = self.http.server_port
        self.host = '127.0.0.1:'+str(args.port)
        thread = threading.Thread(target=self.http.serve_forever, daemon=True); thread.start()
        self.addCleanup(self.http.server_close); self.addCleanup(self.http.shutdown)
        self.cookie, self.token = self.login('offline pilot')
        self.assertEqual(self.request('/api/acquire', {}, self.cookie, self.token)[0], 200)

    def choose(self, scenario):
        status, reply, _ = self.request('/api/mock-scenario', {'scenario': scenario}, self.cookie, self.token)
        self.assertEqual(status, 200, reply)
        return self.request('/api/status', cookie=self.cookie)[1]

    def test_mock_never_writes_real_settings_or_access_code(self):
        self.assertEqual(self.app.code, 'offline')
        status, reply, _ = self.request('/api/start', dict(backend.DEFAULT_CONFIG), self.cookie, self.token)
        self.assertEqual(status, 200, reply)
        self.assertFalse(Path(self.app.args.settings).exists())
        self.assertFalse(Path(self.app.args.access_code).exists())
        snapshot = self.request('/api/status', cookie=self.cookie)[1]
        self.assertFalse(snapshot['status']['state']['value']['armed'])

    def test_all_scenarios_are_finite_json_and_keep_same_session_owner(self):
        from px4_wall_ceiling_control.ceiling_wall_console.mock import SCENARIOS
        for scenario in SCENARIOS:
            snapshot = self.choose(scenario)
            self.assertTrue(snapshot['offline'])
            self.assertTrue(snapshot['session']['control'])
            self.assertEqual(snapshot['mock_scenario'], scenario)
            json.dumps(snapshot, allow_nan=False)


    def test_observer_and_paused_controller_cannot_change_scenarios(self):
        b, bt = self.login('offline observer')
        self.assertEqual(self.request('/api/mock-scenario', {'scenario': 'wall_hold'}, b, bt)[0], 409)
        self.request('/api/push', {'enabled': False}, self.cookie, self.token)
        self.assertEqual(self.request('/api/mock-scenario', {'scenario': 'wall_hold'}, self.cookie, self.token)[0], 409)
        self.assertTrue(self.request('/api/status', cookie=b)[1]['session']['feedback_live'])

    def test_stale_and_disconnected_frames_cannot_authorize_actions(self):
        for scenario in ('stale', 'disconnected'):
            snapshot = self.choose(scenario)
            self.assertFalse(snapshot['session']['feedback_live'])
            for action in ('wall', 'detach', 'ceiling'):
                self.assertEqual(self.request('/api/'+action, {}, self.cookie, self.token)[0], 409)
        self.assertTrue(self.choose('range_control')['session']['feedback_live'])

    def test_removed_takeover_has_no_offline_route_or_scenario(self):
        from px4_wall_ceiling_control.ceiling_wall_console.mock import SCENARIOS, MockRuntime
        self.assertNotIn('rc_takeover', SCENARIOS)
        self.assertFalse(hasattr(MockRuntime, 'rc_takeover'))
        self.choose('range_fault')
        self.assertEqual(self.request('/api/rc-takeover', {}, self.cookie, self.token)[0], 404)
        self.assertEqual(self.request('/api/status', cookie=self.cookie)[1]['mock_scenario'], 'range_fault')
        self.assertEqual(self.request('/api/mock-scenario', {'scenario': 'rc_takeover'}, self.cookie, self.token)[0], 409)

    def test_original_ceiling_wall_flows_in_mock(self):
        for scenario, path, expected in (
                ('ceiling_ready', 'ceiling', 'ceiling_approach'),
                ('ceiling_hold', 'ceiling-detach', 'ceiling_detach'),
                ('wall_ready', 'wall', 'wall_wait'),
                ('wall_wait', 'cancel-wait', 'wall_ready'),
                ('wall_hold', 'detach', 'range_rotating')):
            self.choose(scenario)
            status, reply, _ = self.request('/api/'+path, {}, self.cookie, self.token)
            self.assertEqual(status, 200, (scenario, reply))
            self.assertEqual(self.request('/api/status', cookie=self.cookie)[1]['mock_scenario'], expected)


class MigrationTests(unittest.TestCase):
    def test_offline_launcher_reopens_matching_preview_and_rejects_real_mode(self):
        import io
        from px4_wall_ceiling_control.ceiling_wall_console.server import open_existing_preview
        args = SimpleNamespace(bridge='mock', open_browser=True, bind='127.0.0.1', port=8791)
        with mock.patch('urllib.request.build_opener') as factory, mock.patch('webbrowser.open') as browser:
            factory.return_value.open.return_value = io.BytesIO(b'{"app_id":"ceiling_wall_console","offline":true}')
            self.assertTrue(open_existing_preview(args))
            browser.assert_called_once_with('http://127.0.0.1:8791/login')
            browser.reset_mock()
            factory.return_value.open.return_value = io.BytesIO(b'{"app_id":"ceiling_wall_console","offline":false}')
            with self.assertRaises(ValueError):
                open_existing_preview(args)
            browser.assert_not_called()
            args.bridge = 'local'
            self.assertFalse(open_existing_preview(args))

    def test_existing_access_code_and_settings_are_reused_without_copying(self):
        from px4_wall_ceiling_control.ceiling_wall_console.server import configuration_file
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(Path, 'home', return_value=Path(directory)):
            legacy = Path(directory)/'.config/ceiling-launchpad'; legacy.mkdir(parents=True)
            (legacy/'access-code').write_text('test-legacy-code')
            self.assertEqual(configuration_file('access-code'), str(legacy/'access-code'))
            current = Path(directory)/'.config/ceiling_wall_console'; current.mkdir()
            (current/'access-code').write_text('test-new-code')
            self.assertEqual(configuration_file('access-code'), str(current/'access-code'))

    def test_real_transport_uses_existing_single_ros_bridge(self):
        from px4_wall_ceiling_control.ceiling_wall_console.transport import BridgeTransport
        transport = BridgeTransport.__new__(BridgeTransport)
        transport.args = SimpleNamespace(remote_workspace='/root/ros_ws', container='ros1_noetic', bridge='local')
        command = transport.command()
        self.assertEqual(command[:4], ['docker', 'exec', '-i', 'ros1_noetic'])
        self.assertIn('wall_console_server.py --bridge', command[-1])
        self.assertNotIn('roslaunch', command[-1])


if __name__=='__main__':unittest.main()


class HostRefreshTests(unittest.TestCase):
    def setUp(self):
        import threading
        from px4_wall_ceiling_control.ceiling_wall_console import server
        self.server = server
        self.app = Application.__new__(Application)
        self.app.args = SimpleNamespace(port=8790)
        self.app.offline = False
        self.app.hosts = {'127.0.0.1', 'localhost', '192.168.2.20'}
        self.app.local_hosts = set()
        self.app.host_refresh_at = None
        self.app.host_lock = threading.Lock()
        self.now = 10.
        clock = mock.patch.object(server.time, 'monotonic', side_effect=lambda: self.now)
        clock.start(); self.addCleanup(clock.stop)
        query = mock.patch.object(server.subprocess, 'check_output', return_value='')
        self.query = query.start(); self.addCleanup(query.stop)

    def test_network_appearing_after_startup_becomes_valid_without_restart(self):
        self.app._refresh_local_hosts()
        self.assertFalse(self.app.valid_host('10.52.56.66:8790'))
        self.query.return_value = '10.52.56.66 172.17.0.1 '
        self.now += 2.1
        self.assertTrue(self.app.valid_host('10.52.56.66:8790'))

    def test_address_change_adds_new_ip_and_removes_old_discovery(self):
        self.query.return_value = '10.52.56.66'
        self.assertTrue(self.app.valid_host('10.52.56.66:8790'))
        self.query.return_value = '10.52.56.67'
        self.now += 2.1
        self.assertFalse(self.app.valid_host('10.52.56.66:8790'))
        self.assertTrue(self.app.valid_host('10.52.56.67:8790'))

    def test_refresh_is_rate_limited_and_configured_hosts_remain_valid(self):
        for _ in range(10):
            self.assertFalse(self.app.valid_host('203.0.113.1:8790'))
        self.assertEqual(self.query.call_count, 1)
        self.assertTrue(self.app.valid_host('192.168.2.20:8790'))
        self.assertTrue(self.app.valid_host('localhost:8790'))

    def test_bad_hosts_ports_and_origins_remain_rejected(self):
        for host in ('evil.test:8790', '10.52.56.66:80', 'u@10.52.56.66:8790',
                     '10.52.56.66:8790/path', '10.52.56.66:8790?q=x',
                     '10.52.56.66:8790#x', '10.52.56.66:bad', ''):
            self.assertFalse(self.app.valid_host(host), host)
        self.query.assert_not_called()

    def test_failed_discovery_fails_closed_and_retries_later(self):
        self.query.return_value = '10.52.56.66'
        self.assertTrue(self.app.valid_host('10.52.56.66:8790'))
        self.query.side_effect = self.server.subprocess.TimeoutExpired('hostname', 2)
        self.now += 2.1
        self.assertFalse(self.app.valid_host('10.52.56.66:8790'))
        self.query.side_effect = None
        self.now += 2.1
        self.assertTrue(self.app.valid_host('10.52.56.66:8790'))

    def test_ipv6_and_bad_discovery_tokens_do_not_hide_valid_ipv4(self):
        self.query.return_value = 'bad ::1 0.0.0.0 224.0.0.1 10.52.56.66'
        self.assertTrue(self.app.valid_host('10.52.56.66:8790'))
        self.assertFalse(self.app.valid_host('0.0.0.0:8790'))
        self.assertFalse(self.app.valid_host('224.0.0.1:8790'))

    def test_mock_mode_never_queries_or_accepts_discovered_network_hosts(self):
        self.app.offline = True
        self.app._refresh_local_hosts()
        self.assertFalse(self.app.valid_host('10.52.56.66:8790'))
        self.query.assert_not_called()
