#!/usr/bin/env python3
"""Action admission and live-state regression tests without hardware or ROS."""
import pathlib
import json
import sys
import unittest
from unittest import mock
import xml.etree.ElementTree as ET
import importlib.util
import threading
import os
import subprocess
import tempfile
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))
from px4_wall_ceiling_control.wall_console import ConsoleState, DEFAULT_CONFIG, LEGACY_PORTS, launch_arguments, validate_config

server_path = pathlib.Path(__file__).resolve().parents[1] / "scripts/operator/wall_console_server.py"
spec = importlib.util.spec_from_file_location("wall_console_server", str(server_path))
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class ConsoleTests(unittest.TestCase):
    def test_retired_height_setting_is_migrated_only_when_loading_saved_config(self):
        old = dict(DEFAULT_CONFIG, detach_range_manual_target_m=1.5)
        with self.assertRaises(ValueError):
            validate_config(old)
        migrated = server.migrate_legacy_ports(old)
        self.assertEqual(validate_config(migrated), DEFAULT_CONFIG)
        self.assertFalse(any('manual_target' in arg for arg in launch_arguments(migrated)))




    def test_fusion_status_serializes_ros_times_and_expires(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        fusion = SimpleNamespace(enabled=True, phase="settling", episode_id="wall-1",
            snapshot_valid=True, restore_required=False, parameters_restored=True,
            control_ready=False, original_flow=1, original_range=2, target_flow=1,
            target_range=2, confirmed_flow=1, confirmed_range=2, measured_tilt_deg=float("nan"),
            retry_count=0, reason="application buffer", request_stamp=SimpleNamespace(to_sec=lambda: 8.25),
            confirmed_stamp=SimpleNamespace(to_sec=lambda: 8.5), ready_after=SimpleNamespace(to_sec=lambda: 9.7))
        runtime.callback(fusion, "wall_sensor_fusion")
        item = runtime.state.snapshot()["status"]["wall_sensor_fusion"]
        self.assertTrue(item["fresh"])
        self.assertEqual(item["value"]["confirmed_stamp"], 8.5)
        self.assertEqual(item["value"]["ready_after"], 9.7)
        self.assertEqual(item["value"]["original_range"], 2)
        self.assertIsNone(item["value"]["measured_tilt_deg"])
        self.assertTrue(item["value"]["parameters_restored"])
        self.assertFalse(item["value"]["control_ready"])
        json.dumps(item, allow_nan=False)  # No ROS Time or NaN leaks to HTTP.
        self.now += .51
        self.assertFalse(runtime.state.snapshot()["status"]["wall_sensor_fusion"]["fresh"])

    def test_task_sync_plugins_are_isolated_in_project_launch(self):
        import yaml
        root=server_path.parents[2]
        config=yaml.safe_load((root/'config/mavros_wall_pluginlists.yaml').read_text())
        self.assertTrue({'waypoint','geofence','rallypoint'}.issubset(config['plugin_blacklist']))
        self.assertNotIn('setpoint_raw',config['plugin_blacklist'])
        launch=ET.parse(root/'launch/system/wall_console_mavros.launch')
        args={n.attrib['name']:n.attrib['value'] for n in launch.findall('./include/arg')}
        self.assertIn('mavros_wall_pluginlists.yaml',args['pluginlists_yaml'])
    def test_existing_mavros_must_also_have_task_isolation(self):
        runtime=object.__new__(server.RemoteRuntime);runtime.ros=mock.Mock()
        runtime.ros.get_param.return_value=['waypoint','geofence']
        with self.assertRaisesRegex(ValueError,'任务同步'):runtime.require_mavros_task_isolation()
        runtime.ros.get_param.return_value=['waypoint','geofence','rallypoint']
        runtime.require_mavros_task_isolation()

    def test_wall_wait_cancel_uses_configured_posctl_mode(self):
        state=ConsoleState()
        state.update("runtime",{"wall_pilot_mode":"POSCTL"})
        state.update("state",{"connected":True,"armed":True,"mode":"POSCTL"})
        state.update("phase","WAITING_WALL");state.update("owner",0)
        state.update("operator",{"enabled":True,"mode":3})
        self.assertTrue(state.snapshot()["can_cancel_wait"])
        state.update("state",{"connected":True,"armed":True,"mode":"OFFBOARD"})
        self.assertFalse(state.snapshot()["can_cancel_wait"])

    def test_restart_with_stale_stopped_mavros_feedback_bootstraps_connection(self):
        for old_armed in (False, True):
            now = [0.0]
            runtime = object.__new__(server.RemoteRuntime)
            runtime.state = ConsoleState(clock=lambda: now[0])
            runtime.state.update("state", {"connected": True, "armed": old_armed, "mode": "ALTCTL"})
            now[0] = 10.0
            runtime.ros = None
            runtime.nodes = lambda: set()
            runtime.stop_requested = True
            with mock.patch.object(server.threading, "Thread") as thread:
                runtime.start(dict(DEFAULT_CONFIG))
            thread.return_value.start.assert_called_once()
            self.assertTrue(runtime.state.starting)
            self.assertFalse(runtime.stop_requested)

    def test_restart_fresh_armed_feedback_still_blocks(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = ConsoleState()
        runtime.state.update("state", {"connected": True, "armed": True, "mode": "ALTCTL"})
        runtime.ros = None
        runtime.nodes = lambda: set()
        with mock.patch.object(server.threading, "Thread") as thread:
            with self.assertRaises(ValueError):
                runtime.start(dict(DEFAULT_CONFIG))
        thread.assert_not_called()


    def test_repair_accepts_running_chain_and_stale_armed_cache_only_with_confirmation(self):
        for stale in (False, True):
            runtime = self.stop_runtime()
            runtime.state.update('runtime', {'enabled':True})
            runtime.state.update('phase', 'CANCELING_ENTRY')
            if stale:
                snapshot = runtime.state.snapshot()
                snapshot['status']['state'].update(fresh=False, value={'armed':True})
                runtime.state.snapshot = lambda:snapshot
            with mock.patch.object(server.threading, 'Thread') as thread:
                with self.assertRaises(ValueError): runtime.repair_mavros({})
                runtime.repair_mavros({'grounded_confirmed':True})
                thread.assert_called_once()
            self.assertTrue(runtime.repairing)
            with self.assertRaises(ValueError): runtime.start(dict(DEFAULT_CONFIG))

    def test_repair_rejects_fresh_armed_state(self):
        runtime = self.stop_runtime()
        runtime.state.update('state', {'connected':True, 'armed':True, 'mode':'OFFBOARD'})
        with mock.patch.object(server.threading, 'Thread') as thread:
            with self.assertRaises(ValueError): runtime.repair_mavros({'grounded_confirmed':True})
            thread.assert_not_called()

    def test_repair_stops_old_chain_before_starting_unique_mavros(self):
        runtime = self.stop_runtime()
        runtime.repairing = True
        runtime.state.starting = True
        runtime.state.update('runtime', {'enabled':True})
        runtime.state.update('phase', 'CANCELING_ENTRY')
        runtime.nodes = mock.Mock(return_value={'/rosout'})
        runtime.ensure_ros_master = mock.Mock()
        runtime.init_ros = mock.Mock()
        runtime.port_owners = mock.Mock(return_value=[])
        runtime.mavros_process_conflicts = mock.Mock(return_value=[])
        runtime.require_mavros_task_isolation = mock.Mock()
        order = []
        def stop(*args, **kwargs):
            order.append('stop')
            runtime.state.samples.clear()
            runtime.state.update('runtime', {'enabled':False})
        runtime.stop = mock.Mock(side_effect=stop)
        def spawn(name, args):
            order.append(name)
            self.assertEqual(name, 'mavros')
            runtime.children[name] = mock.Mock(poll=lambda:None)
            runtime.state.update('state', {'connected':True,'armed':False,'mode':'POSCTL'})
        runtime.spawn = mock.Mock(side_effect=spawn)
        with mock.patch.object(server.Path, 'is_char_device', return_value=True):
            runtime.repair_mavros_worker()
        self.assertEqual(order, ['stop', 'mavros'])
        self.assertEqual(runtime.state.mavros_repair['state'], 'SUCCEEDED')
        self.assertFalse(runtime.state.starting)
        self.assertFalse(runtime.repairing)
        self.assertFalse(runtime.state.snapshot()['status']['runtime']['value']['enabled'])

    def test_repair_cleanup_failure_never_spawns_mavros_and_unlocks_retry(self):
        runtime = self.stop_runtime()
        runtime.repairing = True
        runtime.state.starting = True
        runtime.spawn = mock.Mock()
        runtime.stop = mock.Mock(side_effect=ValueError('serial still occupied'))
        runtime.repair_mavros_worker()
        runtime.stop.assert_called_once_with({'grounded_confirmed':True}, _repair=True)
        runtime.spawn.assert_not_called()
        self.assertFalse(runtime.repairing)
        self.assertFalse(runtime.state.starting)
        self.assertEqual(runtime.state.mavros_repair['state'], 'FAILED')

    def test_stop_verification_preserves_master_but_rejects_live_launcher(self):
        runtime = self.stop_runtime()
        runtime.children = {'roscore': SimpleNamespace(poll=lambda: None)}
        with mock.patch.object(runtime, 'stop_processes', return_value={}), \
                mock.patch.object(runtime, 'stop_ros_residuals', return_value=[]), \
                mock.patch.object(runtime, 'port_owners', return_value=[]):
            runtime.verify_stopped()
            runtime.children['wall_flight'] = SimpleNamespace(poll=lambda: None)
            with self.assertRaisesRegex(ValueError, '启动进程仍存活: wall_flight'):
                runtime.verify_stopped()

    def test_cleanup_failure_is_not_overwritten_by_old_launcher_exit(self):
        runtime = self.stop_runtime()
        runtime.stop_requested = True
        runtime.children = {'wall_flight': SimpleNamespace(poll=lambda: 0)}
        runtime.state.launch_error = '停止核验失败：serial still occupied'
        runtime.update_process_errors({'enabled': False, 'nodes': []})
        self.assertEqual(runtime.state.launch_error, '停止核验失败：serial still occupied')

    def test_stop_rejects_armed_or_expired_state(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = ConsoleState()
        runtime.state.update('state', {'connected': True, 'armed': True, 'mode': 'ALTCTL'})
        runtime.children = {}
        with self.assertRaisesRegex(ValueError, '解锁状态'):
            runtime.stop()

    def test_stop_stale_feedback_requires_grounded_confirmation(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = ConsoleState()
        runtime.children = {}
        runtime.ros = None
        with self.assertRaisesRegex(ValueError, '明确确认'):
            runtime.stop()
        with mock.patch.object(server.Path, 'glob', return_value=[]), \
                mock.patch.object(runtime, 'stop_ros_residuals', return_value=[]):
            self.assertIn('已终止', runtime.stop({'grounded_confirmed': True})['message'])

    def test_stop_without_remaining_processes_updates_runtime(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = ConsoleState()
        runtime.state.update('state', {'connected': True, 'armed': False, 'mode': 'ALTCTL'})
        runtime.children = {'mavros': mock.Mock(), 'wall_flight': mock.Mock()}
        runtime.ros = None
        runtime.state.launch_error = 'mavros 已退出，请查看启动日志'
        with mock.patch.object(server.Path, 'glob', return_value=[]), \
                mock.patch.object(runtime, 'stop_ros_residuals', return_value=[]):
            self.assertIn('已终止', runtime.stop()['message'])
        self.assertEqual(runtime.children, {})
        self.assertEqual(runtime.state.launch_error, '')
        self.assertFalse(runtime.state.snapshot()['status']['runtime']['value']['enabled'])
        self.assertTrue(runtime.stop_requested)
        self.assertIsNone(runtime.state.snapshot()["status"]["state"]["value"])

    def stop_runtime(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = ConsoleState()
        runtime.state.update('state', {'connected': True, 'armed': False, 'mode': 'POSCTL'})
        runtime.state.update('wall', {'state_name': 'ABORT', 'reason': 'previous fault'})
        runtime.children = {}
        runtime.ros = None
        return runtime

    def test_stop_does_not_report_success_for_node_missed_by_process_scan(self):
        runtime = self.stop_runtime()
        with mock.patch.object(runtime, 'stop_processes', return_value={}), \
                mock.patch.object(runtime, 'stop_ros_residuals', return_value=['/renamed_wall']), \
                mock.patch.object(runtime, 'port_owners', return_value=[]):
            with self.assertRaisesRegex(ValueError, '仍存活 ROS 节点.*renamed_wall'):
                runtime.stop()
        self.assertEqual(runtime.state.snapshot()['status']['wall']['value']['state_name'], 'ABORT')
        self.assertIn('停止核验失败', runtime.state.launch_error)
        self.assertFalse(runtime.stopping)
        with self.assertRaisesRegex(ValueError, '上次停止未完成'):
            runtime.start(dict(DEFAULT_CONFIG))

    def test_stop_rejects_process_appearing_after_signal_passes(self):
        runtime = self.stop_runtime()
        with mock.patch.object(runtime, 'stop_processes', side_effect=[{}, {}, {}, {}, {987: b'new'}]), \
                mock.patch.object(runtime, 'stop_ros_residuals', return_value=[]), \
                mock.patch.object(runtime, 'port_owners', return_value=[]):
            with self.assertRaisesRegex(ValueError, '残留 PID.*987'):
                runtime.stop()

    def test_stop_rejects_occupied_serial_or_inconclusive_graph(self):
        for graph_error in (False, True):
            runtime = self.stop_runtime()
            with mock.patch.object(runtime, 'stop_processes', return_value={}), \
                    mock.patch.object(runtime, 'stop_ros_residuals',
                        side_effect=TimeoutError('graph timeout') if graph_error else None,
                        return_value=[]), \
                    mock.patch.object(runtime, 'port_owners', return_value=[123] if not graph_error else []):
                with self.assertRaisesRegex(ValueError, 'graph timeout' if graph_error else '仍被 PID'):
                    runtime.stop()

    def test_stop_inventory_finds_alternate_path_and_unknown_launch_child(self):
        runtime = self.stop_runtime()
        with tempfile.TemporaryDirectory() as folder:
            files = []
            for pid, parent, args in [(901, 1, ['python3', '/alternate/install/wall_perch_decision_node.py']),
                                      (902, 901, ['/custom/worker']),
                                      (903, 1, ['python3', '/root/wall_console_server.py']),
                                      (904, 1, ['/usr/bin/roscore'])]:
                directory = pathlib.Path(folder) / str(pid)
                directory.mkdir()
                path = directory / 'cmdline'
                path.write_bytes(b'\0'.join(a.encode() for a in args) + b'\0')
                (directory / 'status').write_text('PPid:\t%d\n' % parent)
                files.append(path)
            with mock.patch.object(server.Path, 'glob', return_value=files):
                targets = runtime.stop_processes()
            self.assertEqual(set(targets), {901, 902})

    def test_graph_check_distinguishes_stale_registration_from_live_renamed_node(self):
        master = mock.Mock()
        master.getSystemState.return_value = (1, '', [[['/wall_perch/status', ['/renamed_wall']]], [], []])
        master.lookupNode.return_value = (1, '', 'http://localhost:4567')
        for live in (True, False):
            node = mock.Mock()
            if live:
                node.getPid.return_value = (1, '', 987)
            else:
                node.getPid.side_effect = ConnectionRefusedError()
            with mock.patch.object(server.xmlrpc.client, 'ServerProxy', side_effect=[master, node]):
                self.assertEqual(server.RemoteRuntime.stop_ros_residuals(), ['/renamed_wall'] if live else [])

    def test_mavros_replacement_clears_old_exit_and_reports_new_failure(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = ConsoleState()
        runtime.children = {"mavros": SimpleNamespace(poll=lambda: -6)}
        runtime.mavros_alive = mock.Mock(return_value=True)
        runtime.fcu_url_matches = mock.Mock(return_value=True)
        runtime.state.launch_error = "mavros 已退出，请查看启动日志"
        live = {"nodes": ["/mavros"], "enabled": True}
        runtime.update_process_errors(live)
        self.assertEqual(runtime.state.launch_error, "")
        self.assertNotIn("mavros", runtime.children)
        runtime.mavros_alive.return_value = False
        runtime.update_process_errors(live)
        self.assertEqual(runtime.state.launch_error, "mavros 无响应，请查看启动日志")
        runtime.mavros_alive.return_value = True
        runtime.update_process_errors(live)
        self.assertEqual(runtime.state.launch_error, "")

    def test_mavros_recovery_preserves_other_errors_and_rejects_wrong_port(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = ConsoleState()
        runtime.children = {"mavros": SimpleNamespace(poll=lambda: -6)}
        runtime.mavros_alive = mock.Mock(return_value=True)
        runtime.fcu_url_matches = mock.Mock(return_value=False)
        live = {"nodes": ["/mavros"], "enabled": True}
        runtime.update_process_errors(live)
        self.assertIn("mavros", runtime.children)
        self.assertEqual(runtime.state.launch_error, "mavros 已退出，请查看启动日志")
        runtime.children = {}
        runtime.fcu_url_matches.return_value = True
        runtime.state.launch_error = "雷达串口错误"
        runtime.update_process_errors(live)
        self.assertEqual(runtime.state.launch_error, "雷达串口错误")

    def test_wait_cancellation_requires_live_altctl_pending_wall_request(self):
        self.ready()
        self.state.update("phase", "WAITING_WALL")
        self.state.update("operator", {"mode": 3, "enabled": True})
        self.assertTrue(self.state.snapshot()["can_cancel_wait"])
        self.state.update("phase", "ENTERING")
        self.assertFalse(self.state.snapshot()["can_cancel_wait"])
        self.state.update("phase", "WAITING_WALL")
        self.state.update("state", {"connected": True, "armed": True, "mode": "OFFBOARD"})
        self.assertFalse(self.state.snapshot()["can_cancel_wait"])

    def test_legacy_default_ports_migrate_without_losing_parameters(self):
        original = dict(DEFAULT_CONFIG, **LEGACY_PORTS, hover_thrust=0.52)
        result = server.migrate_legacy_ports(original)
        self.assertEqual(result["fcu_port"], DEFAULT_CONFIG["fcu_port"])
        self.assertEqual(result["hover_thrust"], 0.52)
        self.assertEqual(original["fcu_port"], "/dev/ttyUSB0")
        alternate = dict(original, fcu_port="/dev/ttyUSB3")
        self.assertEqual(server.migrate_legacy_ports(alternate), alternate)

    def test_fcu_url_compares_resolved_device_and_baud(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.ros = mock.Mock()
        with tempfile.TemporaryDirectory() as folder:
            port = pathlib.Path(folder) / "port"
            runtime.ros.get_param.return_value = str(port) + ":230400"
            self.assertTrue(runtime.fcu_url_matches(str(pathlib.Path(folder) / "child/../port") + ":230400"))
            self.assertFalse(runtime.fcu_url_matches(str(port) + ":921600"))
    def test_mavros_liveness_rejects_dead_registration(self):
        with mock.patch.object(server.subprocess, "run", return_value=SimpleNamespace(returncode=1)):
            self.assertFalse(server.RemoteRuntime.mavros_alive())
        with mock.patch.object(server.subprocess, "run", side_effect=subprocess.TimeoutExpired("rosnode", 4)):
            self.assertFalse(server.RemoteRuntime.mavros_alive())
        with mock.patch.object(server.subprocess, "run", return_value=SimpleNamespace(returncode=0)):
            self.assertTrue(server.RemoteRuntime.mavros_alive())

    def test_password_auth_consumes_environment_without_persisting(self):
        args = SimpleNamespace(ssh_auth="password")
        with mock.patch.dict(os.environ, {"WALL_CONSOLE_SSH_PASSWORD": "fixture-secret"}), mock.patch.object(server.threading, "Thread"):
            transport = server.SSHTransport(args)
            self.assertEqual(transport.password, "fixture-secret")
            self.assertNotIn("WALL_CONSOLE_SSH_PASSWORD", os.environ)
            self.assertNotIn("password", transport.snapshot)

    def test_key_auth_does_not_use_password(self):
        with mock.patch.object(server.threading, "Thread"):
            transport = server.SSHTransport(SimpleNamespace(ssh_auth="key"))
            self.assertIsNone(transport.password)

    def telemetry_runtime(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        runtime.ros = mock.Mock()
        runtime.telemetry_lock = threading.Lock()
        runtime.telemetry_attempt_at = -float("inf")
        runtime.telemetry_connected = False
        return runtime

    def test_reconnect_restores_telemetry_without_start_button(self):
        runtime = self.telemetry_runtime()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        self.state.update("estimator", {"attitude": True, "vertical_velocity": True})
        runtime.configure_telemetry = mock.Mock()
        runtime.maintain_telemetry()
        runtime.configure_telemetry.assert_called_once()
        runtime.configure_telemetry.reset_mock()
        runtime.maintain_telemetry()
        runtime.configure_telemetry.assert_not_called()
        self.state.update("state", {"connected": False, "armed": False, "mode": "ALTCTL"})
        runtime.maintain_telemetry()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        runtime.maintain_telemetry()
        runtime.configure_telemetry.assert_called_once()

    def test_stale_stream_retries_disarmed_with_backoff(self):
        runtime = self.telemetry_runtime()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        runtime.telemetry_connected = True
        runtime.configure_telemetry = mock.Mock()
        with mock.patch.object(server.time, "monotonic", return_value=100):
            runtime.telemetry_attempt_at = 95
            runtime.maintain_telemetry()
            runtime.configure_telemetry.assert_not_called()
            runtime.telemetry_attempt_at = 89
            runtime.maintain_telemetry()
            runtime.configure_telemetry.assert_called_once()

    def test_telemetry_maintenance_never_configures_armed_or_stale_state(self):
        runtime = self.telemetry_runtime()
        runtime.configure_telemetry = mock.Mock()
        self.state.update("state", {"connected": True, "armed": True, "mode": "ALTCTL"})
        runtime.maintain_telemetry()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        self.now += 2
        runtime.maintain_telemetry()
        runtime.configure_telemetry.assert_not_called()

    def test_telemetry_configuration_rechecks_disarmed_after_service_wait(self):
        runtime = self.telemetry_runtime()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        runtime.ros.wait_for_service.side_effect = lambda *a, **k: self.state.update(
            "state", {"connected": True, "armed": True, "mode": "ALTCTL"})
        with mock.patch.dict(sys.modules, {"mavros_msgs.srv": SimpleNamespace(MessageInterval=object)}):
            with self.assertRaises(ValueError):
                runtime.configure_telemetry()
        runtime.ros.ServiceProxy.assert_not_called()

    def test_telemetry_configuration_requires_fcu_confirmation(self):
        runtime = self.telemetry_runtime()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        runtime.ros.ServiceProxy.return_value.return_value.success = False
        with mock.patch.dict(sys.modules, {"mavros_msgs.srv": SimpleNamespace(MessageInterval=object)}):
            with self.assertRaises(ValueError):
                runtime.configure_telemetry()
        runtime.ros.ServiceProxy.return_value.assert_called_once_with(message_id=230, message_rate=5.0)

    def test_telemetry_requests_attitude_target_for_smooth_thrust_takeover(self):
        runtime = self.telemetry_runtime()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        runtime.ros.ServiceProxy.return_value.return_value.success = True
        with mock.patch.dict(sys.modules, {"mavros_msgs.srv": SimpleNamespace(MessageInterval=object)}):
            runtime.configure_telemetry()
        self.assertEqual(runtime.ros.ServiceProxy.return_value.call_args_list,
                         [mock.call(message_id=230, message_rate=5.0), mock.call(message_id=83, message_rate=10.0), mock.call(message_id=105, message_rate=10.0)])

    def test_telemetry_rechecks_disarmed_before_attitude_stream_request(self):
        runtime = self.telemetry_runtime()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        def arm_after_estimator(**unused):
            self.state.update("state", {"connected": True, "armed": True, "mode": "ALTCTL"})
            return SimpleNamespace(success=True)
        runtime.ros.ServiceProxy.return_value.side_effect = arm_after_estimator
        with mock.patch.dict(sys.modules, {"mavros_msgs.srv": SimpleNamespace(MessageInterval=object)}):
            with self.assertRaises(ValueError):
                runtime.configure_telemetry()
        runtime.ros.ServiceProxy.return_value.assert_called_once_with(message_id=230, message_rate=5.0)

    def test_bridge_keeps_catkin_generated_messages_importable(self):
        # Reproduce a sourced Catkin workspace in a fresh interpreter. Importing
        # the bridge must not shadow its relay with the plain source package.
        source = server_path.parents[2] / "src/px4_wall_ceiling_control"
        with tempfile.TemporaryDirectory() as directory:
            relay = pathlib.Path(directory) / "px4_wall_ceiling_control"
            (relay / "msg").mkdir(parents=True)
            (relay / "__init__.py").write_text(
                "__path__ = [" + repr(str(relay)) + ", " + repr(str(source)) + "]\n",
                encoding="utf-8")
            (relay / "msg/__init__.py").write_text("generated = True\n", encoding="utf-8")
            code = ("import runpy; runpy.run_path(" + repr(str(server_path)) + "); "
                    "from px4_wall_ceiling_control.msg import generated; assert generated")
            result = subprocess.run([sys.executable, "-c", code],
                env=dict(os.environ, PYTHONPATH=directory), capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def setUp(self):
        self.now = 100.0
        self.state = ConsoleState(clock=lambda: self.now)
        self.publisher = mock.Mock()

    def ready(self):
        for name, value in {"runtime": {"enabled": True}, "state": {"connected": True, "armed": True, "mode": "ALTCTL"},
               "health": {"ceiling_valid": True, "front_valid": True}, "estimator": {"attitude": True, "vertical_velocity": True},
               "odom": {"finite": True}, "wall": {"state_name": "IDLE", "reason": "initialized",
                   "altitude_reference_valid": True, "height_above_takeoff_m": 1.2,
                   "min_entry_height_m": 0.3, "entry_height_ready": True},
               "phase": "IDLE", "owner": 0, "operator": {"mode": 1, "enabled": False}}.items():
            self.state.update(name, value)
        self.state.update("kill", {"valid": True, "active": False, "channel": 7})

    def ceiling_ready(self):
        self.ready()
        self.state.update("state", {"connected": True, "armed": True, "mode": "POSCTL"})
        self.state.update("runtime", {"enabled": True, "ceiling_enabled": True})
        self.state.update("ceiling_status", {"state_name": "NORMAL_FLIGHT", "reason": "idle"})
        self.state.update("ceiling", {"range_m": .4})
        self.state.update("odom", {"finite": True, "vertical_speed": 0.0})

    def test_ceiling_entry_and_duplicate_request(self):
        self.ceiling_ready()
        self.state.send_ceiling(self.publisher)
        self.publisher.assert_called_once_with("ceiling")
        with self.assertRaises(ValueError):
            self.state.send_ceiling(self.publisher)

    def test_ceiling_request_can_wait_outside_one_metre(self):
        self.ceiling_ready()
        self.state.update("runtime", {"enabled": True, "ceiling_enabled": True, "ceiling_distance_gate_enabled": True, "ceiling_activation_distance_m": 1.0})
        self.state.update("ceiling", {"range_m": 2.0})
        self.assertTrue(self.state.snapshot()["can_ceiling"])
    def test_latched_front_mask_blocks_wall_but_allows_ceiling(self):
        self.ceiling_ready()
        self.state.update("runtime", {"enabled": True, "ceiling_enabled": True, "front_sensor_masked": True})
        self.assertFalse(self.state.snapshot()["can_wall"])
        self.assertTrue(self.state.snapshot()["can_ceiling"])
    def test_ceiling_requires_top_but_not_front_or_wall_height(self):
        self.ceiling_ready()
        self.state.update("health", {"ceiling_valid": True, "front_valid": False})
        self.state.update("wall", {"state_name": "IDLE", "reason": "idle", "entry_height_ready": False})
        self.assertTrue(self.state.snapshot()["can_ceiling"])
        self.state.update("health", {"ceiling_valid": False})
        self.assertFalse(self.state.snapshot()["can_ceiling"])

    def test_ceiling_stale_kill_and_distance_gate_block(self):
        self.ceiling_ready()
        self.state.update("ceiling", {"range_m": .5})
        self.assertFalse(self.state.snapshot()["can_ceiling"])
        self.state.update("ceiling", {"range_m": .4})
        self.state.update("kill", {"valid": True, "active": True})
        self.assertFalse(self.state.snapshot()["can_ceiling"])
        self.ceiling_ready();self.now += 2
        self.assertFalse(self.state.snapshot()["can_ceiling"])

    def test_ceiling_detach_requires_ceiling_owner(self):
        self.ceiling_ready()
        self.state.update("state", {"connected": True, "armed": True, "mode": "OFFBOARD"})
        self.state.update("phase", "ACTIVE");self.state.update("owner", 1)
        self.state.update("operator", {"mode": 2, "enabled": True})
        self.state.update("setpoint", {"thrust": .48})
        self.state.update("ceiling_status", {"state_name": "SURFACE_HOLD", "reason": "hold", "range_fresh": True, "odom_fresh": True, "fault_detected": False})
        self.assertTrue(self.state.snapshot()["can_ceiling_detach"])
        self.state.send_ceiling_detach(self.publisher)
        self.publisher.assert_called_once_with("detach")
        self.state.update("owner", 2)
        self.assertFalse(self.state.snapshot()["can_ceiling_detach"])

    def test_baro_detach_requires_fresh_pressure_reference_without_top_range(self):
        self.ceiling_ready()
        self.state.update("setpoint", {"thrust": .50})
        self.state.update("state", {"connected":True,"armed":True,"mode":"OFFBOARD"})
        self.state.update("phase","ACTIVE");self.state.update("owner",1)
        self.state.update("operator",{"mode":2,"enabled":True})
        value={"state_name":"SURFACE_HOLD","reason":"hold","odom_fresh":True,"fault_detected":False,"range_fresh":False,"height_source":"barometer_hold_relative","baro_fresh":False,"baro_reference_ready":True}
        self.state.update("ceiling_status",value)
        self.assertFalse(self.state.snapshot()["can_ceiling_detach"])
        value["baro_fresh"]=True;self.state.update("ceiling_status",value)
        self.assertTrue(self.state.snapshot()["can_ceiling_detach"])
        value["baro_reference_ready"]=False;self.state.update("ceiling_status",value)
        self.assertFalse(self.state.snapshot()["can_ceiling_detach"])
    def test_ceiling_gated_request_queues_during_motion_and_invalid_range(self):
        self.ceiling_ready()
        self.state.update('runtime',{'enabled':True,'ceiling_enabled':True,'ceiling_distance_gate_enabled':True})
        self.state.update('odom',{'finite':True,'vertical_speed':.6})
        self.state.update('health',{'ceiling_valid':False,'front_valid':False})
        self.assertTrue(self.state.snapshot()['can_ceiling'])
        self.state.send_ceiling(lambda command:{'id':'r1','accepted':True,'reason':'received'})
        self.assertEqual(self.state.snapshot()['ceiling_request']['state'],'ACCEPTED')
    def ack_runtime(self):
        runtime=server.RemoteRuntime.__new__(server.RemoteRuntime)
        runtime.state=self.state;runtime.command_ack_lock=threading.Lock();runtime.command_acks={}
        runtime.publisher=mock.Mock();runtime.publisher.get_num_connections.return_value=1
        return runtime
    def test_backend_waits_for_matching_node_ack(self):
        runtime=self.ack_runtime()
        def receive(payload):
            request=json.loads(payload)
            runtime._on_terminal_ack(SimpleNamespace(data=json.dumps({'id':'wrong','accepted':True})))
            runtime._on_terminal_ack(SimpleNamespace(data=json.dumps({'id':request['id'],'accepted':True,'mode':2,'enable_control':True,'reason':'accepted'})))
        runtime.publisher.publish.side_effect=receive
        reply=runtime._publish_ceiling_request('ceiling')
        self.assertTrue(reply['accepted']);runtime.publisher.publish.assert_called_once()
        self.assertEqual(runtime.command_acks,{})
    def test_backend_unacknowledged_ceiling_has_no_success_or_automatic_retry(self):
        runtime=self.ack_runtime()
        with mock.patch.object(server.threading,'Event') as event:
            event.return_value.wait.return_value=False
            with self.assertRaisesRegex(ValueError,'未收到吸顶节点回执'):
                runtime._publish_ceiling_request('ceiling')
        runtime.publisher.publish.assert_called_once();self.assertEqual(runtime.command_acks,{})
    def test_backend_returns_node_rejection_reason(self):
        runtime=self.ack_runtime()
        def receive(payload):
            request=json.loads(payload)
            runtime._on_terminal_ack(SimpleNamespace(data=json.dumps({'id':request['id'],'accepted':False,'reason':'急停生效'})))
        runtime.publisher.publish.side_effect=receive
        with self.assertRaisesRegex(ValueError,'急停生效'):runtime._publish_ceiling_request('ceiling')
    def test_ready_dispatches_exactly_one_wall(self):
        self.ready()
        self.state.send_wall(self.publisher)
        self.publisher.assert_called_once_with("wall")
        with self.assertRaises(ValueError):
            self.state.send_wall(self.publisher)
        self.publisher.assert_called_once()

    def test_stale_estimator_blocks_even_when_flag_remains_true(self):
        self.ready()
        self.now += 1.6
        for name in ("state", "runtime", "health", "odom", "wall", "phase", "owner", "operator", "kill"):
            self.state.update(name, self.state.samples[name]["value"])
        with self.assertRaisesRegex(ValueError, "估计器"):
            self.state.send_wall(self.publisher)
        self.publisher.assert_not_called()

    def test_height_and_sensor_failures_are_visible_and_block_dispatch(self):
        self.ready()
        self.state.update("wall", dict(self.state.samples["wall"]["value"],
                                      height_above_takeoff_m=0.2, entry_height_ready=False))
        with self.assertRaisesRegex(ValueError, "相对起飞高度"):
            self.state.send_wall(self.publisher)
        self.ready()
        self.state.update("health", {"front_valid": False, "ceiling_valid": True})
        self.state.update("front_diagnostic", {"valid": False, "error": "L1 error 255 (weak reflection)"})
        with self.assertRaisesRegex(ValueError, "前向雷达.*E255"):
            self.state.send_wall(self.publisher)
        self.publisher.assert_not_called()

    def test_wall_range_faults_are_warnings_without_blocking_entry(self):
        self.ready()
        self.state.update('runtime',{'enabled':True,'wall_range_faults_diagnostic_only':True})
        self.state.update('health',{'front_valid':False,'ceiling_valid':False})
        snapshot=self.state.snapshot()
        self.assertTrue(snapshot['can_wall'])
        self.assertEqual(len(snapshot['sensor_warnings']),2)
        self.state.samples['health']['at']=0
        self.assertTrue(self.state.snapshot()['can_wall'])
        self.state.update('odom',{'finite':False})
        self.assertFalse(self.state.snapshot()['can_wall'])

    def test_restrained_bench_ignores_entry_height_and_sensor_not_flight_feedback(self):
        self.ready()
        self.state.config["test_profile"] = "restrained_bench"
        self.state.update("health", {"front_valid": False, "ceiling_valid": False})
        self.state.update("wall", dict(self.state.samples["wall"]["value"], entry_height_ready=False))
        self.assertTrue(self.state.snapshot()["can_wall"])
        self.state.update("state", {"connected": False, "armed": False, "mode": ""})
        self.assertFalse(self.state.snapshot()["can_wall"])
        self.assertIn("restrained_bench:=true", launch_arguments(self.state.config))

    def test_disarmed_wrong_mode_invalid_sensor_and_existing_intent_block(self):
        mutations = [("state", {"connected": True, "armed": False, "mode": "ALTCTL"}),
                     ("state", {"connected": True, "armed": True, "mode": "OFFBOARD"}),
                     ("health", {"ceiling_valid": False, "front_valid": True}),
                     ("operator", {"mode": 3, "enabled": True}), ("runtime", {"enabled": False})]
        for name, value in mutations:
            with self.subTest(name=name):
                self.ready()
                self.state.update(name, value)
                with self.assertRaises(ValueError):
                    self.state.send_wall(self.publisher)
        self.publisher.assert_not_called()

    def test_sent_or_stale_hold_is_not_success(self):
        self.ready()
        self.state.send_wall(self.publisher)
        self.assertFalse(self.state.snapshot()["success"])
        self.state.update("wall", {"state_name": "WALL_HOLD", "reason": "contact confirmed"})
        self.assertFalse(self.state.snapshot()["success"])
        self.state.update("phase", "ACTIVE")
        self.state.update("owner", 2)
        self.state.update("state", {"connected": True, "armed": True, "mode": "OFFBOARD"})
        self.state.update("setpoint", {"thrust": 0.6})
        self.assertTrue(self.state.snapshot()["success"])
        self.now += 0.6
        self.assertFalse(self.state.snapshot()["success"])

    def test_no_rc_mapping_needed_and_shell_input_rejected(self):
        self.assertEqual(validate_config(DEFAULT_CONFIG), DEFAULT_CONFIG)
        for change in ({"fcu_port": "/dev/ttyUSB0; reboot"}, {"front_port": DEFAULT_CONFIG["ceiling_port"]},
                       {"hover_thrust": float("nan")}, {"baud": True}, {"flight_enabled": False}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_config(dict(DEFAULT_CONFIG, **change))

    def holding(self):
        self.ready()
        self.state.update("state", {"connected": True, "armed": True, "mode": "OFFBOARD"})
        self.state.update("phase", "ACTIVE")
        self.state.update("owner", 2)
        self.state.update("operator", {"mode": 3, "enabled": True})
        self.state.update("setpoint", {"thrust": 0.6})
        self.state.update("wall", dict(self.state.samples["wall"]["value"],
            state_name="WALL_HOLD", hold_achieved=True, reached_state_name="WALL_HOLD",
            detach_ready=True, detach_completed=False, fault_detected=False))

    def test_detach_dispatches_once_only_in_current_ready_hold(self):
        self.holding()
        self.assertTrue(self.state.snapshot()["can_detach"])
        self.state.send_detach(self.publisher)
        self.publisher.assert_called_once_with("detach")
        with self.assertRaises(ValueError):
            self.state.send_detach(self.publisher)
        self.state.update("wall", dict(self.state.samples["wall"]["value"], state_name="DETACH_ROTATE"))
        self.now += 6
        self.assertFalse(self.state.snapshot()["can_detach"])

    def test_historical_success_never_authorizes_detach_after_abort(self):
        self.holding()
        self.state.update("wall", dict(self.state.samples["wall"]["value"], state_name="ABORT", fault_detected=True))
        result = self.state.snapshot()
        self.assertTrue(result["status"]["wall"]["value"]["hold_achieved"])
        self.assertFalse(result["success"])
        with self.assertRaises(ValueError):
            self.state.send_detach(self.publisher)
        self.publisher.assert_not_called()

    def test_detach_rejects_unsettled_pose_stale_sensor_and_lost_ssh(self):
        self.holding()
        self.state.update("wall", dict(self.state.samples["wall"]["value"], detach_ready=False))
        self.assertTrue(self.state.snapshot()["can_detach"])
        self.holding()
        self.state.samples["health"]["at"] = 99.0
        self.assertFalse(self.state.snapshot()["can_detach"])
        self.holding()
        bridge = object.__new__(server.SSHTransport)
        bridge.lock = threading.RLock()
        bridge.snapshot = self.state.snapshot()
        bridge.arrived, bridge.connected, bridge.error = 100.0, True, ""
        bridge.heartbeat, bridge.push_enabled = 100.0, True
        bridge.session = {"control": True, "push": True}
        bridge.args = SimpleNamespace(ssh_host="test-host", ssh_user="pi")
        with mock.patch.object(server.time, "monotonic", return_value=103.0):
            result = bridge.status(DEFAULT_CONFIG)
        self.assertFalse(result["can_detach"])
        self.assertTrue(result["status"]["wall"]["value"]["hold_achieved"])

    def test_failed_detach_publish_is_retryable(self):
        self.holding()
        self.publisher.side_effect = ValueError("未订阅")
        with self.assertRaises(ValueError):
            self.state.send_detach(self.publisher)
        self.assertIsNone(self.state.detach_pending_since)

    def test_kill_switch_blocks_detach_before_armed_flag_catches_up(self):
        self.holding()
        self.state.update("kill", {"valid": True, "active": True, "channel": 7})
        status = self.state.snapshot()
        self.assertTrue(status["status"]["state"]["value"]["armed"])
        self.assertFalse(status["success"])
        self.assertFalse(status["can_detach"])
        with self.assertRaisesRegex(ValueError, "急停"):
            self.state.send_detach(self.publisher)
        self.publisher.assert_not_called()

    def test_controller_queued_detach_blocks_duplicate_after_http_pending_expires(self):
        self.holding()
        self.state.update("wall", dict(self.state.samples["wall"]["value"], detach_pending=True))
        self.assertFalse(self.state.snapshot()["can_detach"])

    def test_launch_uses_dedicated_entry_with_explicit_real_output(self):
        args = launch_arguments(DEFAULT_CONFIG)
        self.assertEqual(args[:3], ["roslaunch", "px4_wall_ceiling_control", "wall_console_flight.launch"])
        path = pathlib.Path(__file__).resolve().parents[1] / "launch/system/wall_console_flight.launch"
        values = {arg.attrib["name"]: arg.attrib["value"] for arg in ET.parse(str(path)).findall("./include/arg")}
        for key in ("flight_mode_enabled", "output_enabled", "wall_enabled", "ceiling_enabled", "rc_assist_enabled", "rc_roll_yaw_enabled"):
            self.assertEqual(values[key], "true")
        for key in ("require_rc_center",):
            self.assertEqual(values[key], "false")
        self.assertEqual(values["wall_hold_time_s"], "0.0")

    def test_no_subscriber_cannot_be_reported_as_sent(self):
        self.ready()
        self.publisher.side_effect = ValueError("未订阅")
        with self.assertRaises(ValueError):
            self.state.send_wall(self.publisher)
        self.assertIsNone(self.state.pending_since)

    def test_transport_loss_clears_success_and_admission(self):
        self.ready()
        bridge = object.__new__(server.SSHTransport)
        bridge.lock = threading.RLock()
        bridge.snapshot = self.state.snapshot()
        bridge.snapshot["success"] = True
        bridge.arrived, bridge.connected, bridge.error = 100.0, True, ""
        bridge.heartbeat, bridge.push_enabled = 100.0, True
        bridge.session = {"control": True, "push": True}
        bridge.args = SimpleNamespace(ssh_host="test-host", ssh_user="pi")
        with mock.patch.object(server.time, "monotonic", return_value=103.0):
            status = bridge.status(DEFAULT_CONFIG)
        self.assertFalse(status["success"])
        self.assertFalse(status["can_wall"])
        self.assertTrue(all(not row["ok"] for row in status["checks"]))

    def test_transport_age_invalidates_estimator_before_ssh_disconnect(self):
        self.ready()
        self.state.samples["estimator"]["at"] = 98.6
        bridge = object.__new__(server.SSHTransport)
        bridge.lock = threading.RLock()
        bridge.snapshot = self.state.snapshot()
        bridge.arrived, bridge.connected, bridge.error = 100.0, True, ""
        bridge.heartbeat, bridge.push_enabled = 100.0, True
        bridge.session = {"control": True, "push": True}
        bridge.args = SimpleNamespace(ssh_host="test-host", ssh_user="pi")
        with mock.patch.object(server.time, "monotonic", return_value=100.2):
            status = bridge.status(DEFAULT_CONFIG)
        self.assertTrue(status["ssh"]["connected"])
        self.assertFalse(status["can_wall"])
        self.assertFalse(next(c["ok"] for c in status["checks"] if c["id"] == "estimator"))

    def test_running_pipeline_is_not_restarted(self):
        self.ready()
        self.state.launch_error = "上一次失败的启动错误"
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        runtime.ros = mock.Mock()
        runtime.ros.get_param.return_value = DEFAULT_CONFIG["fcu_port"] + ":" + str(DEFAULT_CONFIG["baud"])
        with mock.patch.object(server.threading, "Thread") as thread, \
             mock.patch.object(runtime, "inspect_runtime", return_value={"enabled": True}):
            reply = runtime.start(DEFAULT_CONFIG)
        self.assertIn("无需重复启动", reply["message"])
        self.assertEqual(self.state.launch_error, "")
        thread.assert_not_called()

    def test_new_computer_adopts_running_configuration(self):
        self.ready()
        running = dict(DEFAULT_CONFIG, test_profile="restrained_bench", hover_thrust=0.52,
                       fcu_port="/dev/ttyUSB3", front_port="/dev/ttyUSB4")
        self.state.config = running
        bridge = object.__new__(server.SSHTransport)
        bridge.lock = threading.RLock()
        bridge.snapshot = self.state.snapshot()
        bridge.arrived, bridge.connected, bridge.error = 100.0, True, ""
        bridge.heartbeat, bridge.push_enabled = 100.0, True
        bridge.session = {"control": True, "push": True}
        bridge.args = SimpleNamespace(ssh_host="test-host", ssh_user="pi")
        with mock.patch.object(server.time, "monotonic", return_value=100.1):
            result = bridge.status(DEFAULT_CONFIG)
        self.assertEqual(result["config"], running)

    def test_runtime_reads_pi_settings_for_another_computer(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        names = {"/mavros", "/terminal_operator_node", "/flight_mode_manager_node",
                 "/attachment_output_mux_node", "/wall_perch_decision_node", "/sensor_range_manager_node",
                 "/ceiling_lidar_node", "/front_lidar_node"}
        values = {"/mavros/fcu_url": "/dev/ttyUSB3:230400",
                  "/ceiling_lidar_node/serial_port": "/dev/ttyUSB1",
                  "/front_lidar_node/serial_port": "/dev/ttyUSB4",
                  "/wall_perch_decision_node/hover_thrust": 0.52}
        runtime.ros = SimpleNamespace(get_param=lambda name, default=False:
            values.get(name, False if name.endswith(("require_rc_center", "rc_assist_enabled")) else True))
        with mock.patch.object(runtime, "nodes", return_value=names):
            self.assertTrue(runtime.inspect_runtime()["enabled"])
        self.assertEqual(runtime.state.config["test_profile"], "restrained_bench")
        self.assertEqual(runtime.state.config["fcu_port"], "/dev/ttyUSB3")
        self.assertEqual(runtime.state.config["hover_thrust"], 0.52)

    def test_runtime_accepts_roll_yaw_assist_but_rejects_legacy_assist(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        names = {"/mavros", "/terminal_operator_node", "/flight_mode_manager_node",
                 "/attachment_output_mux_node", "/wall_perch_decision_node", "/sensor_range_manager_node",
                 "/ceiling_lidar_node", "/front_lidar_node"}
        for roll_yaw in (False, True):
            values = {"/mavros/fcu_url": "/dev/ttyUSB3:230400",
                      "/ceiling_lidar_node/serial_port": "/dev/ttyUSB1",
                      "/front_lidar_node/serial_port": "/dev/ttyUSB4",
                  "/wall_perch_decision_node/hover_thrust": 0.52,
                      "/attachment_output_mux_node/rc_assist_enabled": True,
                      "/attachment_output_mux_node/rc_roll_yaw_enabled": roll_yaw}
            runtime.ros = SimpleNamespace(get_param=lambda name, default=False:
                values.get(name, False if name.endswith("require_rc_center") else True))
            with mock.patch.object(runtime, "nodes", return_value=names):
                self.assertEqual(runtime.inspect_runtime()["enabled"], roll_yaw)

    def test_armed_vehicle_blocks_new_pipeline_start(self):
        self.ready()
        self.state.update("runtime", {"enabled": False})
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        runtime.ros = None
        with mock.patch.object(runtime, "nodes", return_value=set()), \
             mock.patch.object(server.threading, "Thread") as thread, self.assertRaisesRegex(ValueError, "未解锁"):
            runtime.start(DEFAULT_CONFIG)
        thread.assert_not_called()

    def test_reconnect_start_inspects_existing_nodes_before_first_monitor_tick(self):
        self.ready()
        self.state.update("runtime", {"enabled": False})
        runtime = self.telemetry_runtime()
        runtime.ros.get_param.return_value = DEFAULT_CONFIG["fcu_port"] + ":" + str(DEFAULT_CONFIG["baud"])
        with mock.patch.object(runtime, "inspect_runtime", return_value={"enabled": True}) as inspect, \
             mock.patch.object(server.threading, "Thread") as thread:
            self.assertIn("无需重复启动", runtime.start(DEFAULT_CONFIG)["message"])
        inspect.assert_called_once()
        thread.assert_not_called()

    def test_lidar_startup_resets_both_measurements_only_while_disarmed(self):
        self.ready()
        self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        factory = mock.MagicMock()
        yaml = SimpleNamespace(safe_load=lambda text: {
            name: {"input_format": "l1_ascii", "baudrate": 38400}
            for name in ("ceiling_lidar_node", "front_lidar_node")})
        with mock.patch.object(runtime, "port_owners", return_value=[]), \
             mock.patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=factory), "yaml": yaml}), \
             mock.patch.object(server.time, "sleep"):
            runtime.prepare_lidars()
            self.assertEqual(factory.call_count, 2)
            self.assertEqual([call.args[0] for call in factory.call_args_list],
                             [DEFAULT_CONFIG["ceiling_port"], DEFAULT_CONFIG["front_port"]])
            device = factory.return_value.__enter__.return_value
            self.assertEqual(device.write.call_args_list, [mock.call(b"iHALT"), mock.call(b"iHALT")])
            self.state.update("state", {"connected": True, "armed": True, "mode": "ALTCTL"})
            factory.reset_mock()
            with self.assertRaisesRegex(ValueError, "未解锁"):
                runtime.prepare_lidars()
            factory.assert_not_called()

    def test_front_usb_failure_is_reported_without_blocking_ceiling_startup(self):
        self.ready();self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        runtime=object.__new__(server.RemoteRuntime);runtime.state=self.state
        device=mock.MagicMock();factory=mock.Mock(side_effect=[device,OSError("front USB I/O")])
        yaml=SimpleNamespace(safe_load=lambda text:{name:{"input_format":"l1_ascii"} for name in ("ceiling_lidar_node","front_lidar_node")})
        with mock.patch.object(runtime,"port_owners",return_value=[]),mock.patch.dict(sys.modules,{"serial":SimpleNamespace(Serial=factory),"yaml":yaml}),mock.patch.object(server.time,"sleep"):
            runtime.prepare_lidars()
        self.assertTrue(any("front USB I/O" in e["text"] for e in self.state.events))
    def test_top_usb_failure_still_blocks_startup(self):
        self.ready();self.state.update("state", {"connected": True, "armed": False, "mode": "ALTCTL"})
        runtime=object.__new__(server.RemoteRuntime);runtime.state=self.state
        factory=mock.Mock(side_effect=OSError("top USB I/O"))
        yaml=SimpleNamespace(safe_load=lambda text:{name:{"input_format":"l1_ascii"} for name in ("ceiling_lidar_node","front_lidar_node")})
        with mock.patch.object(runtime,"port_owners",return_value=[]),mock.patch.dict(sys.modules,{"serial":SimpleNamespace(Serial=factory),"yaml":yaml}),self.assertRaisesRegex(OSError,"top USB"):
            runtime.prepare_lidars()

    def test_orphan_serial_reader_blocks_startup_reset(self):
        runtime = object.__new__(server.RemoteRuntime)
        runtime.state = self.state
        with mock.patch.object(runtime, "port_owners", return_value=[1234]), \
             self.assertRaisesRegex(ValueError, "旧 roslaunch.*1234"):
            runtime.prepare_lidars()


if __name__ == "__main__":
    unittest.main()
