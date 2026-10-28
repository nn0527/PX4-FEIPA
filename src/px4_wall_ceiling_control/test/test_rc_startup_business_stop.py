"""Business Stop must release controls and sensors while preserving RC transport."""
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from px4_wall_ceiling_control.wall_console import ConsoleState
spec = importlib.util.spec_from_file_location("business_stop_backend", str(ROOT / "scripts/operator/wall_console_server.py"))
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class BusinessStopTests(unittest.TestCase):
    def runtime(self):
        r = object.__new__(server.RemoteRuntime)
        r.state = ConsoleState()
        r.state.update("state", dict(connected=True, armed=False, mode="POSCTL"))
        r.state.update("rc", dict(channels=[1500, 1500, 999, 1500, 999, 999, 999]))
        r.state.update("phase", "ENTERING")
        r.state.update("owner", 2)
        r.state.update("wall", dict(state_name="WALL_HOLD", reason="old task"))
        r.state.ceiling_request = dict(state="ACCEPTED", reason="old request")
        r.state.ceiling_request_seen = True
        r.rc_startup_enabled = True
        r.mavros_alive = mock.Mock(return_value=True)
        r.children = {"mavros": SimpleNamespace(poll=lambda: None),
                      "wall_flight": SimpleNamespace(poll=lambda: 0)}
        r.ros = None
        return r

    def test_success_keeps_rc_and_mavros_but_clears_all_business_state(self):
        r = self.runtime()
        child = r.children["mavros"]
        fcu = r.state.config["fcu_port"]
        with mock.patch.object(r, "stop_processes", return_value={}), \
                mock.patch.object(r, "stop_ros_residuals", return_value=[]) as graph, \
                mock.patch.object(r, "port_owners", side_effect=lambda p: [77] if p == fcu else []), \
                mock.patch.object(server.time, "sleep"):
            reply = r.stop()
        self.assertIn("遥控启动待命", reply["message"])
        self.assertIs(r.children["mavros"], child)
        self.assertNotIn("wall_flight", r.children)
        self.assertFalse(r.stop_requested)
        self.assertFalse(r.stopping)
        self.assertFalse(r.stop_keep_telemetry)
        snapshot = r.state.snapshot()
        self.assertTrue(snapshot["status"]["rc"]["fresh"])
        self.assertTrue(snapshot["status"]["state"]["fresh"])
        self.assertIsNone(snapshot["status"]["wall"]["value"])
        self.assertIsNone(snapshot["status"]["phase"]["value"])
        self.assertIsNone(snapshot["status"]["owner"]["value"])
        self.assertEqual(r.state.ceiling_request["state"], "NOT_SENT")
        self.assertFalse(r.state.ceiling_request_seen)
        graph.assert_called_with(keep_telemetry=True)

    def test_ground_repair_still_releases_fcu_and_discards_cached_feedback(self):
        r = self.runtime()
        r.children = {"mavros": SimpleNamespace(poll=lambda: 0)}
        with mock.patch.object(r, "stop_processes", return_value={}), \
                mock.patch.object(r, "stop_ros_residuals", return_value=[]) as graph, \
                mock.patch.object(r, "port_owners", return_value=[]), \
                mock.patch.object(server.time, "sleep"):
            r.stop({"grounded_confirmed": True}, _repair=True)
        self.assertNotIn("mavros", r.children)
        self.assertIsNone(r.state.snapshot()["status"]["rc"]["value"])
        self.assertTrue(r.stop_requested)
        graph.assert_called_with(keep_telemetry=False)

    def test_dead_mavros_is_cleaned_and_bootstrap_unblocked_after_verified_stop(self):
        r = self.runtime()
        r.mavros_alive.return_value = False
        r.children = {"mavros": SimpleNamespace(poll=lambda: 1)}
        with mock.patch.object(r, "stop_processes", return_value={}), \
                mock.patch.object(r, "stop_ros_residuals", return_value=[]) as graph, \
                mock.patch.object(r, "port_owners", return_value=[]), \
                mock.patch.object(server.time, "sleep"):
            reply = r.stop({"grounded_confirmed": True})
        self.assertNotIn("mavros", r.children)
        self.assertFalse(r.stop_requested)
        self.assertIn("正在恢复遥控通信", reply["message"])
        graph.assert_called_with(keep_telemetry=False)

    def test_stop_does_not_resume_while_startup_thread_can_still_spawn(self):
        r = self.runtime()
        r.startup_thread = mock.Mock()
        r.startup_thread.is_alive.return_value = True
        with mock.patch.object(r, "stop_processes", return_value={}), \
                mock.patch.object(server.time, "sleep"):
            with self.assertRaisesRegex(ValueError, "启动线程尚未退出"):
                r.stop()
        self.assertTrue(r.stop_requested)

    def test_sensor_port_still_occupied_blocks_stop_success_and_rc_resume(self):
        r = self.runtime()
        top = r.state.config["ceiling_port"]
        with mock.patch.object(r, "stop_processes", return_value={}), \
                mock.patch.object(r, "stop_ros_residuals", return_value=[]), \
                mock.patch.object(r, "port_owners", side_effect=lambda p: [88] if p == top else []), \
                mock.patch.object(server.time, "sleep"):
            with self.assertRaisesRegex(ValueError, "停止核验失败"):
                r.stop()
        self.assertTrue(r.stop_requested)
        self.assertIn("停止核验失败", r.state.launch_error)

    def test_remaining_controller_blocks_stop_even_if_mavros_is_allowed(self):
        r = self.runtime()
        with mock.patch.object(r, "stop_processes", return_value={}), \
                mock.patch.object(r, "stop_ros_residuals", return_value=["/renamed_controller"]), \
                mock.patch.object(r, "port_owners", return_value=[]), \
                mock.patch.object(server.time, "sleep"):
            with self.assertRaisesRegex(ValueError, "renamed_controller"):
                r.stop()
        self.assertTrue(r.stop_requested)

    def test_fresh_armed_state_cannot_stop_business_or_transport(self):
        r = self.runtime()
        r.state.update("state", dict(connected=True, armed=True, mode="POSCTL"))
        with mock.patch.object(r, "stop_processes") as inventory:
            with self.assertRaisesRegex(ValueError, "解锁状态"):
                r.stop()
        inventory.assert_not_called()

    def test_inventory_preserves_only_mavros_tree_and_stops_all_business_children(self):
        r = self.runtime()
        processes = [
            (901, 1, ["python3", "/opt/ros/noetic/bin/roslaunch", "px4_wall_ceiling_control", "wall_console_mavros.launch"]),
            (902, 901, ["/opt/ros/noetic/lib/mavros/mavros_node"]),
            (903, 901, ["/custom/transport_helper"]),
            (910, 1, ["python3", "/opt/ros/noetic/bin/roslaunch", "px4_wall_ceiling_control", "wall_console_flight.launch"]),
            (911, 910, ["python3", "/alternate/wall_perch_decision_node.py"]),
            (912, 911, ["/custom/control_helper"]),
            (920, 1, ["/opt/ros/noetic/lib/rosbag/record", "-O", "/root/wall_test_records/log"]),
        ]
        with tempfile.TemporaryDirectory() as folder:
            files = []
            for pid, parent, args in processes:
                directory = Path(folder) / str(pid)
                directory.mkdir()
                p = directory / "cmdline"
                p.write_bytes(b"\0".join(a.encode() for a in args) + b"\0")
                (directory / "status").write_text("PPid:\t%d\n" % parent)
                files.append(p)
            r.stop_keep_telemetry = True
            with mock.patch.object(server.Path, "glob", return_value=files):
                self.assertEqual(set(r.stop_processes()), {910, 911, 912, 920})
            r.stop_keep_telemetry = False
            with mock.patch.object(server.Path, "glob", return_value=files):
                self.assertEqual(set(r.stop_processes()), {901, 902, 903, 910, 911, 912, 920})

    def test_graph_excludes_transport_but_still_checks_controller_publishers(self):
        master = mock.Mock()
        master.getSystemState.return_value = (1, "", [
            [["/mavros/state", ["/mavros"]], ["/mavros/setpoint_raw/attitude", ["/renamed_controller"]]], [], []])
        master.lookupNode.return_value = (1, "", "http://localhost:4567")
        node = mock.Mock()
        node.getPid.return_value = (1, "", 987)
        with mock.patch.object(server.xmlrpc.client, "ServerProxy", side_effect=[master, node]):
            self.assertEqual(server.RemoteRuntime.stop_ros_residuals(keep_telemetry=True), ["/renamed_controller"])
        master.lookupNode.assert_called_once_with("/wall_console_stop_check", "/renamed_controller")


if __name__ == "__main__":
    unittest.main()
