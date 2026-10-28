import sys
import pathlib
import threading
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))
from px4_wall_ceiling_control.wall_console import ConsoleState
from px4_wall_ceiling_control.shared_bridge import SharedBridge


class SharedTests(unittest.TestCase):
    def setUp(self):
        self.now = 100
        self.runtime = Mock()
        self.runtime.state = ConsoleState()
        self.bridge = SharedBridge(self.runtime, lambda: self.now)
        self.bridge.connect("a", "A")
        self.bridge.connect("b", "B")

    def test_simultaneous_acquisition_has_one_winner(self):
        barrier = threading.Barrier(2)
        winners = []
        def acquire(identifier):
            barrier.wait()
            try:
                self.bridge.request(identifier, "acquire")
                winners.append(identifier)
            except ValueError:
                pass
        threads = [threading.Thread(target=acquire, args=(n,)) for n in ("a", "b")]
        for t in threads: t.start()
        for t in threads: t.join()
        self.assertEqual(len(winners), 1)

    def test_observer_cannot_bypass_http_and_send_actions(self):
        self.bridge.request("a", "acquire")
        for action in ("start", "wall", "detach", "cancel_wait", "rc_takeover"):
            with self.assertRaises(ValueError): self.bridge.request("b", action)
        self.runtime.start.assert_not_called()
        self.runtime.wall.assert_not_called()
        self.runtime.detach.assert_not_called()

    def test_pause_is_per_client_and_blocks_controller(self):
        self.bridge.request("a", "acquire")
        self.bridge.request("a", "push", {"enabled": False})
        self.assertFalse(self.bridge.metadata("a")["push"])
        self.assertTrue(self.bridge.metadata("b")["push"])
        self.assertTrue(self.bridge.metadata("a")["control"])
        with self.assertRaises(ValueError): self.bridge.request("a", "wall")
        self.bridge.request("a", "push", {"enabled": True})
        self.bridge.request("a", "wall")
        self.runtime.wall.assert_called_once()

    def test_disconnect_reserves_control_then_expires(self):
        self.bridge.request("a", "acquire")
        self.bridge.disconnect("a")
        with self.assertRaises(ValueError): self.bridge.request("b", "acquire")
        self.bridge.connect("a", "A")
        self.assertTrue(self.bridge.metadata("a")["control"])
        self.bridge.disconnect("a")
        self.now += 31
        self.bridge.request("b", "acquire")
        self.assertTrue(self.bridge.metadata("b")["control"])

    def test_active_flight_cannot_transfer_after_owner_expires(self):
        self.bridge.request("a", "acquire")
        snapshot = self.runtime.state.snapshot()
        snapshot["status"]["state"].update(fresh=True, value={"connected": True, "armed": True, "mode": "OFFBOARD"})
        self.runtime.state = Mock(snapshot=lambda: snapshot)
        with self.assertRaises(ValueError): self.bridge.request("a", "release")
        self.bridge.disconnect("a")
        self.now += 31
        with self.assertRaises(ValueError): self.bridge.request("b", "acquire")
        self.assertIsNone(self.bridge.owner)

    def test_source_posctl_default_and_configured_altctl_transfer_policy(self):
        snapshot=self.runtime.state.snapshot()
        snapshot['status']['state'].update(fresh=True,value=dict(connected=True,armed=True,mode='POSCTL'))
        snapshot['status']['runtime'].update(fresh=False,value={})
        snapshot['status']['phase'].update(fresh=True,value='IDLE')
        snapshot['status']['owner'].update(fresh=True,value=0)
        self.runtime.state=Mock(snapshot=lambda:snapshot)
        self.assertTrue(self.bridge.safe_transfer())  # POSCTL is the source default.
        snapshot['status']['state']['value']['mode']='ALTCTL'
        self.assertFalse(self.bridge.safe_transfer())
        snapshot['status']['runtime'].update(fresh=True,value=dict(wall_pilot_mode='ALTCTL'))
        self.assertTrue(self.bridge.safe_transfer())
        snapshot['status']['runtime']['fresh']=False
        self.assertFalse(self.bridge.safe_transfer())
        snapshot['status']['state']['value']['mode']='POSCTL'
        snapshot['status']['phase']['value']='ACTIVE'
        self.assertFalse(self.bridge.safe_transfer())
        snapshot['status']['phase']['value']='IDLE'
        snapshot['status']['owner']['value']=2
        self.assertFalse(self.bridge.safe_transfer())

    def test_ground_recovery_with_stale_armed_cache_does_not_grant_authority(self):
        self.bridge.request("a", "acquire")
        snapshot = self.runtime.state.snapshot()
        snapshot["status"]["state"].update(fresh=False, value={"connected":True,"armed":True,"mode":"OFFBOARD"})
        self.runtime.state = Mock(snapshot=lambda:snapshot)
        with self.assertRaises(ValueError): self.bridge.request("b", "repair_mavros")
        self.bridge.request("b", "repair_mavros", {"grounded_confirmed":True})
        self.runtime.repair_mavros.assert_called_once()
        self.assertEqual(self.bridge.owner, "a")
        with self.assertRaises(ValueError): self.bridge.request("b", "wall")

    def test_recovery_cannot_override_live_armed_vehicle_or_healthy_other_owner(self):
        self.bridge.request("a", "acquire")
        self.runtime.state.update("state", {"connected":True,"armed":False,"mode":"POSCTL"})
        with self.assertRaises(ValueError):
            self.bridge.request("b", "repair_mavros", {"grounded_confirmed":True})
        self.runtime.state.update("state", {"connected":True,"armed":True,"mode":"OFFBOARD"})
        with self.assertRaises(ValueError):
            self.bridge.request("b", "repair_mavros", {"grounded_confirmed":True})
        self.runtime.repair_mavros.assert_not_called()

    def test_explicit_release_allows_new_controller(self):
        self.bridge.request("a", "acquire")
        self.bridge.request("a", "release")
        self.bridge.request("b", "acquire")
        with self.assertRaises(ValueError): self.bridge.request("a", "detach")


if __name__ == "__main__": unittest.main()
