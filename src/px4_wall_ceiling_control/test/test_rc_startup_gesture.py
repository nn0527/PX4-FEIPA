"""RC startup regression checks; never send commands to flight hardware."""
import copy
import pathlib
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))
from px4_wall_ceiling_control.adapters.kill_switch import KillSwitchConfig
from px4_wall_ceiling_control.rc_startup import StickGesture, audit_parameters, safe_snapshot, RcStartup
from px4_wall_ceiling_control.wall_console import DEFAULT_CONFIG

NEUTRAL = [1500, 1500, 999, 1500, 999, 999, 999, 1999, 1500, 1999, 1500, 999]
GESTURE = [1003, 1999, 999, 1999] + NEUTRAL[4:]


def snapshot(channels=None):
    return {"starting": False, "launch_error": "", "status": {
        "state": {"fresh": True, "value": {"connected": True, "armed": False, "mode": "POSCTL"}},
        "rc": {"fresh": True, "value": {"channels": list(NEUTRAL if channels is None else channels)}},
        "runtime": {"fresh": True, "value": {"enabled": False}}}}


class GestureTests(unittest.TestCase):
    def hold(self, detector, channels, start=0, allowed=True, duration=2.3):
        return [detector.update(channels, allowed, start+i*.1) for i in range(int(duration*10)+1)]

    def test_boot_with_sticks_held_never_starts(self):
        self.assertFalse(any(self.hold(StickGesture(), GESTURE, duration=4)))

    def test_two_seconds_one_shot_and_no_throttle_center_requirement(self):
        detector = StickGesture()
        detector.update(NEUTRAL, True, 0)
        values = self.hold(detector, GESTURE, .1, duration=5)
        self.assertEqual(sum(values), 1)
        self.assertFalse(any(values[:20]))
        detector.update(NEUTRAL, True, 5.2)
        self.assertEqual(sum(self.hold(detector, GESTURE, 5.3)), 1)

    def test_short_or_partial_gesture_does_not_trigger(self):
        for index in range(4):
            detector = StickGesture()
            detector.update(NEUTRAL, True, 0)
            channels = list(GESTURE)
            channels[index] = 1500
            self.assertFalse(any(self.hold(detector, channels, .1)))
        detector = StickGesture()
        detector.update(NEUTRAL, True, 0)
        self.assertFalse(any(self.hold(detector, GESTURE, .1, duration=1.8)))

    def test_stale_signal_or_armed_state_requires_release_before_retry(self):
        for channels, allowed in ((GESTURE, False), ([], True)):
            detector = StickGesture()
            detector.update(NEUTRAL, True, 0)
            self.hold(detector, GESTURE, .1, duration=1)
            detector.update(channels, allowed, 1.2)
            self.assertFalse(any(self.hold(detector, GESTURE, 1.3)))

    def test_sampling_gap_resets_hold_and_release_requirement(self):
        detector = StickGesture()
        detector.update(NEUTRAL, True, 0)
        self.hold(detector, GESTURE, .1, duration=1)
        self.assertFalse(any(self.hold(detector, GESTURE, 1.6)))

    def test_out_of_range_channels_are_rejected(self):
        for value in (0, 65535, float("nan"), float("inf")):
            channels = list(GESTURE)
            channels[0] = value
            self.assertFalse(StickGesture.matches(channels))


class AuditTests(unittest.TestCase):
    def parameters(self):
        values = {"MAN_ARM_GESTURE": 0, "RC_MAP_ROLL": 1, "RC_MAP_PITCH": 2,
                  "RC_MAP_THROTTLE": 3, "RC_MAP_YAW": 4, "RC_MAP_ARM_SW": 6,
                  "RC_MAP_KILL_SW": 7, "RC_MAP_FLTMODE": 5, "COM_RC_IN_MODE": 0,
                  "RC_ARMSWITCH_TH": .75, "RC_KILLSWITCH_TH": .75}
        for channel in (6, 7):
            for key, value in (("MIN", 1000), ("MAX", 2000), ("REV", 1)):
                values["RC%d_%s" % (channel, key)] = value
        return values

    def read(self, values):
        def get(name, optional=False):
            if optional:
                return values.get(name)
            return values[name]
        return get

    def test_current_fcu_mapping_has_no_switch_conflict(self):
        arm, kill = audit_parameters(self.read(self.parameters()))
        self.assertEqual((arm.channel, kill.channel), (6, 7))

    def test_enabling_stick_arming_or_changing_mapping_blocks_start(self):
        for name, value in (("MAN_ARM_GESTURE", 1), ("RC_MAP_ROLL", 2),
                            ("RC_MAP_ARM_SW", 4), ("RC_MAP_KILL_SW", 1),
                            ("RC_MAP_FLTMODE", 3), ("COM_RC_IN_MODE", 1)):
            values = self.parameters()
            values[name] = value
            with self.subTest(name=name), self.assertRaises(ValueError):
                audit_parameters(self.read(values))

    def test_other_functions_cannot_share_stick_channels(self):
        for name in ("RC_MAP_OFFB_SW", "RC_MAP_GEAR_SW", "RC_MAP_PARAM1", "RC_MAP_TRANS_SW"):
            values = self.parameters()
            values[name] = 4
            with self.subTest(name=name), self.assertRaises(ValueError):
                audit_parameters(self.read(values))

    def test_missing_required_parameters_fail_closed(self):
        values = self.parameters()
        del values["MAN_ARM_GESTURE"]
        with self.assertRaises(KeyError):
            audit_parameters(self.read(values))


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.arm, self.kill = KillSwitchConfig(6), KillSwitchConfig(7)

    def safe(self, value):
        return safe_snapshot(value, self.arm, self.kill)

    def test_current_disarmed_state_is_allowed(self):
        self.assertTrue(self.safe(snapshot(GESTURE)))

    def test_armed_disconnected_or_nonpilot_mode_rejects(self):
        for key, value in (("armed", True), ("connected", False), ("mode", "OFFBOARD"), ("mode", "AUTO.MISSION")):
            data = snapshot(GESTURE)
            data["status"]["state"]["value"][key] = value
            self.assertFalse(self.safe(data))

    def test_stale_state_or_rc_rejects(self):
        for name in ("state", "rc"):
            data = snapshot(GESTURE)
            data["status"][name]["fresh"] = False
            self.assertFalse(self.safe(data))

    def test_arm_or_kill_active_rejects(self):
        for index in (5, 6):
            channels = list(GESTURE)
            channels[index] = 1999
            self.assertFalse(self.safe(snapshot(channels)))

    def test_ch12_counted_pulse_and_return_do_not_gate_disarmed_start(self):
        for pwm in (999, 1999, 999, 1999):
            channels = list(GESTURE)
            channels[11] = pwm
            self.assertTrue(self.safe(snapshot(channels)))

    def test_unrelated_auxiliary_values_do_not_block_gesture(self):
        channels = list(GESTURE)
        channels[11] = 0
        self.assertTrue(self.safe(snapshot(channels)))

    def test_intermediate_arm_switch_is_not_accepted_as_low(self):
        channels = list(GESTURE)
        channels[5] = 1500
        self.assertFalse(self.safe(snapshot(channels)))

    def test_reverse_kill_switch_is_decoded_from_confirmed_parameters(self):
        self.kill = KillSwitchConfig(7, reverse=-1)
        self.assertFalse(self.safe(snapshot(GESTURE)))

    def test_recheck_cancels_before_dispatch_if_fcu_or_kill_changed(self):
        for field in ("armed", "kill"):
            data = snapshot(GESTURE)
            if field == "armed":
                data["status"]["state"]["value"]["armed"] = True
            else:
                data["status"]["rc"]["value"]["channels"][6] = 1999
            controller = object.__new__(RcStartup)
            controller.audit = Mock()
            controller.arm, controller.kill = self.arm, self.kill
            controller.runtime = Mock(stop_requested=False, stopping=False, repairing=False)
            controller.runtime.state.snapshot.return_value = data
            controller.clock = lambda: 123
            controller.broker = SimpleNamespace(lock=threading.RLock())
            with self.assertRaises(ValueError):
                controller.trigger()
            controller.runtime.start.assert_not_called()

    def test_parameter_and_bootstrap_calls_do_not_interrupt_two_second_sampling(self):
        controller = object.__new__(RcStartup)
        controller.arm, controller.kill = self.arm, self.kill
        controller.runtime = Mock(stop_requested=False, stopping=False, repairing=False)
        controller.runtime.state.snapshot.return_value = snapshot(NEUTRAL)
        controller.gesture = StickGesture()
        controller.last_connection, controller.pending = True, False
        controller.absent_optional = set()
        controller.audited_at = 0
        controller.last_status = None
        controller.audit, controller.bootstrap, controller.trigger, controller.status = Mock(), Mock(), Mock(), Mock()
        controller.clock = lambda: 0
        controller.tick()
        controller.runtime.state.snapshot.return_value = snapshot(GESTURE)
        for i in range(1, 26):
            controller.clock = lambda i=i: i * .1
            controller.tick()
        controller.trigger.assert_called_once()
        controller.audit.assert_not_called()
        controller.bootstrap.assert_not_called()

    def test_parameter_audit_failure_never_dispatches_start(self):
        controller = object.__new__(RcStartup)
        controller.audit = Mock(side_effect=ValueError("stick arming enabled"))
        controller.runtime = Mock()
        controller.clock = lambda: 123
        with self.assertRaises(ValueError):
            controller.trigger()
        controller.runtime.start.assert_not_called()

    def test_dispatch_reuses_web_start_without_taking_web_ownership(self):
        controller = object.__new__(RcStartup)
        controller.audit = Mock()
        controller.arm, controller.kill = self.arm, self.kill
        controller.runtime = Mock(stop_requested=False, stopping=False, repairing=False)
        controller.runtime.state.snapshot.return_value = snapshot(GESTURE)
        controller.broker = SimpleNamespace(lock=threading.RLock(), owner="existing-web-client")
        controller.config = dict(DEFAULT_CONFIG)
        controller.clock = lambda: 123
        controller.status, controller.sound = Mock(), Mock()
        controller.trigger()
        controller.runtime.start.assert_called_once_with(DEFAULT_CONFIG)
        self.assertEqual(controller.broker.owner, "existing-web-client")
        self.assertTrue(controller.pending)
        controller.runtime.wall.assert_not_called()
        controller.runtime.ceiling.assert_not_called()
        controller.runtime.detach.assert_not_called()
        controller.sound.assert_called_once_with("accepted")

    def test_bootstrap_reuses_live_mavros_without_starting_controls(self):
        controller = object.__new__(RcStartup)
        controller.config = dict(DEFAULT_CONFIG)
        controller.runtime = Mock(stop_requested=False, stopping=False, repairing=False)
        controller.runtime.state.starting = False
        controller.runtime.nodes.return_value = {"/mavros"}
        controller.runtime.mavros_alive.return_value = True
        controller.runtime.fcu_url_matches.return_value = True
        controller.broker = SimpleNamespace(lock=threading.RLock())
        controller.bootstrap()
        controller.runtime.spawn.assert_not_called()
        controller.runtime.start.assert_not_called()

    def test_bootstrap_blocks_unresponsive_mavros_without_duplicate(self):
        controller = object.__new__(RcStartup)
        controller.config = dict(DEFAULT_CONFIG)
        controller.runtime = Mock(stop_requested=False, stopping=False, repairing=False)
        controller.runtime.state.starting = False
        controller.runtime.nodes.return_value = {"/mavros"}
        controller.runtime.mavros_alive.return_value = False
        controller.broker = SimpleNamespace(lock=threading.RLock())
        with self.assertRaises(ValueError):
            controller.bootstrap()
        controller.runtime.spawn.assert_not_called()
        controller.runtime.start.assert_not_called()

    def test_explicit_stop_does_not_bootstrap_or_start(self):
        controller = object.__new__(RcStartup)
        controller.runtime = Mock(stop_requested=True, stopping=False)
        controller.clock = lambda: 1
        controller.gesture = StickGesture()
        controller.status = Mock()
        controller.bootstrap = Mock()
        controller.tick()
        controller.bootstrap.assert_not_called()
        controller.runtime.start.assert_not_called()


if __name__ == "__main__":
    unittest.main()
