"""Parameter transactions, recovery angle evidence, and durable crash recovery."""
import copy
import importlib.util
import json
import math
import os
import queue
import sys
import tempfile
import threading
import types
import unittest
from unittest import mock

from px4_wall_ceiling_control.domain.wall_sensor_fusion import (
    FLOW, RANGE, FreshSample, FusionPolicy, Operation, RestoreJournal, total_tilt_deg,
)


def quaternion(roll=0., pitch=0.):
    r, p = math.radians(roll)/2., math.radians(pitch)/2.
    return (math.sin(r)*math.cos(p), math.cos(r)*math.sin(p),
            -math.sin(r)*math.sin(p), math.cos(r)*math.cos(p))


class MemoryJournal:
    def __init__(self, saved=None):
        self.data = copy.deepcopy(saved)
        self.saves = []
        self.fail_save = False

    def load(self):
        return copy.deepcopy(self.data)

    def save(self, data):
        if self.fail_save:
            raise OSError("disk unavailable")
        self.data = copy.deepcopy(data)
        self.saves.append(copy.deepcopy(data))

    def clear(self):
        self.data = None


class Harness:
    def __init__(self, journal=None, enabled=True, originals=None):
        self.journal = journal or MemoryJournal()
        self.policy = FusionPolicy(self.journal, "test-fcu", enabled=enabled)
        self.now = self.mono = 10.
        self.state, self.tilt, self.armed = "IDLE", 0., True
        self.values = dict(originals or {FLOW: 1, RANGE: 2})
        self.feed()

    def feed(self, dt=.05, state=None, tilt=None, armed=None, connected=True):
        self.now += dt
        self.mono += dt
        self.state = state if state is not None else self.state
        self.tilt = tilt if tilt is not None else self.tilt
        self.armed = armed if armed is not None else self.armed
        self.policy.observe_fcu(connected, self.armed, self.now, self.now, self.mono)
        self.policy.observe_wall(self.state, self.now, self.now, self.mono)
        self.policy.observe_odom(quaternion(self.tilt), self.now, self.now, self.mono)

    def tick(self):
        return self.policy.tick(self.now, self.mono)

    def status(self):
        return self.policy.status(self.now, self.mono)

    def complete(self, op, success=True, values=None):
        if values is None:
            if op.kind == "set":
                if success:
                    self.values[op.parameter] = op.value
                values = {op.parameter: op.value}
            else:
                values = dict(self.values)
        return self.policy.complete(op, success, values, self.now, self.mono, "service rejected")

    def start(self):
        op = self.tick()
        assert op.kind == "snapshot"
        self.complete(op)

    def suspend(self):
        self.start()
        self.feed(state="WALL_HOLD", tilt=90.)
        first = self.tick()
        self.complete(first)
        self.feed()
        second = self.tick()
        self.complete(second)
        return first, second

    def restore(self, tilt=79., state="DETACH_ROTATE"):
        self.feed(state=state, tilt=tilt)
        first = self.tick()
        assert first.parameter == RANGE
        self.complete(first)
        self.feed()
        second = self.tick()
        assert second.parameter == FLOW
        self.complete(second)
        return first, second

    def settle(self):
        self.feed(dt=1.201)
        verification = self.tick()
        assert verification.kind == "verify"
        self.complete(verification)


class TiltAndFreshnessTests(unittest.TestCase):
    def test_total_angle_is_not_individual_euler_limit(self):
        self.assertAlmostEqual(total_tilt_deg(quaternion()), 0.)
        self.assertAlmostEqual(total_tilt_deg(quaternion(80.)), 80.)
        self.assertGreater(total_tilt_deg(quaternion(8., 8.)), 10.)
        self.assertAlmostEqual(total_tilt_deg(quaternion(180.)), 180.)

    def test_invalid_quaternions_are_not_normalized_into_valid_data(self):
        for q in ((0, 0, 0, 0), (0, 0, 0, 2), (0, math.nan, 0, 1), (0, 0, 1)):
            with self.assertRaises(ValueError):
                total_tilt_deg(q)

    def test_source_replay_out_of_order_future_and_receipt_age(self):
        g = FreshSample(.3)
        self.assertTrue(g.observe(10., 10., 10.))
        self.assertFalse(g.observe(10., 10.01, 10.01))
        self.assertFalse(g.observe(9.9, 10.02, 10.02))
        self.assertFalse(g.observe(10.3, 10.03, 10.03))
        self.assertTrue(g.observe(10.04, 10.04, 10.04))
        self.assertFalse(g.fresh(10.04, 10.35))  # paused ROS time still expires
        self.assertFalse(g.observe(10.05, 10.01, 10.36))
        self.assertFalse(g.observe(0., 10.1, 10.4))

    def test_angle_boundary_and_direction_require_new_fresh_evidence(self):
        h = Harness(); h.suspend()
        h.feed(state="DETACH_ROTATE", tilt=85.); self.assertIsNone(h.tick())
        h.feed(tilt=80.01); self.assertIsNone(h.tick())
        h.feed(tilt=80.); self.assertEqual(h.tick().parameter, RANGE)

    def test_entry_already_below_threshold_does_not_require_rate_test(self):
        h = Harness(); h.suspend()
        h.feed(tilt=30.)
        h.feed(state="DETACH_ROTATE", tilt=79.)
        self.assertEqual(h.tick().parameter, RANGE)  # valid at entry even if increasing

    def test_increasing_angle_after_entry_does_not_restore(self):
        h = Harness(); h.suspend()
        h.feed(state="DETACH_ROTATE", tilt=85.); self.assertIsNone(h.tick())
        h.feed(tilt=80.1)
        h.feed(tilt=80.2); self.assertIsNone(h.tick())
        h.feed(tilt=79.4); self.assertEqual(h.tick().parameter, RANGE)

    def test_stale_or_duplicate_orientation_cannot_restore(self):
        for invalid in ("stale", "duplicate", "future", "bad_quaternion"):
            h = Harness(); h.suspend()
            h.feed(state="DETACH_ROTATE", tilt=85.); h.tick()
            h.now += .4; h.mono += .4
            h.policy.observe_wall("DETACH_ROTATE", h.now, h.now, h.mono)
            h.policy.observe_fcu(True, True, h.now, h.now, h.mono)
            stamp = {"stale": h.now-.4, "duplicate": h.policy.odom.source,
                     "future": h.now+.2, "bad_quaternion": h.now}[invalid]
            q = (0, 0, 0, 0) if invalid == "bad_quaternion" else quaternion(79.)
            h.policy.observe_odom(q, stamp, h.now, h.mono)
            self.assertIsNone(h.tick(), invalid)
            h.feed(tilt=79.)
            self.assertEqual(h.tick().parameter, RANGE)  # fresh below-threshold re-entry after gap


class FusionTransactionTests(unittest.TestCase):
    def test_first_hold_only_and_journal_precedes_first_mutation(self):
        h = Harness(); h.start()
        self.assertTrue(h.status()["parameters_restored"])
        self.assertTrue(h.status()["control_ready"])
        for state in ("SLOW_APPROACH", "FLIP_TO_WALL", "WALL_CAPTURE"):
            h.feed(state=state, tilt=90.)
            self.assertIsNone(h.tick())
        h.feed(state="WALL_HOLD")
        op = h.tick()
        self.assertEqual((op.parameter, op.value), (FLOW, 0))
        self.assertEqual(h.journal.data["originals"], {FLOW: 1, RANGE: 2})
        self.assertEqual(h.journal.data["pending"]["parameter"], FLOW)
        self.assertFalse(h.status()["control_ready"])
        h.complete(op); h.feed(); op = h.tick()
        self.assertEqual((op.parameter, op.value), (RANGE, 0))
        h.complete(op)
        for _ in range(5):
            h.feed(); self.assertIsNone(h.tick())
        self.assertEqual(h.status()["phase"], "SUSPENDED")

    def test_restore_range_then_flow_settle_verify_and_latch(self):
        h = Harness(); h.suspend(); h.restore()
        self.assertEqual(h.status()["phase"], "SETTLING")
        self.assertFalse(h.status()["control_ready"])
        self.assertIsNotNone(h.journal.data)
        h.feed(dt=1.19); self.assertIsNone(h.tick())
        h.feed(dt=.011); verify = h.tick()
        self.assertEqual(verify.kind, "verify")
        self.assertFalse(h.status()["control_ready"])
        h.complete(verify)
        self.assertTrue(h.status()["parameters_restored"])
        self.assertTrue(h.status()["control_ready"])
        self.assertFalse(h.status()["restore_required"])
        self.assertEqual(h.status()["ready_after"], h.now)
        self.assertIsNone(h.journal.data)
        for tilt in (84., 70., 86., 79.):
            h.feed(state="WALL_HOLD", tilt=tilt)
            self.assertIsNone(h.tick())

    def test_next_cycle_takes_new_snapshot_in_idle(self):
        h = Harness(); h.suspend(); h.restore(); h.settle()
        h.values = {FLOW: 3, RANGE: 1}
        h.feed(state="IDLE"); self.assertIsNone(h.tick())
        self.assertTrue(h.status()["control_ready"])
        h.feed(armed=False); h.tick()
        h.feed(armed=True); op = h.tick()
        self.assertEqual(op.kind, "snapshot"); h.complete(op)
        h.feed(state="WALL_HOLD", tilt=90.); op = h.tick()
        self.assertEqual(h.journal.data["originals"], {FLOW: 3, RANGE: 1})

    def test_partial_disable_error_still_restores_both_actual_originals(self):
        h = Harness(originals={FLOW: 0, RANGE: 2}); h.start()
        h.feed(state="WALL_HOLD", tilt=90.); h.complete(h.tick())
        h.feed(); op = h.tick(); h.complete(op, success=False)
        self.assertEqual(h.status()["phase"], "ERROR")
        h.feed(dt=1.1, state="DETACH_ROTATE", tilt=70.)
        op = h.tick(); self.assertEqual((op.parameter, op.value), (RANGE, 2)); h.complete(op)
        h.feed(); op = h.tick(); self.assertEqual((op.parameter, op.value), (FLOW, 0)); h.complete(op)
        h.settle(); self.assertTrue(h.status()["control_ready"])

    def test_timeout_keeps_serial_slot_and_late_disable_ack_cannot_satisfy_restore(self):
        h = Harness(); h.start()
        h.feed(state="WALL_HOLD", tilt=90.); disable = h.tick()
        h.feed(dt=5.1, state="DETACH_ROTATE", tilt=70.)
        self.assertIsNone(h.tick())
        self.assertEqual(h.status()["phase"], "ERROR")
        self.assertIn("unresolved", h.status()["reason"])
        self.assertEqual(h.journal.data["desired"], "restore")
        self.assertEqual(h.journal.data["pending"]["value"], 0)
        h.complete(disable)
        self.assertFalse(h.status()["control_ready"])
        op = h.tick(); self.assertEqual(op.parameter, RANGE); h.complete(op)
        h.feed(); op = h.tick(); self.assertEqual((op.parameter, op.value), (FLOW, 1)); h.complete(op)
        self.assertFalse(h.policy.complete(disable, True, {FLOW: 0}, h.now, h.mono))  # duplicate result only
        h.settle(); self.assertEqual(h.values, {FLOW: 1, RANGE: 2})

    def test_failed_requests_back_off_and_value_mismatch_is_not_an_ack(self):
        h = Harness(); h.start(); h.feed(state="WALL_HOLD", tilt=90.)
        op = h.tick(); h.complete(op, values={FLOW: 1})
        self.assertEqual(h.status()["confirmed_flow"], -1)
        self.assertEqual(h.status()["retry_count"], 1)
        for _ in range(10):
            h.feed(); self.assertIsNone(h.tick())
        h.feed(dt=.6); op = h.tick()
        self.assertEqual(op.parameter, FLOW); h.complete(op, success=False)
        h.feed(dt=1.1); self.assertIsNone(h.tick())
        h.feed(dt=1.); self.assertEqual(h.tick().parameter, FLOW)

    def test_post_settle_readback_mismatch_reissues_restores(self):
        h = Harness(); h.suspend(); h.restore()
        h.feed(dt=1.21); op = h.tick(); h.complete(op, values={FLOW: 0, RANGE: 2})
        self.assertFalse(h.status()["control_ready"])
        self.assertIsNotNone(h.journal.data)
        h.feed(dt=1.1); self.assertEqual(h.tick().parameter, RANGE)

    def test_unknown_originals_or_unwritable_journal_never_issue_a_write(self):
        h = Harness(); h.feed(state="WALL_HOLD", tilt=90.)
        self.assertIsNone(h.tick()); self.assertFalse(h.status()["snapshot_valid"])
        h = Harness(); h.start(); h.journal.fail_save = True
        h.feed(state="WALL_HOLD", tilt=90.)
        self.assertIsNone(h.tick()); self.assertIn("journal", h.status()["reason"])
        self.assertIsNone(h.policy.in_flight)

    def test_failed_snapshot_never_uses_partial_original_values(self):
        h = Harness(); op = h.tick(); h.complete(op, values={FLOW: 1})
        self.assertFalse(h.status()["snapshot_valid"])
        h.feed(dt=1.1, state="WALL_HOLD", tilt=90.)
        self.assertIsNone(h.tick())

    def test_abort_and_fresh_disarm_restore_without_waiting_for_wall_transition(self):
        for action in ("ABORT", "RECOVER", "disarm"):
            h = Harness(); h.suspend()
            if action == "disarm":
                h.feed(armed=False, tilt=90.)
            else:
                h.feed(state=action, tilt=70.)
            self.assertEqual(h.tick().parameter, RANGE)

    def test_disconnected_or_stale_disarm_is_not_authorization(self):
        h = Harness(); h.suspend()
        h.feed(armed=False, connected=False)
        self.assertIsNone(h.tick())
        h.feed(armed=False)
        h.now += 1.6; h.mono += 1.6
        self.assertIsNone(h.tick())
        self.assertFalse(h.policy.restore_intent)

    def test_restart_preserves_originals_and_disabled_node_honors_journal(self):
        h = Harness(); h.suspend()
        restarted = Harness(journal=h.journal, enabled=False, originals={FLOW: 0, RANGE: 0})
        restarted.feed(state="WALL_HOLD", tilt=90.)
        op = restarted.tick(); self.assertEqual(op.kind, "reconcile")
        restarted.complete(op)
        self.assertEqual(restarted.policy.originals, {FLOW: 1, RANGE: 2})
        restarted.feed(state="DETACH_ROTATE", tilt=75.)
        op = restarted.tick(); self.assertEqual((op.parameter, op.value), (RANGE, 2))
        restarted.complete(op); restarted.feed(); restarted.complete(restarted.tick())
        restarted.settle()
        self.assertEqual(restarted.status()["phase"], "DISABLED")
        self.assertTrue(restarted.status()["control_ready"])
        self.assertIsNone(restarted.journal.data)

    def test_threshold_crossing_while_disconnected_restores_on_reconnect(self):
        h = Harness(); h.suspend()
        h.feed(state="DETACH_ROTATE", tilt=85., connected=False)
        self.assertIsNone(h.tick())
        h.feed(tilt=80., connected=False)
        self.assertIsNone(h.tick())
        self.assertTrue(h.policy.restore_intent)
        self.assertEqual(h.journal.data["desired"], "restore")
        h.feed(dt=1., tilt=0., connected=False)
        self.assertIsNone(h.tick())
        h.feed(tilt=0., connected=True)
        op = h.tick()
        self.assertEqual((op.parameter, op.value), (RANGE, 2))

    def test_complete_telemetry_gap_can_reconfirm_low_angle_recovery_entry(self):
        h = Harness(); h.suspend()
        h.feed(state="DETACH_ROTATE", tilt=85.)
        self.assertIsNone(h.tick())
        h.now += 2.; h.mono += 2.
        self.assertIsNone(h.tick())
        self.assertFalse(h.policy.restore_intent)  # stale cached pose never restores
        h.feed(tilt=0., connected=True)
        self.assertEqual(h.tick().parameter, RANGE)

    def test_restart_without_fresh_hold_never_reissues_disable(self):
        h = Harness(); h.suspend()
        restarted = Harness(journal=h.journal, enabled=False)
        restarted.feed(state="SLOW_APPROACH", tilt=90.)
        op = restarted.tick(); self.assertEqual(op.kind, "reconcile"); restarted.complete(op)
        for state in ("SLOW_APPROACH", "WALL_PIN", "FRONT_WALL_DETECT"):
            restarted.feed(state=state, tilt=90.)
            self.assertIsNone(restarted.tick())
        restarted.feed(state="WALL_HOLD", tilt=90.)
        self.assertEqual(restarted.tick().parameter, FLOW)

    def test_full_stack_restart_idle_or_exit_can_restore_only_with_fresh_low_tilt(self):
        for state in ("IDLE", "EXIT"):
            h = Harness(); h.suspend()
            restarted = Harness(journal=h.journal, enabled=False)
            restarted.feed(state=state, tilt=85.)
            restarted.complete(restarted.tick())  # read-only reconciliation
            self.assertIsNone(restarted.tick())
            restarted.feed(tilt=80.)
            self.assertEqual(restarted.tick().parameter, RANGE)

    def test_pending_cross_process_request_waits_ten_seconds_before_reconcile(self):
        h = Harness(); h.start()
        h.feed(state="WALL_HOLD", tilt=90.)
        old_disable = h.tick()  # no response before old process died
        restarted = Harness(journal=h.journal, enabled=False, originals={FLOW: 0, RANGE: 2})
        restarted.feed(state="DETACH_ROTATE", tilt=70.)
        self.assertIsNone(restarted.tick())
        self.assertIn("quarantine", restarted.status()["reason"])
        self.assertEqual(restarted.journal.data["desired"], "restore")
        self.assertFalse(restarted.status()["control_ready"])
        restarted.feed(dt=9.99)
        self.assertIsNone(restarted.tick())
        restarted.feed(dt=.02)
        op = restarted.tick(); self.assertEqual(op.kind, "reconcile")
        restarted.complete(op)
        self.assertEqual(restarted.tick().parameter, RANGE)
        self.assertIsNotNone(old_disable)

    def test_restart_during_quarantine_preserves_pending_and_restore_intent(self):
        h = Harness(); h.start()
        h.feed(state="WALL_HOLD", tilt=90.); h.tick()
        pending = copy.deepcopy(h.journal.data["pending"])
        restarted = Harness(journal=h.journal, enabled=False)
        restarted.feed(state="DETACH_ROTATE", tilt=85.)
        self.assertIsNone(restarted.tick())
        restarted.feed(tilt=80.)
        self.assertIsNone(restarted.tick())
        self.assertEqual(restarted.journal.data["pending"], pending)
        self.assertEqual(restarted.journal.data["desired"], "restore")
        again = Harness(journal=h.journal, enabled=False)
        again.feed(state="DETACH_ROTATE", tilt=0.)
        self.assertIsNone(again.tick())
        again.feed(dt=9.9)
        self.assertIsNone(again.tick())
        again.feed(dt=.11)
        op = again.tick(); self.assertEqual(op.kind, "reconcile"); again.complete(op)
        self.assertEqual(again.tick().parameter, RANGE)

    def test_odom_crossing_is_not_lost_when_next_sample_is_equal(self):
        h = Harness(); h.suspend()
        h.feed(state="DETACH_ROTATE", tilt=85.); self.assertIsNone(h.tick())
        h.feed(dt=.02, tilt=80.)
        h.feed(dt=.02, tilt=80.)
        self.assertEqual(h.tick().parameter, RANGE)

    def test_disabled_without_journal_has_no_service_calls(self):
        h = Harness(enabled=False)
        for state in ("IDLE", "WALL_HOLD", "DETACH_ROTATE"):
            h.feed(state=state); self.assertIsNone(h.tick())
        self.assertEqual(h.status()["phase"], "DISABLED")


class EmergencyFusionTests(unittest.TestCase):
    def test_kill_during_flip_restores_only_disabled_parameters(self):
        h = Harness(originals={FLOW: 0, RANGE: 1})
        h.start()
        h.feed(state="FLIP_TO_WALL", tilt=85.)
        h.policy.observe_kill(True, h.mono)
        op = h.tick()
        self.assertEqual(op.kind, "reconcile")
        h.complete(op)
        op = h.tick()
        self.assertEqual((op.parameter, op.value), (FLOW, 1))
        h.complete(op)
        h.feed(dt=1.3)
        op = h.tick()
        self.assertEqual(op.kind, "verify")
        h.complete(op)
        self.assertEqual(h.values, {FLOW: 1, RANGE: 1})
        self.assertTrue(h.status()["parameters_restored"])
        h.feed(state="WALL_HOLD", tilt=90.)
        self.assertIsNone(h.tick())

    def test_kill_waits_for_dispatched_disable_before_reconciling(self):
        h = Harness(originals={FLOW: 1, RANGE: 1})
        h.start()
        h.feed(state="WALL_HOLD", tilt=90.)
        old = h.tick()
        h.policy.observe_kill(True, h.mono)
        self.assertFalse(h.policy.authorize_operation(old, h.now, h.mono))
        self.assertIsNone(h.tick())
        h.complete(old)  # The write had already reached the FCU.
        h.feed(state="ABORT", connected=False)
        self.assertIsNone(h.tick())
        h.feed(connected=True)
        h.complete(h.tick())  # Fresh reconcile sees the late zero.
        restore = h.tick()
        self.assertEqual((restore.parameter, restore.value), (FLOW, 1))
        h.complete(restore)

    def test_kill_leaves_enabled_values_untouched(self):
        h = Harness(originals={FLOW: 1, RANGE: 1})
        h.policy.observe_kill(True, h.mono)
        h.complete(h.tick())
        self.assertIsNone(h.tick())
        h.feed(dt=1.3)
        op = h.tick()
        self.assertEqual(op.kind, "verify")
        h.complete(op)
        self.assertTrue(h.status()["parameters_restored"])


class RestoreJournalTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = os.path.join(self.directory.name, "sub", "restore.json")
        self.record = {"version": 1, "vehicle_id": "fcu-one", "episode_id": "episode",
                       "desired": "suspend", "originals": {FLOW: 1, RANGE: 2}}

    def test_atomic_round_trip_and_clear(self):
        journal = RestoreJournal(self.path, "fcu-one")
        self.assertIsNone(journal.load()); journal.save(self.record)
        self.assertEqual(journal.load(), self.record)
        self.assertEqual(os.listdir(os.path.dirname(self.path)), ["restore.json"])
        journal.clear(); self.assertIsNone(journal.load())

    def test_identity_mismatch_corrupt_and_missing_originals_fail_closed(self):
        for data in (self.record, [], None, {**self.record, "originals": {FLOW: 1}},
                     {**self.record, "originals": {FLOW: True, RANGE: 2}}):
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with open(self.path, "w") as stream: json.dump(data, stream)
            journal = RestoreJournal(self.path, "different-vehicle")
            policy = FusionPolicy(journal, "different-vehicle", enabled=True)
            self.assertTrue(policy.fatal)
            self.assertIsNone(policy.tick(10., 10.))
            self.assertFalse(policy.status(10., 10.)["control_ready"])


    def test_truncated_json_and_malformed_pending_keep_journal_and_fail_closed(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        for text in ('{"originals":', json.dumps({**self.record, "pending": "unknown"})):
            with open(self.path, "w") as stream: stream.write(text)
            policy = FusionPolicy(RestoreJournal(self.path, "fcu-one"), "fcu-one", enabled=True)
            self.assertTrue(policy.fatal)
            self.assertIsNone(policy.tick(10., 10.))
            self.assertTrue(os.path.exists(self.path))

    def test_quarantine_cannot_be_shorter_than_verified_mavros_bound(self):
        with self.assertRaises(ValueError):
            FusionPolicy(MemoryJournal(), "fcu-one", restart_quarantine_s=5.)


def load_node_without_ros():
    """Test service semantics with mocks, without touching a ROS master/FCU."""
    modules = {name: types.ModuleType(name) for name in (
        "rospy", "mavros_msgs", "mavros_msgs.msg", "mavros_msgs.srv",
        "nav_msgs", "nav_msgs.msg", "std_msgs", "std_msgs.msg", "px4_wall_ceiling_control.msg")}
    modules["mavros_msgs.msg"].ParamValue = lambda **kw: types.SimpleNamespace(**kw)
    modules["mavros_msgs.msg"].State = object
    modules["std_msgs.msg"].String = object
    for name in ("ParamGet", "ParamPull", "ParamSet"):
        setattr(modules["mavros_msgs.srv"], name, object)
    modules["nav_msgs.msg"].Odometry = object
    for name in ("WallPerchStatus", "WallSensorFusionStatus"):
        setattr(modules["px4_wall_ceiling_control.msg"], name, object)
    modules["rospy"].wait_for_service = mock.Mock()
    path = os.path.join(os.path.dirname(__file__), "..", "scripts", "sensors", "wall_sensor_fusion_manager_node.py")
    spec = importlib.util.spec_from_file_location("isolated_wall_fusion_node", path)
    node = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, modules):
        spec.loader.exec_module(node)
    return node


class WorkerAndTransportTests(unittest.TestCase):
    def setUp(self):
        self.node = load_node_without_ros()
        self.transport = self.node.MavrosParameterTransport.__new__(self.node.MavrosParameterTransport)
        self.transport._exchange = mock.Mock(side_effect=lambda parameter, expected=None:
            expected if expected is not None else {FLOW: 1, RANGE: 2}[parameter])

    def test_snapshot_and_verify_use_two_fresh_named_reads(self):
        for kind in ("snapshot", "verify", "reconcile"):
            self.transport._exchange.reset_mock()
            self.assertEqual(self.transport.execute(Operation(1, kind, 0, 10., 10.)), {FLOW: 1, RANGE: 2})
            self.assertEqual(self.transport._exchange.call_args_list, [mock.call(FLOW), mock.call(RANGE)])

    def test_named_read_timeout_and_wrong_set_ack_are_not_success(self):
        self.transport._exchange.side_effect = RuntimeError('timeout')
        with self.assertRaises(RuntimeError):
            self.transport.execute(Operation(1, 'snapshot', 0, 10., 10.))
        self.transport._exchange.side_effect = None
        self.transport._exchange.return_value = 1
        with self.assertRaises(ValueError):
            self.transport.execute(Operation(2, 'set', 1, 10., 10., FLOW, 0))

    def test_blocked_service_keeps_status_thread_free_and_has_only_one_worker(self):
        entered, release = threading.Event(), threading.Event()
        calls = []
        def execute(operation):
            calls.append(operation)
            entered.set()
            release.wait(timeout=2.)
            return {FLOW: 0}
        worker = self.node.SerialParameterWorker(execute, lambda: 10.)
        self.addCleanup(worker.close)
        self.addCleanup(release.set)
        operation = Operation(1, "set", 1, 10., 10., FLOW, 0)
        worker.submit(operation)
        self.assertTrue(entered.wait(timeout=1.))
        self.assertTrue(worker.results.empty())
        # Caller remains free to tick/publish while the service is unresolved.
        h = Harness(); h.suspend()
        for _ in range(20):
            h.feed(); h.tick(); self.assertEqual(h.status()["phase"], "SUSPENDED")
        self.assertEqual(calls, [operation])
        release.set()
        result = worker.results.get(timeout=1.)
        self.assertEqual(result[:3], (operation, True, {FLOW: 0}))


if __name__ == "__main__":
    unittest.main()
