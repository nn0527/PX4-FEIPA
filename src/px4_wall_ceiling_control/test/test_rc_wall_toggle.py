#!/usr/bin/env python3
"""CH12 edge handling and mixed web/RC operator cycles; no flight hardware needed."""
import importlib.util
import json
import pathlib
import threading
import unittest
from unittest import mock

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import RCIn, State
from px4_wall_ceiling_control.msg import OperatorCommand as Op, WallPerchStatus as Wall, CeilingAttachmentStatus as Ceiling
from px4_wall_ceiling_control.domain.momentary_switch import MomentarySwitch

path = pathlib.Path(__file__).resolve().parents[1] / 'scripts/operator/terminal_operator_node.py'
spec = importlib.util.spec_from_file_location('ch12_terminal', str(path))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
Terminal = module.TerminalOperatorNode


class SwitchTest(unittest.TestCase):
    def test_startup_high_and_reconnect_require_release(self):
        s = MomentarySwitch()
        for t in [0., .1, .2]: self.assertFalse(s.update(1999, t))
        s.update(999, .3); self.assertFalse(s.update(999, .4))
        s.update(1999, .5); self.assertTrue(s.update(1999, .6))
        self.assertFalse(s.update(1999, .7))
        self.assertFalse(s.update(1999, 2.))
        self.assertFalse(s.update(1999, 2.1))

    def test_bounce_missing_and_backwards_time(self):
        s = MomentarySwitch()
        s.update(999, 0.); s.update(999, .1)
        s.update(1999, .2); s.update(999, .21)
        self.assertFalse(s.update(1999, .22))
        self.assertTrue(s.update(1999, .3))
        s.update(999, .4); s.update(999, .5)
        self.assertFalse(s.update(None, .6))
        s.update(1999, .7); self.assertFalse(s.update(1999, .8))
        s.update(999, 1.); s.update(999, 1.1)
        self.assertFalse(s.update(1999, .1))
        self.assertFalse(s.update(1999, .2))


class MixedOperatorTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.clock = mock.patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(self.now))
        self.clock.start(); self.addCleanup(self.clock.stop)
        self.params = mock.patch.object(rospy, 'get_param', side_effect=lambda name, default=None: default)
        self.get_param = self.params.start(); self.addCleanup(self.params.stop)
        n = self.node = Terminal.__new__(Terminal)
        n.intent_lock = threading.RLock()
        n.pilot_handover_request_time = rospy.Time(0)
        n.mode = Op.MODE_STANDBY
        n.detaching = n.detach_pulse = n.was_offboard = False
        n.front_sensor_masked = False
        n.flight_mode = 'ALTCTL'
        n.flight_connected = n.flight_armed = True
        n.flight_at = rospy.Time.now()
        n.kill_guard_enabled = False
        n.rc_wall_toggle_enabled = True
        n.rc_wall_channel_index = 11
        n.wall_switch = MomentarySwitch()
        n.rc_selected_mode = "wall"
        n.request_acks = {}
        n.selected_mode_pub = mock.Mock()
        n.command_ack_pub = mock.Mock()
        for attr in ['command_pub', 'ceiling_detach_pub', 'wall_detach_pub']:
            setattr(n, attr, mock.Mock())
        self.status(Wall.STATE_IDLE)

    def status(self, state, pending=False):
        w = Wall(state=state, detach_pending=pending, odom_fresh=True)
        w.header.stamp = rospy.Time.now()
        self.node._on_wall_status(w)

    def press(self):
        n = self.node
        for pwm in [999, 999, 1999, 1999]:
            with mock.patch.object(module.time, 'monotonic', return_value=self.now):
                n._on_rc(RCIn(channels=[1500]*11+[pwm]))
            self.now += .1

    def test_rc_start_web_detach_and_next_cycle(self):
        n = self.node
        self.press(); self.assertEqual(n.mode, Op.MODE_WALL)
        n._on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        self.status(Wall.STATE_WALL_HOLD)
        n._on_command(String(data='detach')); n._on_timer(None)
        n.wall_detach_pub.publish.assert_called_once()
        self.status(Wall.STATE_DETACH_ROTATE)
        self.press(); self.assertEqual(n.wall_detach_pub.publish.call_count, 1)
        n._on_state(State(connected=True, armed=True, mode='ALTCTL'))
        self.status(Wall.STATE_IDLE)
        self.press(); self.assertEqual(n.mode, Op.MODE_WALL)

    def test_web_start_rc_detach_and_retry_rejected_request(self):
        n = self.node
        n._on_command(String(data='wall'))
        n._on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        self.status(Wall.STATE_SLOW_APPROACH)
        self.press(); self.assertFalse(n.detach_pulse)
        self.status(Wall.STATE_WALL_HOLD)
        n.flight_at = rospy.Time.now()
        self.press(); self.assertTrue(n.detach_pulse)
        n._on_timer(None); n.wall_detach_pub.publish.assert_called_once()
        # Decision rejects the request, leaving HOLD and no pending request.
        self.status(Wall.STATE_WALL_HOLD)
        n.flight_at = rospy.Time.now()
        self.press(); n._on_timer(None)
        self.assertEqual(n.wall_detach_pub.publish.call_count, 2)

    def test_stale_disarmed_kill_and_other_action_block(self):
        n = self.node
        self.now += 1.; n._on_wall_toggle(); self.assertEqual(n.mode, Op.MODE_STANDBY)
        self.status(Wall.STATE_IDLE); n.flight_at = rospy.Time.now()
        n.flight_armed = False; n._on_wall_toggle(); self.assertEqual(n.mode, Op.MODE_STANDBY)
        n.flight_armed = True; n.kill_guard_enabled = True
        n._kill_report = lambda: {'valid':True,'active':True}
        n._on_wall_toggle(); self.assertEqual(n.mode, Op.MODE_STANDBY)
        n.kill_guard_enabled = False; n.mode = Op.MODE_CEILING
        n._on_wall_toggle(); self.assertEqual(n.mode, Op.MODE_CEILING)

    def test_pending_request_blocks_rc_duplicate(self):
        n = self.node
        n.mode = Op.MODE_WALL
        n._on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        self.status(Wall.STATE_WALL_HOLD, pending=True)
        self.press(); self.assertFalse(n.detach_pulse)

    def posctl(self):
        self.get_param.side_effect = lambda name, default=None: (
            'POSCTL' if name == '/flight_mode_manager_node/wall_pilot_mode' else default)
        self.node._on_state(State(connected=True, armed=True, mode='POSCTL'))

    def test_recorded_posctl_idle_fault_does_not_block_rc_start(self):
        # 2026-10-25: all three CH12 events had these fields. The decision
        # reports fault_detected in POSCTL because require_offboard is true.
        self.posctl()
        n = self.node
        n.wall_status.fault_detected = True
        n.wall_status.flight_state_fresh = False
        n.kill_guard_enabled = True
        n._kill_report = lambda: {'valid': True, 'active': False}
        self.press()
        n._publish_intent()
        intent = n.command_pub.publish.call_args.args[0]
        self.assertEqual(intent.mode, Op.MODE_WALL)
        self.assertTrue(intent.enable_control)

    def test_posctl_rc_cycle_releases_only_after_actual_handover(self):
        self.posctl()
        n = self.node
        n.wall_status.fault_detected = True
        self.press()
        self.assertEqual(n.mode, Op.MODE_WALL)
        n._on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        for state in (Wall.STATE_SLOW_APPROACH, Wall.STATE_FLIP_TO_WALL,
                      Wall.STATE_WALL_CAPTURE):
            self.status(state)
            n.flight_at = rospy.Time.now()
            self.press()
            self.assertFalse(n.detach_pulse)
        self.status(Wall.STATE_WALL_HOLD)
        n.flight_at = rospy.Time.now()
        self.press()
        n._on_timer(None)
        n.wall_detach_pub.publish.assert_called_once()
        self.assertTrue(n.detaching)
        for state in (Wall.STATE_DETACH_ROTATE, Wall.STATE_RECOVER, Wall.STATE_EXIT):
            self.status(state)
            n.flight_at = rospy.Time.now()
            self.press()
            n._on_timer(None)
            self.assertEqual(n.mode, Op.MODE_WALL)
            self.assertEqual(n.wall_detach_pub.publish.call_count, 1)
        # EXIT/recovery alone does not clear the operator request.
        n._on_state(State(connected=True, armed=True, mode='POSCTL'))
        self.assertEqual(n.mode, Op.MODE_STANDBY)
        self.assertFalse(n.detaching)
        self.status(Wall.STATE_IDLE)
        n.wall_status.fault_detected = True
        self.press()
        self.assertEqual(n.mode, Op.MODE_WALL)

    def test_idle_still_requires_odometry_and_configured_pilot_mode(self):
        self.posctl()
        n = self.node
        n.wall_status.fault_detected = True
        n.wall_status.odom_fresh = False
        n._on_wall_toggle()
        self.assertEqual(n.mode, Op.MODE_STANDBY)
        n.wall_status.odom_fresh = True
        n.flight_mode = 'ALTCTL'
        n._on_wall_toggle()
        self.assertEqual(n.mode, Op.MODE_STANDBY)
        n.flight_mode = 'POSCTL'
        n._on_wall_toggle()
        self.assertEqual(n.mode, Op.MODE_WALL)

    def test_hold_fault_still_blocks_detach(self):
        n = self.node
        n.mode = Op.MODE_WALL
        n._on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        self.status(Wall.STATE_WALL_HOLD)
        n.wall_status.fault_detected = True
        self.press()
        self.assertFalse(n.detaching)
        self.assertFalse(n.detach_pulse)

    def test_invalid_kill_disconnected_and_stale_flight_still_block_start(self):
        n = self.node
        n.wall_status.fault_detected = True
        n.flight_connected = False
        n._on_wall_toggle()
        self.assertEqual(n.mode, Op.MODE_STANDBY)
        n.flight_connected = True
        n.flight_at = rospy.Time.from_sec(self.now - 2.)
        n._on_wall_toggle()
        self.assertEqual(n.mode, Op.MODE_STANDBY)
        n.flight_at = rospy.Time.now()
        n.wall_status.header.stamp = rospy.Time.from_sec(self.now + 1.)
        n._on_wall_toggle()
        self.assertEqual(n.mode, Op.MODE_STANDBY)
        self.status(Wall.STATE_IDLE)
        n.kill_guard_enabled = True
        n._kill_report = lambda: {'valid': False, 'active': False}
        n._on_wall_toggle()
        self.assertEqual(n.mode, Op.MODE_STANDBY)


class ModeSelectionTest(unittest.TestCase):
    def setUp(self):
        self.fx = MixedOperatorTest()
        self.fx.setUp()
        self.addCleanup(self.fx.doCleanups)
        self.fx.posctl()
        self.n = n = self.fx.node
        n.rc_selected_mode = None
        n.kill_pub = mock.Mock()
        n.kill_guard_enabled = True
        n._kill_report = lambda: {'valid': True, 'active': False}
        self.ceiling(Ceiling.STATE_NORMAL_FLIGHT)

    def ceiling(self, state, **kwargs):
        m = Ceiling(state=state, odom_fresh=True, **kwargs)
        m.header.stamp = rospy.Time.from_sec(self.fx.now)
        self.n._on_ceiling_status(m)

    def select(self, mode):
        data = {"id": str(self.fx.now) + mode, "command": "select_rc_mode", "mode": mode}
        with mock.patch.object(rospy, "set_param"):
            self.n._on_command(String(data=json.dumps(data)))
        return json.loads(self.n.command_ack_pub.publish.call_args.args[0].data)

    def frame(self, ch8, ch12=999, refresh=True):
        n = self.n
        if refresh:
            n.flight_at = rospy.Time.from_sec(self.fx.now)
            for m in filter(None, (n.wall_status, n.ceiling_status)):
                m.header.stamp = rospy.Time.from_sec(self.fx.now)
            n.wall_status_at = n.ceiling_status_at = rospy.Time.from_sec(self.fx.now)
        channels = [1500]*12
        channels[7], channels[11] = ch8, ch12
        with mock.patch.object(module.time, 'monotonic', return_value=self.fx.now):
            n._on_rc(RCIn(channels=channels))
        self.fx.now += .1

    def press(self, ch8):
        for ch12 in (999, 999, 1999, 1999):
            self.frame(ch8, ch12)

    def test_ch8_alone_never_starts_and_ch12_uses_selection(self):
        self.press(1999)
        self.assertEqual(self.n.mode, Op.MODE_STANDBY)
        self.assertTrue(self.select("ceiling")["accepted"])
        self.assertEqual(self.n.mode, Op.MODE_STANDBY)
        self.assertEqual(self.n.selected_mode_pub.publish.call_args.args[0].data, "ceiling")
        with mock.patch.object(rospy, "set_param"):
            self.press(1999)
        self.assertEqual(self.n.mode, Op.MODE_CEILING)
        self.assertTrue(self.n.front_sensor_masked)

    def test_incline_selection_requires_ready_and_routes_to_incline(self):
        self.assertTrue(self.select("incline")["accepted"])
        self.n._apply_command = mock.Mock()
        self.press(999)
        self.n._apply_command.assert_not_called()
        self.n.wall_status.incline_ready = True
        self.press(1999)
        self.assertEqual(self.n._apply_command.call_args.args[0].data, "incline")

    def test_ceiling_cycle_rejects_selection_during_active_mode(self):
        n = self.n
        self.select("ceiling")
        with mock.patch.object(rospy, "set_param"):
            self.press(999)
        n._on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        self.assertFalse(self.select("wall")["accepted"])
        self.ceiling(Ceiling.STATE_APPROACH)
        self.press(1999)
        self.assertEqual(n.mode, Op.MODE_CEILING)
        self.assertFalse(n.detach_pulse)
        self.ceiling(Ceiling.STATE_SURFACE_HOLD)
        self.press(1999)
        n._on_timer(None)
        n.ceiling_detach_pub.publish.assert_called_once()
        n.wall_detach_pub.publish.assert_not_called()
        self.ceiling(Ceiling.STATE_DETACH)
        self.press(1999); n._on_timer(None)
        self.assertEqual(n.ceiling_detach_pub.publish.call_count, 1)
        self.ceiling(Ceiling.STATE_RECOVERY_HOVER)
        with mock.patch.object(rospy, "set_param"):
            n._on_state(State(connected=True, armed=True, mode='POSCTL'))
        self.ceiling(Ceiling.STATE_NORMAL_FLIGHT)
        self.assertTrue(self.select("wall")["accepted"])
        self.press(1999)
        self.assertEqual(n.mode, Op.MODE_WALL)

    def test_wall_detach_remains_bound_to_active_mode(self):
        n = self.n
        self.select("wall")
        self.press(1999)
        n._on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        self.assertFalse(self.select("ceiling")["accepted"])
        self.fx.status(Wall.STATE_WALL_HOLD)
        self.press(999); n._on_timer(None)
        n.wall_detach_pub.publish.assert_called_once()
        n.ceiling_detach_pub.publish.assert_not_called()
        self.assertEqual(n.mode, Op.MODE_WALL)

    def test_stale_ceiling_does_not_add_a_wall_start_gate(self):
        self.n.ceiling_status = None
        self.n.ceiling_status_at = None
        self.select("wall")
        self.press(1999)
        self.assertEqual(self.n.mode, Op.MODE_WALL)

    def test_unsafe_inputs_and_stale_status_still_block(self):
        n = self.n
        self.select("ceiling")
        self.frame(999); self.frame(999)
        for fault in ('kill', 'disarmed', 'disconnected', 'ceiling_stale', 'odom', 'wrong_mode'):
            with self.subTest(fault=fault):
                n._kill_report = lambda: {'valid': True, 'active': fault == 'kill'}
                n.flight_armed = fault != 'disarmed'
                n.flight_connected = fault != 'disconnected'
                n.flight_mode = 'ALTCTL' if fault == 'wrong_mode' else 'POSCTL'
                n.ceiling_status.odom_fresh = fault != 'odom'
                n.ceiling_status_at = rospy.Time.from_sec(self.fx.now - (1. if fault == 'ceiling_stale' else 0.))
                n.flight_at = n.rc_at = rospy.Time.from_sec(self.fx.now)
                n._on_wall_toggle()
                self.assertEqual(n.mode, Op.MODE_STANDBY)

    def test_ceiling_fault_or_stale_hold_cannot_detach(self):
        n = self.n
        self.select("ceiling")
        with mock.patch.object(rospy, "set_param"):
            self.press(999)
        n._on_state(State(connected=True, armed=True, mode='OFFBOARD'))
        self.assertFalse(self.select("wall")["accepted"])
        self.ceiling(Ceiling.STATE_SURFACE_HOLD, fault_detected=True)
        self.press(999)
        self.assertFalse(n.detaching)
        n.ceiling_status.fault_detected = False
        n.ceiling_status.header.stamp = rospy.Time.from_sec(self.fx.now - 1.)
        n._on_wall_toggle()
        self.assertFalse(n.detaching)

    def test_startup_high_and_reconnect_high_never_start(self):
        self.select("ceiling")
        for _ in range(4): self.frame(999, 1999)
        self.assertEqual(self.n.mode, Op.MODE_STANDBY)
        self.frame(999); self.frame(999)
        self.fx.now += 1.
        self.frame(999, 1999); self.frame(999, 1999)
        self.assertEqual(self.n.mode, Op.MODE_STANDBY)
        with mock.patch.object(rospy, "set_param"):
            self.press(999)
        self.assertEqual(self.n.mode, Op.MODE_CEILING)


if __name__ == '__main__':
    unittest.main()
