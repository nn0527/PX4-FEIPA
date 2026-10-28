"""Stage faults and unavailable FCU links; all publishers/services are mocked."""
import struct
import unittest
from unittest import mock
import rospy
from mavros_msgs.msg import Mavlink
from px4_wall_ceiling_control.msg import CeilingAttachmentStatus as S, OperatorCommand as Op
from px4_wall_ceiling_control.adapters.ceiling_failsafe import decode_failsafe_parameter
import test_ceiling_motion as fixtures
import test_ceiling_hold_abort as landing


class FaultLandingTests(unittest.TestCase):
    def setUp(self):
        self.fx = fixtures.CeilingMotionLifecycleTests()
        self.fx.extra_params = {'~abort_land_on_fault': True}
        self.fx.setUp(); self.addCleanup(self.fx.doCleanups)
        self.fx.hold()
        self.n = self.fx.n

    def assert_landing(self):
        self.assertEqual(self.n.state, S.STATE_ABORT)
        self.assertTrue(self.n._landing_abort())
        self.assertTrue(self.n.abort_maneuver_id)

    def test_contact_fault_outranks_simultaneous_detach(self):
        self.n.detach_requested = True
        self.n._tilt_fault = lambda now: True
        status = self.fx.feed(distance=.055)
        self.assert_landing()
        self.assertTrue(status.abort_land_requested)
        self.assertFalse(status.hold_timed_out)

    def test_detach_and_recovery_abort_always_land(self):
        for state in (S.STATE_DETACH, S.STATE_RECOVERY_HOVER):
            with self.subTest(state=state):
                self.n.state = state
                self.n.flight_state.connected = False
                self.n._update_state_machine(rospy.Time.now())
                self.assert_landing()

    def test_any_abort_from_descent_or_recovery_latches_landing(self):
        for state in (S.STATE_DETACH, S.STATE_RECOVERY_HOVER):
            self.n.state = state
            self.n._enter_state(S.STATE_ABORT, rospy.Time.now(), 'stage timeout')
            self.assert_landing()
            identity = self.n.abort_maneuver_id
            self.n.detach_requested = True
            self.n._update_state_machine(rospy.Time.now())
            self.assertEqual(identity, self.n.abort_maneuver_id)

    def test_usb_disconnect_aborts_during_entry_and_ascent(self):
        for state, phase in ((S.STATE_NORMAL_FLIGHT, 'ENTERING'), (S.STATE_APPROACH, 'ACTIVE')):
            self.n.state, self.n.motion_manager_phase = state, phase
            self.n.flight_state.connected = False
            self.n._update_state_machine(rospy.Time.now())
            self.assert_landing()

    def test_stale_heartbeat_aborts_without_waiting_motion_timeout(self):
        self.n.state = S.STATE_APPROACH
        self.n.flight_state_received_at = rospy.Time.from_sec(self.fx.clock-2.)
        self.n._update_state_machine(rospy.Time.now())
        self.assert_landing()

    def packet(self, ratio=.4, stamp=1000, flags=1):
        raw = struct.pack('<QffffffffH', stamp, .1, .1, .1, ratio, .1, .1, .1, .1, flags)
        msg = Mavlink(msgid=230, sysid=1, compid=1, framing_status=1, len=len(raw))
        msg.header.stamp = rospy.Time.now()
        msg.payload64 = list(struct.unpack('<6Q', raw.ljust(48, b'\0')))
        return msg

    def test_first_bad_mag_report_aborts_and_good_report_cannot_cancel(self):
        self.n.ceiling_ekf_guard_enabled = True
        self.n.state = S.STATE_APPROACH
        self.n._on_ceiling_ekf_safety(self.packet(1.01))
        self.n._update_state_machine(rospy.Time.now())
        self.assert_landing()
        self.assertIn('磁', self.n.last_reason)
        identity = self.n.abort_maneuver_id
        self.n._on_ceiling_ekf_safety(self.packet(.2, 2000))
        self.n._update_state_machine(rospy.Time.now())
        self.assertEqual(identity, self.n.abort_maneuver_id)

    def test_bad_attitude_and_nan_mag_abort(self):
        for ratio, flags in ((.2, 0), (float('nan'), 1), (.2, 2049)):
            self.n.state = S.STATE_APPROACH
            self.n.ceiling_ekf_stamp = None
            self.n.ceiling_ekf_guard_enabled = True
            self.n._on_ceiling_ekf_safety(self.packet(ratio, flags=flags))
            self.n._update_state_machine(rospy.Time.now())
            self.assert_landing()

    def test_disabled_magnetic_ratio_reports_warning_without_aborting(self):
        self.n.ceiling_ekf_guard_enabled = True
        self.n.ceiling_mag_ratio_abort_enabled = False
        self.n.state = S.STATE_APPROACH
        with mock.patch.object(rospy, 'logwarn_throttle') as warning:
            for i, ratio in enumerate((1.27, -0.1, float('nan')), start=1):
                self.n._on_ceiling_ekf_safety(self.packet(ratio, stamp=1000*i))
                self.assertEqual(self.n.ceiling_ekf_fault, '')
                self.assertEqual(self.n.motion_fatal, '')
                self.assertEqual(self.n._ceiling_safety_fault(rospy.Time.now()), '')
            self.assertEqual(warning.call_count, 3)

    def test_disabled_magnetic_abort_still_aborts_on_attitude_and_accel_faults(self):
        for flags in (0, 2049):
            self.n.ceiling_ekf_guard_enabled = True
            self.n.ceiling_mag_ratio_abort_enabled = False
            self.n.ceiling_ekf_stamp = None
            self.n.state = S.STATE_APPROACH
            self.n._on_ceiling_ekf_safety(self.packet(1.27, flags=flags))
            self.assertIn('姿态无效或加速度异常', self.n.ceiling_ekf_fault)
            self.n._update_state_machine(rospy.Time.now())
            self.assert_landing()

    def test_disabled_magnetic_abort_does_not_disable_ekf_heartbeat_timeout(self):
        self.n.ceiling_ekf_guard_enabled = True
        self.n.ceiling_mag_ratio_abort_enabled = False
        self.n.state = S.STATE_APPROACH
        self.n.ceiling_ekf_at = None
        self.n._update_state_machine(rospy.Time.now())
        self.assert_landing()
        self.assertIn('反馈超时', self.n.last_reason)

    def test_wrong_sender_stale_future_and_replay_do_not_refresh_monitor(self):
        self.n._on_ceiling_ekf_safety(self.packet())
        initial = self.n.ceiling_ekf_at
        self.fx.clock += .1
        packets = []
        msg = self.packet(2., 2000); msg.sysid = 2; packets.append(msg)
        msg = self.packet(2., 2000); msg.header.stamp = rospy.Time.from_sec(self.fx.clock-3.); packets.append(msg)
        msg = self.packet(2., 2000); msg.header.stamp = rospy.Time.from_sec(self.fx.clock+1.); packets.append(msg)
        packets.append(self.packet(2., 1000))
        for msg in packets:
            self.n._on_ceiling_ekf_safety(msg)
        self.assertEqual(initial, self.n.ceiling_ekf_at)
        self.assertFalse(self.n.ceiling_ekf_fault)

    def test_missing_mag_telemetry_fails_closed(self):
        self.n.ceiling_ekf_guard_enabled = True
        self.n.state = S.STATE_APPROACH
        self.n._update_state_machine(rospy.Time.now())
        self.assert_landing()
        self.assertIn('反馈超时', self.n.last_reason)


class DisconnectedLandingManagerTests(unittest.TestCase):
    def setUp(self):
        self.fx = landing.CeilingLandingManagerTests()
        self.fx.setUp(); self.addCleanup(self.fx.doCleanups)
        self.n = self.fx.n
        self.now = self.fx.now

    def test_disconnect_does_not_retire_intent_or_send_command(self):
        n = self.n
        n.flight.connected = False; n.flight.armed = False
        n._tick_ceiling_abort_land(self.now)
        self.assertEqual(n.phase, 'CEILING_LANDING')
        n._request_mode.assert_not_called(); n._reset.assert_not_called()
        n._send_mode_request('AUTO.LAND', attempt=3)
        n.set_mode.assert_not_called()
        n.flight.connected = True; n.flight.armed = True
        n._tick_ceiling_abort_land(self.now)
        n._request_mode.assert_called_once_with('AUTO.LAND')

    def test_px4_native_land_on_reconnect_needs_fresh_confirmation(self):
        n = self.n
        n.flight.connected = False
        n._tick_ceiling_abort_land(self.now)
        n.flight.connected = True; n.flight.mode = 'AUTO.LAND'
        n.flight.header.stamp = n.flight_at = rospy.Time.from_sec(10.2)
        n._tick_ceiling_abort_land(rospy.Time.from_sec(10.2))
        self.assertTrue(n.ceiling_land_confirmed)
        n._request_mode.assert_not_called()

    def test_entry_abort_can_request_land_from_initial_posctl(self):
        n = self.n
        n.phase = n.ENTERING; n.flight.mode = 'POSCTL'
        n._tick_ceiling_abort_land(self.now)
        n._request_mode.assert_called_once_with('AUTO.LAND')
        n._send_mode_request('AUTO.LAND', attempt=3)
        n.set_mode.assert_called_once()

    def test_entry_exception_ends_after_offboard_is_observed(self):
        n = self.n
        n.phase = n.ENTERING; n.flight.mode = 'POSCTL'
        n._tick_ceiling_abort_land(self.now)
        n.flight.mode = 'OFFBOARD'
        n._tick_ceiling_abort_land(self.now)
        n._request_mode.reset_mock()
        n.flight.mode = 'POSCTL'
        n._tick_ceiling_abort_land(self.now)
        n._request_mode.assert_not_called(); n._reset.assert_called_once()

    def test_unverified_or_stale_failsafe_blocks_ceiling_only(self):
        n = self.n
        n.ceiling_failsafe_guard_required = True
        self.assertIn('尚未核验', n._preflight_failure_reason(self.now, Op.MODE_CEILING))
        n.ceiling_failsafe_ready = True; n.ceiling_failsafe_at = rospy.Time.from_sec(8.)
        self.assertIn('尚未核验', n._preflight_failure_reason(self.now, Op.MODE_CEILING))

    def test_warning_only_unverified_audit_does_not_hide_other_preflight_failures(self):
        n = self.n
        n.ceiling_failsafe_guard_required = True
        n.ceiling_failsafe_unverified_warning_only = True
        n.ceiling_failsafe_ready = False
        n._incline_intent = lambda: False
        n.enabled = False
        with mock.patch.object(rospy, 'logwarn_throttle') as warning:
            self.assertEqual(n._preflight_failure_reason(self.now, Op.MODE_CEILING), 'flight mode handover disabled')
            warning.assert_called_once()
        self.assertFalse(n.ceiling_failsafe_ready)

    def test_warning_only_stale_audit_still_rejects_disconnected_fcu(self):
        n = self.n
        n.ceiling_failsafe_guard_required = True
        n.ceiling_failsafe_unverified_warning_only = True
        n.ceiling_failsafe_ready = True
        n.ceiling_failsafe_at = self.now-rospy.Duration(2.)
        n._incline_intent = lambda: False
        n.enabled = True
        n.flight.connected = False
        n.flight_at = self.now
        n.state_timeout_s = 1.5
        with mock.patch.object(rospy, 'logwarn_throttle'):
            self.assertEqual(n._preflight_failure_reason(self.now, Op.MODE_CEILING), 'MAVROS disconnected')


class FailsafeWireTests(unittest.TestCase):
    def packet(self, name, value, kind):
        raw = struct.pack('<i' if kind == 6 else '<f', value) + struct.pack('<HH', 20, 1) + name.encode().ljust(16,b'\0') + bytes([kind])
        msg = Mavlink(msgid=22, sysid=1, compid=1, framing_status=1, len=25)
        msg.payload64 = list(struct.unpack('<4Q', raw.ljust(32,b'\0')))
        return msg

    def test_int_and_float_values_decode_only_expected_names_types_and_sender(self):
        for name, value, kind in (('COM_OBL_RC_ACT',4,6), ('COM_OF_LOSS_T',.2,9), ('COM_OBL_ACT',0,6)):
            msg = self.packet(name,value,kind)
            actual = decode_failsafe_parameter(msg,1,1)
            self.assertEqual(actual[0],name); self.assertAlmostEqual(actual[1],value)
            msg.compid = 2
            self.assertIsNone(decode_failsafe_parameter(msg,1,1))
        self.assertIsNone(decode_failsafe_parameter(self.packet('COM_OBL_RC_ACT',4.,9),1,1))
        self.assertIsNone(decode_failsafe_parameter(self.packet('COM_OF_LOSS_T',float('nan'),9),1,1))


class GroundFailsafeGuardTests(unittest.TestCase):
    def setUp(self):
        import importlib.util
        from pathlib import Path
        import threading
        path = Path(__file__).resolve().parents[1]/'scripts/operator/ceiling_failsafe_guard_node.py'
        spec = importlib.util.spec_from_file_location('guard_test_module', str(path))
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        self.n = module.CeilingFailsafeGuard.__new__(module.CeilingFailsafeGuard)
        n = self.n
        clock = mock.patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(10.))
        clock.start(); self.addCleanup(clock.stop)
        n.lock = threading.RLock(); n.epoch = 1; n.verified = False
        n.state = mock.Mock(armed=False)
        n.connected = mock.Mock(return_value=True)
        n.transport = mock.Mock()
        n.modern_px4 = mock.Mock(return_value=True)
        n.transport._exchange.side_effect = lambda name: {'COM_OBL_RC_ACT':4, 'COM_OF_LOSS_T':2.0, 'COM_OBL_ACT':0}[name]

    def run_once(self):
        with mock.patch.object(rospy, 'is_shutdown', side_effect=[False, True]), mock.patch.object(rospy, 'Rate'), mock.patch.object(rospy, 'logwarn_throttle'):
            self.n.audit_loop()

    def test_ground_setup_readback_required_for_ready(self):
        n = self.n
        values = {'COM_OBL_RC_ACT':0, 'COM_OF_LOSS_T':1., 'COM_OBL_ACT':1}
        n.transport._exchange.side_effect = lambda name: values[name]
        n.transport._publish.side_effect = lambda name,value: values.update({name:value})
        self.run_once()
        self.assertTrue(n.verified)
        self.assertEqual(n.transport._publish.call_count,3)
        self.assertEqual(n.transport._exchange.call_count,6)

    def test_warning_only_status_keeps_failed_audit_unverified(self):
        import json
        n = self.n
        n.unverified_warning_only = True
        n.transport._exchange.side_effect = RuntimeError('readback timeout')
        self.run_once()
        n.ready = mock.Mock(); n.detail = mock.Mock(); n.status = mock.Mock()
        n.publish(None)
        payload = json.loads(n.status.publish.call_args.args[0].data)
        self.assertTrue(payload['unverified_warning_only'])
        self.assertFalse(payload['verified'])
        self.assertFalse(n.ready.publish.call_args.args[0].data)

    def test_armed_or_disconnected_never_writes(self):
        for armed, connected in ((True,True),(False,False)):
            n = self.n; n.state.armed = armed; n.connected.return_value = connected
            self.run_once()
        n.transport._exchange.assert_not_called(); n.transport._publish.assert_not_called()
        self.assertFalse(n.verified)

    def test_failed_readback_blocks_acquisition(self):
        self.n.transport._exchange.return_value = 0
        self.n.transport._exchange.side_effect = None
        self.run_once()
        self.assertFalse(self.n.verified)
        self.assertIn('mismatch', self.n.reason)

    def test_unknown_or_old_firmware_cannot_skip_missing_legacy_parameter(self):
        n = self.n
        n.modern_px4.return_value = False
        n.transport._exchange.side_effect = lambda name: (_ for _ in ()).throw(RuntimeError('timeout')) if name=='COM_OBL_ACT' else {'COM_OBL_RC_ACT':4, 'COM_OF_LOSS_T':2.0}[name]
        self.run_once(); self.assertFalse(n.verified)
        n.modern_px4.return_value = True
        self.run_once(); self.assertTrue(n.verified)

    def test_arm_during_parameter_reads_prevents_writes(self):
        n = self.n
        def read(name):
            n.state.armed = True
            return 0
        n.transport._exchange.side_effect = read
        self.run_once()
        n.transport._publish.assert_not_called()
        self.assertFalse(n.verified)
