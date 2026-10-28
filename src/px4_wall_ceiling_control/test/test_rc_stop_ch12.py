"""Disarmed CH12 Stop shares the existing counted switch, never armed tasks."""
import copy
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from px4_wall_ceiling_control.rc_startup import RcStartup, StickGesture, safe_stop_snapshot
from px4_wall_ceiling_control.domain.momentary_switch import MomentarySwitch


def snapshot(pwm=999):
    return {'starting': False, 'launch_error': '', 'status': {
        'state': {'fresh': True, 'value': {'connected': True, 'armed': False, 'mode': 'POSCTL'}},
        'rc': {'fresh': True, 'value': {'channels': [1500,1500,999,1500,999,999,999,1999,1500,1999,1500,pwm]}},
        'runtime': {'fresh': True, 'value': {'enabled': False}}}}


class Ch12StopTests(unittest.TestCase):
    def controller(self):
        c = object.__new__(RcStartup)
        c.runtime = Mock(stopping=False, repairing=False, stop_requested=False)
        c.data = snapshot()
        c.runtime.state.snapshot.side_effect = lambda: copy.deepcopy(c.data)
        c.runtime.stop.return_value = {'message': '基础链路已停止，遥控启动待命'}
        c.broker = SimpleNamespace(lock=threading.RLock(), owner='page-owner')
        c.stop_ch12_enabled = True
        c.stop_switch = MomentarySwitch()
        c.gesture = StickGesture()
        c.pending = True
        c.now = 0.
        c.clock = lambda: c.now
        c.status, c.sound = Mock(), Mock()
        return c

    def sample(self, c, pwm, elapsed=.1):
        c.now += elapsed
        c.data['status']['rc']['value']['channels'][11] = pwm
        return c.handle_stop_switch(copy.deepcopy(c.data), c.now)

    def press(self, c):
        return [self.sample(c, p) for p in [999,999,1999,1999,1999,999,999]]

    def test_one_stop_per_pulse_retains_page_owner_and_never_calls_tasks(self):
        c = self.controller()
        self.assertEqual(sum(self.press(c)), 1)
        c.runtime.stop.assert_called_once_with()
        self.assertEqual(c.broker.owner, 'page-owner')
        self.assertFalse(c.pending)
        self.assertFalse(c.gesture.ready)
        c.sound.assert_called_once_with('stopped')
        for name in ('start','wall','ceiling','detach','repair_mavros'):
            getattr(c.runtime,name).assert_not_called()
        self.assertEqual(sum(self.press(c)), 1)
        self.assertEqual(c.runtime.stop.call_count, 2)

    def test_armed_pulses_are_only_for_existing_attachment_operator(self):
        c = self.controller()
        c.data['status']['state']['value']['armed'] = True
        self.assertFalse(any(self.press(c)))
        c.runtime.stop.assert_not_called()

    def test_no_boot_or_arm_to_disarm_high_level_stop(self):
        c = self.controller()
        for _ in range(5): self.assertFalse(self.sample(c,1999))
        self.press(c)
        self.assertEqual(c.runtime.stop.call_count, 1)
        c.data['status']['state']['value']['armed'] = True
        self.sample(c,999); self.sample(c,999); self.sample(c,1999)
        c.data['status']['state']['value']['armed'] = False
        for _ in range(5): self.assertFalse(self.sample(c,1999))
        self.assertEqual(c.runtime.stop.call_count, 1)

    def test_stale_state_rc_disconnect_unknown_armed_and_missing_channel_block(self):
        cases = [('state_stale',None),('rc_stale',None),('disconnected',None),('unknown',None),('short',None)]
        for name,_ in cases:
            c = self.controller()
            if name == 'state_stale': c.data['status']['state']['fresh'] = False
            if name == 'rc_stale': c.data['status']['rc']['fresh'] = False
            if name == 'disconnected': c.data['status']['state']['value']['connected'] = False
            if name == 'unknown': c.data['status']['state']['value']['armed'] = None
            if name == 'short':
                c.data['status']['rc']['value']['channels'] = [1500]*7
                self.assertFalse(safe_stop_snapshot(c.data))
                continue
            self.assertFalse(any(self.press(c)),name)
            c.runtime.stop.assert_not_called()

    def test_disconnect_midpulse_requires_rest_before_next_press(self):
        c = self.controller()
        self.sample(c,999); self.sample(c,999); self.sample(c,1999)
        c.data['status']['rc']['fresh'] = False
        self.sample(c,1999)
        c.data['status']['rc']['fresh'] = True
        self.assertFalse(self.sample(c,1999))
        self.assertFalse(self.sample(c,1999))
        c.runtime.stop.assert_not_called()

    def test_stopping_or_repairing_blocks_and_discards_pulse(self):
        for flag in ('stopping','repairing'):
            c = self.controller()
            setattr(c.runtime,flag,True)
            self.assertFalse(any(self.press(c)))
            setattr(c.runtime,flag,False)
            self.assertFalse(self.sample(c,1999))
            self.assertFalse(self.sample(c,1999))
            c.runtime.stop.assert_not_called()

    def test_stop_available_during_failed_start_or_cleanup(self):
        c = self.controller()
        c.runtime.stop_requested = True
        c.data['starting'] = True
        c.data['launch_error'] = 'wall_flight 已退出，请查看启动日志'
        self.assertEqual(sum(self.press(c)),1)

    def test_state_change_before_dispatch_cancels_without_ground_override(self):
        c = self.controller()
        c.data['status']['state']['value']['armed'] = True
        with self.assertRaisesRegex(ValueError,'disarmed'):
            c.trigger_stop()
        c.runtime.stop.assert_not_called()

    def test_stop_does_not_require_pilot_mode_low_kill_or_start_audit(self):
        c = self.controller()
        c.data['status']['state']['value']['mode'] = 'OFFBOARD'
        c.data['status']['rc']['value']['channels'][6] = 1999
        self.assertEqual(sum(self.press(c)),1)

    def test_failed_cleanup_never_reports_success_or_plays_stopped_tune(self):
        c = self.controller()
        c.runtime.stop.side_effect = ValueError('停止核验失败')
        with self.assertRaisesRegex(ValueError,'停止核验失败'):
            self.press(c)
        c.sound.assert_not_called()
        self.assertNotIn('STOPPED',[call.args[0] for call in c.status.call_args_list])

    def test_stale_mavros_registration_is_removed_only_with_no_process_or_port_owner(self):
        c = self.controller()
        c.config = {'fcu_port':'/dev/null','baud':230400}
        c.runtime.state.starting = False
        c.runtime.nodes.return_value = {'/mavros'}
        c.runtime.mavros_alive.return_value = False
        c.runtime.mavros_process_conflicts.return_value = []
        c.runtime.port_owners.return_value = []
        c.bootstrap()
        c.runtime.cleanup_stale_mavros.assert_called_once_with()
        c.runtime.spawn.assert_called_once()
        c.runtime.start.assert_not_called()

    def test_existing_output_process_or_serial_owner_blocks_automatic_mavros_recovery(self):
        for conflict,owners in (([123],[]),([],[456])):
            c = self.controller()
            c.config = {'fcu_port':'/dev/null','baud':230400}
            c.runtime.state.starting = False
            c.runtime.nodes.return_value = {'/mavros'}
            c.runtime.mavros_alive.return_value = False
            c.runtime.mavros_process_conflicts.return_value = conflict
            c.runtime.port_owners.return_value = owners
            with self.assertRaisesRegex(ValueError,'ground recovery'): c.bootstrap()
            c.runtime.cleanup_stale_mavros.assert_not_called()
            c.runtime.spawn.assert_not_called()


if __name__ == '__main__': unittest.main()
