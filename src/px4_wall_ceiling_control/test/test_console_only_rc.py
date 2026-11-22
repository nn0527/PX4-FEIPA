import unittest
from unittest import mock
from test_rc_wall_toggle import MixedOperatorTest, Op

class ConsoleOnlyRCTests(unittest.TestCase):
    def setUp(self):
        self.fx=MixedOperatorTest();self.fx.setUp();self.addCleanup(self.fx.doCleanups)
        self.n=self.fx.node;self.n.console_selected_mode=None
        self.n.rc_wall_toggle_enabled=False

    def test_ch12_cannot_start_an_idle_task(self):
        self.n.mode=Op.MODE_STANDBY
        self.n._apply_command=mock.Mock()
        self.n._on_wall_toggle()
        self.n._apply_command.assert_not_called()

    def test_active_task_routes_to_its_own_handler(self):
        for mode in (Op.MODE_CEILING,Op.MODE_WALL,Op.MODE_INCLINE):
            self.n.mode=mode
            self.n._request_ceiling_handover=mock.Mock(return_value=False)
            self.n._on_attachment_toggle=mock.Mock()
            self.n._on_wall_toggle()
            self.n._on_attachment_toggle.assert_called_once()

    def test_ceiling_handover_confirmation_still_has_priority(self):
        self.n.mode=Op.MODE_CEILING
        self.n._request_ceiling_handover=mock.Mock(return_value=True)
        self.n._on_attachment_toggle=mock.Mock()
        self.n._on_wall_toggle()
        self.n._on_attachment_toggle.assert_not_called()

    def test_ch8_has_no_effect_with_console_selection(self):
        from mavros_msgs.msg import RCIn
        self.n._apply_command=mock.Mock()
        for pwm in (999,1999,999):
            channels=[999]*12;channels[7]=pwm
            self.n._on_rc(RCIn(channels=channels))
        self.assertIsNone(self.n.console_selected_mode)
        self.n._apply_command.assert_not_called()
