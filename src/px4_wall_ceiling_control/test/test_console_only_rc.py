import unittest
from unittest import mock
from test_rc_wall_toggle import MixedOperatorTest, Op

class ConsoleOnlyRCTests(unittest.TestCase):
    def setUp(self):
        self.fx=MixedOperatorTest();self.fx.setUp();self.addCleanup(self.fx.doCleanups)
        self.n=self.fx.node;self.n.console_mode_only=True

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
            self.n._on_legacy_wall_toggle=mock.Mock()
            self.n._on_wall_toggle()
            self.n._on_attachment_toggle.assert_called_once()
            self.n._on_legacy_wall_toggle.assert_not_called()

    def test_ceiling_handover_confirmation_still_has_priority(self):
        self.n.mode=Op.MODE_CEILING
        self.n._request_ceiling_handover=mock.Mock(return_value=True)
        self.n._on_attachment_toggle=mock.Mock()
        self.n._on_wall_toggle()
        self.n._on_attachment_toggle.assert_not_called()

    def test_ch8_has_no_effect_with_console_selection(self):
        from mavros_msgs.msg import RCIn
        self.n.rc_mode_selector_enabled=False
        self.n.mode_selector=mock.Mock()
        self.n._apply_command=mock.Mock()
        for pwm in (999,1999,999):
            channels=[999]*12;channels[7]=pwm
            self.n._on_rc(RCIn(channels=channels))
        self.n.mode_selector.update.assert_not_called()
        self.n._apply_command.assert_not_called()
