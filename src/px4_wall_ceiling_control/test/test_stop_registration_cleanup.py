"""Regression coverage for stale registrations and concurrent replacements."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
spec = importlib.util.spec_from_file_location('stop_cleanup_server', ROOT / 'scripts/operator/wall_console_server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class StopRegistrationCleanupTests(unittest.TestCase):
    def setUp(self):
        self.name = '/wall_perch_decision_node'
        self.uri = 'http://localhost:40483/'
        self.graph = [[['/wall_perch/status', [self.name]]],
                      [['/operator/command', [self.name]]],
                      [['/wall_perch_decision_node/get_loggers', [self.name]]]]
        self.master, self.node = mock.Mock(), mock.Mock()
        self.master.getSystemState.side_effect = [(1, '', self.graph), (1, '', self.graph), (1, '', [[], [], []])]
        self.master.lookupNode.return_value = (1, '', self.uri)
        self.master.lookupService.return_value = (1, '', 'rosrpc://localhost:45678')
        for method in ('unregisterPublisher', 'unregisterSubscriber', 'unregisterService'):
            getattr(self.master, method).return_value = (1, '', 1)
        self.node.getPid.side_effect = ConnectionRefusedError()

    def run_check(self, **kwargs):
        with mock.patch.object(server.xmlrpc.client, 'ServerProxy',
                               side_effect=lambda uri, **unused: self.master if uri.endswith(':11311') else self.node):
            return server.RemoteRuntime.stop_ros_residuals(**kwargs)

    def assert_no_cleanup(self):
        for method in ('unregisterPublisher', 'unregisterSubscriber', 'unregisterService'):
            getattr(self.master, method).assert_not_called()

    def test_refused_node_registrations_are_removed_and_rechecked(self):
        self.assertEqual(self.run_check(), [])
        self.master.unregisterPublisher.assert_called_once_with(self.name, '/wall_perch/status', self.uri)
        self.master.unregisterSubscriber.assert_called_once_with(self.name, '/operator/command', self.uri)
        self.master.unregisterService.assert_called_once_with(self.name, '/wall_perch_decision_node/get_loggers', 'rosrpc://localhost:45678')
        self.assertEqual(self.master.getSystemState.call_count, 3)

    def test_live_node_is_reported_without_cleanup(self):
        self.node.getPid.side_effect = None
        self.node.getPid.return_value = (1, '', 123)
        self.assertEqual(self.run_check(), [self.name])
        self.assert_no_cleanup()

    def test_new_uri_is_not_cleaned(self):
        self.master.lookupNode.side_effect = [(1, '', self.uri), (1, '', 'http://localhost:9999/')]
        with self.assertRaisesRegex(ValueError, '同名节点重新注册'):
            self.run_check()
        self.assert_no_cleanup()

    def test_node_responding_again_at_same_uri_is_not_cleaned(self):
        self.node.getPid.side_effect = [ConnectionRefusedError(), (1, '', 789)]
        with self.assertRaisesRegex(ValueError, '重新响应'):
            self.run_check()
        self.assert_no_cleanup()

    def test_timeout_does_not_authorize_cleanup(self):
        self.node.getPid.side_effect = TimeoutError('timed out')
        with self.assertRaisesRegex(ValueError, '无法确认节点已停止'):
            self.run_check()
        self.assert_no_cleanup()

    def test_registration_still_present_fails_verification(self):
        self.master.getSystemState.side_effect = None
        self.master.getSystemState.return_value = (1, '', self.graph)
        with self.assertRaisesRegex(ValueError, '仍有节点注册'):
            self.run_check()

    def test_master_rejecting_cleanup_fails_verification(self):
        self.master.unregisterPublisher.return_value = (0, 'rejected', 0)
        with self.assertRaisesRegex(ValueError, '清理残留注册失败'):
            self.run_check()

    def test_keep_telemetry_does_not_cleanup_mavros(self):
        self.master.getSystemState.side_effect = None
        self.master.getSystemState.return_value = (1, '', [[['/mavros/state', ['/mavros']]], [], []])
        self.assertEqual(self.run_check(keep_telemetry=True), [])
        self.master.lookupNode.assert_not_called()
        self.assert_no_cleanup()


if __name__ == '__main__':
    unittest.main()
