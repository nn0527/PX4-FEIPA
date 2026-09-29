#!/usr/bin/env python3

import contextlib
import importlib.util
import io
import os
import unittest
from threading import Lock

from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

from px4_wall_ceiling_control.msg import (
    AttachmentMechanismStatus,
    CeilingAttachmentStatus,
    SensorHealth,
)


MONITOR_PATH = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "scripts",
        "test_support",
        "ceiling_attachment_state_monitor.py",
    )
)
spec = importlib.util.spec_from_file_location("ceiling_state_monitor", MONITOR_PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class CeilingStateMonitorTest(unittest.TestCase):
    def make_monitor(self):
        monitor = module.CeilingAttachmentStateMonitor.__new__(
            module.CeilingAttachmentStateMonitor
        )
        monitor.previous_state = None
        monitor.previous_name = None
        monitor.lock = Lock()
        monitor.latest = {}
        monitor.bell = False
        monitor.status_topic = "/ceiling_attachment/status"
        monitor.range_topic = "/sensor/ceiling/range"
        monitor.health_topic = "/sensor/health"
        monitor.state_topic = "/mock_mavros/state"
        monitor.odom_topic = "/mock_mavros/local_position/odom"
        monitor.mechanism_topic = "/attachment/mechanism_status"
        return monitor

    def test_fault_prints_one_snapshot_of_latest_inputs(self):
        monitor = self.make_monitor()
        monitor.on_range(Range(range=0.08))
        monitor.on_health(SensorHealth())
        monitor.on_flight_state(State())
        monitor.on_odom(Odometry())
        monitor.on_mechanism(AttachmentMechanismStatus())

        normal = CeilingAttachmentStatus()
        normal.state = CeilingAttachmentStatus.STATE_APPROACH
        normal.state_name = "APPROACH"
        fault = CeilingAttachmentStatus()
        fault.state = CeilingAttachmentStatus.STATE_FAULT
        fault.state_name = "FAULT"
        fault.reason = "test fault"
        fault.fault_count = 1

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            monitor.on_status(normal)
            monitor.on_status(fault)
            monitor.on_status(fault)

        text = output.getvalue()
        self.assertEqual(text.count("[FAULT 快照]"), 1)
        self.assertIn("APPROACH -> FAULT", text)
        for topic in (
            monitor.status_topic,
            monitor.range_topic,
            monitor.health_topic,
            monitor.state_topic,
            monitor.odom_topic,
            monitor.mechanism_topic,
        ):
            self.assertIn(topic, text)

    def test_fault_marks_missing_inputs(self):
        monitor = self.make_monitor()
        fault = CeilingAttachmentStatus()
        fault.state = CeilingAttachmentStatus.STATE_FAULT
        fault.state_name = "FAULT"

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            monitor.on_status(fault)

        self.assertIn("/sensor/ceiling/range: 尚未收到消息", output.getvalue())


if __name__ == "__main__":
    unittest.main()
