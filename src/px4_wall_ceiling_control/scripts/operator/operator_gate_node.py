#!/usr/bin/env python3
"""Convert raw MAVROS RC channels into a debounced, fail-closed command."""

import rospy
from mavros_msgs.msg import RCIn

from px4_wall_ceiling_control.msg import OperatorCommand


class OperatorGateNode:
    def __init__(self):
        rc_topic = rospy.get_param("~rc_topic", "/mavros/rc/in")
        command_topic = rospy.get_param("~command_topic", "/operator/command")
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))
        self.rc_timeout_s = float(rospy.get_param("~rc_timeout_s", 0.5))
        self.control_scheme = str(
            rospy.get_param("~control_scheme", "independent_switches")
        )
        self.ceiling_switch_channel_index = int(
            rospy.get_param("~ceiling_switch_channel_index", 4)
        )
        self.wall_switch_channel_index = int(
            rospy.get_param("~wall_switch_channel_index", 5)
        )
        self.switch_pwm_min = int(rospy.get_param("~switch_pwm_min", 1700))
        self.mode_channel_index = int(rospy.get_param("~mode_channel_index", 4))
        self.enable_channel_index = int(rospy.get_param("~enable_channel_index", 5))
        self.enable_pwm_min = int(rospy.get_param("~enable_pwm_min", 1700))
        self.manual_pwm_max = int(rospy.get_param("~manual_pwm_max", 1200))
        self.standby_pwm_max = int(rospy.get_param("~standby_pwm_max", 1450))
        self.ceiling_pwm_max = int(rospy.get_param("~ceiling_pwm_max", 1750))
        self.debounce_samples = max(1, int(rospy.get_param("~debounce_samples", 3)))

        self.channels = []
        self.received_at = None
        self.stable_mode = (
            OperatorCommand.MODE_STANDBY
            if self.control_scheme == "independent_switches"
            else OperatorCommand.MODE_MANUAL
        )
        self.candidate_mode = self.stable_mode
        self.candidate_count = 0
        self.live_request = OperatorCommand.MODE_STANDBY

        self.publisher = rospy.Publisher(command_topic, OperatorCommand, queue_size=10)
        rospy.Subscriber(rc_topic, RCIn, self._on_rc, queue_size=10)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(publish_rate_hz, 1.0)), self._on_timer
        )

    def _mode_from_pwm(self, pwm):
        if pwm <= self.manual_pwm_max:
            return OperatorCommand.MODE_MANUAL
        if pwm <= self.standby_pwm_max:
            return OperatorCommand.MODE_STANDBY
        if pwm <= self.ceiling_pwm_max:
            return OperatorCommand.MODE_CEILING
        return OperatorCommand.MODE_WALL

    def _independent_switch_mode(self):
        required = (
            self.ceiling_switch_channel_index,
            self.wall_switch_channel_index,
        )
        if any(index < 0 or index >= len(self.channels) for index in required):
            return None
        ceiling_on = (
            self.channels[self.ceiling_switch_channel_index] >= self.switch_pwm_min
        )
        wall_on = self.channels[self.wall_switch_channel_index] >= self.switch_pwm_min
        if ceiling_on and wall_on:
            return OperatorCommand.MODE_ABORT
        if ceiling_on:
            return OperatorCommand.MODE_CEILING
        if wall_on:
            return OperatorCommand.MODE_WALL
        return OperatorCommand.MODE_STANDBY

    def _requested_mode(self):
        if self.control_scheme == "independent_switches":
            return self._independent_switch_mode()
        if self.control_scheme == "selector_deadman":
            if self.mode_channel_index < 0 or self.mode_channel_index >= len(
                self.channels
            ):
                return None
            return self._mode_from_pwm(self.channels[self.mode_channel_index])
        return None

    def _on_rc(self, message):
        self.channels = list(message.channels)
        self.received_at = rospy.Time.now()
        requested = self._requested_mode()
        if requested is None:
            return
        self.live_request = requested
        # A simultaneous switch request must inhibit motion without debounce delay.
        if requested == OperatorCommand.MODE_ABORT:
            self.stable_mode = requested
            self.candidate_mode = requested
            self.candidate_count = self.debounce_samples
            return
        if requested == self.candidate_mode:
            self.candidate_count += 1
        else:
            self.candidate_mode = requested
            self.candidate_count = 1
        if self.candidate_count >= self.debounce_samples:
            self.stable_mode = requested

    def _on_timer(self, _event):
        now = rospy.Time.now()
        command = OperatorCommand()
        command.header.stamp = now
        command.source = "mavros_rc"
        command.mode = OperatorCommand.MODE_ABORT
        command.enable_control = False
        command.reason = "RC data unavailable"

        if self.received_at is None:
            self.publisher.publish(command)
            return
        age_s = (now - self.received_at).to_sec()
        if age_s < 0.0 or age_s > self.rc_timeout_s:
            command.reason = "RC timeout"
            self.publisher.publish(command)
            return
        live_request = self._requested_mode()
        if live_request is None:
            command.reason = "RC mode switch channel missing or scheme invalid"
            self.publisher.publish(command)
            return
        if live_request == OperatorCommand.MODE_ABORT:
            command.reason = "ceiling and wall switches cannot be active together"
            self.publisher.publish(command)
            return

        command.mode = self.stable_mode
        if self.control_scheme == "independent_switches":
            # Starting is debounced, while releasing/changing a switch inhibits
            # motion immediately instead of keeping the previous mode alive.
            command.enable_control = (
                live_request == self.stable_mode
                and self.stable_mode
                in (OperatorCommand.MODE_CEILING, OperatorCommand.MODE_WALL)
            )
        else:
            if (
                self.enable_channel_index < 0
                or self.enable_channel_index >= len(self.channels)
            ):
                command.reason = "deadman/enable channel missing"
                self.publisher.publish(command)
                return
            deadman_held = (
                self.channels[self.enable_channel_index] >= self.enable_pwm_min
            )
            command.enable_control = deadman_held and self.stable_mode in (
                OperatorCommand.MODE_CEILING,
                OperatorCommand.MODE_WALL,
            )
        command.reason = "accepted" if command.enable_control else "motion not enabled"
        self.publisher.publish(command)


if __name__ == "__main__":
    rospy.init_node("operator_gate_node")
    OperatorGateNode()
    rospy.spin()
