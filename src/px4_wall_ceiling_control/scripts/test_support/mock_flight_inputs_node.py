#!/usr/bin/env python3
"""Publish isolated MAVROS-like feedback for real-sensor bench testing only."""

import rospy
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry


class MockFlightInputsNode:
    def __init__(self):
        rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))
        self.ceiling_switch_channel_index = int(
            rospy.get_param("~ceiling_switch_channel_index", 4)
        )
        self.wall_switch_channel_index = int(
            rospy.get_param("~wall_switch_channel_index", 5)
        )
        self.ceiling_switch_pwm = int(
            rospy.get_param("~ceiling_switch_pwm", 1800)
        )
        self.wall_switch_pwm = int(rospy.get_param("~wall_switch_pwm", 1000))
        self.altitude_m = float(rospy.get_param("~altitude_m", 1.0))

        self.state_publisher = rospy.Publisher(
            "/mock_mavros/state", State, queue_size=10
        )
        self.rc_publisher = rospy.Publisher(
            "/mock_mavros/rc/in", RCIn, queue_size=10
        )
        self.odom_publisher = rospy.Publisher(
            "/mock_mavros/local_position/odom", Odometry, queue_size=10
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(rate_hz, 2.0)), self._publish
        )
        rospy.logwarn(
            "mock flight feedback active; this node is for isolated bench tests only"
        )

    def _publish(self, _event):
        now = rospy.Time.now()

        state = State()
        state.connected = True
        state.armed = True
        state.guided = True
        state.manual_input = True
        state.mode = "OFFBOARD"
        state.system_status = 4
        self.state_publisher.publish(state)

        channel_count = max(
            self.ceiling_switch_channel_index, self.wall_switch_channel_index, 7
        ) + 1
        channels = [1500] * channel_count
        channels[self.ceiling_switch_channel_index] = self.ceiling_switch_pwm
        channels[self.wall_switch_channel_index] = self.wall_switch_pwm
        rc = RCIn()
        rc.header.stamp = now
        rc.rssi = 255
        rc.channels = channels
        self.rc_publisher.publish(rc)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.position.z = self.altitude_m
        self.odom_publisher.publish(odom)


if __name__ == "__main__":
    rospy.init_node("mock_flight_inputs_node")
    MockFlightInputsNode()
    rospy.spin()
