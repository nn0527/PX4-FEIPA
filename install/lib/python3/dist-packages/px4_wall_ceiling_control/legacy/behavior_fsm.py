"""Legacy behavior-state-machine shell for ceiling and wall tracking."""

import rospy
from sensor_msgs.msg import Range

from px4_wall_ceiling_control.freshness import is_fresh, is_valid_range
from px4_wall_ceiling_control.msg import BehaviorCommand, SupervisorState


class BehaviorFsm:
    """Turn a selected high-level mode into a surface-distance behavior goal."""

    def __init__(self, surface):
        if surface not in ("ceiling", "wall"):
            raise ValueError("surface must be 'ceiling' or 'wall'")

        self.surface = surface
        self.source = (
            BehaviorCommand.SOURCE_CEILING
            if surface == "ceiling"
            else BehaviorCommand.SOURCE_WALL
        )
        self.required_supervisor_state = (
            SupervisorState.STATE_CEILING_ACTIVE
            if surface == "ceiling"
            else SupervisorState.STATE_WALL_ACTIVE
        )

        default_range_topic = "/sensor/{}/range".format(surface)
        default_command_topic = "/{}_fsm/command".format(surface)
        self.target_distance_m = float(rospy.get_param("~target_distance_m", 1.0))
        self.tolerance_m = float(rospy.get_param("~distance_tolerance_m", 0.1))
        self.max_speed_mps = float(rospy.get_param("~max_speed_mps", 0.3))
        self.range_timeout_s = float(rospy.get_param("~range_timeout_s", 0.3))
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))

        supervisor_topic = rospy.get_param("~supervisor_topic", "/supervisor/state")
        range_topic = rospy.get_param("~range_topic", default_range_topic)
        command_topic = rospy.get_param("~command_topic", default_command_topic)

        self.supervisor = None
        self.range_message = None
        self.range_received_at = None
        self.last_state = None

        self.publisher = rospy.Publisher(command_topic, BehaviorCommand, queue_size=10)
        rospy.Subscriber(
            supervisor_topic, SupervisorState, self._on_supervisor, queue_size=10
        )
        rospy.Subscriber(range_topic, Range, self._on_range, queue_size=10)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(publish_rate_hz, 1.0)), self._on_timer
        )

    def _on_supervisor(self, message):
        self.supervisor = message

    def _on_range(self, message):
        self.range_message = message
        self.range_received_at = rospy.Time.now()

    def _on_timer(self, _event):
        command = self._build_command(rospy.Time.now())
        self.publisher.publish(command)
        if command.state != self.last_state:
            rospy.loginfo(
                "%s FSM state=%d (%s)", self.surface, command.state, command.reason
            )
            self.last_state = command.state

    def _build_command(self, now):
        command = BehaviorCommand()
        command.header.stamp = now
        command.source = self.source
        command.state = BehaviorCommand.STATE_DISABLED
        command.active = False
        command.target_distance_m = self.target_distance_m
        command.max_speed_mps = self.max_speed_mps
        command.reason = "not selected"

        if self.supervisor is None:
            command.reason = "waiting for supervisor"
            return command

        if (
            self.supervisor.state != self.required_supervisor_state
            or not self.supervisor.motion_allowed
        ):
            if self.supervisor.state == SupervisorState.STATE_PRESTREAM:
                command.reason = "PX4 offboard pre-stream; motion inhibited"
            return command

        if not is_fresh(self.range_received_at, self.range_timeout_s, now):
            command.state = BehaviorCommand.STATE_FAULT
            command.reason = "range timeout"
            return command
        if not is_valid_range(self.range_message):
            command.state = BehaviorCommand.STATE_FAULT
            command.reason = "invalid range"
            return command

        # TODO: add SEARCH/lost-target handling, hysteresis and timed transitions.
        error = self.range_message.range - self.target_distance_m
        command.active = True
        if error > self.tolerance_m:
            command.state = BehaviorCommand.STATE_APPROACH
            command.reason = "surface farther than target"
        elif error < -self.tolerance_m:
            command.state = BehaviorCommand.STATE_RETREAT
            command.reason = "surface closer than target"
        else:
            command.state = BehaviorCommand.STATE_TRACK
            command.reason = "inside tracking band"
        return command


def run_behavior_fsm(surface):
    rospy.init_node("{}_fsm_node".format(surface))
    BehaviorFsm(surface)
    rospy.spin()
