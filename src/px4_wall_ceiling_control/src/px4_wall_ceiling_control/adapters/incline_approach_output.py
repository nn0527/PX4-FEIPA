"""Incline-owned approach output, copied from the side-wall output behavior."""
import math
import rospy
from geometry_msgs.msg import Quaternion, Vector3Stamped
from std_msgs.msg import Float32
from px4_wall_ceiling_control.domain.geometry import quaternion_multiply, quaternion_normalize, euler_from_quaternion
from px4_wall_ceiling_control.domain.recovery_attitude_limits import limit_recovery_attitude, axis_yaw
from px4_wall_ceiling_control.msg import AttachmentControlCandidate as C, WallPerchStatus as W
from px4_wall_ceiling_control.freshness import is_fresh_receipt


class InclineApproachOutput:
    def _incline_approach_output_settings(self):
        if not hasattr(self, 'incline_approach_output_params'):
            defaults = dict(rc_assist_enabled=True, rc_roll_deg=10., rc_yaw_deg=15.,
                rc_roll_slew_deg_s=90., rc_yaw_slew_deg_s=20., max_tilt_deg=10.,
                tilt_slew_deg_s=10., yaw_slew_deg_s=20.)
            p = {k: rospy.get_param('/incline_attachment/approach/' + k, v) for k,v in defaults.items()}
            for key,value in p.items():
                if key == 'rc_assist_enabled':
                    if type(value) is not bool: raise ValueError('invalid incline RC enable')
                elif isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value) or value <= 0:
                    raise ValueError('invalid incline output parameter: ' + key)
            if p['max_tilt_deg'] > 10. or p['tilt_slew_deg_s'] > 10. or p['yaw_slew_deg_s'] > 20. or p['rc_roll_deg'] > 10. or p['rc_yaw_deg'] > 30.:
                raise ValueError('incline approach output exceeds bounds')
            self.incline_approach_output_params = p
        return self.incline_approach_output_params

    def _incline_approach_assisted_attitude(self, now, quaternion):
        p = self._incline_approach_output_settings()
        if not p['rc_assist_enabled'] or not getattr(self, 'rc_assist_enabled', False):
            return quaternion
        flight = self.flight_state
        if (flight is None or not is_fresh_receipt(self.flight_state_at, self.state_timeout_s, now)
                or flight.mode.upper() != 'OFFBOARD'):
            self.incline_rc_roll_rad = self.incline_rc_yaw_rad = 0.
            self.incline_rc_at = now
            return quaternion
        ready = self._rc_ready(now, C.SOURCE_WALL)
        previous_at = getattr(self, 'incline_rc_at', None)
        elapsed = max(0., min((now-previous_at).to_sec(), .1)) if previous_at is not None else .02
        applied = {}
        for axis in ('roll', 'yaw'):
            limit = math.radians(p['rc_' + axis + '_deg'])
            fraction = self._calibrated_rc_fraction(axis) if ready else 0.
            desired = fraction * limit
            previous = max(-limit, min(limit, getattr(self, 'incline_rc_' + axis + '_rad', 0.)))
            step = math.radians(p['rc_' + axis + '_slew_deg_s'])*elapsed
            applied[axis] = max(previous-step, min(previous+step, desired))
            setattr(self, 'incline_rc_' + axis + '_rad', applied[axis])
        self.incline_rc_at = now
        q = tuple(getattr(quaternion,k) for k in 'xyzw')
        roll = (math.sin(applied['roll']/2), 0., 0., math.cos(applied['roll']/2))
        yaw = (0., 0., math.sin(applied['yaw']/2), math.cos(applied['yaw']/2))
        q = quaternion_normalize(quaternion_multiply(yaw, quaternion_multiply(q, roll)))
        preview = Vector3Stamped(); preview.header.stamp = now
        preview.vector.x = math.degrees(applied['roll']); preview.vector.z = math.degrees(applied['yaw'])
        self.rc_preview_publisher.publish(preview)
        self.rc_age_publisher.publish(Float32(data=(now-self.rc_at).to_sec()*1000 if ready else -1.))
        return Quaternion(*q)

    def _limit_incline_approach_target(self, now, target, source, candidate=None):
        p = self._incline_approach_output_settings()
        status = getattr(self, 'wall_status', None)
        flight = self.flight_state
        active = bool(flight is not None and flight.connected and flight.armed
            and is_fresh_receipt(self.flight_state_at, self.state_timeout_s, now)
            and ((candidate is not None and candidate.control_phase == W.STATE_SLOW_APPROACH
                  and flight.mode.upper() == 'OFFBOARD')
                 or (source == C.SOURCE_NONE and status is not None
                     and is_fresh_receipt(self.wall_status_at, self.status_timeout_s, now)
                     and status.incline_active and status.horizontal_equilibrium_enabled
                     and status.state in (W.STATE_IDLE,W.STATE_SLOW_APPROACH)
                     and flight.mode.upper() in ('POSCTL','OFFBOARD'))))
        if not active: return target
        previous = getattr(self, 'incline_approach_previous_target', None)
        previous_at = getattr(self, 'incline_approach_previous_at', None)
        dt = max(0., min((now-previous_at).to_sec(), .1)) if previous_at is not None else .02
        if previous is None or not is_fresh_receipt(previous_at,self.status_timeout_s,now):
            o = self.odom.pose.pose.orientation
            previous = (o.x,o.y,o.z,o.w); dt = .02
        q = tuple(getattr(target.orientation,k) for k in 'xyzw')
        q,_ = limit_recovery_attitude(q,previous,dt,math.radians(p['max_tilt_deg']),
            math.radians(p['tilt_slew_deg_s']),math.radians(p['yaw_slew_deg_s']))
        target.orientation = Quaternion(*q)
        roll,pitch,_ = euler_from_quaternion(q);axis,_ = axis_yaw(q)
        preview = Vector3Stamped();preview.header.stamp=now
        preview.vector.x,preview.vector.y=math.degrees(roll),math.degrees(pitch)
        preview.vector.z=math.degrees(math.acos(max(-1.,min(1.,axis[2]))))
        self.approach_target_publisher.publish(preview)
        return target
