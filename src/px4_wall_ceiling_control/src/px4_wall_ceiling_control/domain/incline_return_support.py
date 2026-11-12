"""Incline return collective seeded from HOLD, with hover and tilt floors.

Normalized collective is not a measurement of lift or contact force. This
controller does not command descent or introduce a separate release phase.
"""
from .geometry import clamp


class InclineReturnSupport:
    def __init__(self, controller, entry_thrust, entry_vertical_fraction,
                 rise_slew, fall_slew):
        self.controller = controller
        self.hover_floor = controller.base
        self.entry_thrust = self.value = entry_thrust
        self.vertical_base = max(self.hover_floor,
                                 entry_thrust*clamp(entry_vertical_fraction, 0., 1.))
        self.controller.base = self.vertical_base
        self.rise_slew, self.fall_slew = rise_slew, fall_slew
        self.at = controller.at
        self.requested = entry_thrust
        self.slew_limited = self.saturated = False
        self.reason = '倾角补偿回平：保留入口衔接，推力不低于悬停基准'

    def update(self, now, height, vz, vertical_fraction, actual_thrust,
               height_kp, velocity_kp):
        if now < self.at:
            raise ValueError('incline return support clock moved backwards')
        if now == self.at:
            return self.value
        dt = min(.05, now-self.at)
        self.at = now
        self.controller.update_range(height, vz, vertical_fraction, now,
            actual_thrust, height_kp, velocity_kp)
        # A negative height/vz correction cannot cancel attitude support.
        tilt_floor = self.hover_floor/max(clamp(vertical_fraction, 0., 1.), .001)
        unlimited = max(self.controller.requested, tilt_floor)
        self.requested = clamp(unlimited, self.hover_floor, self.controller.max_thrust)
        self.value = clamp(self.requested,
            self.value-self.fall_slew*dt, self.value+self.rise_slew*dt)
        self.value = clamp(self.value, self.hover_floor, self.controller.max_thrust)
        self.slew_limited = abs(self.requested-self.value) > 1e-9
        self.saturated = unlimited > self.controller.max_thrust or self.slew_limited
        self.controller.value = self.value
        return self.value
