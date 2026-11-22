"""Debounce the attachment selector and momentary switch."""

import math


class ThreePositionSelector:
    def __init__(self, up_is_high=False):
        self.positions = ("incline", "wall", "ceiling") if up_is_high else ("ceiling", "wall", "incline")
        self.reset()

    def reset(self):
        self.selected = self.candidate = None
        self.candidate_at = self.last_at = None

    def update(self, pwm, now):
        if self.last_at is not None and not 0 <= now - self.last_at <= .25:
            self.reset()
        self.last_at = now
        position = None
        if pwm is not None:
            for index, (low, high) in enumerate(((800, 1200), (1350, 1650), (1800, 2200))):
                if low <= pwm <= high:
                    position = self.positions[index]
                    break
        if position != self.candidate:
            self.selected = None
            self.candidate, self.candidate_at = position, now
        elif position is not None and now - self.candidate_at >= .05:
            self.selected = position


class MomentarySwitch:
    def __init__(self, debounce_s=0.05, timeout_s=0.5):
        self.debounce_s = debounce_s
        self.timeout_s = timeout_s
        self.reset()

    def reset(self):
        self.last_at = None
        self.candidate = None
        self.candidate_at = None
        self.armed = False

    def update(self, pwm, now):
        if (pwm is None or not math.isfinite(pwm) or not 800 <= pwm <= 2200
                or not math.isfinite(now)):
            self.reset()
            return False
        if self.last_at is not None and not 0 <= now-self.last_at <= self.timeout_s:
            self.reset()
        self.last_at = now
        level = 'low' if pwm <= 1200 else 'high' if pwm >= 1800 else None
        if level != self.candidate:
            self.candidate, self.candidate_at = level, now
        if level is None or now-self.candidate_at < self.debounce_s:
            return False
        if level == 'low':
            self.armed = True
            return False
        trigger = self.armed
        self.armed = False
        return trigger
