"""Debounce a CH12 low-rest/high-pressed switch without startup/reconnect edges."""

import math


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
