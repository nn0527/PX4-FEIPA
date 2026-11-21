"""Stable POSCTL thrust-axis feedforward, frozen for one takeover attempt."""
import math
from collections import deque
from statistics import median
from px4_wall_ceiling_control.domain.geometry import quaternion_normalize, rotate_vector

class HorizontalEquilibrium:
    def __init__(self, window=.5, max_gap=.35, max_tilt_deg=5., max_spread_deg=1.5):
        self.window,self.max_gap=window,max_gap
        self.max_tilt=math.radians(max_tilt_deg);self.max_spread=math.radians(max_spread_deg)
        self.clear()
    def clear(self):
        self.samples=deque();self.axis=(0.,0.,1.);self.ready=False;self.frozen=False
        self.last_stamp=None
        self.thrust = None
    def observe(self, stamp, quaternion, eligible, thrust=None):
        if self.frozen:return
        if not eligible:
            self.clear();return
        if self.last_stamp == stamp:return
        q=quaternion_normalize(quaternion);axis=rotate_vector(q,(0.,0.,1.))
        if not all(math.isfinite(v) for v in axis) or axis[2] < math.cos(self.max_tilt):
            self.clear();return
        if self.last_stamp is not None and (stamp <= self.last_stamp or stamp-self.last_stamp > self.max_gap):
            self.clear()
        if self.samples and any(sum(a*b for a,b in zip(axis,old)) < math.cos(self.max_spread) for _,old,_ in self.samples):
            self.clear()
        self.samples.append((stamp,axis,thrust));self.last_stamp=stamp
        # Keep at least the oldest sample needed to span the confirmation window.
        while len(self.samples)>3 and stamp-self.samples[1][0]>=self.window:self.samples.popleft()
        self.ready=len(self.samples)>=3 and stamp-self.samples[0][0]>=self.window
        if self.ready:
            slopes=[(a[0]/a[2],a[1]/a[2]) for _,a,_ in self.samples]
            v=(sum(a[0] for a in slopes)/len(slopes),sum(a[1] for a in slopes)/len(slopes),1.)
            length=math.sqrt(sum(a*a for a in v));self.axis=tuple(a/length for a in v)
            thrusts = [value for _, _, value in self.samples if value is not None]
            self.thrust = median(thrusts) if len(thrusts) == len(self.samples) else None
    def freeze(self, now):
        if self.frozen:return True
        if self.ready and self.last_stamp is not None and 0<=now-self.last_stamp<=self.max_gap:
            self.frozen=True;return True
        self.clear();return False
