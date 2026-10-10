"""Causal current-state anchored variant of the frozen W1 world model.

W1 estimates velocity and heading from a 0.6 s position chord.  That estimate
is deliberately retained by :class:`WorldModel`; this variant replaces only
the present velocity vector with the observation's current ``v/yaw`` and
estimates acceleration/yaw rate from the causal history of those observations.
The inherited prediction model and its uncertainty limitations are unchanged.
"""
import math
from collections import deque
from typing import Dict, List

import numpy as np

from .world import Agent, WorldModel


class CurrentStateWorldModel(WorldModel):
    """W1 update with current speed/heading anchoring and causal state diffs."""

    STATE_HIST_S = 0.4
    STATE_ACC_CLAMP = 3.5
    STATE_YAW_RATE_CLAMP = 0.8

    def __init__(self):
        super().__init__()
        # id -> (t, speed, yaw), retained only while the actor is observed.
        self.state_hist: Dict[str, deque] = {}

    def update(self, t: float, agents: List[Agent]):
        super().update(t, agents)
        seen = set()
        for a in agents:
            seen.add(a.id)
            history = self.state_hist.setdefault(a.id, deque())
            if history and t < history[-1][0]:
                history.clear()
            if history and t == history[-1][0]:
                history.pop()
            history.append((float(t), float(a.v), float(a.yaw)))
            while len(history) > 2 and t - history[0][0] > self.STATE_HIST_S + 1e-9:
                history.popleft()

            tr = self.tracks[a.id]
            # The observed state is current by contract; do not reuse the W1
            # position-chord vector for the present heading or speed.
            tr.vx = float(a.v * math.cos(a.yaw))
            tr.vy = float(a.v * math.sin(a.yaw))
            # Do not blend with super().update's already filtered estimates:
            # that would mix two different derivative clocks and histories.
            tr.accel = 0.0
            tr.yaw_rate = 0.0
            if len(history) < 2:
                continue
            t0, v0, yaw0 = history[0]
            dt = float(t - t0)
            if dt <= 1e-3:
                continue
            raw_acc = float(np.clip((a.v - v0) / dt,
                                    -self.STATE_ACC_CLAMP, self.STATE_ACC_CLAMP))
            tr.accel = raw_acc
            # Agent headings may wrap at 2π; use the shortest causal delta.
            dyaw = (float(a.yaw) - yaw0 + math.pi) % (2.0 * math.pi) - math.pi
            raw_rate = float(np.clip(dyaw / dt,
                                     -self.STATE_YAW_RATE_CLAMP,
                                     self.STATE_YAW_RATE_CLAMP))
            tr.yaw_rate = raw_rate

        for key in list(self.state_hist):
            if key not in seen:
                del self.state_hist[key]

