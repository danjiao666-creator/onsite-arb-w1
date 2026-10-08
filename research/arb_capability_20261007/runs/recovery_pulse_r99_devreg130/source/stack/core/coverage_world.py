"""Retain W1 and add causal occupancy coverage from current observations.

Optional branches anchor W1 to current speed/heading and retain constant-speed
motion while an actor is braking. They use no future actor observations.
Original W1 rows stay first and unchanged. Extra rows are collision hypotheses,
not extra physical actors: consumers must use headway_mask for following costs.
"""
import math

import numpy as np

from .world import WorldModel


class CoverageWorldModel(WorldModel):
    # A 0.1 m/s^2 deceleration changes a 6 s displacement by 1.8 m before stopping.
    # The threshold excludes tiny estimate noise while covering a vehicle-scale
    # separation. At >1 m/s, continued motion over the horizon is material.
    # These are hypothesis activation thresholds, not calibrated safety limits.
    BRAKING_THRESHOLD = -0.1
    MOVING_THRESHOLD = 1.0

    def __init__(self, anchored=True, braking_cv=True):
        super().__init__()
        self.anchored = bool(anchored)
        self.braking_cv = bool(braking_cv)

    def predict(self, times, near_xy=None, radius=150.0):
        original = super().predict(times, near_xy=near_xy, radius=radius)
        count = len(original['ids'])
        original['headway_mask'] = np.ones(count, dtype=bool)
        original['branch_mask'] = np.zeros(count, dtype=bool)
        original['source_ids'] = list(original['ids'])
        original['hypothesis'] = ['w1'] * count
        if not self.anchored and not self.braking_cv:
            return original

        times = np.asarray(times, dtype=float)
        dts = np.diff(np.concatenate([[0.0],times]))
        added = {key: [] for key in ('ids','pos','yaw','L','W','v','vxy','now','yaw_now')}
        branch_sources, kinds = [], []
        for index, key in enumerate(original['ids']):
            track = self.tracks[key]
            actor = track.agent
            # Observed current anchor and heading avoid the delayed direction
            # inherent in W1's trailing finite-difference velocity estimate.
            velocity = np.array([actor.v * math.cos(actor.yaw),
                                 actor.v * math.sin(actor.yaw)])
            anchor = np.array([actor.x, actor.y])
            variants = []
            if self.anchored:
                # Match W1's right-endpoint integration and decaying turn rate.
                # Only the initial speed/heading source changes to current GT.
                turn = track.yaw_rate if actor.v > 2.0 else 0.0
                heading = actor.yaw + turn * 2.0 * (1 - np.exp(-times / 2.0))
                speed = np.maximum(0.0, actor.v + track.accel * times)
                positions = anchor[None,:] + np.cumsum(
                    np.column_stack((speed*np.cos(heading),speed*np.sin(heading))) * dts[:,None],axis=0)
                if (not np.allclose(positions,original['pos'][index],rtol=0,atol=1e-9)
                        or not np.allclose(heading,original['yaw'][index],rtol=0,atol=1e-9)):
                    variants.append(('anchored_acceleration',positions,heading))
            if (self.braking_cv and track.accel < self.BRAKING_THRESHOLD
                    and actor.v > self.MOVING_THRESHOLD):
                variants.append(('cv_braking',anchor[None,:]+times[:,None]*velocity[None,:],
                                 np.full(len(times),actor.yaw)))
            for kind,positions,heading in variants:
                added['ids'].append(key + '#' + kind)
                added['pos'].append(positions)
                added['yaw'].append(heading)
                added['L'].append(actor.length)
                added['W'].append(actor.width)
                added['v'].append(actor.v)
                added['vxy'].append(velocity)
                added['now'].append(anchor)
                added['yaw_now'].append(actor.yaw)
                branch_sources.append(key)
                kinds.append(kind)

        if not kinds:
            return original
        original['ids'] = original['ids'] + added['ids']
        for key in ('pos','yaw','L','W','v','vxy','now','yaw_now'):
            original[key] = np.concatenate([original[key], np.asarray(added[key])], axis=0)
        original['headway_mask'] = np.concatenate([original['headway_mask'],np.zeros(len(kinds),dtype=bool)])
        original['branch_mask'] = ~original['headway_mask']
        original['source_ids'] += branch_sources
        original['hypothesis'] += kinds
        return original
