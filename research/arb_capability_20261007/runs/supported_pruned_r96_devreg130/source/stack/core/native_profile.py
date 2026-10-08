"""Compile the existing longitudinal profile loop without changing its algebra."""
import numpy as np
from numba import njit

from .arbiter import Arbiter
from .capability_arbiter import FeasibleArbiter


@njit
def profile(speed, acceleration, station, env_s, env_v, desired,
            cruise, first, second, steps, dt, switch, jerk, emergency_jerk):
    count = len(first)
    S, V, A = np.zeros((count, steps)), np.zeros((count, steps)), np.zeros((count, steps))
    s, v, a = np.full(count, station), np.full(count, speed), np.full(count, acceleration)
    for index in range(steps):
        tk = (index + 1) * dt
        target = np.where(cruise, np.clip(.8 * (desired - v), -2., 1.6),
                          first if tk <= switch else second)
        delta = np.where(target < -3.5, emergency_jerk, jerk) * dt
        a = a + np.minimum(np.maximum(target - a, -delta), delta)
        cap = np.interp(s, env_s, env_v)
        a = np.where(v > cap, np.minimum(a, np.maximum(-4., 1.5 * (cap - v))), a)
        a = np.where(v + a * dt < 0., -v / dt, a)
        s = s + v * dt + .5 * a * dt * dt
        v = np.maximum(0., v + a * dt)
        S[:, index], V[:, index], A[:, index] = s, v, a
    return S, V, A


class NativeProfileMixin:
    def rollout(self, ego, s0, env_s, env_v, v_des):
        cfg = self.cfg
        cruise = np.array([candidate[0] == 'cruise' for candidate in self.cands])
        first = np.array([0. if candidate[0] == 'cruise' else candidate[0] for candidate in self.cands])
        second = np.array([0. if candidate[0] == 'cruise' else candidate[1] for candidate in self.cands])
        return profile(float(ego.v), float(ego.a), float(s0), env_s, env_v, float(v_des),
                       cruise, first, second, self.n, cfg.dt, cfg.t_switch, cfg.jerk, cfg.jerk_emerg)


class NativeArbiter(NativeProfileMixin, Arbiter):
    pass


class NativeFeasibleArbiter(NativeProfileMixin, FeasibleArbiter):
    pass
