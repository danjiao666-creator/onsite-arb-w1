"""Finite speed-envelope proposals with the unchanged native execution law."""
import numpy as np
from numba import njit
from .native_execution import execute, interpolate


@njit
def _profiles(initial, points, cumulative, headings, curvature, station0, memory0,
              dt, length, ke, kpsi, preview, acc_min, acc_max, steer_max, steer_rate,
              wheelbase_factor, env_s, env_v, desired, count):
    peaks = np.array([2.5, 4.5, 9.8])
    proposals = np.zeros((3, count))
    for pi in range(3):
        current = initial.copy()
        memory, station = memory0, station0
        for k in range(count):
            cap = min(desired, interpolate(station, env_s, env_v))
            target = min(peaks[pi], max(-9.8, 1.5*(cap-current[3])))
            jerk = 20. if target < -3.5 else 6.
            command = current[4]+min(max(target-current[4], -jerk*dt), jerk*dt)
            control = np.array([[command]])
            states, controls, stations = execute(current, control, points, cumulative,
                headings, curvature, station, memory, dt, length, ke, kpsi, preview,
                acc_min, acc_max, steer_max, steer_rate, wheelbase_factor,
                np.zeros(1, dtype=np.int64), np.zeros(1), np.full(1, np.nan))
            proposals[pi, k] = command
            current = states[0, 1].copy()
            station = stations[0, 0]
            memory = controls[0, 0, 1]
    return proposals


def generate_progress(arb, ego, station, env_s, env_v, desired, path, measured, ctrl, lat):
    limits = ctrl.lim
    initial = np.array([ego.x, ego.y, ego.yaw, ego.v, ego.a, measured])
    proposals = _profiles(initial, path.pts, path.cum, path._hd_unwrap, path._curv,
        station, ctrl.prev_steer, arb.cfg.dt, ego.length, lat.k_e, lat.k_psi, lat.preview_s,
        limits.acc_min, limits.acc_max, limits.steer_max, limits.steer_rate,
        limits.wheelbase_factor, env_s, env_v, desired, arb.n)
    return list(zip((2.5, 4.5, 9.8), proposals))
