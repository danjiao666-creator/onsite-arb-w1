"""Control proposals: family never changes evaluation or safety ordering."""
from dataclasses import dataclass
import numpy as np


@dataclass
class Candidate:
    id: str
    family: str
    acceleration: np.ndarray
    steering_mode: int = 0
    steering_duration: float = 2.0
    steering_target: float = np.nan
    event: dict = None
    original_reference: bool = True


def generate(arb, ego, station, env_s, env_v, desired, previous=None, execution_context=None):
    _, _, accelerations = arb.rollout(ego, station, env_s, env_v, desired)
    result = [Candidate('main:%d' % i, 'main', values.copy())
              for i, values in enumerate(accelerations)]
    n = arb.n
    for acc in (-9.8, 0., 2.5):
        for mode in (0, 1, 2):
            result.append(Candidate('recovery:constant:%g:%d' % (acc, mode),
                                    'recovery', np.full(n, acc), mode))
    # Delayed braking is generated from every main profile instead of the
    # already-chosen nominal profile. Equal sequences are deduplicated below.
    for i, values in enumerate(accelerations):
        for delay in (.3, .6, 1.):
            result.append(Candidate('recovery:delay:%d:%g' % (i, delay), 'recovery',
                np.where(arb.t <= delay+1e-9, values, -9.8)))
    for acc in (4.5, 9.8):
        for duration in (.3, .6, 1.):
            result.append(Candidate('recovery:pulse:%g:%g' % (acc, duration), 'recovery',
                np.where(arb.t <= duration+1e-9, acc, -9.8)))
    # The inherited main family forbids positive acceleration after strong
    # braking. Provide explicit yielding/restart witnesses in the same model.
    for brake in (-4.,-8.):
        for duration in (.5,1.,2.):
            result.append(Candidate('recovery:restart:%g:%g'%(brake,duration),'recovery',
                np.where(arb.t<=duration+1e-9,brake,2.5)))
    if previous is not None:
        values, mode = previous[:2]
        duration=previous[2] if len(previous)>2 else 2.
        target=previous[3] if len(previous)>3 else np.nan
        shifted = np.r_[values[1:], values[-1]]
        result.append(Candidate('continuation', 'continuation', shifted, mode,
                                max(0.,duration-arb.cfg.dt),target,
                                event=previous[5] if len(previous)>5 else None,
                                original_reference=bool(previous[4]) if len(previous)>4 else True))
    if execution_context is not None:
        from .progress_profile import generate_progress
        path, measured, ctrl, lat = execution_context
        for peak, values in generate_progress(arb, ego, station, env_s, env_v, desired, path, measured, ctrl, lat):
            result.append(Candidate('progress:envelope:%g' % peak, 'envelope_progress', values, original_reference=False))
    unique, seen = [], set()
    for item in result:
        key = (item.steering_mode, item.acceleration.tobytes(),
               item.steering_duration if item.steering_mode else 0.,
               float(item.steering_target) if np.isfinite(item.steering_target) else None)
        if key not in seen:
            seen.add(key)
            unique.append(item)
    return unique
