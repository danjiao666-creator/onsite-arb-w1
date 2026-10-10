"""Separate a nominal proposal from the actual execution decision and feedback."""
from dataclasses import dataclass, asdict
import numpy as np


def finite(value):
    return float(value) if np.isfinite(value) else None


@dataclass
class ExecutionDecision:
    t: float
    candidate_id: str
    family: str
    action: list
    checked_action: list
    expected_next_state: list
    status: str
    route_id: str
    nominal_action: float
    source: str = 'unified'

    def snapshot(self):
        return asdict(self)


def feedback(previous, ego):
    if previous is None or not previous.expected_next_state:
        return {'status': 'initial'}
    actual = np.array([ego.x, ego.y, ego.yaw, ego.v, ego.a, ego.rot], float)
    error = actual - np.asarray(previous.expected_next_state)
    error[2] = (error[2] + np.pi) % (2*np.pi) - np.pi
    return {'status': 'matched' if np.max(np.abs(error)) <= .00101 else 'mismatch',
            'previous_candidate': previous.candidate_id, 'state_error': error.tolist()}


def next_state(ego, checked, dt):
    acc, rot = checked
    return [round(ego.x + ego.v*np.cos(ego.yaw)*dt, 3),
            round(ego.y + ego.v*np.sin(ego.yaw)*dt, 3),
            round((ego.yaw + ego.v/ego.length*1.7*np.tan(rot)*dt) % (2*np.pi), 3),
            round(max(0., ego.v+acc*dt), 3), round(acc, 3), round(rot, 3)]
