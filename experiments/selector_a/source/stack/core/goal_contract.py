"""Official ordered target predicate; never repair task input implicitly."""
import numpy as np


def goal_status(rect):
    try:
        points = np.asarray(rect, dtype=float)
    except (TypeError, ValueError):
        return 'invalid_shape'
    if points.shape != (2, 2):
        return 'invalid_shape'
    if not np.isfinite(points).all():
        return 'nonfinite'
    if np.any(points[0] > points[1]):
        return 'reversed_bounds'
    return 'valid'


def validate_goal(rect):
    status = goal_status(rect)
    if status != 'valid':
        raise ValueError('Invalid official target rectangle: ' + status)
    return np.asarray(rect, dtype=float).tolist()


def goal_contains(rect, x, y):
    x, y = np.broadcast_arrays(x, y)
    if goal_status(rect) != 'valid':
        return np.zeros(x.shape, dtype=bool)
    lower, upper = np.asarray(rect, dtype=float)
    return ((x >= lower[0]) & (x <= upper[0]) &
            (y >= lower[1]) & (y <= upper[1]))
