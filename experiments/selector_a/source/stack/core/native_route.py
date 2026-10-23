"""Exact nearest sampled-point distances used by route assembly."""
import numpy as np
from numba import njit


@njit
def _nearest_distances(queries, points):
    result = np.empty(len(queries))
    for qi in range(len(queries)):
        minimum = np.inf
        for pi in range(len(points)):
            dx = points[pi, 0] - queries[qi, 0]
            dy = points[pi, 1] - queries[qi, 1]
            minimum = min(minimum, dx * dx + dy * dy)
        result[qi] = np.sqrt(minimum)
    return result


def nearest_distances(queries, points):
    return _nearest_distances(np.ascontiguousarray(queries), np.ascontiguousarray(points))
