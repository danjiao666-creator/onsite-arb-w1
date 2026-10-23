"""Exact five-circle minimum without materializing the 25-pair tensor."""
import numpy as np
from numba import njit


@njit
def circle_distance(ego_circles, actor_circles, ego_radius, actor_radii):
    count, times, circles = ego_circles.shape[:3]
    actors = len(actor_circles)
    result = np.empty((count, actors, times))
    for candidate in range(count):
        for actor in range(actors):
            for ti in range(times):
                minimum = np.inf
                for ei in range(circles):
                    for ai in range(actor_circles.shape[2]):
                        dx = ego_circles[candidate, ti, ei, 0] - actor_circles[actor, ti, ai, 0]
                        dy = ego_circles[candidate, ti, ei, 1] - actor_circles[actor, ti, ai, 1]
                        minimum = min(minimum, dx * dx + dy * dy)
                result[candidate, actor, ti] = np.sqrt(minimum) - (ego_radius + actor_radii[actor])
    return result
