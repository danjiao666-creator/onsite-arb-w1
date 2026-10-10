"""Optimistic traffic-free continuation using the existing route speed caps."""
import numpy as np
from numba import njit


@njit
def continuation(stations, speeds, goal, env_s, env_v, desired):
    result = np.zeros(len(stations))
    for index in range(len(stations)):
        station, speed, time = stations[index], speeds[index], 0.
        for _ in range(2000):
            if station >= goal:
                break
            cap = min(desired, np.interp(station, env_s, env_v))
            acceleration = 2.5 if speed < cap else max(-4., 1.5 * (cap - speed))
            next_speed = max(0., speed + acceleration * .1)
            distance = (speed + next_speed) * .05
            if distance > 0. and station + distance >= goal:
                time += .1 * (goal - station) / distance
                station = goal
                break
            station += distance
            speed = next_speed
            time += .1
        if station < goal:
            time += (goal - station) / max(speed, 1.)
        result[index] = time
    return result
