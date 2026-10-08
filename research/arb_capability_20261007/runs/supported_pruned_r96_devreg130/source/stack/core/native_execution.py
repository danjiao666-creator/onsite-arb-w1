"""Compiled replay execution, preserving control and rounding order.

No fastmath: projection ties, actuator memory and millimetre rounding are part
of the contract. Compilation is warmed during init, outside act timing.
"""
import numpy as np
from numba import njit


@njit
def clip(value, lower, upper):
    return min(max(value, lower), upper)


@njit
def interpolate(station, cumulative, values):
    if station <= cumulative[0]:
        return values[0]
    if station >= cumulative[-1]:
        return values[-1]
    index = np.searchsorted(cumulative, station, side='right') - 1
    return values[index] + (values[index + 1] - values[index]) / (cumulative[index + 1] - cumulative[index]) * (station - cumulative[index])


@njit
def project(x, y, hint, points, cumulative):
    lower = max(0, np.searchsorted(cumulative, hint - 15.) - 1)
    upper = min(len(points) - 1, np.searchsorted(cumulative, hint + 60.) + 1)
    best_distance = np.inf
    station, lateral = 0., 0.
    for index in range(lower, upper):
        dx, dy = points[index + 1, 0] - points[index, 0], points[index + 1, 1] - points[index, 1]
        length2 = max(dx * dx + dy * dy, 1e-9)
        qx, qy = x - points[index, 0], y - points[index, 1]
        fraction = clip((qx * dx + qy * dy) / length2, 0., 1.)
        ex, ey = qx - fraction * dx, qy - fraction * dy
        distance = ex * ex + ey * ey
        if distance < best_distance:
            best_distance = distance
            station = cumulative[index] + fraction * np.sqrt(length2)
            heading = np.arctan2(dy, dx)
            lateral = -ex * np.sin(heading) + ey * np.cos(heading)
    return station, lateral


@njit
def execute(initial, accelerations, points, cumulative, headings, curvature,
            station0, memory0, dt, length, ke, kpsi, preview,
            acc_min, acc_max, steer_max, steer_rate, wheelbase_factor, steering_modes):
    count, steps = accelerations.shape
    states = np.empty((count, steps + 1, 6))
    commands = np.empty((count, steps, 2))
    for candidate in range(count):
        states[candidate, 0] = initial
        memory, hint = memory0, station0
        for index in range(steps):
            x, y, yaw, speed, previous_acc, measured = states[candidate, index]
            station, lateral = project(x, y, hint, points, cumulative)
            hint = station
            ahead = station + max(speed, 1.) * preview
            heading = (interpolate(ahead, cumulative, headings) + np.pi) % (2 * np.pi) - np.pi
            kappa = interpolate(ahead, cumulative, curvature)
            desired = heading - np.arctan(ke * lateral / (max(speed, 0.) + 2.))
            error = (desired - yaw + np.pi) % (2 * np.pi) - np.pi
            yaw_rate = speed * kappa + kpsi * error
            if steering_modes[candidate] != 0:
                target = initial[5] if steering_modes[candidate] == 1 else 0.
                yaw_rate = speed * wheelbase_factor / length * np.tan(target)
            steer = np.arctan(yaw_rate * (length / wheelbase_factor) / speed) if speed > .3 else memory
            steer = clip(steer, -steer_max, steer_max)
            rate = steer_rate * dt * .98
            steer = clip(steer, memory - rate, memory + rate)
            memory = steer
            command_acc = clip(accelerations[candidate, index], acc_min, acc_max)
            acc = -speed / dt if speed + command_acc * dt < 0. else previous_acc + clip(command_acc - previous_acc, -49 * dt, 49 * dt)
            acc = clip(acc, -9.8, 9.8)
            rot = clip(measured + clip(steer - measured, -1.397 * dt, 1.397 * dt), -.699, .699)
            states[candidate, index + 1, 0] = np.round(x + speed * np.cos(yaw) * dt, 3)
            states[candidate, index + 1, 1] = np.round(y + speed * np.sin(yaw) * dt, 3)
            states[candidate, index + 1, 2] = np.round((yaw + speed / length * 1.7 * np.tan(rot) * dt) % (2 * np.pi), 3)
            states[candidate, index + 1, 3] = np.round(max(0., speed + acc * dt), 3)
            states[candidate, index + 1, 4] = np.round(acc, 3)
            states[candidate, index + 1, 5] = np.round(rot, 3)
            commands[candidate, index, 0] = command_acc
            commands[candidate, index, 1] = steer
    return states, commands


def rollout(initial, accelerations, path, station, controller, lateral, dt, length, steering_modes=None):
    lim = controller.lim
    modes = np.zeros(len(accelerations), dtype=np.int64) if steering_modes is None else steering_modes
    return execute(initial, np.ascontiguousarray(accelerations), path.pts, path.cum,
                   path._hd_unwrap, path._curv, float(station), controller.prev_steer,
                   float(dt), float(length), lateral.k_e, lateral.k_psi, lateral.preview_s,
                   lim.acc_min, lim.acc_max, lim.steer_max, lim.steer_rate, lim.wheelbase_factor, modes)


@njit
def project_states(states, station0, points, cumulative):
    stations = np.empty(states.shape[:2])
    for candidate in range(len(states)):
        hint = station0
        for index in range(states.shape[1]):
            hint, _ = project(states[candidate, index, 0], states[candidate, index, 1],
                              hint, points, cumulative)
            stations[candidate, index] = hint
    return stations
