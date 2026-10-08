"""Compiled actuator/rectangle recovery for actors hidden by circle overlap."""
import numpy as np

from .onsite_cadence import CadencePlanner
from ..core.arbiter import _circles, _radius, _obb_separation
from ..core.native_execution import rollout


class CloseGuardPlanner(CadencePlanner):
    def init(self, scenario_info):
        super().init(scenario_info)
        self._pulse_recovery_until = -np.inf
        rollout(np.zeros(6), np.zeros((10, 10)), self.route.path, self.s,
                self.ctrl, self.lat, self.dt, 4.5, np.zeros(10, dtype=np.int64))

    def _execute(self, plan, path, station, error, ego, dt):
        actors = list(self.world.tracks.values())
        if not actors:
            return super()._execute(plan, path, station, error, ego, dt)
        now = np.array([[tr.agent.x, tr.agent.y] for tr in actors])
        yaw = np.array([tr.agent.yaw for tr in actors])
        length = np.array([tr.agent.length for tr in actors])
        width = np.array([tr.agent.width for tr in actors])
        actor_circles = _circles(now[:, 0], now[:, 1], yaw, length)
        ego_circles = _circles(np.array([ego.x]), np.array([ego.y]), np.array([ego.yaw]), ego.length)
        gap = np.linalg.norm(actor_circles[:, :, None] - ego_circles[0][None, None], axis=-1).min((1, 2))
        gap -= _radius(length, width) + _radius(ego.length, ego.width)
        if not np.any(gap <= 15.):
            return super()._execute(plan, path, station, error, ego, dt)
        count = max(1, int(round(2. / dt)))
        times = np.arange(1, count + 1) * dt
        pred = self.world.predict(times, near_xy=(ego.x, ego.y), near_speed=ego.v,
                                  near_extent=.5 * np.hypot(ego.length, ego.width))
        profiles = [(None, 0)] + [(acc, mode) for acc in (-9.8, 0., 2.5) for mode in (0, 1, 2)]
        accelerations = np.array([plan.a[:count] if acc is None else np.full(count, acc)
                                  for acc, _ in profiles])
        # Preserve an already safe immediate action when a later brake can
        # remove the short-window conflict; constant controls cannot express it.
        delayed = []
        for delay in (.3, .6, 1.):
            sequence = plan.a[:count].copy()
            sequence[times > delay + 1e-9] = -9.8
            delayed.append(sequence)
            profiles.append((None, 0))
        accelerations = np.vstack((accelerations, delayed))
        pulses = []
        for acceleration in (4.5, 9.8):
            for duration in (.3, .6, 1.):
                sequence = np.where(times <= duration + 1e-9, acceleration, -9.8)
                pulses.append(sequence)
                profiles.append((acceleration, 0))
        accelerations = np.vstack((accelerations, pulses))
        modes = np.array([mode for _, mode in profiles], dtype=np.int64)
        initial = np.array([ego.x, ego.y, ego.yaw, ego.v, ego.a, self._guard_measured_steer])
        states, controls = rollout(initial, accelerations, path, station, self.ctrl,
                                   self.lat, dt, ego.length, modes)
        separation = _obb_separation(states[:, 1:, 0][:, None], states[:, 1:, 1][:, None],
            states[:, 1:, 2][:, None], ego.length, ego.width,
            pred['pos'][None, :, :, 0], pred['pos'][None, :, :, 1], pred['yaw'][None],
            pred['L'][None, :, None], pred['W'][None, :, None])
        observed = [self.world.tracks[key].agent for key in pred['ids']]
        current_heading = np.array([actor.yaw for actor in observed])
        distance = np.array([actor.v for actor in observed])[:, None] * times[None]
        current_x = pred['now'][:, 0, None] + distance * np.cos(current_heading[:, None])
        current_y = pred['now'][:, 1, None] + distance * np.sin(current_heading[:, None])
        current_separation = _obb_separation(
            states[:, 1:, 0][:, None], states[:, 1:, 1][:, None], states[:, 1:, 2][:, None],
            ego.length, ego.width, current_x[None], current_y[None], current_heading[None, :, None],
            pred['L'][None, :, None], pred['W'][None, :, None])
        separation = np.minimum(separation, current_separation)
        acceleration_distance = .5 * 1.5 * times[None] ** 2
        accelerating_x = current_x + acceleration_distance * np.cos(current_heading[:, None])
        accelerating_y = current_y + acceleration_distance * np.sin(current_heading[:, None])
        accelerating_separation = _obb_separation(
            states[:, 1:, 0][:, None], states[:, 1:, 1][:, None], states[:, 1:, 2][:, None],
            ego.length, ego.width, accelerating_x[None], accelerating_y[None], current_heading[None, :, None],
            pred['L'][None, :, None], pred['W'][None, :, None])
        supported = np.array([self.world.tracks[key].accel >= 0. for key in pred['ids']])
        accelerating_separation = np.where(supported[None, :, None], accelerating_separation, np.inf)
        separation = np.minimum(separation, accelerating_separation)
        clear = separation.min(axis=1)
        lower, upper = np.asarray(self.rect).min(0), np.asarray(self.rect).max(0)
        inside = ((states[:, 1:, :2] >= lower) & (states[:, 1:, :2] <= upper)).all(-1)
        terminal = np.where(inside.any(-1), inside.argmax(-1), count - 1)
        valid = np.arange(count)[None] <= terminal[:, None]
        hit = (clear <= 0) & valid
        if not hit[0].any():
            return super()._execute(plan, path, station, error, ego, dt)
        # High-acceleration escape is an emergency action. Longer-range
        # forecast conflicts retain the ordinary controls while we reobserve.
        recovering = self.world.t < self._pulse_recovery_until
        pulse_allowed = recovering or times[np.flatnonzero(hit[0])[0]] <= 1. + 1e-9
        if not pulse_allowed:
            hit[13:] = True
        clear_candidates = np.flatnonzero(~hit.any(-1))
        if not len(clear_candidates):
            return super()._execute(plan, path, station, error, ego, dt)
        max_error = np.full(len(profiles), abs(error))
        for ci in clear_candidates:
            hint = station
            for k in range(int(terminal[ci]) + 1):
                hint, lateral = path.project(states[ci, k + 1, 0], states[ci, k + 1, 1], s_hint=hint)
                max_error[ci] = max(max_error[ci], abs(lateral))
        eligible = np.flatnonzero(~hit.any(-1) & (max_error <= max(abs(error) + .5, 1.5)))
        if not len(eligible):
            return super()._execute(plan, path, station, error, ego, dt)
        nominal = controls[0, 0]
        clearance = np.where(valid, clear, np.inf).min(-1)
        chosen = min(eligible, key=lambda ci: (abs(controls[ci, 0, 1] - nominal[1]),
                                             abs(controls[ci, 0, 0] - nominal[0]), -clearance[ci]))
        acc, mode = profiles[chosen]
        if chosen >= 13 and not recovering:
            # Retain the recovery family for one checked horizon. Every
            # control is still re-evaluated against the latest observation.
            self._pulse_recovery_until = self.world.t + 2.
        yaw_rate = self.lat.yaw_rate(path, station, error, ego.yaw, ego.v)
        if mode:
            target = self._guard_measured_steer if mode == 1 else 0.
            yaw_rate = ego.v * self.ctrl.lim.wheelbase_factor / ego.length * np.tan(target)
        action = self.ctrl.command(float(plan.a_cmd if acc is None else acc), yaw_rate,
                                   ego.v, ego.length, dt)
        self.arb.stats['close_guard_overrides'] = self.arb.stats.get('close_guard_overrides', 0) + 1
        return action

    def act(self, observation):
        self._guard_measured_steer = float(observation.ego_info.rot or 0.)
        return super().act(observation)
