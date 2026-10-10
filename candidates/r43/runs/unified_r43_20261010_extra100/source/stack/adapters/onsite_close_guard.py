"""Compiled actuator/rectangle recovery with explicit checked and failed states."""
from copy import deepcopy
import numpy as np

from .onsite_cadence import CadencePlanner
from ..core.arbiter import _circles, _radius, _obb_separation
from ..core.native_execution import rollout
from ..core.goal_contract import goal_contains
from ..core.replay_contract import checked_action


class CloseGuardPlanner(CadencePlanner):
    def __init__(self):
        super().__init__()
        self._guard_status = {'status': 'not_run', 'checked': False, 'failed': False}
        self._guard_counts = {}

    def init(self, scenario_info):
        self._guard_status = {'status': 'not_run', 'checked': False, 'failed': False}
        self._guard_counts = {}
        super().init(scenario_info)
        self._pulse_recovery_until = -np.inf
        rollout(np.zeros(6), np.zeros((10, 10)), self.route.path, self.s,
                self.ctrl, self.lat, self.dt, 4.5, np.zeros(10, dtype=np.int64))

    def _execute(self, plan, path, station, error, ego, dt):
        self._guard_status = {
            't': float(self.world.t), 'status': 'assessing', 'checked': False, 'failed': False,
            'scope': 'execution_model_on_checked_grid', 'horizon': 2., 'dt': float(dt),
            'nominal_first_hit': None, 'candidate_count': 0, 'collision_free_count': None,
            'available_clear_count': None, 'eligible_count': None, 'chosen_index': None,
            'pulse_allowed': None, 'recovering': None, 'returned_action_status': 'unverified',
        }
        actors = list(self.world.tracks.values())
        if not actors:
            return self._nominal_return('not_checked_no_actors', plan, path, station, error, ego, dt)
        now = np.array([[tr.agent.x, tr.agent.y] for tr in actors])
        yaw = np.array([tr.agent.yaw for tr in actors])
        length = np.array([tr.agent.length for tr in actors])
        width = np.array([tr.agent.width for tr in actors])
        actor_circles = _circles(now[:, 0], now[:, 1], yaw, length)
        ego_circles = _circles(np.array([ego.x]), np.array([ego.y]), np.array([ego.yaw]), ego.length)
        gap = np.linalg.norm(actor_circles[:, :, None] - ego_circles[0][None, None], axis=-1).min((1, 2))
        gap -= _radius(length, width) + _radius(ego.length, ego.width)
        if not np.any(gap <= 15.):
            return self._nominal_return('not_checked_outside_trigger', plan, path, station, error, ego, dt)
        count = max(1, int(round(2. / dt)))
        times = np.arange(1, count + 1) * dt
        self._guard_status['horizon'] = float(times[-1])
        pred = self.world.predict(times, near_xy=(ego.x, ego.y), near_speed=ego.v,
                                  near_extent=.5 * np.hypot(ego.length, ego.width))
        if not pred['ids']:
            return self._nominal_return('not_checked_no_predictions', plan, path, station, error, ego, dt)
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
        inside = goal_contains(self.rect, states[:, 1:, 0], states[:, 1:, 1])
        terminal = np.where(inside.any(-1), inside.argmax(-1), count - 1)
        valid = np.arange(count)[None] <= terminal[:, None]
        hit = (clear <= 0) & valid
        self._guard_status.update(checked=True, candidate_count=len(profiles),
            collision_free_count=int((~hit.any(-1)).sum()))
        if not hit[0].any():
            self._guard_status['returned_action_status'] = 'execution_model_checked_on_grid'
            return self._nominal_return('nominal_clear', plan, path, station, error, ego, dt)
        # High-acceleration escape is an emergency action. Longer-range
        # forecast conflicts retain the ordinary controls while we reobserve.
        recovering = self.world.t < self._pulse_recovery_until
        pulse_allowed = recovering or times[np.flatnonzero(hit[0])[0]] <= 1. + 1e-9
        self._guard_status.update(nominal_first_hit=float(times[np.flatnonzero(hit[0])[0]]),
                                  recovering=bool(recovering), pulse_allowed=bool(pulse_allowed))
        if not pulse_allowed:
            hit[13:] = True
        clear_candidates = np.flatnonzero(~hit.any(-1))
        self._guard_status['available_clear_count'] = int(len(clear_candidates))
        if not len(clear_candidates):
            reason = ('failed_pulse_gate' if self._guard_status['collision_free_count']
                      else 'failed_no_collision_free_candidate')
            self._guard_status.update(failed=True, returned_action_status='risky_nominal_fallback')
            return self._nominal_return(reason, plan, path, station, error, ego, dt)
        max_error = np.full(len(profiles), abs(error))
        for ci in clear_candidates:
            hint = station
            for k in range(int(terminal[ci]) + 1):
                hint, lateral = path.project(states[ci, k + 1, 0], states[ci, k + 1, 1], s_hint=hint)
                max_error[ci] = max(max_error[ci], abs(lateral))
        eligible = np.flatnonzero(~hit.any(-1) & (max_error <= max(abs(error) + .5, 1.5)))
        self._guard_status['eligible_count'] = int(len(eligible))
        if not len(eligible):
            self._guard_status.update(failed=True, returned_action_status='risky_nominal_fallback')
            return self._nominal_return('failed_lateral_eligibility', plan, path, station, error, ego, dt)
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
        self._guard_status.update(chosen_index=int(chosen), returned_action_status='execution_model_checked_on_grid')
        return self._record_guard_return('override', action, ego, dt)

    def _nominal_return(self, status, plan, path, station, error, ego, dt):
        action = super()._execute(plan, path, station, error, ego, dt)
        return self._record_guard_return(status, action, ego, dt)

    def _record_guard_return(self, status, action, ego, dt):
        self._guard_status['status'] = status
        self._guard_status['commanded_action'] = [float(value) for value in action]
        measured = getattr(self, '_guard_measured_steer', 0.)
        checked = checked_action(dt, ego.v, ego.a, measured, float(action[0]), float(action[1]))
        self._guard_status['expected_checked_action'] = [float(value) for value in checked]
        self._guard_counts[status] = self._guard_counts.get(status, 0) + 1
        if self._guard_status['failed']:
            self.arb.stats['close_guard_failures'] = self.arb.stats.get('close_guard_failures', 0) + 1
        return action

    def safety_diagnostics(self):
        """Per-frame JSON-safe status; recorder must capture after every act."""
        plan = self.arb.last
        return {'close_guard': deepcopy(self._guard_status),
                'current_overlap': {
                    'circle_ids': list(plan.current_circle_overlap_ids) if plan else [],
                    'rectangle_ids': list(plan.current_rectangle_overlap_ids) if plan else [],
                    'future_occupancy_suppressed': False}}

    def diagnostics(self):
        result = super().diagnostics()
        result.update(self.safety_diagnostics())
        result['close_guard_counts'] = dict(self._guard_counts)
        return result

    def act(self, observation):
        self._guard_measured_steer = float(observation.ego_info.rot or 0.)
        return super().act(observation)
