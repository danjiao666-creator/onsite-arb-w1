"""Bounded speed-profile rescue for the frozen ARB-W1 candidate generator.

The original 27 candidates are evaluated without modification. Only when all
of them predict collision do we evaluate twelve additional speed primitives.
Prediction, collision checks, soft costs, and controller contracts are inherited.
This fills holes in the candidate family; it does not certify future safety.
"""
import numpy as np

from .arbiter import Arbiter


class CapabilityArbiter(Arbiter):
    """Add short pulses, delayed stopping, and brake/restart speed profiles.

    A fixed 2 s switch forces even a small speed correction to persist for 2 s.
    The half-second and one-second switches permit bounded short corrections;
    the 3 s switch permits coasting until a later stop. Brake/restart removes
    the old prohibition on positive second acceleration after strong braking.

    Four intent primitives x three switch times = 12 additional candidates.
    This bounded set is a useful sparse cover, not a completeness claim.
    """

    rescue_switches = (0.5, 1.0, 3.0)
    rescue_primitives = ((0.8, 0.0), (-1.0, 0.0), (0.0, -3.0), (-4.0, 1.5))
    feasible_first = False

    def __init__(self, cfg=None):
        super().__init__(cfg)
        self._rescue_specs = None
        self._selected_index = None
        self.capability_stats = {'evaluated_frames': 0, 'selected_frames': 0,
                                 'rescued_feasible_frames': 0}

    def _select_index(self, cost, collision_times):
        feasible = np.isposinf(collision_times)
        if self.feasible_first and feasible.any():
            indices = np.flatnonzero(feasible)
            index = int(indices[np.argmin(cost[indices])])
        else:
            index = super()._select_index(cost, collision_times)
        self._selected_index = index
        return index

    def decide(self, ego, path, s0, e0, pred, *, v_des, v_goal_cap, goal_in,
               time_left, offset_fn=None, commit=True, extra_cost=0.0):
        kwargs = dict(v_des=v_des, v_goal_cap=v_goal_cap, goal_in=goal_in,
                      time_left=time_left, offset_fn=offset_fn, commit=False,
                      extra_cost=extra_cost)
        base = super().decide(ego, path, s0, e0, pred, **kwargs)
        selected = base
        self.last_capability = {'triggered': False, 'original_candidates': len(self.cands),
                                'additional_candidates': 0, 'selected_additional': False}
        if base.all_collide:
            original = self.cands
            specs = [(a1, a2, switch) for a1, a2 in self.rescue_primitives
                     for switch in self.rescue_switches]
            self.cands = [(a1, a2) for a1, a2, _ in specs]
            self._rescue_specs = specs
            try:
                rescue = super().decide(ego, path, s0, e0, pred, **kwargs)
                a1, a2, switch = specs[self._selected_index]
                rescue.label = f'{a1:+.1f}/{a2:+.1f}@{switch:.1f}s'
            finally:
                self.cands = original
                self._rescue_specs = None
            # Preserve frozen finite-cost ordering; ties retain the original.
            if self._rank(rescue) < self._rank(base):
                selected = rescue
            selected.all_collide = base.all_collide and rescue.all_collide
            self.last_capability.update(triggered=True, additional_candidates=len(specs),
                                        selected_additional=selected is rescue)
            if commit:
                self.capability_stats['evaluated_frames'] += 1
                self.capability_stats['selected_frames'] += int(selected is rescue)
                self.capability_stats['rescued_feasible_frames'] += int(not rescue.all_collide)
        if commit:
            self.commit(selected)
        return selected

    def _rank(self, plan):
        if self.feasible_first:
            return (not np.isposinf(plan.t_collide), plan.cost)
        return (False, plan.cost)

    def rollout(self, ego, s0, env_s, env_v, v_des):
        if self._rescue_specs is None:
            return super().rollout(ego, s0, env_s, env_v, v_des)
        c = self.cfg
        values = np.asarray(self._rescue_specs, dtype=float)
        first, second, switches = values.T
        count = len(values)
        S, V, A = (np.zeros((count, self.n)) for _ in range(3))
        s = np.full(count, float(s0))
        v = np.full(count, float(ego.v))
        a = np.full(count, float(ego.a))
        for k in range(self.n):
            target = np.where((k + 1) * c.dt <= switches, first, second)
            delta = np.where(target < -3.5, c.jerk_emerg, c.jerk) * c.dt
            a = a + np.clip(target - a, -delta, delta)
            cap = np.interp(s, env_s, env_v)
            a = np.where(v > cap, np.minimum(a, np.maximum(-4.0, 1.5 * (cap - v))), a)
            a = np.where(v + a * c.dt < 0.0, -v / c.dt, a)
            s = s + v * c.dt + 0.5 * a * c.dt * c.dt
            v = np.maximum(0.0, v + a * c.dt)
            S[:, k], V[:, k], A[:, k] = s, v, a
        return S, V, A


class FeasibleCapabilityArbiter(CapabilityArbiter):
    """Never trade a known collision against finite comfort/progress costs."""

    feasible_first = True


class FeasibleArbiter(Arbiter):
    """Selection-only ablation; the frozen candidate generator is unchanged."""

    def _select_index(self, cost, collision_times):
        indices = np.flatnonzero(np.isposinf(collision_times))
        if len(indices):
            return int(indices[np.argmin(cost[indices])])
        return super()._select_index(cost, collision_times)
