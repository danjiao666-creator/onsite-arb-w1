"""Restore the historical dynamic route-timing mechanism as an isolated candidate.

Source of the route policy: research/arbiter_execution_20260925/before_sources.zip,
stack/adapters/onsite_replay.py, lines 96-165. The implementation is explicit;
the archive is never loaded or executed at runtime. Candidate geometry is made
at initialization and after each lateral edge commitment, evaluated every frame,
and committed only when the chosen transition is about to start.

Two deliberate additions are geometry deduplication and a feature gate: only a
scene with at least two valid, distinct initial route variants uses this policy
and FeasibleArbiter. Other scenes call the frozen parent act unchanged. Once
activated, feasibility ordering stays active even after all route edges lock.
Prediction, longitudinal candidates, collision geometry, speed rules, and the
execution controller are inherited without alteration.
"""
import re

import numpy as np

from .onsite_replay import ArbiterPlanner, _read
from ..core.arbiter import Arbiter, EgoState
from ..core.capability_arbiter import FeasibleArbiter
from ..core.route import RouteError
from ..core.world import Agent


class DynamicRouteCapabilityPlanner(ArbiterPlanner):
    """Dynamic route timing plus feasibility, gated by actual route branching."""

    FRACS = (0.0, 0.25, 0.5, 0.75, 0.95)
    W_LATE = 1.5

    def init(self, scenario_info):
        # A reused planner must not carry the feature gate into another scene.
        if isinstance(self.arb, FeasibleArbiter):
            self.arb = Arbiter(self.arb.cfg)
        super().init(scenario_info)
        self._start = scenario_info['task_info']['startPos']
        text = _read(scenario_info['source_file']['xosc'])
        match = re.search(r'(?:h|heading)_init\s*=\s*(-?[\d.]+(?:[eE][-+]?\d+)?)', text)
        self._heading = float(match.group(1)) if match else None
        self._ego_xy = tuple(self._start)
        self.fixed = {}
        self.n_lock = 0
        self._dynamic_active = False
        self._route_diagnostics = {
            'policy': 'evaluate_each_frame_lock_at_transition_start',
            'ranking': 'predicted_feasibility_then_cost_plus_1p5f',
            'active': False,
            'reason': 'no_distinct_initial_route_branch',
            'initial_distinct_candidates': 0,
            'candidate_count': 1,
            'evaluations': [],
            'rejected_variants': [],
        }
        self._make_variants()
        count = len(self.cands)
        self._route_diagnostics['initial_distinct_candidates'] = count
        if count >= 2:
            original_arb = self.arb
            self.arb = FeasibleArbiter(original_arb.cfg)
            self.arb.last = original_arb.last
            self.arb.stats = original_arb.stats
            self._dynamic_active = True
            self._route_diagnostics.update(active=True, reason='distinct_route_branch')
        else:
            # Do not silently change the baseline route when only one generated
            # geometry survives validation/deduplication.
            self.cands = [(self.route, self.s, None, 0.0)]
            self._route_diagnostics['candidate_count'] = 1

    @staticmethod
    def _same_geometry(first, second):
        return (first.path.pts.shape == second.path.pts.shape
                and np.allclose(first.path.pts, second.path.pts, rtol=0, atol=1e-6))

    def _make_variants(self):
        """Historical first-pending-edge expansion, with geometry deduplication."""
        route = self.route
        self.cands = [(route, self.s, None, 0.0)]
        rejected = []
        if route.method != 'graph' or route.issues:
            self._route_diagnostics.update(candidate_count=1,
                                           rejected_variants=rejected)
            return
        pending = [edge for edge in self.rp.lateral_edges(route.lane_seq)
                   if edge not in self.fixed]
        if not pending:
            self._route_diagnostics.update(candidate_count=1,
                                           rejected_variants=rejected)
            return
        edge = pending[0]
        candidates = []
        for fraction in self.FRACS:
            try:
                variant = self.rp.variant(
                    route.lane_seq, self._start, self._heading, self.rect,
                    {**self.fixed, edge: fraction}, anchor=self._ego_xy)
            except (RouteError, ValueError, FloatingPointError) as exc:
                rejected.append({'fraction': fraction, 'reason': str(exc)})
                continue
            if variant.issues or edge not in variant.spans:
                rejected.append({'fraction': fraction,
                                 'reason': list(variant.issues) or 'missing_transition_span'})
                continue
            station, _ = variant.path.project(*self._ego_xy, s_hint=self.s)
            if variant.spans[edge][0] < station - 1.0:
                rejected.append({'fraction': fraction, 'reason': 'transition_start_passed'})
                continue
            if any(self._same_geometry(variant, previous[0]) for previous in candidates):
                rejected.append({'fraction': fraction, 'reason': 'duplicate_geometry'})
                continue
            candidates.append((variant, station, edge, fraction))
        if candidates:
            self.cands = candidates
        self._route_diagnostics.update(candidate_count=len(self.cands),
                                       rejected_variants=rejected)

    def act(self, observation):
        if not self._dynamic_active:
            return super().act(observation)
        ego = observation.ego_info
        timestamp = float(observation.test_info['t'])
        dt = float(observation.test_info.get('dt') or self.dt)
        agents = [Agent('{}/{}'.format(category, key), category,
                        float(obj.x), float(obj.y), float(obj.yaw), float(obj.v),
                        float(obj.length), float(obj.width))
                  for category, objects in observation.object_info.items()
                  for key, obj in objects.items()]
        self.world.update(timestamp, agents)
        self._ego_xy = (float(ego.x), float(ego.y))
        state = EgoState(float(ego.x), float(ego.y), float(ego.yaw), float(ego.v),
                         float(ego.a or 0.0), float(ego.length), float(ego.width))
        prediction = self.world.predict(self.arb.t, near_xy=(state.x, state.y),
                                        near_speed=state.v,
                                        near_extent=0.5 * float(np.hypot(state.length, state.width)))
        time_left = self.max_t - timestamp if self.max_t > 0.5 else 0.0
        best = None
        evaluations = []
        def evaluate(route, station, error, fraction):
            desired = self.v_des
            if time_left > 0.3:
                need = max(0.0, route.goal_in - station) / time_left
                desired = min(max(desired, need * 1.1),
                              max(self.v_des, 1.45 * self.limit))
            # Each geometry must retain the parent's target-box sampling rule.
            span = max(route.goal_out - route.goal_in, 0.5)
            goal_cap = max(3.0, (span - 0.4) / self.dt)
            plan = self.arb.decide(
                state, route.path, station, error, prediction,
                v_des=desired, v_goal_cap=goal_cap, goal_in=route.goal_in,
                time_left=time_left, commit=False, extra_cost=self.W_LATE * fraction,
                goal_rect=self.rect)
            return plan, goal_cap

        for index, (route, previous_s, edge, fraction) in enumerate(self.cands):
            station, error = route.path.project(ego.x, ego.y, s_hint=previous_s)
            self.cands[index] = (route, station, edge, fraction)
            if (edge is not None and route.spans[edge][0] < station - 1.0
                    and route is not self.route):
                continue
            plan, goal_cap = evaluate(route, station, error, fraction)
            collision = not np.isposinf(plan.t_collide)
            rank = (collision, not plan.on_time, plan.cost)
            evaluations.append({'edge': edge, 'fraction': fraction, 'cost': plan.cost,
                                'delay_cost': self.W_LATE * fraction,
                                'label': plan.label, 'a_cmd': plan.a_cmd,
                                'predicted_collision': bool(collision),
                                'all_collide': plan.all_collide})
            if best is None or rank < best[0]:
                best = (rank, plan, route, station, error, edge, fraction, goal_cap)
        # A rebuild can replace every candidate object. If their transition
        # starts are all behind us, continue the current geometry using this
        # frame's already updated world. Calling parent act would update twice.
        if best is None:
            route = self.route
            station, error = route.path.project(ego.x, ego.y, s_hint=self.s)
            pending = [i for i in route.spans if i not in self.fixed]
            edge = min(pending, key=lambda i: route.spans[i][0]) if pending else None
            fraction = 0.0
            plan, goal_cap = evaluate(route, station, error, fraction)
            best = ((not np.isposinf(plan.t_collide), not plan.on_time, plan.cost), plan, route,
                    station, error, edge, fraction, goal_cap)
            self.cands = [(route, station, edge, fraction)]
            self._route_diagnostics['current_route_recoveries'] = (
                self._route_diagnostics.get('current_route_recoveries', 0) + 1)
        _, plan, route, self.s, error, edge, fraction, self.v_goal_cap = best
        self.arb.commit(plan)
        self.route = route
        if edge is not None and self.s >= route.spans[edge][0] - 0.5:
            self.fixed[edge] = tuple(route.span_xy[edge][0])
            self.n_lock += 1
            self._make_variants()
        self._route_diagnostics.update(selected_at=timestamp, selected_edge=edge,
                                       selected_fraction=fraction,
                                       evaluations=evaluations,
                                       locked_edges=len(self.fixed), lock_count=self.n_lock)
        return self._execute(plan, route.path, self.s, error, state, dt)

    def diagnostics(self):
        result = super().diagnostics()
        result['dynamic_route_capability'] = dict(self._route_diagnostics)
        result['lc_fixed'] = dict(self.fixed)
        return result
