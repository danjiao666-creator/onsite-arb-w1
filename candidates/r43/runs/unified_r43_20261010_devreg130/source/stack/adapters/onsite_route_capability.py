"""One-factor route timing experiment, using the existing planner unchanged.

Geometry is prepared in init. On the first real Observation, at most three
routes (original, earliest, later) share one WorldModel update/prediction and the
existing longitudinal candidate evaluation. The chosen route is then locked;
subsequent calls use ArbiterPlanner.act without route search or switching.

Only graph routes that already require a lateral transition are eligible. This
does not add overtaking lanes, road permissions, new prediction, controllers,
weights, or longitudinal candidates. All lateral edges use the same fraction,
so the search stays bounded rather than multiplying choices at each edge.

Limits: first-frame work includes up to three arbiter evaluations; its latency
must be measured. First-frame history is sparse and later traffic may invalidate
the route preference; locking avoids switching but is not a safety guarantee.
"""
import math
import re

import numpy as np

from .onsite_replay import ArbiterPlanner, _read
from ..core.arbiter import EgoState
from ..core.capability_arbiter import FeasibleArbiter
from ..core.route import RouteError
from ..core.world import Agent


class RouteCapabilityPlanner(ArbiterPlanner):
    """Original Arbiter cost ordering, with one observed route-timing choice."""

    route_fractions = (0.0, 0.9)

    def init(self, scenario_info):
        super().init(scenario_info)
        self._route_candidates = [("original", None, self.route)]
        self._route_choice_pending = False
        self._route_diagnostics = {
            "policy": "first_observation_then_lock",
            "ranking": "original_cost",
            "selected": "original",
            "selected_fraction": None,
            "evaluations": [],
            "rejected_variants": [],
            "candidate_count": 1,
        }
        if self.route.method != "graph" or self.route.issues:
            self._route_diagnostics["reason"] = "original_route_has_no_verified_graph_geometry"
            return
        edges = self.rp.lateral_edges(self.route.lane_seq)
        if not edges:
            self._route_diagnostics["reason"] = "no_existing_lateral_edge"
            return
        task = scenario_info["task_info"]
        start = task["startPos"]
        text = _read(scenario_info["source_file"]["xosc"])
        match = re.search(r"(?:h|heading)_init\s*=\s*(-?[\d.]+(?:[eE][-+]?\d+)?)", text)
        heading = float(match.group(1)) if match else None
        for fraction in self.route_fractions:
            try:
                route = self.rp.variant(self.route.lane_seq, start, heading, self.rect,
                                        {edge: fraction for edge in edges})
            except (RouteError, ValueError, FloatingPointError) as exc:
                self._route_diagnostics["rejected_variants"].append({
                    "fraction": fraction, "reason": str(exc)})
                continue
            if route.issues:
                self._route_diagnostics["rejected_variants"].append({
                    "fraction": fraction, "reason": list(route.issues)})
                continue
            duplicate = any(route.path.pts.shape == previous.path.pts.shape
                            and np.allclose(route.path.pts, previous.path.pts, rtol=0, atol=1e-6)
                            for _, _, previous in self._route_candidates)
            if duplicate:
                self._route_diagnostics["rejected_variants"].append({
                    "fraction": fraction, "reason": "same_geometry_as_existing_candidate"})
                continue
            self._route_candidates.append(("fraction_{:.1f}".format(fraction), fraction, route))
        self._route_choice_pending = len(self._route_candidates) > 1
        self._route_diagnostics["candidate_count"] = len(self._route_candidates)
        self._route_diagnostics["reason"] = ("awaiting_first_observation" if self._route_choice_pending
                                             else "no_distinct_valid_variant")

    def _route_rank(self, plan):
        return (plan.cost,)

    def _route_delay_cost(self, fraction):
        return 0.0

    def _route_speed_parameters(self, route):
        # Same formulas as ArbiterPlanner.init, recalculated for each geometry.
        span = max(route.goal_out - route.goal_in, 0.5)
        goal_cap = max(3.0, (span - 0.4) / self.dt)
        desired = self.limit
        distance = max(0.0, route.goal_in - route.s0)
        if self.max_t > 0.5 and distance / self.max_t * 1.1 > desired:
            desired = min(distance / self.max_t * 1.1, 1.45 * self.limit)
        return desired, goal_cap

    def act(self, observation):
        if not self._route_choice_pending:
            return super().act(observation)
        ego = observation.ego_info
        timestamp = float(observation.test_info["t"])
        dt = float(observation.test_info.get("dt") or self.dt)
        agents = [Agent("{}/{}".format(category, key), category, float(obj.x), float(obj.y),
                        float(obj.yaw), float(obj.v), float(obj.length), float(obj.width))
                  for category, objects in observation.object_info.items() for key, obj in objects.items()]
        self.world.update(timestamp, agents)
        state = EgoState(float(ego.x), float(ego.y), float(ego.yaw), float(ego.v), float(ego.a or 0.0),
                         float(ego.length), float(ego.width))
        prediction = self.world.predict(self.arb.t, near_xy=(state.x, state.y))
        time_left = self.max_t - timestamp if self.max_t > 0.5 else 0.0
        best = None
        evaluations = []
        for name, fraction, route in self._route_candidates:
            station, error = route.path.project(state.x, state.y, s_hint=route.s0)
            desired_base, goal_cap = self._route_speed_parameters(route)
            desired = desired_base
            if time_left > 0.3:
                need = max(0.0, route.goal_in - station) / time_left
                desired = min(max(desired, need * 1.1), max(desired_base, 1.45 * self.limit))
            plan = self.arb.decide(state, route.path, station, error, prediction, v_des=desired,
                                   v_goal_cap=goal_cap, goal_in=route.goal_in, time_left=time_left,
                                   commit=False)
            delay_cost = self._route_delay_cost(fraction)
            plan.cost += delay_cost
            rank = self._route_rank(plan)
            evaluations.append({"name": name, "fraction": fraction, "cost": plan.cost,
                                "delay_cost": delay_cost,
                                "label": plan.label, "a_cmd": plan.a_cmd,
                                "predicted_collision": math.isfinite(plan.t_collide),
                                "all_collide": plan.all_collide})
            if best is None or rank < best[0]:
                best = (rank, name, fraction, route, station, error, desired_base, goal_cap, plan)
        _, name, fraction, route, station, error, desired_base, goal_cap, plan = best
        self.route, self.s = route, station
        self.v_des, self.v_goal_cap = desired_base, goal_cap
        self.arb.commit(plan)
        self._route_choice_pending = False
        self._route_diagnostics.update(reason="selected_and_locked", selected=name,
                                       selected_fraction=fraction, selected_at=timestamp,
                                       evaluations=evaluations)
        # Commit and execute exactly the plan that was evaluated on this route.
        return self._execute(plan, route.path, station, error, state, dt)

    def diagnostics(self):
        result = super().diagnostics()
        result["route_capability"] = dict(self._route_diagnostics)
        return result


class FeasibleRouteCapabilityPlanner(RouteCapabilityPlanner):
    """FeasibleArbiter within routes; the same feasibility order across routes."""

    def __init__(self):
        super().__init__()
        self.arb = FeasibleArbiter()

    def init(self, scenario_info):
        super().init(scenario_info)
        self._route_diagnostics["ranking"] = "predicted_feasibility_then_original_cost"

    def _route_rank(self, plan):
        return (math.isfinite(plan.t_collide), plan.cost)


class RouteCommitmentPlanner(RouteCapabilityPlanner):
    """Restore the existing 1.5-second late-maneuver cost when ranking routes.

    The parent graph builder uses fraction 0.1 for its default lateral edges.
    The cost reflects spending the available lane-change corridor before acting;
    it is inherited from the earlier multiroute design, not fitted to outcomes.
    """

    def _route_delay_cost(self, fraction):
        return 1.5 * (0.1 if fraction is None else fraction)
