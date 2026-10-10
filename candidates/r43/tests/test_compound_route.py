"""Contracts for missing consecutive lateral geometry and its commitment."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE/'source'))
from stack.core.geometry import Path as GeometryPath
from stack.core.route import Lane, Route, RoutePlanner
from stack.core.compound_route import build_compound_routes, commit_compound, consecutive_lateral_chain, route_cusps


class CompoundContracts(unittest.TestCase):
    def fixture(self):
        x = np.arange(0., 101.)
        lanes = {name: Lane(name, np.column_stack((x, np.full(len(x), y))))
                 for name, y in (('s', 0.), ('m', 3.))}
        final = np.vstack((np.column_stack((np.arange(31.), np.full(31, 6.))),
            np.column_stack((np.full(54, 30.), np.arange(7., 61.))),
            np.column_stack((np.arange(31., 101.), np.full(70, 60.)))))
        lanes['t'] = Lane('t', final)
        planner = RoutePlanner(lanes, v_ref=8.)
        planner.lat_pairs = {'s': {'m'}, 'm': {'s', 't'}, 't': {'m'}}
        route = planner.variant([('s', 0), ('m', 0), ('t', 0)], (10., 0.), 0.,
                                 [[89., 59.], [93., 61.]], {})
        return planner, route

    def test_missing_transition_becomes_continuous_goal_route(self):
        planner, route = self.fixture()
        self.assertNotEqual(set(planner.lateral_edges(route.lane_seq)), set(route.spans))
        candidates, evidence = build_compound_routes(planner, route, (10., 0.), 0.,
            [[89., 59.], [93., 61.]], (0., .5))
        self.assertEqual(evidence['graph_edges'], [0, 1])
        self.assertTrue(candidates)
        for candidate in candidates:
            self.assertEqual(candidate.compound_edges, (0, 1))
            self.assertFalse(candidate.issues)
            self.assertEqual(len(route_cusps(candidate)), 0)
            self.assertLess(candidate.goal_in, route.goal_in)

    def test_unchanged_single_lateral_requirement(self):
        planner, route = self.fixture()
        route.lane_seq = [('s', 0), ('m', 0)]
        candidates, evidence = build_compound_routes(planner, route, (10., 0.), 0.,
            [[39., 2.], [43., 4.]], (0.,))
        self.assertEqual(candidates, [])
        self.assertEqual(evidence['reason'], 'no_consecutive_lateral_chain')

    def test_separated_maneuvers_are_not_coalesced(self):
        planner = SimpleNamespace(lateral_edges=lambda _: [0, 2])
        self.assertIsNone(consecutive_lateral_chain(planner, ['a', 'b', 'c', 'd']))

    def test_cusp_is_detected_even_when_goal_projection_is_valid(self):
        path = GeometryPath(np.array([[0., 0.], [20., 0.], [1., .1], [25., 1.]]))
        route = Route(path, 0., 20., 19., 21.)
        self.assertGreater(len(route_cusps(route)), 0)

    def test_commitment_preserves_selected_geometry_and_all_graph_edges(self):
        planner, route = self.fixture()
        candidates, _ = build_compound_routes(planner, route, (10., 0.), 0.,
            [[89., 59.], [93., 61.]], (0.,))
        chosen = candidates[0]
        state = SimpleNamespace(route=chosen, s=12., fixed={}, compound_route_evidence={})
        self.assertTrue(commit_compound(state))
        self.assertIs(state.cands[0][0], chosen)
        self.assertEqual(set(state.fixed), {0, 1})
        self.assertIsNone(state.cands[0][2])
        self.assertEqual(state.compound_route_evidence['committed_edges'], [0, 1])

    def test_complete_smooth_chain_is_unchanged(self):
        planner, original = self.fixture()
        repaired, _ = build_compound_routes(planner, original, (10., 0.), 0.,
            [[89., 59.], [93., 61.]], (0.,))
        route = repaired[0]
        route.spans[1] = route.spans[0]
        candidates, evidence = build_compound_routes(planner, route, (10., 0.), 0.,
            [[89., 59.], [93., 61.]], (0.,))
        self.assertEqual(candidates, [])
        self.assertEqual(evidence['reason'], 'existing_geometry_realizes_all_transitions')


if __name__ == '__main__':
    unittest.main(verbosity=2)
