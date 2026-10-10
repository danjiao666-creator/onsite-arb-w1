"""Regression tests for disappearing-lane endpoint jumps and valid seams."""
from pathlib import Path
import sys
import unittest

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(HERE/'source'))
from stack.core.route import Lane, RoutePlanner, _smoothstep5


def line(start, end, y):
    x = np.arange(float(start), float(end)+.01, .5)
    return np.column_stack((x, np.full(len(x), y)))


def merge_lanes():
    x = np.arange(210., 300.01, .5)
    taper = np.column_stack((x, -7.5 + 1.5*_smoothstep5((x-210.)/90.)))
    return {'approach': Lane('approach', line(0, 200, -7.5), ['entry']),
        'entry': Lane('entry', line(200, 210, -7.5), ['taper']),
        'taper': Lane('taper', taper),
        'through': Lane('through', line(210, 300, -4.5), ['exit']),
        'exit': Lane('exit', line(300, 900, -4.5))}


class RouteSeamTests(unittest.TestCase):
    def test_disappearing_lane_cannot_jump_sideways_to_exit(self):
        planner = RoutePlanner(merge_lanes(), v_ref=22.185)
        self.assertNotIn(('exit', 0), planner._topo_edges('taper', 0))
        self.assertFalse(planner._topo_connected(('taper', 0), ('exit', 0)))

    def test_merge_is_expressed_as_a_smooth_lateral_transition(self):
        planner = RoutePlanner(merge_lanes(), v_ref=22.185)
        route = planner.plan((198., -7.5), 0., [[483., -5.5], [493., -3.5]])
        self.assertFalse(route.issues)
        self.assertTrue(route.spans)
        self.assertEqual(set(route.spans), set(planner.lateral_edges(route.lane_seq)))
        self.assertLess(np.max(np.abs(route.path._curv)), .02)
        for first, second in zip(route.lane_seq, route.lane_seq[1:]):
            if not planner.is_lateral(first, second):
                self.assertTrue(planner._topo_connected(first, second))

    def test_continuous_topological_connection_stays_valid(self):
        lanes = {'first': Lane('first', line(0, 100, 0), ['second']),
            'second': Lane('second', line(100, 200, 0))}
        planner = RoutePlanner(lanes)
        self.assertIn(('second', 0), planner._topo_edges('first', 0))
        self.assertFalse(planner.is_lateral(('first', 0), ('second', 0)))
        route = planner.plan((10., 0.), 0., [[150., -1.], [160., 1.]])
        self.assertFalse(route.issues)
        self.assertFalse(route.spans)
        np.testing.assert_array_equal(route.path._curv, np.zeros(len(route.path.pts)))

    def test_small_longitudinal_sampling_gap_does_not_create_lane_change(self):
        lanes = {'first': Lane('first', line(0, 100, 0), ['second']),
            'second': Lane('second', line(101, 200, 0))}
        planner = RoutePlanner(lanes)
        self.assertIn(('second', 0), planner._topo_edges('first', 0))
        self.assertFalse(planner.is_lateral(('first', 0), ('second', 0)))


if __name__ == '__main__':
    unittest.main()
