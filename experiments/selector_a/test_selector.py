"""Eight focused selector contracts, no planner runs or scene simulation."""
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent / 'source'))
from stack.core.arbiter import EgoState
from stack.core.road_boundary import RoadBoundary
from stack.core.unified_evaluation import select_execution, select_original_execution


class SelectorContracts(unittest.TestCase):
    def make(self, etas=(12., 7.), costs=(1., 1000.), dt=.1, missing=False):
        n, count = int(round(6./dt)), len(etas)
        states = np.zeros((count, n+1, 6))
        states[:, :, 0] = 10.+np.arange(n+1)*dt
        states[:, :, 3] = 1.
        commands = np.zeros((count, n, 2))
        commands[:, :, 0] = np.arange(count)[:, None]
        a = SimpleNamespace(first_hit=np.full(count, np.inf), eligible=np.ones(count, dtype=bool),
            goal_time=np.full(count, np.inf), eta=np.asarray(etas), cost=np.asarray(costs),
            contact_first_hit=np.full(count, np.inf), next_clearance=np.ones(count),
            prefix_clearance=np.ones(count), states=states, commands=commands,
            terminal=np.full(count, n-1, dtype=int))
        rows = [[dict(id='r/%d'%i, rejected_by=[]) for i in range(count)]]
        lane = SimpleNamespace(lane_id='a', left_vertices=np.array([[0., 3.], [100., 3.]]),
            right_vertices=np.array([[0., -3.], [100., -3.]]), successor=[], predecessor=[])
        boundary = RoadBoundary(SimpleNamespace(discretelanes=[] if missing else [lane]))
        return [a], rows, boundary, EgoState(10., 0., 0., 1., 0., 4.5, 1.8), dt, 10.

    def test_01_late_occupancy_is_not_a_progress_witness(self):
        args = self.make()
        args[0][0].first_hit[1] = 3.
        selected = select_execution(*args)
        self.assertEqual(selected.candidate, 0)
        self.assertEqual(selected.progress['rejection_counts']['future_occupancy'], 1)

    def test_02_late_road_exit_cannot_replace_original(self):
        args = self.make()
        args[0][0].states[1, 31:, 1] = 5.
        selected = select_execution(*args)
        self.assertEqual(selected.candidate, 0)
        self.assertEqual(args[1][0][1]['progress_status'], 'full_road_failed')

    def test_03_tracking_and_nonfinite_commands_rejected(self):
        args = self.make(etas=(12., 7., 6.), costs=(1., 1000., 2.))
        args[0][0].eligible[1] = False
        args[0][0].commands[2, 0, 0] = np.nan
        selected = select_execution(*args)
        self.assertEqual(selected.candidate, 0)
        self.assertEqual(selected.progress['full_window_admitted_count'], 1)

    def test_04_unknown_road_never_admitted(self):
        args = self.make(missing=True)
        selected = select_execution(*args)
        self.assertEqual(selected.progress['full_window_admitted_count'], 0)
        self.assertFalse(selected.feasible)

    def test_05_progress_precedes_soft_cost(self):
        args = self.make()
        selected = select_execution(*args)
        self.assertEqual(selected.candidate, 1)
        self.assertTrue(selected.progress['action_changed'])
        self.assertEqual(selected.progress['original_candidate'], 0)

    def test_06_same_time_grid_uses_original_cost(self):
        args = self.make(etas=(7.02, 7.01))
        selected = select_execution(*args)
        self.assertEqual(selected.candidate, 0)

    def test_07_empty_pool_preserves_exact_original_action(self):
        args = self.make()
        args[0][0].first_hit[:] = [3., 4.]
        original = select_original_execution(args[0], deepcopy(args[1]), *args[2:])
        selected = select_execution(*args)
        self.assertEqual((original.batch, original.candidate, original.feasible),
                         (selected.batch, selected.candidate, selected.feasible))
        self.assertEqual(selected.progress['status'], 'no_full_window_progress_candidate')
        self.assertFalse(selected.progress['action_changed'])

    def test_08_original_terminal_and_dt_are_preserved(self):
        args = self.make(dt=.04)
        a = args[0][0]
        a.terminal[1] = 24
        a.goal_time[1] = 1.
        a.states[1, 26:, 1] = 5.
        selected = select_execution(*args)
        self.assertEqual(selected.candidate, 1)
        self.assertEqual(args[1][0][1]['full_window_first_outside_step'], -1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
