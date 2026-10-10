"""P0 contracts on isolated synthetic states; no scene replay or map loading."""
import ast
import gzip
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(HERE / 'source'))
from stack.core.arbiter import Arbiter, ArbiterConfig, EgoState, _obb_separation
from stack.core.control import ControlAdapter, LateralTracker
from stack.core.geometry import Path as RoutePath
from stack.core.goal_contract import goal_contains, goal_status, validate_goal
from stack.core.native_execution import rollout
from stack.core.native_profile import NativeArbiter
from stack.core.replay_contract import checked_action, execution_limits_diagnostic
from stack.core.route import _box_span, RoutePlanner, chain_fallback
from stack.adapters.onsite_close_guard import CloseGuardPlanner
from stack.adapters.onsite_cadence import CadencePlanner
from stack.adapters.onsite_replay import ArbiterPlanner

PROJECT = HERE.parents[1]
OFFICIAL = Path(os.environ.get('ONSITE_OFFICIAL_FUNCTIONS', str(PROJECT / 'code/onsite-structured-test-3.1.5/utils/functions.py')))
VALID = PROJECT / 'research/arb_capability_20261007/runs/recovery_pulse_r99_devreg130/validation'


def official_functions():
    # Execute only the two pure functions; importing the official module writes logs.
    tree = ast.parse(OFFICIAL.read_text(encoding='utf-8'))
    names = ('is_point_inside_rect', 'check_action')
    body = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
    namespace = {'np': np}
    exec(compile(ast.Module(body=body, type_ignores=[]), str(OFFICIAL), 'exec'), namespace)
    return namespace['is_point_inside_rect'], namespace['check_action']


OFFICIAL_INSIDE, OFFICIAL_CHECK = official_functions()


def empty_prediction(n):
    return dict(ids=[], pos=np.empty((0, n, 2)), yaw=np.empty((0, n)),
                L=np.empty(0), W=np.empty(0), now=np.empty((0, 2)),
                yaw_now=np.empty(0), vxy=np.empty((0, 2)))


class GoalContracts(unittest.TestCase):
    def test_predicate_matches_official_boundaries_and_reversed_rectangles(self):
        random = np.random.RandomState(10608)
        for _ in range(100):
            rect = random.uniform(-10, 10, (2, 2))
            points = np.vstack((rect, random.uniform(-15, 15, (30, 2))))
            expected = [OFFICIAL_INSIDE(rect.tolist(), point.tolist()) for point in points]
            np.testing.assert_array_equal(goal_contains(rect, points[:, 0], points[:, 1]), expected)
        rect = [[0, 0], [1, 1]]
        self.assertTrue(goal_contains(rect, 0, 1))
        self.assertTrue(goal_contains(rect, 1, 0))
        self.assertFalse(goal_contains(rect, 1.001, .5))

    def test_invalid_inputs_have_explicit_status(self):
        for rect, status in (([[0, 1], [1, 0]], 'reversed_bounds'),
                             ([[0, 0]], 'invalid_shape'),
                             ([[0, 0], [1, np.inf]], 'nonfinite')):
            with self.subTest(status=status):
                self.assertEqual(goal_status(rect), status)
                with self.assertRaisesRegex(ValueError, status):
                    validate_goal(rect)
                self.assertFalse(goal_contains(rect, .5, .5))

    def test_invalid_init_fails_before_importing_official_or_loading_map(self):
        planner = CloseGuardPlanner()
        info = {'task_info': {'startPos': [0, 0], 'targetPos': [[0, 1], [1, 0]]}}
        with self.assertRaisesRegex(ValueError, 'reversed_bounds'):
            planner.init(info)
        diag = planner.diagnostics()
        self.assertEqual(diag['goal_contract']['status'], 'reversed_bounds')
        self.assertFalse(diag['goal_contract']['implicit_normalization'])
        self.assertEqual(diag['close_guard']['status'], 'not_run')
        json.dumps(diag, allow_nan=False)

    def test_route_span_keeps_ordered_predicate(self):
        path = RoutePath([[0, 0], [20, 0]])
        self.assertNotEqual(_box_span(path, [[5, -1], [7, 1]], 6), (None, None))
        self.assertEqual(_box_span(path, [[5, 1], [7, -1]], 6), (None, None))

    def test_all_route_entry_points_reject_invalid_goal(self):
        rect = [[5, 1], [7, -1]]
        planner = RoutePlanner({})
        calls = (lambda: planner.plan([0, 0], 0, rect),
                 lambda: planner.variant([], [0, 0], 0, rect, {}),
                 lambda: planner.finish([[0, 0], [10, 0]], [0, 0], 0, rect, [], 'chain'),
                 lambda: chain_fallback({}, [0, 0], rect))
        for call in calls:
            with self.assertRaisesRegex(ValueError, 'reversed_bounds'):
                call()

    def test_arb_invalid_goal_neither_promotes_arrival_nor_truncates_future_risk(self):
        path = RoutePath([[0, 0], [100, 0]])
        arb = Arbiter()
        arb.cands = [(0., 0.)]
        s = np.broadcast_to(2 * arb.t, (1, arb.n)).copy()
        arb.rollout = lambda *args: (s, np.full_like(s, 2.), np.zeros_like(s))
        pred = empty_prediction(arb.n)
        pos = np.full((1, arb.n, 2), 100.)
        pos[0, np.argmin(abs(arb.t - 2.)), :] = [4., 0.]
        pred.update(ids=['future'], pos=pos, yaw=np.zeros((1, arb.n)), L=np.array([4.]), W=np.array([1.5]),
                    now=np.array([[100., 100.]]), yaw_now=np.zeros(1), vxy=np.zeros((1, 2)))
        plan = arb.decide(EgoState(0, 0, 0, 2, 0, 4, 1.5), path, 0, 0, pred,
                          v_des=2, v_goal_cap=100, goal_in=1, time_left=3,
                          goal_rect=[[1., 1.], [2., -1.]])
        self.assertFalse(plan.on_time)
        self.assertAlmostEqual(plan.t_collide, 2.)


class ObjectRetention(unittest.TestCase):
    def retained_case(self, name, actor_id):
        data = json.load(gzip.open(str(VALID / (name + '.json.gz')), 'rt', encoding='utf-8'))
        f = data['frames'][-2]
        a = f['objs'][actor_id]
        arb = Arbiter()
        arb.cands = [(0., 0.)]
        s = np.full((1, arb.n), 10.)
        arb.rollout = lambda *args: (s, np.zeros_like(s), np.zeros_like(s))
        path = RoutePath([[f['x'] - 10*np.cos(f['yaw']), f['y'] - 10*np.sin(f['yaw'])],
                          [f['x'] + 100*np.cos(f['yaw']), f['y'] + 100*np.sin(f['yaw'])]])
        # Retain recorded current geometry; controlled future occupancy hits ego.
        pred = dict(ids=[actor_id], pos=np.tile([f['x'], f['y']], (1, arb.n, 1)),
                    yaw=np.full((1, arb.n), f['yaw']), L=np.array([a[4]]), W=np.array([a[5]]),
                    now=np.array([a[:2]]), yaw_now=np.array([a[3]]), vxy=np.zeros((1, 2)))
        ego = EgoState(f['x'], f['y'], f['yaw'], f['v'], f['a'], f['L'], f['W'])
        plan = arb.decide(ego, path, 10, 0, pred, v_des=10, v_goal_cap=100,
                          goal_in=200, time_left=20)
        return plan

    def test_three_recorded_circle_false_overlaps_retain_future_actor(self):
        for name, actor in (('14_14_left_straight_in_adjacent_38', 'vehicle/car11'),
                            ('20_94_straight_straight_97', 'vehicle/car12'),
                            ('8_282_merge_291', 'vehicle/car10')):
            with self.subTest(scene=name):
                plan = self.retained_case(name, actor)
                self.assertIn(actor, plan.current_circle_overlap_ids)
                self.assertNotIn(actor, plan.current_rectangle_overlap_ids)
                self.assertTrue(plan.all_collide)
                self.assertAlmostEqual(plan.t_collide, .2)

    def test_true_overlap_does_not_erase_future_or_headway(self):
        path = RoutePath([[0, 0], [200, 0]])
        arb = Arbiter()
        arb.cands = [(0., 0.)]
        shape = (1, arb.n)
        arb.rollout = lambda *args: (np.zeros(shape), np.full(shape, 2.), np.zeros(shape))
        pred = dict(ids=['overlap'], pos=np.tile([3., 0.], (1, arb.n, 1)),
                    yaw=np.zeros(shape), L=np.array([4.]), W=np.array([1.5]),
                    now=np.array([[3., 0.]]), yaw_now=np.zeros(1), vxy=np.zeros((1, 2)))
        ego = EgoState(0, 0, 0, 2, 0, 4, 1.5)
        plan = arb.decide(ego, path, 0, 0, pred, v_des=10, v_goal_cap=100, goal_in=100, time_left=20)
        self.assertIn('overlap', plan.current_rectangle_overlap_ids)
        self.assertTrue(plan.all_collide)
        cost = arb._headway_cost(path, 0, np.zeros((1, len(arb.tc))),
                                 np.full((1, len(arb.tc)), 2.), pred, ego, np.ones(1, dtype=bool))
        self.assertGreater(float(cost[0]), 0.)


class GuardStateContracts(unittest.TestCase):
    def setUp(self):
        self.planner = CloseGuardPlanner()
        self.planner.rect = [[500., -1.], [510., 1.]]
        self.planner._pulse_recovery_until = -np.inf
        self.planner._guard_measured_steer = 0.
        self.ego = EgoState(0, 0, 0, 2, 0, 4.5, 1.8)
        self.path = RoutePath([[0, 0], [1000, 0]])
        self.plan = SimpleNamespace(a=np.zeros(60), a_cmd=0.)
        actor = SimpleNamespace(x=5., y=0., yaw=0., v=0., length=4.5, width=1.8)
        self.planner.world.t = 1.
        self.planner.world.tracks = {'actor': SimpleNamespace(agent=actor, accel=0.)}
        pred = dict(ids=['actor'], now=np.array([[5., 0.]]), L=np.array([4.5]), W=np.array([1.8]),
                    pos=np.full((1, 20, 2), 5.), yaw=np.zeros((1, 20)))
        self.planner.world.predict = lambda *args, **kwargs: pred

    def execute_with_separation(self, separation):
        def fake_rollout(initial, accelerations, path, station, ctrl, lat, dt, length, modes):
            count, steps = accelerations.shape
            states = np.zeros((count, steps+1, 6))
            states[:, :, 0] = np.arange(steps+1)[None] * dt * self.ego.v
            commands = np.zeros((count, steps, 2))
            commands[:, :, 0] = np.clip(accelerations, -9.8, 9.8)
            return states, commands
        inf = np.full_like(separation, np.inf)
        with patch('stack.adapters.onsite_close_guard.rollout', side_effect=fake_rollout), \
             patch('stack.adapters.onsite_close_guard._obb_separation', side_effect=[separation, inf, inf]):
            return self.planner._execute(self.plan, self.path, 0, 0, self.ego, .1)

    def status(self):
        row = self.planner.safety_diagnostics()['close_guard']
        json.dumps(row, allow_nan=False)
        return row

    def test_no_actors_is_not_checked_not_safe(self):
        self.planner.world.tracks = {}
        self.planner._execute(self.plan, self.path, 0, 0, self.ego, .1)
        row = self.status()
        self.assertEqual(row['status'], 'not_checked_no_actors')
        self.assertFalse(row['checked'])
        self.assertEqual(row['returned_action_status'], 'unverified')

    def test_distant_actors_are_not_checked(self):
        self.planner.world.tracks['actor'].agent.x = 100.
        self.planner._execute(self.plan, self.path, 0, 0, self.ego, .1)
        self.assertEqual(self.status()['status'], 'not_checked_outside_trigger')

    def test_empty_prediction_is_not_checked_and_never_reduced(self):
        self.planner.world.predict = lambda *args, **kwargs: {'ids': []}
        self.planner._execute(self.plan, self.path, 0, 0, self.ego, .1)
        self.assertEqual(self.status()['status'], 'not_checked_no_predictions')

    def test_nominal_clear_has_explicit_checked_scope(self):
        self.execute_with_separation(np.ones((19, 1, 20)))
        row = self.status()
        self.assertEqual(row['status'], 'nominal_clear')
        self.assertTrue(row['checked'])
        self.assertEqual(row['collision_free_count'], 19)
        self.assertIsNone(row['eligible_count'])

    def test_all_candidates_collide_explicit_failure_preserves_nominal_action(self):
        expected = ControlAdapter().command(0., self.planner.lat.yaw_rate(self.path, 0, 0, 0, 2), 2, 4.5, .1)
        action = self.execute_with_separation(-np.ones((19, 1, 20)))
        np.testing.assert_array_equal(action, expected)
        row = self.status()
        self.assertEqual(row['status'], 'failed_no_collision_free_candidate')
        self.assertTrue(row['failed'])
        self.assertEqual(row['returned_action_status'], 'risky_nominal_fallback')
        self.assertEqual(self.planner.arb.stats['close_guard_failures'], 1)

    def test_pulse_gate_is_distinct_from_no_collision_free_candidate(self):
        separation = np.ones((19, 1, 20))
        separation[:, :, 14] = -1.
        separation[13, :, :] = 1.
        self.execute_with_separation(separation)
        row = self.status()
        self.assertEqual(row['status'], 'failed_pulse_gate')
        self.assertEqual(row['collision_free_count'], 1)
        self.assertEqual(row['available_clear_count'], 0)
        self.assertFalse(row['pulse_allowed'])

    def test_lateral_rejection_is_distinct(self):
        separation = -np.ones((19, 1, 20))
        separation[1] = 1.
        with patch.object(self.path, 'project', return_value=(0, 2.)):
            self.execute_with_separation(separation)
        row = self.status()
        self.assertEqual(row['status'], 'failed_lateral_eligibility')
        self.assertEqual(row['available_clear_count'], 1)
        self.assertEqual(row['eligible_count'], 0)

    def test_override_records_chosen_candidate_and_checked_action(self):
        separation = -np.ones((19, 1, 20))
        separation[1] = 1.
        self.execute_with_separation(separation)
        row = self.status()
        self.assertEqual(row['status'], 'override')
        self.assertEqual(row['chosen_index'], 1)
        self.assertFalse(row['failed'])
        self.assertEqual(row['commanded_action'][0], -9.8)
        self.assertAlmostEqual(row['expected_checked_action'][0], -4.9)

    def test_failure_resets_next_cycle_and_counts_survive(self):
        self.execute_with_separation(-np.ones((19, 1, 20)))
        self.planner.world.tracks = {}
        self.planner._execute(self.plan, self.path, 0, 0, self.ego, .1)
        self.assertFalse(self.status()['failed'])
        self.assertIsNone(self.status()['nominal_first_hit'])
        self.assertEqual(self.planner._guard_counts['failed_no_collision_free_candidate'], 1)
        self.assertEqual(self.planner._guard_counts['not_checked_no_actors'], 1)

    def test_diagnostic_snapshot_cannot_mutate_guard_state(self):
        self.execute_with_separation(-np.ones((19, 1, 20)))
        snapshot = self.planner.safety_diagnostics()
        snapshot['close_guard']['commanded_action'][0] = 123.
        snapshot['close_guard']['status'] = 'changed_by_recorder'
        self.assertNotEqual(self.status()['commanded_action'][0], 123.)
        self.assertEqual(self.status()['status'], 'failed_no_collision_free_candidate')

    def test_reversed_goal_cannot_hide_guard_collision_after_normalized_entry(self):
        separation = np.ones((19, 1, 20))
        separation[:, :, 5] = -1.
        self.planner.rect = [[.1, 1.], [.3, -1.]]
        self.execute_with_separation(separation)
        self.assertEqual(self.status()['status'], 'failed_no_collision_free_candidate')


class ExecutionContracts(unittest.TestCase):
    def test_checked_action_matches_official_at_both_cadences_and_nan_initial_memory(self):
        random = np.random.RandomState(4998)
        for dt in (.04, .1):
            for i in range(200):
                v = random.uniform(0, 45)
                prev = random.uniform([-9.8, -.699], [9.8, .699])
                cmd = random.uniform([-15, -1], [15, 1])
                if i % 30 == 0:
                    prev[:] = np.nan
                actual = checked_action(dt, v, prev[0], prev[1], cmd[0], cmd[1])
                expected = OFFICIAL_CHECK(dt, v, prev, cmd)
                np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)

    def test_stop_correction_precedes_jerk(self):
        for dt in (.04, .1):
            action = checked_action(dt, .1, 9.8, .0, -9.8, .0)
            self.assertAlmostEqual(action[0], -.1 / dt)

    def test_compiled_guard_execution_matches_official_checks_at_every_step(self):
        path = RoutePath([[0, 0], [10, 0], [25, 6], [50, 20]])
        for dt in (.04, .1):
            ctrl = ControlAdapter()
            ctrl.reset(.12031)
            steps = int(round(2 / dt))
            commands_acc = np.tile(np.r_[np.full(steps//2, -9.8), np.full(steps-steps//2, 9.8)], (3, 1))
            initial = np.array([1., .4, 6.2, 6., -1., .12])
            states, commands = rollout(initial, commands_acc, path, 1, ctrl, LateralTracker(), dt, 4.5,
                                       np.array([0, 1, 2], dtype=np.int64))
            for ci in range(3):
                for k in range(steps):
                    x, y, yaw, v, acc, rot = states[ci, k]
                    checked = OFFICIAL_CHECK(dt, v, [acc, rot], commands[ci, k])
                    nx = round(x + v*np.cos(yaw)*dt, 3)
                    ny = round(y + v*np.sin(yaw)*dt, 3)
                    nh = round((yaw + v/4.5*1.7*np.tan(checked[1])*dt) % (2*np.pi), 3)
                    np.testing.assert_allclose(states[ci, k+1],
                        [nx, ny, nh, round(max(0, v+checked[0]*dt), 3), round(checked[0], 3), round(checked[1], 3)],
                        rtol=0, atol=1e-12)
            self.assertEqual(ctrl.prev_steer, .12031)

    def test_main_profile_reference_and_native_unchanged_algebra(self):
        random = np.random.RandomState(3249)
        for dt in (.04, .1):
            cfg = ArbiterConfig(dt=dt, jerk=6*.1/dt, jerk_emerg=20*.1/dt)
            ref, native = Arbiter(cfg), NativeArbiter(cfg)
            for _ in range(4):
                ego = EgoState(0, 0, 0, random.uniform(0, 30), random.uniform(-8, 8), 4.5, 1.8)
                env_s = np.arange(250, dtype=float)
                env_v = 10+8*np.sin(env_s/20)
                for expected, actual in zip(ref.rollout(ego, 0, env_s, env_v, 20),
                                            native.rollout(ego, 0, env_s, env_v, 20)):
                    np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-10)

    def test_diagnostic_exposes_main_jerk_and_integration_differences(self):
        cfg = ArbiterConfig(dt=.04, jerk=15, jerk_emerg=50)
        d = execution_limits_diagnostic(cfg, ControlAdapter().lim)
        self.assertTrue(d['main_profile']['emergency_jerk_exceeds_official'])
        self.assertFalse(d['guard']['main_profile_execution_equivalence'])
        self.assertNotEqual(d['official']['position_integration'], d['main_profile']['position_integration'])
        json.dumps(d, allow_nan=False)


if __name__ == '__main__':
    unittest.main(verbosity=2)
