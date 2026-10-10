"""Execution witnesses and evaluator invariants, independent of driving scores."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
import numpy as np

sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'source'))
from stack.core.arbiter import EgoState, ArbiterConfig
from stack.core.native_profile import NativeArbiter
from stack.core.geometry import Path as DrivingPath
from stack.core.control import ControlAdapter,LateralTracker
from stack.core.world import WorldModel,Agent
from stack.core.candidates import Candidate,generate
from stack.core.unified_evaluation import ExecutionEvaluator, execution_rank, occupancy, select_execution
from stack.core.decision_record import ExecutionDecision,feedback
from stack.core.road_boundary import RoadBoundary
from stack.core.native_execution import rollout_projected, project_states


class UnifiedContracts(unittest.TestCase):
    def setUp(self):
        self.path=DrivingPath(np.array([[0.,0.],[200.,0.]]))
        self.cfg=ArbiterConfig(horizon=2.,dt=.1,check_every=1)
        self.arb=NativeArbiter(self.cfg)
        self.world=WorldModel()
        self.world.update(0.,[])
        self.ctrl=ControlAdapter()
        self.lat=LateralTracker()

    def assess(self,ego,candidates,world=None):
        ev=ExecutionEvaluator(world or self.world,self.arb.t,ego)
        a=ev.evaluate(candidates,self.path,0.,ego.y,ego,0.,self.ctrl,self.lat,.1,
            [[100.,-1.],[110.,1.]],100.,20.,np.array([0.,200.]),np.array([30.,30.]),
            self.cfg,10.,0.,0.)
        return ev,a

    def test_identical_proposals_have_identical_assessment_across_families(self):
        ego=EgoState(0.,0.,0.,10.,0.,4.5,1.8)
        c=[Candidate('main','main',np.zeros(20)),Candidate('recovery','recovery',np.zeros(20))]
        _,a=self.assess(ego,c)
        for value in (a.states,a.commands,a.first_hit,a.clearance,a.cost,a.eligible):
            np.testing.assert_array_equal(value[0],value[1])

    def test_stopped_vehicle_does_not_converge_to_path(self):
        ego=EgoState(0.,1.,.3,0.,0.,4.5,1.8)
        _,a=self.assess(ego,[Candidate('stop','main',np.zeros(20))])
        np.testing.assert_array_equal(a.states[0,-1,:3],[0.,1.,.3])

    def test_jerk_changes_occupancy_state_not_only_returned_action(self):
        ego=EgoState(0.,0.,0.,20.,9.8,4.5,1.8)
        _,a=self.assess(ego,[Candidate('brake','main',np.full(20,-9.8))])
        self.assertAlmostEqual(a.states[0,1,4],4.9)
        self.assertAlmostEqual(a.states[0,1,0],2.)
        self.assertAlmostEqual(a.states[0,1,3],20.49)

    def test_conflict_identifies_actor_and_time(self):
        self.world.update(0.,[Agent('vehicle/block','vehicle',8.,0.,0.,0.,4.5,1.8)])
        ego=EgoState(0.,0.,0.,10.,0.,4.5,1.8)
        ev,a=self.assess(ego,[Candidate('coast','main',np.zeros(20))])
        row=ev.rows([Candidate('coast','main',np.zeros(20))],a,'r')[0]
        self.assertEqual(row['actor'],'vehicle/block')
        self.assertEqual(row['branch'],'W1')
        self.assertLess(row['first_hit'],1.)

    def test_comfort_cannot_advance_impact_when_all_proposals_conflict(self):
        a=SimpleNamespace(first_hit=np.array([.8,1.2]),eligible=np.ones(2,dtype=bool),
            goal_time=np.full(2,np.inf),eta=np.full(2,20.),cost=np.array([0.,100000.]),
            next_clearance=np.array([1.,1.]),prefix_clearance=np.array([-1.,-1.]))
        self.assertLess(execution_rank(a,1,10.),execution_rank(a,0,10.))

    def test_immediate_separation_breaks_same_step_impact_tie(self):
        a=SimpleNamespace(first_hit=np.array([.1,.1]),eligible=np.ones(2,dtype=bool),
            goal_time=np.full(2,np.inf),eta=np.full(2,20.),cost=np.array([0.,100000.]),
            next_clearance=np.array([-.1,.05]),prefix_clearance=np.array([-1.,-2.]))
        self.assertLess(execution_rank(a,1,10.),execution_rank(a,0,10.))

    def test_hard_prefix_conflict_cannot_replace_prefix_clear_proposal(self):
        a=SimpleNamespace(first_hit=np.array([1.,np.inf]),eligible=np.ones(2,dtype=bool),
            goal_time=np.array([4.,np.inf]),eta=np.array([4.,9.]),cost=np.array([0.,100000.]),
            next_clearance=np.array([1.,1.]),prefix_clearance=np.array([1.,1.]))
        self.assertLess(execution_rank(a,1,10.),execution_rank(a,0,10.))

    def joint_selection(self,hits,heights,terminal=None,missing_map=False,contacts=None):
        lane=SimpleNamespace(lane_id='a',left_vertices=np.array([[0.,2.],[50.,2.]]),
            right_vertices=np.array([[0.,-2.],[50.,-2.]]),successor=[],predecessor=[])
        boundary=RoadBoundary(SimpleNamespace(discretelanes=[] if missing_map else [lane]))
        states=np.zeros((len(hits),3,6))
        states[:,:,0]=[10.,11.,12.]
        states[:,1:,1]=heights
        a=SimpleNamespace(first_hit=np.array(hits),eligible=np.ones(len(hits),dtype=bool),
            goal_time=np.full(len(hits),np.inf),cost=np.arange(len(hits))*100000.,
            eta=np.full(len(hits),20.),
            contact_first_hit=np.array(contacts if contacts is not None else np.full(len(hits),np.inf)),
            next_clearance=np.ones(len(hits)),prefix_clearance=np.ones(len(hits)),
            states=states,terminal=np.full(len(hits),1) if terminal is None else np.array(terminal))
        rows=[[dict(rejected_by=[]) for _ in hits]]
        ego=EgoState(10.,0.,0.,10.,0.,4.5,1.8)
        selected=select_execution([a],rows,boundary,ego,.1,10.,.2)
        return selected,rows[0]

    def test_road_rejected_clear_proposals_do_not_hide_later_joint_risk_witness(self):
        selected,rows=self.joint_selection([.3,.2],[[1.5,1.5],[0.,0.]])
        self.assertFalse(selected.feasible)
        self.assertEqual(selected.candidate,1)
        self.assertEqual(selected.checked_count,2)
        self.assertAlmostEqual(rows[0]['joint_first_violation'],.1)
        self.assertAlmostEqual(rows[1]['joint_first_violation'],.2)

    def test_all_occupancy_rejected_proposals_still_receive_road_checks(self):
        selected,rows=self.joint_selection([.2,.15],[[1.5,1.5],[0.,0.]])
        self.assertEqual(selected.candidate,1)
        self.assertTrue(all(row['road_checked'] for row in rows))
        self.assertEqual(selected.road_rejected,1)

    def test_imminent_body_contact_is_not_advanced_to_avoid_earlier_map_overhang(self):
        selected,rows=self.joint_selection([.2,.15],[[1.5,1.5],[0.,0.]],contacts=[.2,.15])
        self.assertFalse(selected.feasible)
        self.assertEqual(selected.candidate,0)
        self.assertEqual(rows[0]['rejected_by'],['mapped_road_body_outside'])

    def test_joint_feasible_prefix_keeps_lazy_gate_and_dominates_risk(self):
        selected,rows=self.joint_selection([.1,np.inf,np.inf],[[0.,0.],[0.,0.],[0.,0.]])
        self.assertTrue(selected.feasible)
        self.assertEqual(selected.candidate,1)
        self.assertEqual(selected.checked_count,1)
        self.assertNotIn('road_checked',rows[2])

    def test_executed_goal_terminal_masks_later_road_exit(self):
        selected,rows=self.joint_selection([np.inf],[[0.,1.5]],terminal=[0])
        self.assertTrue(selected.feasible)
        self.assertEqual(rows[0]['road_first_outside_step'],-1)

    def test_unknown_map_cannot_be_a_joint_feasible_witness(self):
        selected,rows=self.joint_selection([np.inf],[[0.,0.]],missing_map=True)
        self.assertFalse(selected.feasible)
        self.assertEqual(rows[0]['rejected_by'],['mapped_road_body_unverified'])

    def test_budget_and_efficiency_use_the_same_eta_contract(self):
        ego=EgoState(0.,0.,0.,10.,0.,4.5,1.8)
        _,a=self.assess(ego,[Candidate('stop','main',np.full(20,-9.8))])
        self.assertAlmostEqual(a.eta[0],a.budget_eta[0])
        self.assertEqual(a.cost_terms['eta'][0],a.eta[0])

    def test_goal_witness_and_optimistic_tail_estimate_have_distinct_priority(self):
        a=SimpleNamespace(first_hit=np.array([3.,np.inf]),eligible=np.ones(2,dtype=bool),
            goal_time=np.array([6.,np.inf]),eta=np.array([6.,15.]),cost=np.array([700.,16.]),
            next_clearance=np.array([1.,1.]),prefix_clearance=np.array([1.,1.]))
        self.assertLess(execution_rank(a,0,25.),execution_rank(a,1,25.))

    def test_estimated_budget_does_not_replace_forecast_clear_with_conflict(self):
        a=SimpleNamespace(first_hit=np.array([3.,np.inf]),eligible=np.ones(2,dtype=bool),
            goal_time=np.full(2,np.inf),eta=np.array([8.,12.]),cost=np.array([0.,100000.]),
            next_clearance=np.ones(2),prefix_clearance=np.ones(2))
        self.assertLess(execution_rank(a,1,10.),execution_rank(a,0,10.))

    def test_buffer_and_contact_can_identify_different_actors(self):
        self.world.update(0.,[Agent('vehicle/near','vehicle',4.6,0.,0.,0.,4.5,1.8),
                             Agent('vehicle/touch','vehicle',4.4,0.,0.,0.,4.5,1.8)])
        ego=EgoState(0.,0.,0.,0.,0.,4.5,1.8)
        c=[Candidate('stop','main',np.zeros(20))]
        ev,a=self.assess(ego,c)
        row=ev.rows(c,a,'r')[0]
        self.assertEqual(row['actor'],'vehicle/near')
        self.assertEqual(row['contact_actor'],'vehicle/touch')
        self.assertAlmostEqual(row['contact_first_hit'],.1)

    def test_buffer_shortfall_is_distinct_from_predicted_body_contact(self):
        states=np.array([[[0.,0.,0.,0.,0.,0.],[0.,0.,0.,0.,0.,0.]]])
        positions=np.array([[[[4.6,0.]]]])
        result=occupancy(states,positions,np.zeros((1,1,1)),np.array([4.5]),
            np.array([1.8]),np.ones((1,1,1),dtype=bool),np.array([0]),4.5,1.8,.1,.15,.06)
        self.assertAlmostEqual(result[0][0],.1)
        self.assertAlmostEqual(result[-2][0],.1)
        self.assertTrue(np.isposinf(result[-1][0]))

    def test_initial_boundary_recovery_cannot_grow_overhang_or_remain_outside(self):
        lane=SimpleNamespace(lane_id='a',left_vertices=np.array([[0.,2.],[30.,2.]]),
            right_vertices=np.array([[0.,-2.],[30.,-2.]]),successor=[],predecessor=[])
        boundary=RoadBoundary(SimpleNamespace(discretelanes=[lane]))
        initial=np.array([[10.,1.5,0.]])
        area=float(boundary.outside_area(initial,4.5,1.8)[0])
        self.assertTrue(boundary.recovers_initial_exterior(np.array([[11.,1.4,0.],[12.,0.,0.]]),4.5,1.8,area))
        self.assertFalse(boundary.recovers_initial_exterior(np.array([[11.,1.6,0.],[12.,0.,0.]]),4.5,1.8,area))
        self.assertFalse(boundary.recovers_initial_exterior(np.array([[11.,1.4,0.],[12.,1.2,0.]]),4.5,1.8,area))

    def test_feedback_exposes_physical_mismatch(self):
        d=ExecutionDecision(0.,'a','main',[0.,0.],[0.,0.],[1.,0.,0.,10.,0.,0.],'ok','r',0.)
        ego=SimpleNamespace(x=1.,y=.2,yaw=0.,v=10.,a=0.,rot=0.)
        self.assertEqual(feedback(d,ego)['status'],'mismatch')

    def test_shifted_continuation_advances_one_step(self):
        ego=EgoState(0.,0.,0.,10.,0.,4.5,1.8)
        seq=np.arange(20,dtype=float)/10
        cs=generate(self.arb,ego,0.,np.array([0.,200.]),np.array([30.,30.]),20.,(seq,1))
        c=next(c for c in cs if c.id=='continuation')
        np.testing.assert_array_equal(c.acceleration,np.r_[seq[1:],seq[-1]])
        self.assertAlmostEqual(c.steering_duration,1.9)

    def test_continuation_keeps_original_steering_target_and_remaining_time(self):
        ego=EgoState(0.,0.,0.,10.,0.,4.5,1.8)
        seq=np.arange(20,dtype=float)/10
        cs=generate(self.arb,ego,0.,np.array([0.,200.]),np.array([30.,30.]),20.,(seq,1,.3,.12))
        c=next(c for c in cs if c.id=='continuation')
        self.assertAlmostEqual(c.steering_duration,.2)
        self.assertAlmostEqual(c.steering_target,.12)

    def test_returned_projection_matches_independent_projection(self):
        states,_,stations=rollout_projected(np.array([0.,.5,.1,8.,0.,0.]),
            np.ones((2,20)),self.path,0.,self.ctrl,self.lat,.1,4.5)
        reference=project_states(states[:,1:],0.,self.path.pts,self.path.cum)
        np.testing.assert_array_equal(stations,reference)

    def test_body_boundary_rejects_width_overhang_and_keeps_map_hole(self):
        def lane(key,x0,x1):
            return SimpleNamespace(lane_id=key,left_vertices=np.array([[x0,2.],[x1,2.]]),
                right_vertices=np.array([[x0,-2.],[x1,-2.]]),successor=[],predecessor=[])
        boundary=RoadBoundary(SimpleNamespace(discretelanes=[lane('a',0.,20.),lane('b',30.,50.)]))
        states=np.array([[[10.,0.,0.,0.,0.,0.],[10.,1.5,0.,0.,0.,0.]],
                         [[25.,0.,0.,0.,0.,0.],[35.,0.,0.,0.,0.,0.]]])
        result=boundary.inspect(states,4.5,1.8,np.array([1,1]))
        np.testing.assert_array_equal(result['covered'],[[True,False],[False,True]])

    def test_missing_road_is_unknown(self):
        boundary=RoadBoundary(SimpleNamespace(discretelanes=[]))
        self.assertFalse(boundary.inspect(np.zeros((1,1,6)),4.5,1.8)['checked'])

    def test_exterior_area_keeps_overhang_and_map_hole_with_covered_fast_path(self):
        def lane(key,x0,x1):
            return SimpleNamespace(lane_id=key,left_vertices=np.array([[x0,2.],[x1,2.]]),
                right_vertices=np.array([[x0,-2.],[x1,-2.]]),successor=[],predecessor=[])
        boundary=RoadBoundary(SimpleNamespace(discretelanes=[lane('a',0.,20.),lane('b',30.,50.)]))
        states=np.array([[[10.,0.,0.],[10.,1.5,0.]],[[25.,0.,0.],[35.,0.,0.]]])
        np.testing.assert_allclose(boundary.outside_area(states,4.5,1.8),
                                   [[0.,1.8],[8.1,0.]],rtol=0.,atol=1e-12)

    def test_shared_border_requires_parser_identity_and_reciprocity(self):
        from stack.core.shared_border import _shared_border_vertices
        border=object()
        def group(side,neighbour):
            bg=SimpleNamespace(**{side+'_border':border,side+'_border_offset':0.})
            g=SimpleNamespace(parametric_lanes=[SimpleNamespace(border_group=bg,reverse=False,length=10.)])
            if side=='inner':
                g.inner_neighbour=neighbour;g.inner_neighbour_same_direction=True
            else:g.outer_neighbour=neighbour
            return g
        owner=SimpleNamespace(right_vertices=np.array([[0.,0.],[5.,.01],[10.,0.]]),
            _parametric_lane_group=group('outer','outer'))
        lane=SimpleNamespace(left_vertices=np.array([[0.,0.],[10.,0.]]),
            right_vertices=np.array([[0.,-3.],[10.,-3.]]),
            _parametric_lane_group=group('inner','inner'))
        replacements,records=_shared_border_vertices({'inner':owner,'outer':lane})
        self.assertEqual(list(replacements),['outer'])
        self.assertEqual(len(records),1)
        owner._parametric_lane_group.parametric_lanes[0].border_group.outer_border=object()
        self.assertFalse(_shared_border_vertices({'inner':owner,'outer':lane})[0])


if __name__=='__main__': unittest.main(verbosity=2)
