"""Single arbiter over executable main/recovery proposals on every route."""
from copy import deepcopy
import time
import numpy as np

from .onsite_cadence import CadencePlanner
from ..core.arbiter import EgoState, Plan
from ..core.world import Agent
from ..core.candidates import generate
from ..core.unified_evaluation import ExecutionEvaluator, occupancy, select_execution
from ..core.native_execution import rollout, project_states
from ..core.decision_record import ExecutionDecision, feedback, finite
from ..core.replay_contract import checked_action
from ..core.road_boundary import RoadBoundary
from ..core.compound_route import install_compound, commit_compound


class UnifiedPlanner(CadencePlanner):
    EXECUTION_HORIZON = 2.0
    def __init__(self):
        super().__init__()
        self.execution_decision=None
        self._cycle={}

    def init(self, scenario_info):
        super().init(scenario_info)
        from utils.opendrive2discretenet import parse_opendrive
        self.road_boundary=RoadBoundary(parse_opendrive(scenario_info['source_file']['xodr']))
        self.execution_decision = None
        self.execution_feedback = {'status':'initial'}
        self._continuation = None
        self._cycle = {}
        self._pending_forecast = None
        initial=np.zeros(6)
        states,_=rollout(initial,np.zeros((1,2)),self.route.path,self.s,
                         self.ctrl,self.lat,self.dt,4.5)
        project_states(states[:,1:],self.s,self.route.path.pts,self.route.path.cum)
        occupancy(states,np.zeros((3,0,2,2)),np.zeros((3,0,2)),np.zeros(0),np.zeros(0),
                  np.zeros((3,0,2),dtype=np.bool_),np.array([1],dtype=np.int64),4.5,1.8,self.dt,.15,.06)
        install_compound(self)

    def _make_variants(self):
        if commit_compound(self):
            return
        return super()._make_variants()

    def _prediction_feedback(self, timestamp, agents):
        pending=self._pending_forecast
        if pending is None: return {'status':'initial'}
        if abs(timestamp-pending['t'])>1e-6: return {'status':'time_mismatch'}
        previous=pending['actors']
        errors=[]
        for actor in agents:
            if actor.id in previous:
                predicted=previous[actor.id]
                errors.append(dict(actor=actor.id,
                    position_error=float(np.hypot(actor.x-predicted[0],actor.y-predicted[1])),
                    heading_error=float((actor.yaw-predicted[2]+np.pi)%(2*np.pi)-np.pi)))
        return dict(status='observed',errors=errors,
                    new_actors=[actor.id for actor in agents if actor.id not in pending['observed_ids']])

    def act(self, observation):
        started=time.perf_counter()
        ego=observation.ego_info
        timestamp=float(observation.test_info['t'])
        dt=float(observation.test_info.get('dt') or self.dt)
        if abs(dt-self.dt)>1e-9:
            raise ValueError('Control cadence changed after init')
        self.execution_feedback=feedback(self.execution_decision,ego)
        actors=[Agent('%s/%s'%(category,key),category,float(o.x),float(o.y),float(o.yaw),
                      float(o.v),float(o.length),float(o.width))
                for category,objects in observation.object_info.items() for key,o in objects.items()]
        forecast_feedback=self._prediction_feedback(timestamp,actors)
        self.world.update(timestamp,actors)
        state=EgoState(float(ego.x),float(ego.y),float(ego.yaw),float(ego.v),
                       float(ego.a or 0.),float(ego.length),float(ego.width))
        measured=float(ego.rot or 0.)
        evaluator=ExecutionEvaluator(self.world,self.arb.t,state)
        pred=evaluator.prediction
        self._pending_forecast=dict(t=timestamp+dt,observed_ids=[a.id for a in actors],actors={key:[*pred['pos'][i,0],pred['yaw'][i,0]]
            for i,key in enumerate(pred['ids'])})
        world_ms=(time.perf_counter()-started)*1000
        self._ego_xy=(state.x,state.y)
        time_left=max(0.,self.max_t-timestamp)
        previous_action=self.execution_decision.action[0] if self.execution_decision else state.a
        if not self._dynamic_active:
            self.cands=[(self.route,self.s,None,0.)]
        batches=[]
        all_rows=[]
        timings=dict(world_ms=world_ms,generation_ms=0.,evaluation_ms=0.)
        for index,(route,hint,edge,fraction) in enumerate(self.cands):
            station,error=route.path.project(state.x,state.y,s_hint=hint)
            self.cands[index]=(route,station,edge,fraction)
            if edge is not None and route.spans[edge][0]<station-1. and route is not self.route:
                continue
            generation_started=time.perf_counter()
            desired=self.v_des
            if time_left>.3:
                desired=min(max(desired,max(0.,route.goal_in-station)/time_left*1.1),
                            max(self.v_des,1.45*self.limit))
            goal_cap=max(3.,(max(route.goal_out-route.goal_in,.5)-.4)/dt)
            env_s,env_v=self.arb.envelope(route.path,station,desired,goal_cap,route.goal_in)
            suffix=None
            if self._continuation is not None and self._continuation[0] is route:
                suffix=self._continuation[1:]
            candidates=generate(self.arb,state,station,env_s,env_v,desired,suffix)
            timings['generation_ms']+=(time.perf_counter()-generation_started)*1000
            evaluation_started=time.perf_counter()
            assessed=evaluator.evaluate(candidates,route.path,station,error,state,measured,
                self.ctrl,self.lat,dt,self.rect,route.goal_in,desired,env_s,env_v,self.arb.cfg,
                time_left,previous_action,self.W_LATE*fraction)
            timings['evaluation_ms']+=(time.perf_counter()-evaluation_started)*1000
            route_id='route:%d:%.2f'%(index,fraction)
            rows=evaluator.rows(candidates,assessed,route_id)
            all_rows.extend(rows)
            batches.append((route,station,error,edge,fraction,candidates,assessed,rows,route_id))
        if not batches:
            raise RuntimeError('No route proposals remain; route generation failure')
        selection_started=time.perf_counter()
        selection=select_execution([b[6] for b in batches],[b[7] for b in batches],
            self.road_boundary,state,dt,time_left,self.EXECUTION_HORIZON)
        timings['selection_ms']=(time.perf_counter()-selection_started)*1000
        bi,ci=selection.batch,selection.candidate
        route,station,error,edge,fraction,candidates,assessed,rows,route_id=batches[bi]
        candidate=candidates[ci]
        action=assessed.commands[ci,0].tolist()
        safe=selection.feasible
        status='execution_prefix_checked_on_grid' if safe else 'risk_mitigation_no_feasible_candidate'
        if safe and rows[ci].get('road_status')=='initial_exterior_recovery_no_growth':
            status='initial_exterior_recovery_checked_on_grid'
        # Commit exactly one selected execution decision, after all routes and
        # candidate families have been evaluated under the same contract.
        checked=list(checked_action(dt,state.v,state.a,measured,*action))
        self.execution_decision=ExecutionDecision(timestamp,route_id+'/'+candidate.id,
            candidate.family,action,checked,assessed.states[ci,1].tolist(),status,route_id,
            float(candidate.acceleration[0]))
        self.ctrl.prev_steer=action[1]
        self.route,self.s=route,station
        held_target=measured if np.isnan(candidate.steering_target) else candidate.steering_target
        self._continuation=(route,candidate.acceleration.copy(),candidate.steering_mode,
                            candidate.steering_duration,held_target)
        plan=Plan(a_cmd=action[0],label=candidate.id,cost=float(assessed.cost[ci]),
            t_collide=float(assessed.first_hit[ci]),min_clear=float(assessed.clearance[ci]),
            s=assessed.stations[ci].copy(),v=assessed.states[ci,1:,3].copy(),
            a=assessed.states[ci,1:,4].copy(),all_collide=not safe,
            on_time=bool(safe and assessed.goal_time[ci]<=time_left+1e-7))
        self.arb.commit(plan)
        if edge is not None and self.s>=route.spans[edge][0]-.5:
            self.fixed[edge]=tuple(route.span_xy[edge][0])
            self.n_lock+=1
            self._make_variants()
        self._route_diagnostics.update(selected_at=timestamp,selected_edge=edge,
            selected_fraction=fraction,locked_edges=len(self.fixed),lock_count=self.n_lock)
        selected=deepcopy(rows[ci])
        selected.update(acceleration_proposal=candidate.acceleration.tolist(),
            commands=assessed.commands[ci].tolist(),states=assessed.states[ci].tolist())
        road=self.road_boundary.inspect(assessed.states[ci:ci+1,1:],state.length,state.width,
                                       assessed.terminal[ci:ci+1])
        road.pop('covered',None)
        road={key:value.tolist() if isinstance(value,np.ndarray) else value for key,value in road.items()}
        self._cycle=dict(schema='unified-execution-v1',t=timestamp,
            execution_decision=self.execution_decision.snapshot(),execution_feedback=self.execution_feedback,
            prediction_feedback=forecast_feedback,selected=selected,candidates=all_rows,
            assessment_contract=dict(horizon=float(self.arb.t[-1]),hard_execution_horizon=self.EXECUTION_HORIZON,
                future_risk_policy='forecast_clear_preferred_not_certified',dt=dt,geometry='SAT rectangles',
                selection_policy='prefix_then_executed_goal_then_forecast_clear_then_cost; risk_body_contact_delay_then_joint_road_buffer_delay_then_next_clearance',
                eta_policy='same_traffic_free_tail_estimate_for_efficiency_and_budget',
                margin0=self.arb.cfg.buf0,margin_t=self.arb.cfg.buf_t,
                hard_collision_margin='margin0+margin_t*t',buffer_policy='buffer_shortfall_and_body_contact_recorded_separately',
                local_cost_horizon=6.,comfort_cost_horizon=2.,
                extra_prediction_horizon=2.,termination='first_executed_goal_entry_inclusive'),
            module_evidence=dict(route_count=len(batches),candidate_count=len(all_rows),
                feasible_count=int(safe),feasible_count_scope='lower_bound_witness_count_lazy_road_checks',
                collision_tracking_feasible_count=sum(bool(a.eligible[i] and a.first_hit[i]>self.EXECUTION_HORIZON+1e-9)
                    for a in (b[6] for b in batches) for i in range(len(a.cost))),
                road_checked_rejected=selection.road_rejected,road_checked_count=selection.checked_count,
                road_remaining_unchecked=selection.checked_count<len(all_rows),
                initial_body_outside_area=finite(selection.initial_exterior),
                occupancy_rejected=sum('execution_prefix_occupancy_conflict' in r['rejected_by'] for r in all_rows),
                tracking_rejected=sum('tracking_corridor_exceeded' in r['rejected_by'] for r in all_rows),
                on_time_count=sum(r['budget_eta'] is not None and r['budget_eta']<=time_left+1e-7 for r in all_rows),
                executed_goal_on_time_count=sum(r['goal_time'] is not None and r['goal_time']<=time_left+1e-7 for r in all_rows),
                on_time_count_scope='includes_traffic_free_tail_estimates_not_certified_arrival',
                road_boundary=road,road_policy='lazy_feasible_gate_then_full_joint_risk_grid',time_left=time_left,
                route_issues=list(route.issues)),timings=timings)
        return action

    def safety_diagnostics(self):
        result=deepcopy(self._cycle)
        result['compound_route_geometry']=deepcopy(self.compound_route_evidence)
        return result

    def diagnostics(self):
        result=super().diagnostics()
        result['unified']=dict(entry='UnifiedPlanner',execution=self.execution_decision.snapshot()
            if self.execution_decision else None,modules=self._cycle.get('module_evidence',{}))
        return result
