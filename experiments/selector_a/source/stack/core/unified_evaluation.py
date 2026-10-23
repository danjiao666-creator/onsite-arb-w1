"""One execution/occupancy assessment for main, recovery and continuation."""
from dataclasses import dataclass
import numpy as np
from numba import njit
from .native_execution import rollout_projected
from .goal_contract import goal_contains
from .tail_eta import continuation
from .decision_record import finite


@njit
def occupancy(states, positions, headings, lengths, widths, enabled, terminal,
              ego_length, ego_width, dt, margin0, margin_t):
    candidates, count = states.shape[0], states.shape[1]-1
    actors = positions.shape[1]
    first = np.full(candidates, np.inf)
    buffer_first = np.full(candidates, np.inf)
    contact_first = np.full(candidates, np.inf)
    clearance = np.full(candidates, np.inf)
    prefix_clearance = np.full(candidates, np.inf)
    next_clearance = np.full(candidates, np.inf)
    actor_index = np.full(candidates, -1, dtype=np.int64)
    branch_index = np.full(candidates, -1, dtype=np.int64)
    contact_actor = np.full(candidates, -1, dtype=np.int64)
    contact_branch = np.full(candidates, -1, dtype=np.int64)
    clearance_cost = np.zeros(candidates)
    headway_cost = np.zeros(candidates)
    actor_cos, actor_sin = np.cos(headings), np.sin(headings)
    for ci in range(candidates):
        for k in range(count):
            if k > terminal[ci]:
                break
            x, y, yaw, speed = states[ci,k+1,:4]
            ce, se = np.cos(yaw), np.sin(yaw)
            local_min = np.inf
            for branch in range(positions.shape[0]):
                for ai in range(actors):
                    if not enabled[branch,ai,k]:
                        continue
                    dx, dy = positions[branch,ai,k,0]-x, positions[branch,ai,k,1]-y
                    h = headings[branch,ai,k]
                    ca, sa = actor_cos[branch,ai,k], actor_sin[branch,ai,k]
                    para, cross = abs(ce*ca+se*sa), abs(ce*sa-se*ca)
                    el, ew = ego_length*.5, ego_width*.5
                    al, aw = lengths[ai]*.5, widths[ai]*.5
                    gap = max(abs(dx*ce+dy*se)-el-al*para-aw*cross,
                              abs(-dx*se+dy*ce)-ew-al*cross-aw*para,
                              abs(dx*ca+dy*sa)-al-el*para-ew*cross,
                              abs(-dx*sa+dy*ca)-aw-el*cross-ew*para)
                    local_min = min(local_min, gap)
                    if gap <= margin0+margin_t*(k+1)*dt and not np.isfinite(buffer_first[ci]):
                        buffer_first[ci]=(k+1)*dt
                    if gap <= 0. and not np.isfinite(contact_first[ci]):
                        contact_first[ci]=(k+1)*dt
                        contact_actor[ci],contact_branch[ci]=ai,branch
                    if gap <= margin0+margin_t*(k+1)*dt and not np.isfinite(first[ci]):
                        first[ci]=(k+1)*dt
                        actor_index[ci], branch_index[ci] = ai, branch
                    # Headway uses the same predicted occupancy at each time.
                    # Apply once to W1, for ahead, aligned, laterally overlapping bodies.
                    longitudinal = dx*ce+dy*se
                    if branch == 0 and longitudinal > 0. and ca*ce+sa*se > .5:
                        lateral = abs(-dx*se+dy*ce)
                        if lateral < ew+al*cross+aw*para:
                            front_gap = longitudinal-el-al*para-aw*cross
                            deficit = max(0., .8*speed+2.-front_gap)
                            headway_cost[ci] += deficit*deficit*dt
            clearance[ci] = min(clearance[ci], local_min)
            if (k+1)*dt <= 2.+1e-9:
                prefix_clearance[ci] = min(prefix_clearance[ci], local_min)
            if k == 0:
                next_clearance[ci] = local_min
            clearance_cost[ci] += max(0., 1.-local_min)**2*dt
    return (first, clearance, actor_index, branch_index, clearance_cost,
            headway_cost, prefix_clearance, next_clearance, contact_actor, contact_branch,
            buffer_first, contact_first)


@dataclass
class Assessment:
    states: np.ndarray
    commands: np.ndarray
    stations: np.ndarray
    first_hit: np.ndarray
    clearance: np.ndarray
    collision_actor: np.ndarray
    collision_branch: np.ndarray
    goal_time: np.ndarray
    eta: np.ndarray
    cost: np.ndarray
    lateral_error: np.ndarray
    eligible: np.ndarray
    cost_terms: dict
    terminal: np.ndarray
    prefix_clearance: np.ndarray
    next_clearance: np.ndarray
    buffer_first_hit: np.ndarray
    contact_first_hit: np.ndarray
    budget_eta: np.ndarray
    contact_actor: np.ndarray
    contact_branch: np.ndarray


def execution_rank(assessment, index, time_left, hard_horizon=2.):
    hit = assessment.first_hit[index]
    eligible = bool(assessment.eligible[index])
    goal_witness = assessment.goal_time[index] <= time_left+1e-7
    if eligible and hit > hard_horizon+1e-9:
        return (0, not goal_witness, np.isfinite(hit), assessment.cost[index])
    # Comfort cannot buy an earlier predicted impact. Within one grid step,
    # protect immediate separation before comparing the rest of the prefix.
    return (1, -min(hit, hard_horizon), -assessment.next_clearance[index],
            -assessment.prefix_clearance[index], not eligible, assessment.cost[index])


@dataclass
class Selection:
    batch: int
    candidate: int
    feasible: bool
    checked_count: int
    road_rejected: int
    initial_exterior: float
    progress: dict = None


def select_original_execution(assessments, rows, boundary, ego, dt, time_left, hard_horizon=2.):
    steps=int(round(hard_horizon/dt))
    initial=float(boundary.outside_area(np.array([[ego.x,ego.y,ego.yaw]]),
                                        ego.length,ego.width)[0])
    ordered=sorted((execution_rank(a,ci,time_left,hard_horizon),bi,ci)
                   for bi,a in enumerate(assessments) for ci in range(len(a.cost)))
    checked={}
    exterior={}

    def inspect(bi,indices):
        a=assessments[bi]
        indices=np.asarray(indices,dtype=int)
        terminal=np.minimum(a.terminal[indices],steps-1)
        states=a.states[indices,1:steps+1]
        result=boundary.inspect(states,ego.length,ego.width,terminal)
        outside=result.get('first_outside_step')
        for offset,ci in enumerate(indices):
            ci=int(ci)
            row=rows[bi][ci]
            first=int(outside[offset]) if outside is not None else None
            row.update(road_status=result['status'],road_checked=result['checked'],
                       road_first_outside_step=first)
            good=bool(result['checked'] and first<0)
            if not good and result['checked'] and initial>0.:
                good=boundary.recovers_initial_exterior(states[offset],ego.length,ego.width,
                                                        initial,terminal[offset])
                if good:row['road_status']='initial_exterior_recovery_no_growth'
            road_hit=(first+1)*dt if result['checked'] and not good else np.inf
            row['road_first_violation']=finite(road_hit)
            checked[bi,ci]=(good,road_hit)
            if not good:
                row['rejected_by'].append('mapped_road_body_outside' if result['checked']
                                          else 'mapped_road_body_unverified')

    def selected(bi,ci,feasible):
        return Selection(bi,ci,feasible,len(checked),
                         sum(not value[0] for value in checked.values()),initial)

    for rank,bi,ci in ordered:
        if rank[0]!=0:break
        inspect(bi,[ci])
        if checked[bi,ci][0]:return selected(bi,ci,True)

    # No joint feasible witness: include every proposal in the same road and
    # occupancy comparison, including proposals rejected by only one gate.
    for bi,a in enumerate(assessments):
        pending=[ci for ci in range(len(a.cost)) if (bi,ci) not in checked]
        if pending:inspect(bi,pending)
        rejected=[ci for ci in range(len(a.cost)) if not checked[bi,ci][0]]
        if rejected:
            areas=boundary.outside_area(a.states[rejected,:steps+1],ego.length,ego.width)
            for offset,ci in enumerate(rejected):
                valid=areas[offset,:min(int(a.terminal[ci])+2,steps+1)]
                exterior[bi,ci]=(float(np.max(valid)),float(np.sum(valid)))
    risks=[]
    for _,bi,ci in ordered:
        a=assessments[bi]
        impact=min(a.first_hit[ci],checked[bi,ci][1],hard_horizon)
        area=exterior.get((bi,ci),(0.,0.))
        contact=min(a.contact_first_hit[ci],hard_horizon)
        rank=(-contact,-impact,-a.next_clearance[ci],-a.prefix_clearance[ci],
              not bool(a.eligible[ci]),*area,a.cost[ci])
        row=rows[bi][ci]
        row['joint_first_violation']=finite(min(a.first_hit[ci],checked[bi,ci][1]))
        row['risk_body_contact_horizon']=hard_horizon
        row['road_max_exterior_area']=area[0]
        risks.append((rank,bi,ci))
    _,bi,ci=min(risks)
    return selected(bi,ci,False)


def select_execution(assessments, rows, boundary, ego, dt, time_left, hard_horizon=2.):
    from copy import deepcopy
    original = select_original_execution(assessments, rows, boundary, ego, dt, time_left, hard_horizon)
    baseline_row = deepcopy(rows[original.batch][original.candidate])
    admitted = []
    counts = dict(prefix_ineligible=0, future_occupancy=0, nonfinite=0,
                  prefix_road_failed=0, full_road_failed=0, full_road_unknown=0)
    prefix_steps = int(round(hard_horizon / dt))
    for bi, a in enumerate(assessments):
        pending = []
        for ci in range(len(a.cost)):
            row = rows[bi][ci]
            if not bool(a.eligible[ci]):
                counts['prefix_ineligible'] += 1
                row['progress_status'] = 'prefix_ineligible'
            elif not np.isposinf(a.first_hit[ci]):
                counts['future_occupancy'] += 1
                row['progress_status'] = 'forecast_occupancy_conflict'
            elif not (np.isfinite(a.eta[ci]) and np.isfinite(a.cost[ci]) and
                      np.isfinite(a.states[ci]).all() and np.isfinite(a.commands[ci]).all()):
                counts['nonfinite'] += 1
                row['progress_status'] = 'nonfinite_execution'
            else:
                pending.append(ci)
        if not pending:
            continue
        indices = np.asarray(pending, dtype=int)
        prefix = boundary.inspect(a.states[indices, 1:prefix_steps+1], ego.length, ego.width,
                                  np.minimum(a.terminal[indices], prefix_steps-1))
        full = boundary.inspect(a.states[indices, 1:], ego.length, ego.width, a.terminal[indices])
        for offset, ci in enumerate(pending):
            row = rows[bi][ci]
            pfirst = int(prefix['first_outside_step'][offset]) if prefix.get('checked') else None
            ffirst = int(full['first_outside_step'][offset]) if full.get('checked') else None
            pg = bool(prefix.get('checked') and pfirst < 0)
            fg = bool(full.get('checked') and ffirst < 0)
            if not pg and prefix.get('checked') and original.initial_exterior > 0.:
                pg = boundary.recovers_initial_exterior(a.states[ci, 1:prefix_steps+1],
                    ego.length, ego.width, original.initial_exterior, min(a.terminal[ci], prefix_steps-1))
            if not fg and full.get('checked') and original.initial_exterior > 0.:
                fg = boundary.recovers_initial_exterior(a.states[ci, 1:], ego.length, ego.width,
                                                       original.initial_exterior, a.terminal[ci])
            row.update(road_status=prefix['status'], road_checked=bool(prefix.get('checked')),
                       road_first_outside_step=pfirst,
                       road_first_violation=finite((pfirst+1)*dt) if not pg and pfirst is not None else None,
                       full_window_road_checked=bool(full.get('checked')),
                       full_window_first_outside_step=ffirst,
                       full_window_road_pass=bool(fg))
            if pg and pfirst >= 0:
                row['road_status'] = 'initial_exterior_recovery_no_growth'
            if not pg:
                tag = 'mapped_road_body_outside' if prefix.get('checked') else 'mapped_road_body_unverified'
                if tag not in row['rejected_by']:
                    row['rejected_by'].append(tag)
                counts['prefix_road_failed'] += 1
                row['progress_status'] = 'prefix_road_failed'
            elif not full.get('checked'):
                counts['full_road_unknown'] += 1
                row['progress_status'] = 'full_road_unknown'
            elif not fg:
                counts['full_road_failed'] += 1
                row['progress_status'] = 'full_road_failed'
            else:
                row['progress_status'] = 'full_window_admitted'
                key = (bool(a.eta[ci] > time_left+1e-7),
                       int(np.floor((a.eta[ci]+1e-7)/dt)), float(a.cost[ci]), bi, ci)
                admitted.append((key, bi, ci))
    chosen = original
    if admitted:
        _, bi, ci = min(admitted)
        checked = sum(bool(r.get('road_checked')) for group in rows for r in group)
        rejected = sum(any(x.startswith('mapped_road_body_') for x in r['rejected_by'])
                       for group in rows for r in group)
        chosen = Selection(bi, ci, True, checked, rejected, original.initial_exterior)
    chosen.checked_count = sum(bool(r.get('road_checked')) for group in rows for r in group)
    chosen.road_rejected = sum(any(x.startswith('mapped_road_body_') for x in r['rejected_by'])
                              for group in rows for r in group)
    changed = (chosen.batch, chosen.candidate) != (original.batch, original.candidate)
    changed_action = not np.array_equal(assessments[chosen.batch].commands[chosen.candidate, 0],
                                       assessments[original.batch].commands[original.candidate, 0])
    chosen.progress = dict(status='full_window_eta_ranked' if admitted else 'no_full_window_progress_candidate',
        full_window_admitted_count=len(admitted), rejection_counts=counts,
        original_batch=original.batch, original_candidate=original.candidate,
        original_feasible=original.feasible, original_selected=baseline_row,
        candidate_changed=changed, action_changed=changed_action,
        selected_full_window_admitted=bool(admitted),
        forecast_scope='original_enabled_branches_only', eta_scope='original_traffic_free_tail_estimate')
    return chosen


class ExecutionEvaluator:
    branch_names = ('W1', 'observed_heading_cv', 'supported_acceleration')

    def __init__(self, world, times, ego):
        self.times = times
        self.prediction = p = world.predict(times, near_xy=(ego.x,ego.y),
            near_speed=ego.v, near_extent=.5*np.hypot(ego.length,ego.width))
        n, k = len(p['ids']), len(times)
        self.positions = np.zeros((3,n,k,2))
        self.headings = np.zeros((3,n,k))
        self.enabled = np.ones((3,n,k), dtype=np.bool_)
        if n:
            self.positions[0], self.headings[0] = p['pos'], p['yaw']
            actors = [world.tracks[key].agent for key in p['ids']]
            yaw = np.array([a.yaw for a in actors])
            speed = np.array([a.v for a in actors])
            direction = np.stack((np.cos(yaw),np.sin(yaw)),axis=1)
            cv = p['now'][:,None,:]+speed[:,None,None]*times[None,:,None]*direction[:,None,:]
            self.positions[1] = cv
            self.positions[2] = cv+.75*times[None,:,None]**2*direction[:,None,:]
            self.headings[1:] = yaw[None,:,None]
            self.enabled[1:] &= times[None,None,:] <= 2.+1e-9
            supported = np.array([world.tracks[key].accel>=0. for key in p['ids']])
            self.enabled[2] &= supported[:,None]

    def evaluate(self, candidates, path, station, initial_error, ego, measured,
                 ctrl, lat, dt, rect, goal_in, desired, env_s, env_v, cfg,
                 time_left, previous_action, extra_cost):
        initial = np.array([ego.x,ego.y,ego.yaw,ego.v,ego.a,measured])
        commands = np.stack([c.acceleration for c in candidates])
        modes = np.array([c.steering_mode for c in candidates],dtype=np.int64)
        durations=np.array([c.steering_duration for c in candidates])
        targets=np.array([c.steering_target for c in candidates])
        states, controls, stations = rollout_projected(initial,commands,path,station,ctrl,lat,dt,ego.length,modes,durations,targets)
        inside = goal_contains(rect,states[:,1:,0],states[:,1:,1])
        terminal = np.where(inside.any(-1),inside.argmax(-1),len(self.times)-1)
        goal_time = np.where(inside.any(-1),self.times[terminal],np.inf)
        p = self.prediction
        first, clear, actor, branch, clearance_cost, headway, prefix_clear, next_clear, contact_actor, contact_branch, buffer_first, contact_first = occupancy(
            states,self.positions,self.headings,p['L'],p['W'],self.enabled,terminal,
            ego.length,ego.width,dt,cfg.buf0,cfg.buf_t)
        px,py = path.xy(stations)
        hd = path.heading(stations)
        lateral = np.abs(-(states[:,1:,0]-px)*np.sin(hd)+(states[:,1:,1]-py)*np.cos(hd))
        valid = np.arange(len(self.times))[None,:] <= terminal[:,None]
        max_error = np.where(valid & (self.times[None,:]<=2.+1e-9),lateral,0.).max(-1)
        eligible = max_error <= max(abs(initial_error)+.5,1.5)
        remaining = continuation(stations[:,-1],states[:,-1,3],goal_in+.3,env_s,env_v,max(desired,1.))
        budget_eta = np.where(np.isfinite(goal_time),goal_time,self.times[-1]+remaining)
        eta = budget_eta.copy()
        short = min(len(self.times),int(round(2./dt)))
        actual_a = states[:,1:,4]
        terms = dict(eta=eta,
            budget=cfg.w_budget*np.maximum(eta-max(time_left,0.),0.) if time_left>0 else np.zeros(len(candidates)),
            acceleration=cfg.w_acc*(actual_a[:,:short]**2).sum(-1)*dt,
            jerk=cfg.w_jerk*(np.diff(states[:,:short+1,4],axis=1)/dt)**2,
            clearance=cfg.w_clear*clearance_cost, headway=cfg.w_headway*headway,
            overspeed=cfg.w_over*(np.maximum(states[:,1:,3]-desired,0.)**2*valid).sum(-1)*dt,
            consistency=cfg.w_cons*np.abs(controls[:,0,0]-previous_action),
            tracking=2.*(lateral*lateral*valid).sum(-1)*dt,
            steering_override=np.array([0. if c.steering_mode==0 else 10. for c in candidates]),
            future_risk=np.where(np.isposinf(first),0.,20./(first+.1)),
            route=np.full(len(candidates),extra_cost))
        terms['jerk'] = terms['jerk'].sum(-1)*dt
        cost = sum(terms.values())
        return Assessment(states,controls,stations,first,clear,actor,branch,goal_time,
                          eta,cost,max_error,eligible,terms,terminal,prefix_clear,next_clear,buffer_first,contact_first,budget_eta,
                          contact_actor,contact_branch)

    def rows(self,candidates,assessment,route_id):
        p = self.prediction
        rows=[]
        for i,c in enumerate(candidates):
            ai,bi=assessment.collision_actor[i],assessment.collision_branch[i]
            ca,cb=assessment.contact_actor[i],assessment.contact_branch[i]
            reasons=[]
            if assessment.first_hit[i]<=2.+1e-9: reasons.append('execution_prefix_occupancy_conflict')
            if not assessment.eligible[i]: reasons.append('tracking_corridor_exceeded')
            rows.append(dict(id=route_id+'/'+c.id,family=c.family,
                steering_mode=c.steering_mode,steering_duration=c.steering_duration,
                steering_target=finite(c.steering_target),
                first_hit=finite(assessment.first_hit[i]),clearance=finite(assessment.clearance[i]),
                prefix_clearance=finite(assessment.prefix_clearance[i]),
                next_clearance=finite(assessment.next_clearance[i]),
                buffer_first_hit=finite(assessment.buffer_first_hit[i]),
                contact_first_hit=finite(assessment.contact_first_hit[i]),
                execution_prefix_buffer_shortfall=bool(assessment.buffer_first_hit[i]<=2.+1e-9),
                actor=p['ids'][ai] if ai>=0 else None,
                branch=self.branch_names[bi] if bi>=0 else None,
                contact_actor=p['ids'][ca] if ca>=0 else None,
                contact_branch=self.branch_names[cb] if cb>=0 else None,
                goal_time=finite(assessment.goal_time[i]),eta=finite(assessment.eta[i]),
                budget_eta=finite(assessment.budget_eta[i]),
                arrival_evidence='executed_goal_entry' if np.isfinite(assessment.goal_time[i]) else 'traffic_free_tail_estimate',
                cost=float(assessment.cost[i]), costs={key:float(value[i]) for key,value in assessment.cost_terms.items()},
                max_lateral_error=float(assessment.lateral_error[i]),rejected_by=reasons,
                future_risk=bool(np.isfinite(assessment.first_hit[i]) and assessment.first_hit[i]>2.+1e-9),
                first_command=assessment.commands[i,0].tolist(),
                expected_first_state=assessment.states[i,1].tolist()))
        return rows
