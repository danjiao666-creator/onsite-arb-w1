"""Forecast-derived lane-occupancy events; proposals still require full assessment."""
from dataclasses import fields
import numpy as np
from numba import njit
from .candidates import Candidate
from .native_execution import execute, interpolate


def lane_interval(path, station, actor_xy, actor_heading, ego_length, ego_width,
                  actor_width, margin, seed_station):
    grid = np.r_[np.arange(station, path.length, .5), path.length]
    points = np.column_stack(path.xy(grid))
    normal = np.array([-np.sin(actor_heading), np.cos(actor_heading)])
    cross = np.abs(np.sin(path.heading(grid)-actor_heading))
    extent = .5*(ego_width+actor_width)+.5*ego_length*cross+margin
    overlap = np.abs((points-actor_xy) @ normal) <= extent
    blocks, start = [], None
    for index, flag in enumerate(np.r_[overlap, False]):
        if flag and start is None:
            start = index
        elif not flag and start is not None:
            blocks.append((start, index-1))
            start = None
    if not blocks:
        return None
    lo, hi = min(blocks, key=lambda b: abs(.5*(grid[b[0]]+grid[b[1]])-seed_station))
    return float(max(station, grid[lo]-.5)), float(min(path.length, grid[hi]+.5))


def forecast_events(evaluator, assessed, candidates, path, station, ego, timestamp, cfg):
    indices = [i for i,c in enumerate(candidates)
               if c.family == 'envelope_progress' and np.isfinite(assessed.first_hit[i])]
    indices.sort(key=lambda i: (assessed.first_hit[i], assessed.eta[i], i))
    events, used = [], set()
    for ci in indices:
        ai, bi = int(assessed.collision_actor[ci]), int(assessed.collision_branch[ci])
        if ai < 0 or bi < 0:
            continue
        actor_id = evaluator.prediction['ids'][ai]
        if actor_id in used:
            continue
        used.add(actor_id)
        hit = float(assessed.first_hit[ci])
        k = min(len(evaluator.times)-1, max(0, int(round(hit/cfg.dt))-1))
        heading = float(evaluator.headings[bi,ai,k])
        actor_xy = evaluator.prediction['now'][ai]
        margin = cfg.buf0+cfg.buf_t*hit
        interval = lane_interval(path, station, actor_xy, heading, ego.length, ego.width,
            evaluator.prediction['W'][ai], margin, assessed.stations[ci,k])
        if interval is None:
            continue
        enter, leave = interval
        direction = np.array([np.cos(heading), np.sin(heading)])
        active = np.flatnonzero(evaluator.enabled[bi,ai])
        if not len(active):
            continue
        times = evaluator.times[active]
        positions = evaluator.positions[bi,ai,active]
        extent = .5*(ego.length+evaluator.prediction['L'][ai])+margin
        before_projection = (positions-np.asarray(path.xy(leave))) @ direction
        event_station = float(assessed.stations[ci,k])
        after_projection = (positions-np.asarray(path.xy(event_station))) @ direction
        before = np.flatnonzero(before_projection >= -extent)
        after = np.flatnonzero(after_projection >= extent)
        if len(before) and before[0] > 0 and leave < path.length-.5:
            deadline = float(times[before[0]-1])
            events.append(dict(side='before', actor=actor_id, branch=bi,
                station=leave, deadline_abs=timestamp+deadline, created_at=timestamp,
                interval=[enter,leave], seed_hit=hit, seed_station=float(assessed.stations[ci,k]), seed_candidate=candidates[ci].id,
                forecast_last_enabled=float(times[-1]), time_basis='forecast_grid_previous_clear_step'))
        if len(after) and event_station > station+.5:
            deadline = float(times[after[0]])
            events.append(dict(side='after', actor=actor_id, branch=bi,
                station=event_station, deadline_abs=timestamp+deadline, created_at=timestamp,
                interval=[enter,leave], seed_hit=hit, seed_station=float(assessed.stations[ci,k]), seed_candidate=candidates[ci].id,
                forecast_last_enabled=float(times[-1]), time_basis='forecast_grid_first_actor_ahead'))
        if len(used) >= 2:
            break
    return events[:4]


@njit
def _event_profile(initial, points, cumulative, headings, curvature, station0, memory0,
                   dt, length, ke, kpsi, preview, acc_min, acc_max, steer_max, steer_rate,
                   wheelbase_factor, env_s, env_v, desired, count, event_station, event_left,
                   before, normal_jerk, emergency_jerk):
    current = initial.copy()
    memory, station = memory0, station0
    proposal = np.zeros(count)
    for k in range(count):
        cap = interpolate(station, env_s, env_v)
        remaining = event_left-k*dt
        if station < event_station and remaining > dt*.5:
            distance = max(0., event_station-station)
            target_speed = max(0., 2.*distance/max(remaining,dt)-current[3])
            if before:
                target_speed = max(desired, target_speed)
            target_speed = min(cap, target_speed)
        else:
            target_speed = min(cap,desired)
        target = min(9.8,max(-9.8,1.5*(target_speed-current[3])))
        jerk = emergency_jerk if target < -3.5 else normal_jerk
        command = current[4]+min(max(target-current[4],-jerk*dt),jerk*dt)
        states, controls, stations = execute(current,np.array([[command]]),points,cumulative,
            headings,curvature,station,memory,dt,length,ke,kpsi,preview,
            acc_min,acc_max,steer_max,steer_rate,wheelbase_factor,
            np.zeros(1,dtype=np.int64),np.zeros(1),np.full(1,np.nan))
        next_cap = interpolate(stations[0,0],env_s,env_v)
        if states[0,1,3] > next_cap+1e-7:
            rounded_cap = np.floor(next_cap*1000.+1e-9)/1000.
            command = min(command,(rounded_cap-current[3])/dt)
            states,controls,stations = execute(current,np.array([[command]]),points,cumulative,
                headings,curvature,station,memory,dt,length,ke,kpsi,preview,
                acc_min,acc_max,steer_max,steer_rate,wheelbase_factor,
                np.zeros(1,dtype=np.int64),np.zeros(1),np.full(1,np.nan))
        proposal[k] = command
        current = states[0,1].copy()
        station = stations[0,0]
        memory = controls[0,0,1]
    return proposal


def generate_gap_events(evaluator, assessed, candidates, arb, ego, station, env_s, env_v,
                        desired, path, measured, ctrl, lat, timestamp, committed=None):
    events = forecast_events(evaluator,assessed,candidates,path,station,ego,timestamp,arb.cfg)
    if committed is not None and committed['station'] > station and committed['deadline_abs'] > timestamp:
        events = [dict(committed)]+[e for e in events
            if (e['actor'],e['side']) != (committed['actor'],committed['side'])]
    events = events[:4]
    initial = np.array([ego.x,ego.y,ego.yaw,ego.v,ego.a,measured])
    lim = ctrl.lim
    added = []
    for index,event in enumerate(events):
        proposal = _event_profile(initial,path.pts,path.cum,path._hd_unwrap,path._curv,
            station,ctrl.prev_steer,arb.cfg.dt,ego.length,lat.k_e,lat.k_psi,lat.preview_s,
            lim.acc_min,lim.acc_max,lim.steer_max,lim.steer_rate,lim.wheelbase_factor,
            env_s,env_v,desired,arb.n,event['station'],event['deadline_abs']-timestamp,
            event['side']=='before',arb.cfg.jerk,arb.cfg.jerk_emerg)
        added.append(Candidate('gap:%s:%s:%d'%(event['actor'],event['side'],index),
            'gap_event',proposal,event=event,original_reference=False))
    return added


def combine_assessments(first, second):
    for field in fields(first):
        left,right = getattr(first,field.name),getattr(second,field.name)
        if isinstance(left,dict):
            setattr(first,field.name,{k:np.concatenate((v,right[k])) for k,v in left.items()})
        elif isinstance(left,np.ndarray):
            setattr(first,field.name,np.concatenate((left,right)))
    return first
