"""Reproducible paired review and evidence-based module attribution for every run."""
from collections import Counter
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

PROJECT=next(parent for parent in Path(__file__).resolve().parents if (parent/'bench/sim.py').is_file())
PARENT=None


def read(path): return json.loads(path.read_text(encoding='utf-8'))
def trace(run,name):
    with gzip.open(str(run/'validation'/(name+'.json.gz')),'rt',encoding='utf-8') as f:
        return json.load(f)


def compact_candidate(row):
    return {key:value for key,value in row.items() if key not in
            ('states','commands','acceleration_proposal')}


def compact_cycle(cycle):
    return {key:compact_candidate(value) if key=='selected' else value
            for key,value in cycle.items() if key!='candidates'}


def review(run, baseline='route_seam_20261008_devreg130', output=None, mapped_audit=None):
    global PARENT
    run=Path(run).resolve()
    path=Path(baseline)
    choices=([path] if path.is_absolute() else [run.parent/path,
        PROJECT/'research/arb_capability_20261007/runs'/path])
    PARENT=next((choice.resolve() for choice in choices if (choice/'manifest.json').is_file()),None)
    if PARENT is None:
        raise FileNotFoundError('Comparison baseline not found: '+str(baseline))
    output=run if output is None else Path(output)
    output.mkdir(parents=True,exist_ok=True)
    manifest=read(run/'manifest.json')
    source_ok=all(hashlib.sha256((run/'source'/key).read_bytes()).hexdigest()==value
                  for key,value in manifest['sources'].items())
    verification=read(run/'validation/verification.json')
    validation_ok=all(verification.get(key,False) for key in ('sources_unchanged',
        'frozen_planner_unchanged','inputs_unchanged','existing_map_caches_unchanged'))
    rows=read(run/'validation/results.json')['rows']
    mapped_path=Path(mapped_audit).resolve() if mapped_audit else run/'mapped_body_audit/summary.json'
    actual_mapped={}
    mapped_identity=None
    if mapped_path.is_file():
        mapped=read(mapped_path)
        identity=next(item for item in mapped['runs'] if item['run']==run.name)
        assert mapped['source_identity_passed'] and identity['source_archive_sha256']==manifest['archive_sha256']
        assert identity['manifest_sha256']==hashlib.sha256((run/'manifest.json').read_bytes()).hexdigest()
        for item in mapped['scenes']:
            if item['run']!=run.name:continue
            trace_path=run/'validation'/(item['scene']+'.json.gz')
            assert item['trace_sha256']==hashlib.sha256(trace_path.read_bytes()).hexdigest()
            actual_mapped[item['scene']]=item['actual_mapped_boundary']
        assert {row['name'] for row in rows}<=set(actual_mapped),'Mapped audit must cover every reviewed scene'
        mapped_identity=dict(path=str(mapped_path),sha256=hashlib.sha256(mapped_path.read_bytes()).hexdigest(),
            tool_sha256=mapped['tool_sha256'],geometry_tool_sha256=mapped['geometry_tool_sha256'])
    records=[]
    failures=[]
    for result in rows:
        name=result['name']
        data,parent=trace(run,name),trace(PARENT,name)
        cycles,frames=data['cycles'],data['frames']
        actual_by_time={round(fr['t'],6):fr for fr in frames}
        state_equal=frames==parent['frames']
        action_equal=[c['action'] for c in cycles]==[c['action'] for c in parent['cycles']]
        counts=Counter()
        mismatch=[]
        risk_windows=[]
        boundary=[]
        families=Counter()
        first_budget_loss=None
        first_prefix_loss=None
        first_road_loss=None
        budget_witness=None
        impact_witness=None
        joint_witness=None
        buffer_only=None
        last_feasible=None
        for c in cycles:
            safety=c['safety']
            decision=safety.get('execution_decision')
            nxt=actual_by_time.get(round(c['t']+result['dt'],6))
            if decision and nxt:
                expected=np.asarray(decision['expected_next_state'])
                actual=np.array([nxt[key] for key in ('x','y','yaw','v','a','rot')])
                error=actual-expected
                error[2]=(error[2]+np.pi)%(2*np.pi)-np.pi
                if np.max(abs(error))>.00101:
                    mismatch.append(dict(t=c['t'],state_error=error.tolist()))
                families[decision['family']]+=1
            module=safety.get('module_evidence')
            if module:
                selected=safety['selected']
                horizon=safety['assessment_contract']['hard_execution_horizon']
                candidates=safety['candidates']
                prefix_eligible=[r for r in candidates if not any(reason in r['rejected_by']
                    for reason in ('execution_prefix_occupancy_conflict','tracking_corridor_exceeded'))]
                eligible=[r for r in candidates if not r['rejected_by']]
                estimated_on_time=[r for r in eligible if r.get('budget_eta',r['eta']) is not None
                                   and r.get('budget_eta',r['eta'])<=module['time_left']+1e-7]
                if first_prefix_loss is None and not prefix_eligible:
                    first_prefix_loss=dict(t=c['t'],time_left=module['time_left'],
                        selected=compact_candidate(selected),occupancy_rejected=module['occupancy_rejected'],
                        tracking_rejected=module['tracking_rejected'],
                        scope='No generated occupancy/tracking prefix witness; not a physical infeasibility proof')
                if not prefix_eligible:counts['no_occupancy_tracking_prefix_witness']+=1
                if prefix_eligible and module['feasible_count']==0:
                    counts['occupancy_tracking_witnesses_rejected_by_road']+=1
                    if first_road_loss is None:
                        first_road_loss=dict(t=c['t'],prefix_witness_count=len(prefix_eligible),
                            road_checked_rejected=module['road_checked_rejected'],selected=compact_candidate(selected),
                            scope='Road gate rejected generated prefix witnesses; map, route and controller may couple')
                if first_budget_loss is None and eligible and not estimated_on_time:
                    first_budget_loss=dict(t=c['t'],time_left=module['time_left'],
                        selected=compact_candidate(selected),
                        fastest_generated_prefix_eligible=min(eligible,key=lambda r:r.get('budget_eta',r['eta']) or float('inf'))
                            if eligible else None,
                        scope='Uninspected roads are candidate witnesses, not certified feasibility')
                if budget_witness is None and selected.get('budget_eta',selected['eta'])>module['time_left']+1e-7 and estimated_on_time:
                    budget_witness=dict(t=c['t'],selected=compact_candidate(selected),
                        alternative=compact_candidate(min(estimated_on_time,key=lambda r:r['cost'])),
                        scope='Estimated budget alternative; road or future conflict may explain rejection')
                if module['feasible_count']:
                    last_feasible=dict(t=c['t'],selected=compact_candidate(selected),
                        status=safety['execution_decision']['status'])
                elif selected['first_hit'] is not None:
                    later=[r for r in candidates if r['first_hit'] is None or
                           r['first_hit']>selected['first_hit']+1e-7]
                    if impact_witness is None and later:
                        impact_witness=dict(t=c['t'],selected=compact_candidate(selected),
                            alternative=compact_candidate(max(later,key=lambda r:r['first_hit'] or float('inf'))),
                            scope='Impact ordering witness; tracking and road constraints recorded separately')
                if module['feasible_count']==0 and selected.get('joint_first_violation') is not None and selected['joint_first_violation']<=horizon+1e-7:
                    contact_priority='risk_body_contact_delay' in safety['assessment_contract'].get('selection_policy','')
                    selected_contact=min(selected.get('contact_first_hit') if selected.get('contact_first_hit') is not None else float('inf'),horizon)
                    later=[r for r in candidates if r.get('road_checked') and
                        (not contact_priority or min(r.get('contact_first_hit') if r.get('contact_first_hit') is not None else float('inf'),horizon)>=selected_contact-1e-7) and
                        min(r['joint_first_violation'] if r.get('joint_first_violation') is not None else float('inf'),horizon)
                         >min(selected['joint_first_violation'],horizon)+1e-7]
                    if joint_witness is None and later:
                        joint_witness=dict(t=c['t'],selected=compact_candidate(selected),
                            alternative=compact_candidate(max(later,key=lambda r:
                                r['joint_first_violation'] if r['joint_first_violation'] is not None else float('inf'))),
                            scope='Joint road/occupancy ordering witness; not a closed-loop avoidance result')
                if selected.get('buffer_first_hit') is not None and selected['buffer_first_hit']<=horizon+1e-7:
                    contact=selected.get('contact_first_hit')
                    if contact is None or contact>horizon+1e-7:
                        counts['selected_prefix_buffer_shortfall_without_contact']+=1
                        if buffer_only is None:
                            buffer_only=dict(t=c['t'],selected=compact_candidate(selected))
                if module['feasible_count']==0:
                    counts['no_feasible_proposal']+=1
                    risk_windows.append(dict(t=c['t'],evidence=module,
                        selected={k:v for k,v in safety['selected'].items() if k not in
                            ('states','commands','acceleration_proposal')}))
                if module['on_time_count']==0: counts['no_predicted_arrival_within_remaining_budget']+=1
                road=module['road_boundary']
                if road.get('checked') and road['first_outside_step'][0]>=0:
                    counts['selected_future_body_outside_map']+=1
                    boundary.append(dict(t=c['t'],first_outside_step=road['first_outside_step'][0]))
            else:
                counts[safety.get('close_guard',{}).get('status','unknown')]+=1
        last=cycles[-1]['safety'] if cycles else {}
        collision_history=result.get('collision_history',[])
        actual_boundary=actual_mapped.get(name)
        earliest_plan_divergence=next((dict(t=c['t'],candidate=c['safety'].get('execution_decision',{}).get('candidate_id'))
            for c,old in zip(cycles,parent['cycles']) if c['action']!=old['action']),None)
        tags=[]
        input_invalid=result.get('outcome_class')=='TASK_INPUT_INVALID'
        if input_invalid:
            tags.append(dict(module='task_input_contract',evidence=result.get('task_contract_status'),
                certainty='invalid_official_target_rectangle_observed'))
        elif result.get('error'):
            tags.append(dict(module='runtime_exception',evidence=result['error'],certainty='observed'))
        if mismatch: tags.append(dict(module='execution_contract',evidence='expected_next_state_mismatch',certainty='observed'))
        if counts['no_occupancy_tracking_prefix_witness']:
            tags.append(dict(module='candidate_generation + occupancy_prediction + execution_constraints',
                evidence='no_generated_occupancy_tracking_prefix_witness',certainty='coupled_failure_requires_counterfactual'))
        if counts['occupancy_tracking_witnesses_rejected_by_road']:
            tags.append(dict(module='route + controller + mapped_boundary',
                evidence='occupancy_tracking_prefix_witnesses_rejected_by_road',
                certainty='coupled_failure_requires_geometry_counterfactual'))
        if boundary:
            tags.append(dict(module='route + controller + mapped_boundary',
                evidence='selected_future_body_leaves_mapped_union',certainty='diagnostic_not_proven_incident_cause'))
        if actual_boundary and (actual_boundary.get('first_exterior_growth_t') is not None or
                                actual_boundary.get('initial_exterior_unrecovered')):
            tags.append(dict(module='route + controller + mapped_boundary + risk_selection',
                evidence=dict(first_exterior_growth_t=actual_boundary['first_exterior_growth_t'],
                    max_exterior_area=actual_boundary['max_exterior_area'],
                    outside_frame_count=actual_boundary['outside_frame_count']),
                certainty='actual_mapped_body_exterior_observed_geometry_requires_counterfactual'))
        if result['end']=='超时':
            tags.append(dict(module='route + candidate_progress + selection_budget',
                evidence='deadline_reached_without_goal_entry',certainty='outcome_requires_no_traffic_or_candidate_witness'))
        if result['end']=='碰撞' and len(frames)>1:
            before,terminal=frames[-2],frames[-1]
            new=set(terminal['objs'])-set(before['objs'])
            tags.append(dict(module='visibility_birth' if any(h['observed_lead_time']==0 for h in collision_history) else 'prediction + occupancy + candidate_selection',
                evidence={'collision_history':collision_history,'new_actors_at_terminal':sorted(new),'last_selected':last.get('selected',{}).get('id'),
                    'predicted_first_hit':last.get('selected',{}).get('first_hit')},
                certainty='collision_partner_unobserved_before_terminal' if any(h['observed_lead_time']==0 for h in collision_history) else 'requires_actor_geometry_comparison'))
        record=dict(name=name,before=parent['result']['end'],after=result['end'],
            input_correction_equal=result.get('input_fix')==parent['result'].get('input_fix'),
            outcome_class=result.get('outcome_class',result['end']),task_input_invalid=input_invalid,
            before_t=parent['frames'][-1]['t'] if parent['frames'] else None,after_t=frames[-1]['t'] if frames else None,
            before_comfort=parent['result'].get('score',{}),after_comfort=result.get('score',{}),
            actual_mapped_boundary=actual_boundary,
            state_equal=state_equal,action_equal=action_equal,error=result.get('error'),
            latency=result.get('latency'),feedback_mismatches=mismatch,counts=dict(counts),families=dict(families),
            module_attribution=tags,first_risk=risk_windows[0] if risk_windows else None,
            last_risk=risk_windows[-1] if risk_windows else None,
            first_boundary=boundary[0] if boundary else None,
            last_cycle=compact_cycle(last),collision_history=collision_history,first_action_divergence=earliest_plan_divergence,
            decision_frontier=dict(first_generated_budget_loss=first_budget_loss,
                first_occupancy_tracking_witness_loss=first_prefix_loss,
                first_road_witness_loss=first_road_loss,
                first_budget_alternative=budget_witness,first_later_impact_alternative=impact_witness,
                first_later_joint_risk_alternative=joint_witness,
                first_buffer_only_risk=buffer_only,last_prefix_feasible_decision=last_feasible))
        records.append(record)
        if result['end']!='到达' or (actual_boundary and (actual_boundary.get('first_exterior_growth_t') is not None or
                                                       actual_boundary.get('initial_exterior_unrecovered'))):
            failures.append(record)
    lost=[r['name'] for r in records if r['before']=='到达' and r['after']!='到达']
    new_coll=[r['name'] for r in records if r['before']!='碰撞' and r['after']=='碰撞']
    errors=[r['name'] for r in records if r['error']]
    invalid=[r['name'] for r in records if r['task_input_invalid']]
    input_protocol_mismatches=[r['name'] for r in records if not r['input_correction_equal']]
    mismatch_count=sum(len(r['feedback_mismatches']) for r in records)
    mapped_checked=sum(bool(r['actual_mapped_boundary'] and r['actual_mapped_boundary'].get('checked')) for r in records)
    mapped_growth=[r['name'] for r in records if r['actual_mapped_boundary'] and
        r['actual_mapped_boundary'].get('first_exterior_growth_t') is not None]
    mapped_unrecovered=[r['name'] for r in records if r['actual_mapped_boundary'] and
        r['actual_mapped_boundary'].get('initial_exterior_unrecovered')]
    summary=dict(run=run.name,parent=PARENT.name,scenes=len(records),
        set_scope=manifest.get('set_scope','original_devreg130'),
        comparison_scope='baseline_self_attribution_only' if PARENT==run else 'paired_driving',
        outcomes=dict(Counter(r['after'] for r in records)),
        source_identity_passed=source_ok,runtime_identity_passed=validation_ok,
        state_identical_count=sum(r['state_equal'] for r in records),
        action_identical_count=sum(r['action_equal'] for r in records),
        lost_successes=lost,new_collisions=new_coll,errors=errors,
        task_input_invalid=invalid,algorithm_errors=[r['name'] for r in records if r['error'] and not r['task_input_invalid']],
        outcome_classes=dict(Counter(r['outcome_class'] for r in records)),
        input_protocol_mismatches=input_protocol_mismatches,
        gained_successes=[r['name'] for r in records if r['before']!='到达' and r['after']=='到达'],
        execution_state_mismatches=mismatch_count,
        mapped_audit_identity=mapped_identity,
        mapped_body_checked_count=mapped_checked,
        mapped_body_exterior_growth=mapped_growth,
        mapped_body_initial_exterior_unrecovered=mapped_unrecovered,
        mapped_body_grid_passed=bool(mapped_checked==len(records) and records and not(mapped_growth or mapped_unrecovered)),
        mapped_body_status='issues_observed' if mapped_growth or mapped_unrecovered else
            'no_exterior_growth_on_grid' if mapped_checked==len(records) and records else 'not_fully_checked',
        arrived_with_mapped_exterior_growth=[r['name'] for r in records if r['after']=='到达' and
            r['actual_mapped_boundary'] and r['actual_mapped_boundary'].get('first_exterior_growth_t') is not None],
        latency_pass_count=sum(bool(r['latency'] and r['latency']['passed']) for r in records),
        promotion_allowed=False,
        driving_pair_passed=PARENT!=run and not(lost or new_coll or errors or mismatch_count or input_protocol_mismatches) and source_ok and validation_ok,
        driving_pair_scope='Official recorded outcomes and execution/input parity; actual mapped-body acceptance is separate',
        review_tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=['Module labels are evidence or coupled hypotheses, not automatic causal proofs.',
                     'Mapped boundary diagnostic is not a legal-road or continuous-sweep certificate.'])
    (output/'comparison_review.json').write_text(json.dumps(dict(summary=summary,scenes=records),ensure_ascii=False,indent=2),encoding='utf-8')
    (output/'failure_attribution.json').write_text(json.dumps(dict(summary=summary,failures=failures),ensure_ascii=False,indent=2),encoding='utf-8')
    lines=['# 运行配对与失败模块证据','',f"运行：{run.name}；对照：{PARENT.name}。",'',
        '```json',json.dumps(summary,ensure_ascii=False,indent=2),'```','',
        '模块归因是证据分类；耦合问题需要逐因素反事实诊断，不把候选无解等同于物理无解。','',
        '| 场景 | 原结果→本次 | 无可行候选帧 | 未来车身出地图帧 | 实际外部面积首次增长 | 模块证据 |',
        '|---|---|---:|---:|---:|---|']
    for r in failures:
        actual=r['actual_mapped_boundary']
        lines.append('| %s | %s→%s | %s | %s | %s | %s |'%(r['name'],r['before'],r['after'],
            r['counts'].get('no_feasible_proposal',0),r['counts'].get('selected_future_body_outside_map',0),
            actual.get('first_exterior_growth_t') if actual else '未核查',
            '; '.join(t['module'] for t in r['module_attribution'])))
    lines+=['','逐候选淘汰原因、成本分项、首冲突对象/分支、执行序列及下一帧对账在每场原始压缩日志中。',
            '首次/末次风险帧与全部失败分类见 `failure_attribution.json`。']
    lines+=['','官方回放结局保留；到达但实际车身外部面积增长的场景也进入问题表。',
            '初始外伸单独记录，地图覆盖核查不等同于合法道路或连续扫掠认证。']
    lines+=['','## 决策链首次分歧','',
        '| 场景 | 首次动作分歧 | 占用/跟踪见证丢失 | 道路见证丢失 | 有候选但预算丢失 | 最后前缀可行决策 |',
        '|---|---:|---:|---:|---:|---:|']
    for r in failures:
        frontier=r['decision_frontier']
        def moment(value):return value['t'] if value else ''
        lines.append('| %s | %s | %s | %s | %s | %s |'%(r['name'],
            moment(r['first_action_divergence']),moment(frontier['first_occupancy_tracking_witness_loss']),
            moment(frontier['first_road_witness_loss']),moment(frontier['first_generated_budget_loss']),
            moment(frontier['last_prefix_feasible_decision'])))
    lines+=['','表中候选见证来自当帧已保存的评估，不是反事实闭环结果；道路未查、预测支和成本分项均在JSON中保留。']
    (output/'REVIEW.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2))
    return summary


if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    parser=argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    parser.add_argument('--baseline',default='route_seam_20261008_devreg130')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--mapped-audit',type=Path,help='Identity-bound independent mapped-body audit summary')
    args=parser.parse_args()
    review(args.run,args.baseline,args.output,args.mapped_audit)
