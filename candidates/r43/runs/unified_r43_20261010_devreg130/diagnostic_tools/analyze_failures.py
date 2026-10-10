"""Collision-partner first-step counterfactuals from frozen observations only."""
import argparse,gzip,hashlib,json,sys
from pathlib import Path
from collections import Counter
import numpy as np

sys.dont_write_bytecode=True
sys.stdout.reconfigure(encoding='utf-8')


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--review',type=Path,help='Use an independent review instead of the frozen run report')
    ap.add_argument('--output',type=Path,help='Write diagnostics outside the frozen run')
    args=ap.parse_args()
    run=Path(args.run).resolve()
    review_path=args.review.resolve() if args.review else run/'comparison_review.json'
    output=args.output.resolve() if args.output else run
    output.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
    assert all(hashlib.sha256((run/'source'/key).read_bytes()).hexdigest()==value
               for key,value in manifest['sources'].items())
    sys.path.insert(0,str(run/'source'))
    from stack.core.world import Agent,WorldModel
    from stack.core.arbiter import _obb_separation
    review=json.loads(review_path.read_text(encoding='utf-8'))
    evidence=[]
    for row in review['scenes']:
        if row['after']=='到达': continue
        with gzip.open(run/'validation'/(row['name']+'.json.gz'),'rt',encoding='utf-8') as f:
            data=json.load(f)
        cycles,frames=data['cycles'],data['frames']
        record=dict(scene=row['name'],before=row['before'],outcome=row['after'],
            outcome_class=row.get('outcome_class',row['after']),
            module_attribution=row['module_attribution'],
            module_counts=row['counts'],execution_mismatches=row['feedback_mismatches'],
            first_risk=row['first_risk'],first_boundary=row['first_boundary'],
            decision_frontier=row.get('decision_frontier'),
            collision_partners=[])
        if row['after']=='碰撞' and len(frames)>1:
            world=WorldModel()
            for frame in frames[:-1]:
                agents=[Agent(key,key.split('/')[0],value[0],value[1],value[3],value[2],value[4],value[5])
                        for key,value in frame['objs'].items()]
                world.update(frame['t'],agents)
            previous,actual=frames[-2],frames[-1]
            dt=data['result']['dt']
            pred=world.predict(np.arange(1,int(round(6./dt))+1)*dt,
                near_xy=(previous['x'],previous['y']),near_speed=previous['v'],
                near_extent=.5*np.hypot(previous['L'],previous['W']))
            for history in data['result'].get('collision_history',[]):
                key=history['id']
                obj=actual['objs'][key]
                true_gap=float(_obb_separation(actual['x'],actual['y'],actual['yaw'],actual['L'],actual['W'],
                                              obj[0],obj[1],obj[3],obj[4],obj[5]))
                partner=dict(actor=key,observed_lead_time=history['observed_lead_time'],
                             actual_next_gap=true_gap,stopped_duration=history.get('stopped_duration'))
                if key not in previous['objs']:
                    partner.update(classification='unobserved_collision_partner_birth',
                        causal_boundary='No prior observation for this actor; earlier policy may still affect exposure.')
                elif key not in pred['ids']:
                    partner.update(classification='prediction_broadphase_excluded_visible_actor')
                else:
                    index=pred['ids'].index(key)
                    old=previous['objs'][key]
                    direction=np.array([np.cos(old[3]),np.sin(old[3])])
                    branches=[('W1',pred['pos'][index,0],pred['yaw'][index,0]),
                              ('observed_heading_cv',np.array(old[:2])+old[2]*dt*direction,old[3])]
                    if world.tracks[key].accel>=0.:
                        branches.append(('supported_acceleration',np.array(old[:2])+(old[2]*dt+.75*dt*dt)*direction,old[3]))
                    branch_rows=[]
                    for name,position,yaw in branches:
                        gap=float(_obb_separation(actual['x'],actual['y'],actual['yaw'],actual['L'],actual['W'],
                                                 position[0],position[1],yaw,obj[4],obj[5]))
                        branch_rows.append(dict(branch=name,gap=gap,
                            position_error=float(np.linalg.norm(position-np.array(obj[:2]))),
                            heading_error=float((yaw-obj[3]+np.pi)%(2*np.pi)-np.pi)))
                    minimum=min(b['gap'] for b in branch_rows)
                    margin=cycles[-1]['safety'].get('assessment_contract',{}).get('margin0',.15)+dt*.06
                    partner.update(branches=branch_rows,margin=margin,
                        classification='prediction_footprint_missed_first_step' if minimum>margin else 'predicted_first_step_conflict_returned_as_risk',
                        causal_boundary='Conditioned on final ego state; does not prove early avoidance feasibility.')
                record['collision_partners'].append(partner)
        evidence.append(record)
    counts=Counter(p['classification'] for r in evidence for p in r['collision_partners'])
    result=dict(run=run.name,method='Rebuild WorldModel from saved past observations; no future actor inputs in predictions',
                source_archive_sha256=manifest['archive_sha256'],
                review_sha256=hashlib.sha256(review_path.read_bytes()).hexdigest(),
                tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                collision_partner_classes=dict(counts),failures=evidence)
    (output/'module_forensics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    lines=['# 失败模块与最后一步反事实证据','',
           '预测只用保存的历史观测重建；下一帧真实周车仅用于事后误差对照，不反馈算法。',
           '最后一步分析不能代替早期可恢复性证明。','',
           '| 场景 | 结果 | 碰撞对象/机制 | 执行模型失配 |',
           '|---|---|---|---:|']
    for row in evidence:
        labels='; '.join(p['actor']+': '+p['classification'] for p in row['collision_partners'])
        if row['outcome_class']=='TASK_INPUT_INVALID':labels='task_input_contract: invalid target rectangle; no driving frames'
        if not labels: labels='见首次无解/边界/预算证据；尚无唯一根因'
        lines.append('| %s | %s | %s | %s |'%(row['scene'],row['outcome'],labels,len(row['execution_mismatches'])))
    (output/'MODULE_FORENSICS.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps(dict(run=run.name,collision_partner_classes=dict(counts)),ensure_ascii=False))


if __name__=='__main__': main()
