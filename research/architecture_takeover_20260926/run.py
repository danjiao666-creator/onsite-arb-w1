"""Development-only capability comparison. Never reads the holdout set."""
import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
from multiprocessing import Pool
import os
from pathlib import Path
import subprocess
import sys
import time
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT), str(ROOT / 'bench')]
# ruff: noqa: E402 -- standalone runner imports the local harness after path setup.
import sim
from score import score_run


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')


def hashes():
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in list((ROOT / 'stack').rglob('*.py')) + list((ROOT / 'bench').glob('*.py'))}


def act_latency(calls, dt, runtime_error=None):
    """Include every timed act attempt; an exception can never pass acceptance."""
    if not calls:
        return None
    durations = np.asarray([r['ms'] for r in calls])
    period_ms = dt * 1000
    missed = durations > period_ms
    consecutive = longest = 0
    for overtime in missed:
        consecutive = consecutive + 1 if overtime else 0
        longest = max(longest, consecutive)
    raised = sum(c.get('status') == 'raised' for c in calls)
    return dict(
        mean=float(durations.mean()), p95=float(np.percentile(durations, 95)),
        p99=float(np.percentile(durations, 99)), maximum=float(durations.max()),
        first_call=float(durations[0]), cycles=len(calls), control_period_ms=period_ms,
        overtime_count=int(missed.sum()), overtime_ratio=float(missed.mean()),
        longest_overtime_streak=longest, ge200=int((durations >= 200).sum()),
        returned_calls=len(calls) - raised, raised_calls=raised,
        timing_scope='all act attempts, including raised calls; excludes recording IO',
        passed=bool(not raised and runtime_error is None
                    and np.percentile(durations, 95) <= min(30, 0.75 * period_ms)
                    and np.percentile(durations, 99) <= min(40, period_ms)
                    and missed.mean() <= 0.01))


def collision_history(frames):
    import cv2
    final = frames[-1]
    ego = ((final['x'], final['y']), (final['L'], final['W']), math.degrees(final['yaw']))
    found = []
    for key, obj in final['objs'].items():
        x, y, v, yaw, length, width = obj
        if not cv2.rotatedRectangleIntersection(ego, ((x, y), (length, width), math.degrees(yaw)))[0]:
            continue
        visible = [f for f in frames if key in f['objs']]
        first = visible[0]
        stop = final['t']
        for f in reversed(frames):
            if f['v'] >= 1.0:
                break
            stop = f['t']
        found.append(dict(id=key, first_visible=first['t'], collision_t=final['t'],
            observed_lead_time=final['t'] - first['t'], visible_frames=len(visible),
            first_distance=math.hypot(first['objs'][key][0]-first['x'], first['objs'][key][1]-first['y']),
            ego_final_speed=final['v'], other_final_speed=v, stop_start=stop,
            stopped_duration=final['t'] - stop))
    return found


def worker(args):
    name, planner, out = args
    out = Path(out)
    base = sim.load_planner(planner)
    cycles = []
    act_calls = []
    trace = None

    class Recorded(base):
        def act(self, obs):
            nonlocal trace
            started = time.perf_counter()
            error = None
            try:
                action = super().act(obs)
            except Exception as exc:
                error = dict(type=type(exc).__name__, message=str(exc))
                raise
            finally:
                ms = (time.perf_counter()-started)*1000
                act_calls.append(dict(t=float(obs.test_info['t']), ms=ms,
                                      status='returned' if error is None else 'raised',
                                      error=error))
            if hasattr(self, 'cycle_summary'):
                row = dict(t=float(obs.test_info['t']), ms=ms, action=list(action), **self.cycle_summary())
                row['command_error'] = float(np.max(np.abs(np.asarray(action)-self.last_plan.selected.controls[0])))
                if trace is None:
                    trace = gzip.open(str(out / (name + '.trace.jsonl.gz')), 'wt', encoding='utf-8')
                    # Geometry is static and belongs in the first record only.
                    roads = {k: dict(left=lane.left.tolist(), right=lane.right.tolist(), center=lane.path.pts.tolist(),
                        width_samples=lane.widths.tolist(),
                        successors=lane.successors, predecessors=lane.predecessors, metadata=lane.metadata)
                        for k, lane in self.roads.lanes.items()}
                    trace.write(json.dumps(dict(schema='traffic-map-v1', lanes=roads,
                        boundary_seams=getattr(self.roads, 'boundary_seams', []),
                        shared_boundaries=getattr(self.roads, 'shared_boundaries', []),
                        configuration=self.config.manifest())) + '\n')
                trace.write(json.dumps(self.replay_record(), ensure_ascii=False, allow_nan=False) + '\n')
                cycles.append(row)
                return action
            p = self.arb.last
            row = dict(t=float(obs.test_info['t']), ms=ms, action=list(action),
                label=p.label, predicted_collision=None if not np.isfinite(p.t_collide) else p.t_collide,
                all_collide=bool(p.all_collide))
            if hasattr(p, 'controls'):
                row['command_error'] = float(np.max(np.abs(np.asarray(action)-p.controls[0])))
                prev = getattr(self, '_previous_expected', None)
                if prev is not None:
                    e = obs.ego_info
                    current = np.array([e.x,e.y,e.yaw,e.v,e.a or 0,e.rot or 0])
                    diff = current-prev
                    diff[2] = (diff[2]+np.pi)%(2*np.pi)-np.pi
                    row['state_error'] = diff.tolist()
                self._previous_expected = p.states[1].copy()
            cycles.append(row)
            return action

    original = sim.load_planner
    sim.load_planner = lambda key: Recorded
    try:
        result = sim.run_scene(name, planner, record_objects=True)
        result['score'] = score_run(result)
        frames = result.pop('frames')
        if result['end'] == '碰撞':
            result['collision_history'] = collision_history(frames)
        result['latency'] = act_latency(act_calls, result['dt'], result.get('error'))
        result['act_call_counts'] = dict(
            total=len(act_calls), returned=sum(c['status'] == 'returned' for c in act_calls),
            raised=sum(c['status'] == 'raised' for c in act_calls), recorded_actions=len(cycles))
        with gzip.open(str(out / (name+'.json.gz')), 'wt', encoding='utf-8') as f:
            json.dump(dict(result=result, cycles=cycles, act_calls=act_calls, frames=frames),
                      f, ensure_ascii=False)
        return result
    finally:
        if trace is not None:
            trace.close()
        sim.load_planner = original


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    ap=argparse.ArgumentParser()
    ap.add_argument('--tag', required=True)
    ap.add_argument('--planner', default='ARB')
    ap.add_argument('--sets', default='DEV+REG')
    ap.add_argument('--jobs', type=int, default=4)
    a=ap.parse_args()
    out=HERE/a.tag
    out.mkdir(exist_ok=False)
    sets=json.loads((ROOT/'research/official_tool_patches/bench_manifest.json').read_text(encoding='utf-8'))['sets']
    if a.sets in ('DEV','REG','DEV+REG'):
        names=[n for s in a.sets.split('+') for n in sets[s]]
    else:
        names=a.sets.split(',')
        if any(n not in sets['DEV']+sets['REG'] for n in names):
            raise ValueError('Only DEV and REG allowed')
    idx=json.loads((ROOT/'scenes/official_catalog/index.json').read_text(encoding='utf-8'))
    before=hashes()
    save(out/'manifest.json',dict(sources=before,python=sys.version,numpy=np.__version__,
        head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=str(ROOT),text=True).strip(),
        jobs=a.jobs,planner=a.planner,scenes=names,
        timing_scope='complete act, excludes harness and disk IO; parallel throughput run' if a.jobs>1 else 'sequential complete act, excludes harness and disk IO',
        environment={k:v for k,v in os.environ.items() if k.startswith(('ARB_','EXEC_','OMP_','MKL_','OPENBLAS_'))}))
    evidence_files = list(HERE.glob('*.py')) + list((HERE.parent/'traffic_architecture_20260928').glob('*.py')) + list((HERE.parent/'traffic_generation_20260930').glob('*.py')) + list((HERE.parent/'traffic_boundary_20260930').glob('*.py')) + list((HERE.parent/'traffic_progress_20260930').glob('*.py')) + list((HERE.parent/'traffic_performance_20260930').glob('*.py')) + list((HERE.parent/'traffic_curve_20260930').glob('*.py')) + [
        HERE.parent/'arbiter_execution_20260925/before_sources.zip', HERE.parent/'traffic_progress_20260930/before_sources.zip', ROOT/'stack/traffic/requirements.txt']
    evidence_files += list((HERE.parent/'traffic_lightweight_20260930').glob('*.py'))
    save(out/'experiment_manifest.json', {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                       for p in evidence_files})
    with zipfile.ZipFile(out/'sources.zip','w',zipfile.ZIP_DEFLATED) as z:
        for p in before:
            z.write(ROOT/p,p)
        for p in evidence_files:
            z.write(p, p.relative_to(ROOT))
    rows=[]
    tasks=[(n,a.planner,str(out)) for n in names]
    def collect(results):
        for r in results:
            r['roadtype']=idx[r['name']]['roadtype']
            rows.append(r)
            save(out/'results.json',dict(rows=rows))
            if len(rows)%10==0 or len(rows)==len(names):
                print(len(rows),dict(Counter(r['end'] for r in rows)),flush=True)
    if a.jobs == 1:
        collect(map(worker, tasks))
    else:
        with Pool(a.jobs) as pool:
            collect(pool.imap_unordered(worker,tasks,chunksize=1))
    save(out/'verification.json',dict(sources_unchanged=before==hashes(),after=hashes()))


if __name__=='__main__':
    main()
