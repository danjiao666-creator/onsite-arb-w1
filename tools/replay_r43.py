"""Explicit, serial replay of the isolated r43 candidate; external OnSite assets required."""
import argparse
import gzip
import importlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = Path(__file__).resolve()
BASE = ROOT / 'candidates/r43'
SELECTED_RUN = None
sys.dont_write_bytecode = True


def worker(args):
    from research.architecture_takeover_20260926 import run as harness
    import sim
    import numpy as np
    from score import score_run
    name, entry, output = args
    sim._boot()
    from utils.ScenarioManager.ScenarioManagerForReplay import ScenarioManagerForReplay
    original_parser = ScenarioManagerForReplay._parse_openscenario
    input_contract = json.loads((SELECTED_RUN / 'manifest.json').read_text(encoding='utf-8'))['input_correction']
    correction = {}

    def corrected_parser(self, file_dir):
        info = original_parser(self, file_dir)
        target = np.asarray(info['targetPos'])
        reversed_extension = (input_contract['normalize_finite_reversed_extension_targets']
            and target.shape == (2, 2) and np.isfinite(target).all() and (target[0] > target[1]).any())
        if Path(file_dir).parent.name in input_contract['scenes'] or reversed_extension:
            before = [point[:] for point in info['targetPos']]
            info['targetPos'] = [[min(before[0][axis], before[1][axis]) for axis in (0, 1)],
                                 [max(before[0][axis], before[1][axis]) for axis in (0, 1)]]
            correction.update(applied=True, original_target=before, corrected_target=info['targetPos'],
                method='sort_each_target_axis_in_memory', source_xml_unchanged=True)
        return info

    planner_class = sim.load_planner(entry)
    calls, cycles = [], []

    class Recorded(planner_class):
        def act(self, observation):
            started, error = time.perf_counter(), None
            try:
                action = super().act(observation)
            except Exception as exc:
                error = dict(type=type(exc).__name__, message=str(exc))
                raise
            finally:
                elapsed = (time.perf_counter() - started) * 1000
                calls.append(dict(t=float(observation.test_info['t']), ms=elapsed,
                    status='returned' if error is None else 'raised', error=error))
            plan = self.arb.last
            cycles.append(dict(t=float(observation.test_info['t']), ms=elapsed, action=list(action),
                label=plan.label, predicted_collision=None if not np.isfinite(plan.t_collide) else float(plan.t_collide),
                all_collide=bool(plan.all_collide), safety=self.safety_diagnostics()))
            return action

    original_loader = sim.load_planner
    sim.load_planner = lambda key: Recorded
    ScenarioManagerForReplay._parse_openscenario = corrected_parser
    try:
        result = sim.run_scene(name, entry, record_objects=True)
    finally:
        sim.load_planner = original_loader
        ScenarioManagerForReplay._parse_openscenario = original_parser
    if correction:
        result['input_fix'] = correction
    result['score'] = score_run(result)
    frames = result.pop('frames')
    if result['end'] == '碰撞':
        result['collision_history'] = harness.collision_history(frames)
    goal = result.get('diag', {}).get('goal_contract', {}).get('status')
    result['task_contract_status'] = goal
    result['outcome_class'] = ('TASK_INPUT_INVALID' if goal in ('reversed_bounds', 'invalid_shape', 'nonfinite')
        and (result.get('error') or '').startswith('init: ValueError: Invalid official target rectangle:') else result['end'])
    result['latency'] = harness.act_latency(calls, result['dt'], result.get('error'))
    result['act_call_counts'] = dict(total=len(calls), returned=sum(c['status'] == 'returned' for c in calls),
        raised=sum(c['status'] == 'raised' for c in calls), recorded_actions=len(cycles))
    with gzip.open(str(Path(output) / (name + '.json.gz')), 'wt', encoding='utf-8') as stream:
        json.dump(dict(result=result, cycles=cycles, act_calls=calls, frames=frames), stream, ensure_ascii=False)
    return result


def main():
    global SELECTED_RUN
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenes', help='Unique comma-separated names from the selected frozen development set')
    parser.add_argument('--set', choices=('original130', 'extra100'), default='original130')
    parser.add_argument('--allow-shared-load', action='store_true')
    args = parser.parse_args()
    args.output = args.output.absolute().resolve()
    if (ROOT / 'new_runs').resolve() not in args.output.parents:
        parser.error('Choose an output directory inside new_runs/ to preserve the snapshot')
    if args.output.exists():
        parser.error('Choose a new output directory')
    for path in ('code/onsite-structured-test-3.1.5', 'scenes/official_catalog/extracted', 'scenes/official_catalog/index.json'):
        if not (ROOT / path).exists():
            parser.error('Missing external dependency: ' + path)
    tag = 'unified_r43_20261010_devreg130' if args.set == 'original130' else 'unified_r43_20261010_extra100'
    SELECTED_RUN = BASE / 'runs' / tag
    sys.path.insert(0, str(ROOT / 'bench'))
    import run_baseline as protocol
    protocol.BASE = SELECTED_RUN
    protocol.ENTRY = 'stack.adapters.onsite_unified:UnifiedPlanner'
    manifest = protocol.verify()
    sets = protocol.read(SELECTED_RUN / 'sets.json')
    selected_names = sets['DEV'] + sets['REG'] if args.scenes is None else args.scenes.split(',')
    if len(set(selected_names)) != len(selected_names) or not set(selected_names).issubset(sets['DEV'] + sets['REG']):
        parser.error('Select unique scene names from the selected frozen development set')
    original_validation = protocol.read(SELECTED_RUN / 'validation/manifest.json')
    original_inputs = original_validation['inputs']
    selected_inputs = {name: expected for name, expected in original_inputs.items()
        if len(Path(name).parts) >= 4 and Path(name).parts[3] in selected_names}
    for name, expected in selected_inputs.items():
        path = ROOT / name
        if not path.is_file() or protocol.digest(path) != expected:
            parser.error('Replay input differs from the frozen r43 input: ' + name)
    if protocol.runtime_hashes() != original_validation['harness_sources']:
        parser.error('Replay platform/harness sources differ from the frozen r43 runtime')
    sys.path.insert(0, str(SELECTED_RUN / 'source'))
    importlib.import_module('stack.adapters.onsite_unified')
    protocol.check_imports()
    sys.path.insert(0, str(ROOT))
    from research.architecture_takeover_20260926 import run as harness
    harness.worker = worker
    try:
        protocol.run(args, manifest)
        summary = protocol.read(args.output / 'summary.json')
        summary.update(set_scope=manifest['set_scope'],
            complete_original_devreg130=bool(args.set == 'original130' and summary['complete_dev_reg']),
            complete_extension_set=bool(args.set == 'extra100' and summary['complete_dev_reg']))
        if args.set == 'extra100':
            summary['complete_dev_reg'] = False
        protocol.save(args.output / 'summary.json', summary)
        new_inputs = protocol.read(args.output / 'manifest.json')['inputs']
        if new_inputs != selected_inputs:
            raise ValueError('Replay inputs differ from frozen r43 inputs; preserve output as a different-input run')
    finally:
        if args.output.is_dir():
            launcher = SCRIPT
            (args.output / 'launcher.py').write_bytes(launcher.read_bytes())
            protocol.save(args.output / 'candidate_replay_identity.json', dict(
                source_run=tag, launcher_sha256=protocol.digest(launcher),
                manifest_sha256=protocol.digest(SELECTED_RUN / 'manifest.json'),
                input_correction=manifest['input_correction'], source_archive_sha256=manifest['archive_sha256'],
                driving_scope='Serial local replay; inspect output for completion and compare original input/platform hashes.'))


if __name__ == '__main__':
    main()
