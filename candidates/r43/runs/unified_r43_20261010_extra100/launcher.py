"""Isolated DEV/REG replay with per-cycle P0 diagnostics and baseline protocol."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import zipfile

HERE = Path(__file__).resolve().parents[1]
SCRIPT = Path(__file__).resolve()
PROJECT = HERE.parents[2]
sys.dont_write_bytecode = True
sys.path.insert(0, str(PROJECT / 'bench'))
import run_baseline as protocol

FIXED_SCENES = {'highway_merge_3_1_197', 'highway_merge_3_1_332',
    'highway_merge_3_1_391', 'highway_merge_3_1_86', 'highway_merge_3_1_87',
    'highway_merge_3_1_301', 'highway_merge_3_1_68'}
NORMALIZE_EXTENSION_TARGETS = False


def resolve_baseline(value):
    path = Path(value)
    if path.is_absolute():
        choices = [path]
    else:
        choices = [HERE/'runs'/path, HERE.parent/'runs'/path]
    for choice in choices:
        if (choice/'manifest.json').is_file():
            return choice.resolve()
    raise FileNotFoundError('Comparison baseline not found: '+str(value))


def worker(args):
    from research.architecture_takeover_20260926 import run as harness
    import sim
    import numpy as np
    from score import score_run
    name, entry, output = args
    sim._boot()
    from utils.ScenarioManager.ScenarioManagerForReplay import ScenarioManagerForReplay
    original_parser = ScenarioManagerForReplay._parse_openscenario
    correction = {}
    def corrected_parser(self, file_dir):
        info = original_parser(self, file_dir)
        target=np.asarray(info['targetPos'])
        reversed_extension=(NORMALIZE_EXTENSION_TARGETS and target.shape==(2,2) and
            np.isfinite(target).all() and (target[0]>target[1]).any())
        if Path(file_dir).parent.name in FIXED_SCENES or reversed_extension:
            before = [point[:] for point in info['targetPos']]
            info['targetPos'] = [[min(before[0][axis], before[1][axis]) for axis in (0, 1)],
                                 [max(before[0][axis], before[1][axis]) for axis in (0, 1)]]
            correction.update(applied=True, original_target=before,
                corrected_target=info['targetPos'], method='sort_each_target_axis_in_memory',
                source_xml_unchanged=True)
        return info
    base = sim.load_planner(entry)
    calls, cycles = [], []

    class Recorded(base):
        def act(self, observation):
            started = time.perf_counter()
            error = None
            try:
                action = super().act(observation)
            except Exception as exc:
                error = {'type': type(exc).__name__, 'message': str(exc)}
                raise
            finally:
                elapsed = (time.perf_counter() - started) * 1000
                calls.append({'t': float(observation.test_info['t']), 'ms': elapsed,
                              'status': 'returned' if error is None else 'raised', 'error': error})
            plan = self.arb.last
            cycles.append(dict(t=float(observation.test_info['t']), ms=elapsed,
                action=list(action), label=plan.label,
                predicted_collision=None if not np.isfinite(plan.t_collide) else float(plan.t_collide),
                all_collide=bool(plan.all_collide), safety=self.safety_diagnostics()))
            return action

    original = sim.load_planner
    sim.load_planner = lambda key: Recorded
    ScenarioManagerForReplay._parse_openscenario = corrected_parser
    try:
        result = sim.run_scene(name, entry, record_objects=True)
    finally:
        sim.load_planner = original
        ScenarioManagerForReplay._parse_openscenario = original_parser
    if correction:
        result['input_fix'] = correction
    result['score'] = score_run(result)
    frames = result.pop('frames')
    if result['end'] == '碰撞':
        result['collision_history'] = harness.collision_history(frames)
    goal = result.get('diag', {}).get('goal_contract', {}).get('status')
    result['task_contract_status'] = goal
    result['outcome_class'] = ('TASK_INPUT_INVALID' if goal in
        ('reversed_bounds', 'invalid_shape', 'nonfinite') and
        (result.get('error') or '').startswith('init: ValueError: Invalid official target rectangle:')
        else result['end'])
    result['latency'] = harness.act_latency(calls, result['dt'], result.get('error'))
    result['act_call_counts'] = dict(total=len(calls),
        returned=sum(call['status'] == 'returned' for call in calls),
        raised=sum(call['status'] == 'raised' for call in calls), recorded_actions=len(cycles))
    with gzip.open(str(Path(output) / (name + '.json.gz')), 'wt', encoding='utf-8') as stream:
        json.dump(dict(result=result, cycles=cycles, act_calls=calls, frames=frames),
                  stream, ensure_ascii=False)
    return result


def main():
    global NORMALIZE_EXTENSION_TARGETS
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tag', required=True)
    parser.add_argument('--mode', choices=('observe', 'unified'), default='unified')
    parser.add_argument('--scenes')
    parser.add_argument('--allow-shared-load', action='store_true')
    parser.add_argument('--compare-to', default='unified_r9_20261009_devreg130')
    parser.add_argument('--source-run', type=Path, help='Verified frozen run to copy instead of current candidate source')
    parser.add_argument('--extension-set', type=Path, help='Verified frozen additional-development selection')
    parser.add_argument('--baseline-only', action='store_true', help='Build extension baseline and attribution without claiming a policy comparison')
    parser.add_argument('--normalize-extension-targets',action='store_true',
        help='Explicit paired protocol: sort reversed finite extension target corners in memory, recording every correction')
    args = parser.parse_args()
    if not args.tag.replace('_', '').replace('-', '').isalnum():
        parser.error('Invalid tag')
    protocol.verify()
    extension = None
    if args.extension_set:
        from select_extended_set import verify_extension
        args.extension_set = args.extension_set.resolve()
        extension = verify_extension(args.extension_set)
    if args.baseline_only and (not extension or not args.source_run):
        parser.error('Baseline-only requires frozen source and an extension selection')
    if args.normalize_extension_targets and not extension:
        parser.error('Additional target normalization is restricted to the extension protocol')
    NORMALIZE_EXTENSION_TARGETS=args.normalize_extension_targets
    paired = resolve_baseline(args.compare_to) if not args.baseline_only else None
    if extension and not args.baseline_only:
        paired_manifest=protocol.read(paired/'manifest.json')
        if paired_manifest.get('extension_selection_sha256')!=protocol.digest(args.extension_set):
            raise ValueError('Extension paired selections differ')
        if paired_manifest.get('input_correction',{}).get('normalize_finite_reversed_extension_targets',False)!=args.normalize_extension_targets:
            raise ValueError('Extension paired target protocols differ')
    target = HERE / 'runs' / args.tag
    target.mkdir(parents=True, exist_ok=False)
    source = HERE / 'source'
    source_origin = None
    candidate_verification = None
    if args.source_run:
        origin = args.source_run.resolve()
        source_origin = protocol.read(origin/'manifest.json')
        if protocol.digest(origin/'sources.zip') != source_origin['archive_sha256'] or not all(
                protocol.digest(origin/'source'/key) == value for key,value in source_origin['sources'].items()):
            raise ValueError('Frozen source origin changed')
        source = origin/'source'
    else:
        candidate_verification = protocol.read(HERE/'verification.json')
        actual = {path.relative_to(source).as_posix():protocol.digest(path)
                  for path in source.rglob('*.py')}
        if actual != candidate_verification['source_sha256'] or any(
                row['exit_code'] for row in candidate_verification['tests']):
            raise ValueError('Current source must match passing candidate verification')
    hashes = {}
    with zipfile.ZipFile(target / 'sources.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for file in sorted(source.rglob('*.py')):
            relative = file.relative_to(source).as_posix()
            content = file.read_bytes()
            hashes[relative] = hashlib.sha256(content).hexdigest()
            dest = target / 'source' / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(content)
            archive.writestr(relative, content)
    parent = PROJECT / 'research/arb_capability_20261007/runs/route_seam_20261008_devreg130'
    if extension:
        protocol.save(target/'sets.json', dict(DEV=extension['scenes'], REG=[]))
        shutil.copyfile(args.extension_set, target/'extension_selection.json')
    else:
        shutil.copyfile(parent / 'sets.json', target / 'sets.json')
    entry = ('stack.adapters.onsite_observed:ObservedPlanner' if args.mode == 'observe' else 'stack.adapters.onsite_unified:UnifiedPlanner')
    manifest = dict(baseline_id='ARB-CAPABILITY-' + args.tag,
        parent_baseline='ARB-CAPABILITY-route_seam_20261008_devreg130', planner=entry,
        parent_archive_sha256=protocol.digest(parent / 'sources.zip'),
        archive_sha256=protocol.digest(target / 'sources.zip'), sources=hashes,
        sets_sha256=protocol.digest(target / 'sets.json'))
    manifest['input_correction'] = dict(method='sort_each_target_axis_in_memory',
        scenes=sorted(FIXED_SCENES), source_files_unchanged=True,
        normalize_finite_reversed_extension_targets=args.normalize_extension_targets)
    manifest.update(set_scope='additional_development_not_holdout' if extension else 'original_devreg130',
        extension_selection_sha256=protocol.digest(args.extension_set) if extension else None,
        source_origin_path=str(origin) if source_origin else None,
        source_origin_archive_sha256=source_origin['archive_sha256'] if source_origin else None,
        baseline_only=args.baseline_only)
    manifest.update(comparison_baseline_path=str(paired) if paired else None,
        candidate_verification_sha256=protocol.digest(HERE/'verification.json')
            if candidate_verification else None)
    if candidate_verification:
        shutil.copyfile(HERE/'verification.json',target/'candidate_verification.json')
    protocol.save(target / 'manifest.json', manifest)
    shutil.copyfile(SCRIPT, target / 'launcher.py')
    protocol.save(target / 'launcher_identity.json', dict(path=str(SCRIPT),
        sha256=protocol.digest(SCRIPT), additional_diagnostics='safety_diagnostics after timed act'))
    if extension:
        shutil.copyfile(SCRIPT.parent/'select_extended_set.py',target/'select_extended_set.py')
    diagnostic_tools = target / 'diagnostic_tools'
    diagnostic_tools.mkdir()
    diagnostic_identity = {}
    for name in ('review_run.py', 'analyze_failures.py', 'audit_mapped_boundaries.py', 'mapped_body_audit.py'):
        tool = Path(__file__).resolve().parent / name
        shutil.copyfile(tool, diagnostic_tools / name)
        diagnostic_identity[name] = protocol.digest(tool)
    protocol.save(target / 'diagnostic_tools_identity.json', diagnostic_identity)
    protocol.BASE, protocol.ENTRY = target, entry
    protocol.verify()
    sys.path.insert(0, str(PROJECT))
    from research.architecture_takeover_20260926 import run as harness
    harness.worker = worker
    args.output = target / 'validation'
    protocol.run(args, manifest)
    validation_summary = protocol.read(args.output/'summary.json')
    validation_summary.update(set_scope=manifest['set_scope'],
        complete_original_devreg130=bool(not extension and validation_summary['complete_dev_reg']),
        complete_extension_set=bool(extension and validation_summary['complete_dev_reg']))
    if extension:
        validation_summary['complete_dev_reg'] = False
    protocol.save(args.output/'summary.json',validation_summary)
    # Postprocessing tools are frozen with the run; historical reports can be
    # reproduced without whichever workspace helper happens to be current.
    comparison = str(target if args.baseline_only else paired) if extension else str(parent)
    if extension and not args.baseline_only:
        actual_inputs = protocol.read(target/'validation/manifest.json')['inputs']
        paired_inputs = protocol.read(paired/'validation/manifest.json')['inputs']
        if any(paired_inputs.get(key) != value for key,value in actual_inputs.items()):
            raise ValueError('Extension paired inputs differ')
    subprocess.run([sys.executable, '-B', str(diagnostic_tools / 'audit_mapped_boundaries.py'),
                    '--runs', str(target), '--output', str(target/'mapped_body_audit')], check=True)
    subprocess.run([sys.executable, '-B', str(diagnostic_tools / 'review_run.py'),
                    str(target),'--baseline',comparison], check=True)
    subprocess.run([sys.executable, '-B', str(diagnostic_tools / 'analyze_failures.py'),
                    str(target)], check=True)
    if args.mode=='unified' and not extension:
        subprocess.run([sys.executable, '-B', str(diagnostic_tools / 'review_run.py'),
            str(target),'--baseline',str(paired),'--output',
            str(target/'comparisons'/args.compare_to)],check=True)


if __name__ == '__main__':
    main()
