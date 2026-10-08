"""Compare an explicit DEV/REG run with the exact matching frozen subset."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BASE = ROOT / 'baselines/arb_w1_v1/validation/devreg_20261007_r1'
sys.path.insert(0, str(ROOT / 'bench'))
from compare_baseline import compare_documents  # noqa: E402


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tag')
    args = parser.parse_args()
    run = HERE / 'runs' / args.tag / 'validation'
    candidate = read(run / 'results.json')
    names = [r['name'] for r in candidate['rows']]
    baseline = read(BASE / 'results.json')
    before = dict(rows=[r for r in baseline['rows'] if r['name'] in names])
    report = compare_documents(before, candidate)
    bm, cm = read(BASE / 'manifest.json'), read(run / 'manifest.json')
    bv, cv = read(BASE / 'verification.json'), read(run / 'verification.json')
    assert set(cm['scenes']) == set(names), 'Run is incomplete'
    matched_inputs = all(bm['inputs'].get(k) == v for k, v in cm['inputs'].items())
    assert matched_inputs, 'Different scene inputs'
    report['evidence'].update(same_inputs=matched_inputs,
        input_hashes_unchanged=bool(bv['inputs_unchanged'] and cv['inputs_unchanged']),
        fixed_source_identity=cv['frozen_planner_unchanged'],
        baseline_results_sha256=digest(BASE / 'results.json'),
        candidate_results_sha256=digest(run / 'results.json'),
        baseline_archive_sha256=bm['archive_sha256'],
        candidate_archive_sha256=cm['archive_sha256'],
        same_python_numpy=all(bm[k] == cm[k] for k in ('python', 'executable', 'numpy')),
        competing_python_before=cm['competing_python_before'],
        competing_python_after=cv['competing_python_after'])
    timing, equality = {'baseline': [], 'candidate': []}, []
    for name in names:
        old, new = [json.load(gzip.open(str(folder / (name + '.json.gz')), 'rt', encoding='utf-8'))
                    for folder in (BASE, run)]
        for key, data in [('baseline', old), ('candidate', new)]:
            timing[key].extend(row['ms'] for row in data['act_calls'])
        first_action_difference = next((a['t'] for a, b in zip(old['cycles'], new['cycles'])
                                       if a['t'] != b['t'] or a['action'] != b['action']), None)
        first_state_difference = next((a['t'] for a, b in zip(old['frames'], new['frames']) if a != b), None)
        equality.append(dict(name=name, baseline_cycles=len(old['cycles']), candidate_cycles=len(new['cycles']),
            first_action_difference=first_action_difference, first_state_difference=first_state_difference,
            states_actions_equal=bool(old['frames'] == new['frames'] and first_action_difference is None
                                     and len(old['cycles']) == len(new['cycles']))))
    report['frame_comparison'] = equality
    report['pooled_complete_act_ms'] = {
        key: dict(n=len(values), mean=float(np.mean(values)), p95=float(np.percentile(values, 95)),
                  p99=float(np.percentile(values, 99)), maximum=float(max(values)))
        for key, values in timing.items()}
    report['timing_note'] = ('Complete act observations; shared host load may differ. '
                             'No paired performance improvement or acceptance is inferred.')
    output = run.parent / 'comparison.json'
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key: report[key] for key in ('total', 'gained', 'lost', 'new_collisions',
                                                 'new_errors', 'pooled_complete_act_ms')},
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
