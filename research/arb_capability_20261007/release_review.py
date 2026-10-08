"""Verify the final driving candidate and the claimed arithmetic equivalence."""
import gzip
import json
from pathlib import Path


HERE = Path(__file__).resolve().parent
OLD = HERE / 'runs/dynamic_route_r4_devreg130'
FINAL = HERE / 'runs/dynamic_route_r5_devreg130'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    audit = read(FINAL / 'final_audit.json')
    assert not audit['lost'] and not audit['new_collisions'] and not audit['new_errors']
    assert not audit['exceptions'] and not audit['record_integrity_issues']
    assert audit['feature_gate']['inactive_all_states_actions_equal']
    assert audit['feature_gate']['active_known_collision_with_clear_candidate_count'] == 0
    assert not audit['source_identity']['manifest_differences']
    assert not audit['source_identity']['archive_entry_differences']
    assert audit['source_identity']['archive_hash_matches']
    assert not audit['source_identity']['imported_outside_snapshot']
    assert not audit['inputs']['baseline_candidate_differences']
    assert not audit['inputs']['candidate_current_differences']
    assert audit['map_caches']['all_match_baseline_postrun']
    assert audit['map_caches']['all_candidate_before_after_equal']
    assert not audit['comparison_crosscheck']['differences']
    old_sources, new_sources = read(OLD / 'manifest.json')['sources'], read(FINAL / 'manifest.json')['sources']
    assert old_sources.keys() == new_sources.keys()
    changed = [name for name in old_sources if old_sources[name] != new_sources[name]]
    assert changed == ['stack/core/arbiter.py'], changed
    rows, cycles = [], 0
    for result in read(FINAL / 'validation/results.json')['rows']:
        name = result['name']
        logs = []
        for folder in (OLD, FINAL):
            with gzip.open(folder / 'validation' / (name + '.json.gz'), 'rt', encoding='utf-8') as stream:
                logs.append(json.load(stream))
        before, after = logs
        assert before['frames'] == after['frames'], name
        decisions = [[{key: value for key, value in row.items() if key != 'ms'}
                      for row in log['cycles']] for log in logs]
        assert decisions[0] == decisions[1], name
        cycles += len(decisions[0])
        rows.append(dict(name=name, states_equal=True, decisions_equal=True, cycles=len(decisions[0])))
    failures = {side: {row['name'] for row in audit['latency'][side]['failures']}
                for side in ('baseline', 'candidate')}
    report = dict(
        candidate='ARB-CAPABILITY-dynamic_route_r5_devreg130',
        driving_regression_passed=True, full_realtime_passed=False,
        default_promotion=False,
        reason='Retain as the validated next capability candidate; 22 scenes still fail the timing contract.',
        parent_baseline='ARB-W1-BASE-v1', unchanged_arithmetic_run='dynamic_route_r4_devreg130',
        equivalent_scenes=len(rows), equivalent_cycles=cycles, source_changes_from_r4=changed,
        per_scene_equivalence=rows,
        newly_failing_latency_scenes=sorted(failures['candidate'] - failures['baseline']),
        recovered_latency_scenes=sorted(failures['baseline'] - failures['candidate']),
        timing_scope='Separate sequential observations, not a controlled timing ratio.',
        comparison='comparison.json', audit='final_audit.json')
    (FINAL / 'release_review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key: value for key, value in report.items() if key != 'per_scene_equivalence'},
                     ensure_ascii=True, indent=2))


if __name__ == '__main__':
    main()
