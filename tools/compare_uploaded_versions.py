"""Read-only comparison of the uploaded r99 and the frozen local r43 evidence."""
from collections import Counter
import ast
import hashlib
import json
from pathlib import Path

CHECKOUT = Path(__file__).resolve().parents[1]
HERE = CHECKOUT / 'outputs/version_comparison'
LOCAL = CHECKOUT / 'candidates/r43'
R99 = CHECKOUT / 'research/arb_capability_20261007/runs/recovery_pulse_r99_devreg130'
R43 = LOCAL / 'runs/unified_r43_20261010_devreg130'
EXTRA = LOCAL / 'runs/unified_r43_20261010_extra100'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def definitions(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    result = {}
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            result[node.name] = ast.dump(node, include_attributes=False)
        elif isinstance(node, ast.ClassDef):
            for child in node.body:
                if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    result[node.name + '.' + child.name] = ast.dump(child, include_attributes=False)
    return result


def main():
    old = read(R99 / 'manifest.json')
    new = read(R43 / 'manifest.json')
    assert all(digest(R99 / 'source' / name) == value for name, value in old['sources'].items())
    assert all(digest(LOCAL / 'source' / name) == value for name, value in new['sources'].items())
    assert read(LOCAL / 'verification.json')['source_sha256'] == new['sources']
    assert digest(LOCAL / 'verification.json') == '9c1efce67345686ef7a27fa0a2a53f0dd69c60a40e851616ae2a7d0271c8d10d'
    file_rows = []
    for name in sorted(set(old['sources']) | set(new['sources'])):
        before, after = old['sources'].get(name), new['sources'].get(name)
        status = 'added' if before is None else 'removed' if after is None else 'unchanged' if before == after else 'modified'
        row = dict(path=name, status=status, r99_sha256=before, r43_sha256=after)
        if status == 'modified':
            left, right = definitions(R99 / 'source' / name), definitions(LOCAL / 'source' / name)
            row['added_definitions'] = sorted(set(right) - set(left))
            row['changed_definitions'] = sorted(key for key in left.keys() & right.keys() if left[key] != right[key])
            row['removed_definitions'] = sorted(set(left) - set(right))
        file_rows.append(row)
    before = {row['name']: row for row in read(R99 / 'validation/results.json')['rows']}
    after = {row['name']: row for row in read(R43 / 'validation/results.json')['rows']}
    assert set(before) == set(after) and len(before) == 130
    old_inputs = read(R99 / 'validation/manifest.json')['inputs']
    new_inputs = read(R43 / 'validation/manifest.json')['inputs']
    scene_rows = []
    for name in sorted(before):
        a, b = before[name], after[name]
        scene_rows.append(dict(scene=name, before=a['end'], after=b['end'],
            before_t=a['t_end'], after_t=b['t_end'],
            lost_success=a['end'] == '到达' and b['end'] != '到达',
            gained_success=a['end'] != '到达' and b['end'] == '到达',
            new_collision=a['end'] != '碰撞' and b['end'] == '碰撞',
            resolved_collision=a['end'] == '碰撞' and b['end'] != '碰撞',
            r43_execution_mismatches=b.get('execution_mismatches'),
            r43_input_fix=b.get('input_fix')))
    extra = read(EXTRA / 'validation/results.json')['rows']
    summary = dict(old_commit='4fbe34fbe5dcfbfdb232bfe5406dfadd4efff1d6',
        r99_files=len(old['sources']), r43_files=len(new['sources']),
        source_counts=dict(Counter(row['status'] for row in file_rows)),
        original_130=dict(r99=dict(Counter(row['end'] for row in before.values())),
                          r43=dict(Counter(row['end'] for row in after.values()))),
        extra_100=dict(Counter(row['end'] for row in extra)),
        same_scene_set=True, same_original_input_hashes=old_inputs == new_inputs,
        r99_successes_retained=sum(a['end'] == '到达' and after[n]['end'] == '到达' for n, a in before.items()),
        gained_successes=[r['scene'] for r in scene_rows if r['gained_success']],
        lost_successes=[r['scene'] for r in scene_rows if r['lost_success']],
        new_collisions=[r['scene'] for r in scene_rows if r['new_collision']],
        resolved_collisions=[r['scene'] for r in scene_rows if r['resolved_collision']],
        old_timing_pass_count=sum(bool(row.get('latency', {}).get('passed')) for row in before.values()),
        new_timing_pass_count=sum(bool(row.get('latency', {}).get('passed')) for row in after.values()),
        extra_timing_pass_count=sum(bool(row.get('latency', {}).get('passed')) for row in extra),
        limitations=['Existing local closed-loop records; no baseline was rerun.',
                     'r43 explicitly orders seven original target rectangles in memory; input XML is unchanged.',
                     'Outcome improvement alone does not meet retained-success/no-new-collision promotion requirements.'])
    corrections = set(new['input_correction']['scenes'])
    without_corrections = [r for r in scene_rows if r['scene'] not in corrections]
    summary['unchanged_target_protocol_123'] = dict(
        r99=dict(Counter(r['before'] for r in without_corrections)),
        r43=dict(Counter(r['after'] for r in without_corrections)))
    result = dict(summary=summary, sources=file_rows, scenes=scene_rows,
        input_keys_only_r99=sorted(set(old_inputs) - set(new_inputs)),
        input_keys_only_r43=sorted(set(new_inputs) - set(old_inputs)),
        changed_input_hashes=[key for key in old_inputs.keys() & new_inputs.keys() if old_inputs[key] != new_inputs[key]],
        evidence_sha256={('uploaded_r99/' + path.relative_to(CHECKOUT).as_posix() if path == R99 or R99 in path.parents else 'local_r43/' + path.relative_to(LOCAL).as_posix()):digest(path)
            for path in (R99 / 'manifest.json', R99 / 'validation/results.json', R99 / 'validation/manifest.json',
                         R43 / 'manifest.json', R43 / 'validation/results.json', R43 / 'validation/manifest.json',
                         EXTRA / 'validation/results.json', LOCAL / 'verification.json')})
    HERE.mkdir(parents=True, exist_ok=True)
    (HERE / 'r99_vs_r43.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print('Changed source files:', ', '.join(row['path'] for row in file_rows if row['status'] != 'unchanged'))
    print('Frozen candidate evidence MB:', round(sum(p.stat().st_size for p in R43.rglob('*') if p.is_file()) / 1024**2, 2),
          round(sum(p.stat().st_size for p in EXTRA.rglob('*') if p.is_file()) / 1024**2, 2))


if __name__ == '__main__':
    main()
