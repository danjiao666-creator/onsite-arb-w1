"""Keep subset outcomes separate from complete parent regression evidence."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent


def main():
    sys.stdout.reconfigure(encoding='utf8')
    parser = argparse.ArgumentParser()
    parser.add_argument('tags', nargs='+')
    parser.add_argument('--parent', default='exact_envelope_r53_devreg130')
    args = parser.parse_args()
    parent = HERE / 'runs' / args.parent
    before = json.loads((parent / 'validation/results.json').read_text(encoding='utf8'))
    bm = json.loads((parent / 'validation/manifest.json').read_text(encoding='utf8'))
    index = {row['name']: row for row in before['rows']}
    for tag in args.tags:
        run = HERE / 'runs' / tag
        result = json.loads((run / 'validation/results.json').read_text(encoding='utf8'))
        manifest = json.loads((run / 'manifest.json').read_text(encoding='utf8'))
        runtime = json.loads((run / 'validation/manifest.json').read_text(encoding='utf8'))
        verification = json.loads((run / 'validation/verification.json').read_text(encoding='utf8'))
        rows = result['rows']
        names = [row['name'] for row in rows]
        assert len(set(names)) == len(names)
        assert set(names) == set(runtime['scenes'])
        assert set(names).issubset(index)
        assert all(bm['inputs'].get(key) == value for key, value in runtime['inputs'].items())
        assert verification['inputs_unchanged'] and verification['frozen_planner_unchanged']
        assert hashlib.sha256((run / 'sources.zip').read_bytes()).hexdigest() == manifest['archive_sha256']
        for name, digest in manifest['sources'].items():
            assert hashlib.sha256((run / 'source' / name).read_bytes()).hexdigest() == digest
        changed = [dict(name=row['name'], before=index[row['name']]['end'], after=row['end'])
                   for row in rows if row['end'] != index[row['name']]['end']]
        lost = [item['name'] for item in changed if item['before'] == '到达']
        collisions = [item['name'] for item in changed if item['after'] == '碰撞']
        errors = [row['name'] for row in rows if row.get('error')]
        report = dict(parent=args.parent, candidate=tag, count=len(rows),
                      full_parent_set=set(names) == set(index), identity_passed=True,
                      before=dict(Counter(index[name]['end'] for name in names)),
                      after=dict(Counter(row['end'] for row in rows)),
                      changed=changed, lost=lost, new_collisions=collisions, errors=errors,
                      subset_regression_passed=not (lost or collisions or errors),
                      full_regression_passed=set(names) == set(index) and not (lost or collisions or errors),
                      promotion=False)
        (run / 'parent_comparison.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
        print(json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
