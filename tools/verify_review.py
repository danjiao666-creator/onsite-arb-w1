"""Verify exported saved evidence, never import or run a planner/simulator."""
import ast
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load(path):
    return json.loads((ROOT / path).read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def main():
    manifest = load('REVIEW_MANIFEST.json')
    for name, expected in manifest['files'].items():
        path = (ROOT / name).resolve()
        require(ROOT in path.parents, 'Path outside review package: ' + name)
        require(path.is_file() and digest(path) == expected, 'Hash mismatch: ' + name)
    source = ROOT / 'candidates/r43/source'
    expected = load('candidates/r43/verification.json')['source_sha256']
    actual = {p.relative_to(source).as_posix(): digest(p) for p in source.rglob('*.py')}
    require(actual == expected and len(actual) == 34, 'Frozen R43 source identity mismatch')
    for name in ('selector_a', 'gap_event_v2'):
        base = ROOT / 'experiments' / name
        identity = load('experiments/' + name + '/candidate_identity.json')['files']
        exported = {p.relative_to(base).as_posix(): digest(p) for p in (base / 'source').rglob('*.py')}
        frozen = {k.replace('\\', '/'): v for k, v in identity.items()
                  if k.replace('\\', '/').startswith('source/')}
        require(exported == frozen, 'Frozen experimental source mismatch: ' + name)
    counts = Counter()
    scenes = []
    for run in (ROOT / 'candidates/r43/runs').iterdir():
        data = json.loads((run / 'validation/results.json').read_text(encoding='utf-8'))
        scenes.extend(data['rows'])
    require(len(scenes) == len(set(row['name'] for row in scenes)) == 230, 'Original scene scope changed')
    counts.update(row['end'] for row in scenes)
    require(dict(counts) == {'到达': 199, '碰撞': 8, '超时': 23}, 'Original outcomes changed')
    timeouts = [r for r in scenes if r['end'] == '超时']
    require(sum('merge' in r['name'] for r in timeouts) == 18, 'Merge timeout count changed')
    a = load('experiments/selector_a/verification.json')
    require(a['new_arrivals'] == 2 and not a['algorithm_accepted'], 'A result scope changed')
    latest = load('experiments/gap_event_v2/verification.json')
    require(latest['submitted_steps'] == 230 and latest['new_arrivals'] == 0
            and latest['failed_at'] == 'highway_merge_3_1_332@3.3'
            and not latest['algorithm_accepted'], 'Latest failure scope changed')
    provenance = load('review/EXPORT_PROVENANCE.json')['copies']
    for name, record in provenance.items():
        path = ROOT / name
        if record['copy'] == 'exact':
            require(digest(path) == record['sha256'], 'Saved artifact changed: ' + name)
        else:
            data = gzip.decompress(path.read_bytes())
            require(hashlib.sha256(data).hexdigest() == record['decoded_sha256'],
                    'Compressed replay changed: ' + name)
    syntax = 0
    for directory in ('tools', 'candidates/r43/source', 'experiments'):
        for path in (ROOT / directory).rglob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
            syntax += 1
    require(not any((ROOT / p).exists() for p in ('stack', 'baselines', 'research', 'docs')),
            'Obsolete baseline material remains in the review tree')
    print(json.dumps({'passed': True, 'manifest_files': len(manifest['files']),
                      'r43_source_files': len(actual), 'original_scenes': len(scenes),
                      'original_outcomes': dict(counts), 'merge_timeouts': 18,
                      'syntax_files': syntax, 'new_driving': 0, 'new_solves': 0,
                      'scope': 'Packaging integrity only, not algorithm acceptance'},
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
