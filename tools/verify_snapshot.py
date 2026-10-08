"""Verify archive and evidence identities without importing or running planners."""
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / 'research/arb_capability_20261007/runs'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def snapshot(base, validation, expected):
    manifest = read(base / 'manifest.json')
    require(digest(base / 'sources.zip') == manifest['archive_sha256'],
            'Archive changed: ' + str(base))
    require(digest(base / 'sets.json') == manifest['sets_sha256'],
            'Scene set changed: ' + str(base))
    files = manifest['sources']
    actual = {p.relative_to(base / 'source').as_posix()
              for p in (base / 'source').rglob('*.py')}
    require(actual == set(files), 'Source set changed: ' + str(base))
    with zipfile.ZipFile(base / 'sources.zip') as archive:
        require(set(archive.namelist()) == set(files), 'Archive entries changed')
        for name, expected_hash in files.items():
            require(digest(base / 'source' / name) == expected_hash, 'Source changed: ' + name)
            require(hashlib.sha256(archive.read(name)).hexdigest() == expected_hash,
                    'Archive source changed: ' + name)
    rows = read(validation / 'results.json')['rows']
    names = [row['name'] for row in rows]
    sets = read(base / 'sets.json')
    require(len(names) == len(set(names)) == 130, 'Missing/duplicate result scenes')
    require(set(names) == set(sets['DEV'] + sets['REG']), 'Result scene set mismatch')
    require(dict(Counter(row['end'] for row in rows)) == expected, 'Outcomes changed')
    require(all(row.get('error') is None for row in rows), 'Recorded exception present')
    require({p.name[:-8] for p in validation.glob('*.json.gz')} == set(names),
            'Trace scene set mismatch')
    calls = 0
    for name in names:
        with gzip.open(validation / (name + '.json.gz'), 'rt', encoding='utf-8') as stream:
            trace = json.load(stream)
        require(bool(trace['frames']), 'Empty trace: ' + name)
        require(trace['result']['name'] == name, 'Trace identity mismatch: ' + name)
        require(trace['result']['end'] == next(r['end'] for r in rows if r['name'] == name),
                'Trace result mismatch: ' + name)
        calls += len(trace.get('act_calls', []))
    return manifest, calls


def main():
    export = read(ROOT / 'EXPORT_MANIFEST.json')
    recorded = export['files']
    actual = {p.relative_to(ROOT).as_posix() for p in ROOT.rglob('*')
              if p.is_file() and '.git' not in p.relative_to(ROOT).parts
              and '__pycache__' not in p.relative_to(ROOT).parts
              and p.name != 'EXPORT_MANIFEST.json'}
    require(actual == set(recorded), 'Export file set changed')
    for name, expected in recorded.items():
        require(digest(ROOT / name) == expected, 'Export file changed: ' + name)
    targets = [
        (ROOT / 'baselines/arb_w1_v1', 'validation/devreg_20261007_r1',
         {'到达': 72, '碰撞': 24, '超时': 34}),
        (RUNS / 'dynamic_route_r5_devreg130', 'validation',
         {'到达': 76, '碰撞': 22, '超时': 32}),
        (RUNS / 'supported_pruned_r96_devreg130', 'validation',
         {'到达': 104, '碰撞': 10, '超时': 16}),
        (RUNS / 'recovery_pulse_r99_devreg130', 'validation',
         {'到达': 107, '碰撞': 7, '超时': 16}),
    ]
    for base, subdir, expected in targets:
        manifest, calls = snapshot(base, base / subdir, expected)
        print(manifest['baseline_id'] + ': source/archive/130-scene evidence verified')
    r99 = targets[-1][0]
    manifest = read(r99 / 'manifest.json')
    require({p.relative_to(ROOT).as_posix() for p in (ROOT / 'stack').rglob('*.py')}
            == set(manifest['sources']), 'Root stack source set changed')
    for name, expected in manifest['sources'].items():
        require(digest(ROOT / name) == expected, 'Root stack differs from r99: ' + name)
    require(calls == 28659, 'r99 act-call count changed')
    review = read(r99 / 'release_review.json')
    for name, expected in review['log_sha256']['candidate'].items():
        require(digest(r99 / 'validation' / (name + '.json.gz')) == expected,
                'Audited log changed: ' + name)
    print('Root stack: 24 r99 sources verified; r99 act calls: 28,659; export files: '
          + str(len(recorded)))
    print('Offline verification passed. No planner or replay was run.')


if __name__ == '__main__':
    main()
