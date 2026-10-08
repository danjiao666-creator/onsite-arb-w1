"""Run the immutable ARB-W1 baseline with the current, recorded replay harness.

The planner is imported from its verified snapshot before the harness boots.
No imports of the working-tree planner are allowed. DEV/REG only, one process.
"""
import argparse
from collections import Counter
import hashlib
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import zipfile


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'baselines' / 'arb_w1_v1'
ENTRY = 'stack.adapters.onsite_replay:ArbiterPlanner'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def save(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')


def verify(check_source=True):
    manifest = read(BASE / 'manifest.json')
    if digest(BASE / 'sources.zip') != manifest['archive_sha256']:
        raise ValueError('Frozen source archive changed')
    with zipfile.ZipFile(BASE / 'sources.zip') as archive:
        if set(archive.namelist()) != set(manifest['sources']):
            raise ValueError('Archive source list changed')
        for name, expected in manifest['sources'].items():
            if Path(name).is_absolute() or '..' in Path(name).parts or ':' in name:
                raise ValueError('Unsafe archive path: ' + name)
            if hashlib.sha256(archive.read(name)).hexdigest() != expected:
                raise ValueError('Archive source changed: ' + name)
            if check_source and digest(BASE / 'source' / name) != expected:
                raise ValueError('Frozen planner changed: ' + name)
    actual = {p.relative_to(BASE / 'source').as_posix()
              for p in (BASE / 'source').rglob('*.py')}
    if check_source and actual != set(manifest['sources']):
        raise ValueError('Unexpected or missing frozen Python source')
    if digest(BASE / 'sets.json') != manifest['sets_sha256']:
        raise ValueError('Frozen DEV/REG list changed')
    return manifest


def runtime_hashes():
    paths = list((ROOT / 'bench').glob('*.py'))
    paths += [ROOT / 'research/architecture_takeover_20260926/run.py']
    paths += list((ROOT / 'code/onsite-structured-test-3.1.5').rglob('*.py'))
    return {p.relative_to(ROOT).as_posix(): digest(p) for p in sorted(paths)}


def map_cache_identity(inputs):
    """Record the cache actually addressed by bench.sim, including missing entries."""
    records = {}
    for name in inputs:
        if not name.endswith('.xodr'):
            continue
        source = ROOT / name
        st = source.stat()
        key = hashlib.md5('{}|{}|{}'.format(
            os.path.abspath(str(source)), st.st_size, st.st_mtime_ns).encode()).hexdigest()
        cache = ROOT / 'bench/.cache/xodr' / (key + '.pkl')
        records[name] = dict(path=cache.relative_to(ROOT).as_posix(),
                             sha256=digest(cache) if cache.exists() else None)
    return records


def archive_files(target, files):
    with zipfile.ZipFile(target, 'w', zipfile.ZIP_DEFLATED) as archive:
        for name, expected in files.items():
            if digest(ROOT / name) != expected:
                raise ValueError('File changed while archiving: ' + name)
            archive.write(ROOT / name, name)


def other_python_processes():
    """Observe competing Python work without interrupting it."""
    command = ("[Console]::OutputEncoding=[System.Text.Encoding]::UTF8; "
               "$items=@(Get-CimInstance Win32_Process | Where-Object {"
               "$_.Name -match '^python(w)?\\.exe$' -and $_.ProcessId -ne "
               + str(os.getpid()) + "}); $before=@{}; foreach($item in $items){"
               "$p=Get-Process -Id $item.ProcessId -ErrorAction SilentlyContinue; "
               "if($p){$before[$item.ProcessId]=$p.CPU}}; Start-Sleep -Milliseconds 500; "
               "$out=foreach($item in $items){$p=Get-Process -Id $item.ProcessId "
               "-ErrorAction SilentlyContinue; if($p -and $before.ContainsKey($item.ProcessId)){"
               "$delta=$p.CPU-$before[$item.ProcessId]; "
               "if($delta -gt 0.05 -or $item.CommandLine -match 'architecture_takeover|run_baseline|onsite38'){"
               "[pscustomobject]@{ProcessId=$item.ProcessId;CommandLine=$item.CommandLine;"
               "cpu_seconds_in_half_second=$delta}}}}; $out | ConvertTo-Json -Compress")
    try:
        result = subprocess.run(['powershell', '-NoProfile', '-Command', command],
                                capture_output=True, text=True, encoding='utf-8', timeout=20, check=True)
        value = json.loads(result.stdout) if result.stdout.strip() else []
        return value if isinstance(value, list) else [value]
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        return [{'inspection_error': str(exc)}]


def check_imports():
    prefix = str((BASE / 'source').resolve()).lower() + os.sep
    for name, module in list(sys.modules.items()):
        if name == 'stack' or name.startswith('stack.'):
            location = str(Path(module.__file__).resolve()).lower()
            if not location.startswith(prefix):
                raise RuntimeError('Working-tree planner leaked into baseline: ' + name)


def run(args, manifest):
    overrides = {k: v for k, v in os.environ.items() if k.startswith(('ARB_', 'EXEC_'))}
    if overrides:
        raise ValueError('Baseline does not allow planner environment overrides: ' + repr(overrides))
    sets = read(BASE / 'sets.json')
    all_names = sets['DEV'] + sets['REG']
    names = all_names if args.scenes is None else args.scenes.split(',')
    if len(set(names)) != len(names) or not set(names).issubset(all_names):
        raise ValueError('Select unique scene names from the frozen DEV/REG list')
    competitors = other_python_processes()
    if competitors and not args.allow_shared_load:
        raise RuntimeError('Other Python work is active; wait before timing: ' + repr(competitors))
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    before = runtime_hashes()
    input_hashes = {}
    for name in names:
        scene = ROOT / 'scenes/official_catalog/extracted' / name
        if not scene.is_dir():
            raise FileNotFoundError(str(scene))
        for path in sorted(scene.rglob('*')):
            if path.is_file():
                input_hashes[path.relative_to(ROOT).as_posix()] = digest(path)
    caches_before = map_cache_identity(input_hashes)
    archive_files(out / 'runtime_sources.zip', before)
    archive_files(out / 'map_cache_before.zip', {
        value['path']: value['sha256'] for value in caches_before.values() if value['sha256']})
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(BASE / 'source'))
    importlib.import_module('stack.adapters.onsite_replay')
    check_imports()
    sys.path.insert(0, str(ROOT))
    from research.architecture_takeover_20260926 import run as harness
    import numpy as np
    meta = dict(baseline_id=manifest['baseline_id'], planner=ENTRY, jobs=1,
                frozen_planner_sources=manifest['sources'], archive_sha256=manifest['archive_sha256'],
                scenes=names, inputs=input_hashes, harness_sources=before,
                map_cache_before=caches_before,
                runtime_archive_sha256=digest(out / 'runtime_sources.zip'),
                map_cache_archive_sha256=digest(out / 'map_cache_before.zip'),
                python=sys.version, executable=sys.executable, numpy=np.__version__,
                environment={k: v for k, v in os.environ.items()
                             if k.startswith(('ARB_', 'EXEC_', 'OMP_', 'MKL_', 'OPENBLAS_'))},
                competing_python_before=competitors,
                timing_scope='complete act including raised calls; excludes init, harness and recording IO',
                started_at=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))
    save(out / 'manifest.json', meta)
    rows = []
    catalog = read(ROOT / 'scenes/official_catalog/index.json')
    for i, name in enumerate(names, 1):
        row = harness.worker((name, ENTRY, str(out)))
        row['roadtype'] = catalog[name]['roadtype']
        rows.append(row)
        save(out / 'results.json', dict(rows=rows))
        check_imports()
        if i % 10 == 0 or i == len(names):
            print(i, '/', len(names), dict(Counter(r['end'] for r in rows)), flush=True)
    after = runtime_hashes()
    verify()
    inputs_unchanged = all(digest(ROOT / p) == h for p, h in input_hashes.items())
    caches_after = map_cache_identity(input_hashes)
    caches_unchanged = all(value['sha256'] is None or value == caches_after[name]
                           for name, value in caches_before.items())
    created_caches = {value['path']: value['sha256'] for name, value in caches_after.items()
                      if caches_before[name]['sha256'] is None and value['sha256']}
    if created_caches:
        archive_files(out / 'map_cache_created.zip', created_caches)
    competitors_after = other_python_processes()
    unchanged = before == after and inputs_unchanged and caches_unchanged
    save(out / 'verification.json', dict(sources_unchanged=before == after,
        frozen_planner_unchanged=True, inputs_unchanged=inputs_unchanged,
        existing_map_caches_unchanged=caches_unchanged, map_cache_after=caches_after,
        imported_stack_sources={name: str(module.__file__) for name, module in sys.modules.items()
                                if name == 'stack' or name.startswith('stack.')},
        competing_python_after=competitors_after))
    summary = dict(baseline_id=manifest['baseline_id'], count=len(rows),
                   complete_dev_reg=len(rows) == len(all_names),
                   outcomes=dict(Counter(r['end'] for r in rows)),
                   runtime_identity_unchanged=unchanged,
                   measured_latency_pass_count=sum(bool(r.get('latency', {}).get('passed'))
                                                   for r in rows if r.get('latency')),
                   no_competing_python_observed_at_boundaries=not competitors and not competitors_after,
                   limitations=['Local replay, not official VTS scores.',
                                'Process observations do not prove exclusive host load.',
                                'Baseline retains known driving failures; not a safety certification.'])
    save(out / 'summary.json', summary)
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    if not unchanged:
        raise RuntimeError('Harness, scene inputs or existing map caches changed during the run')


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('command', choices=['verify', 'run', 'restore'])
    ap.add_argument('--output', type=Path)
    ap.add_argument('--scenes', help='Comma-separated DEV/REG names; default all 130')
    ap.add_argument('--allow-shared-load', action='store_true',
                    help='Driving check only: timing will retain shared-load evidence')
    args = ap.parse_args()
    manifest = verify(check_source=args.command != 'restore')
    if args.command == 'verify':
        print(manifest['baseline_id'], 'verified', len(manifest['sources']), 'files')
    elif args.command == 'restore':
        if args.output is None:
            ap.error('restore requires --output pointing to a NEW directory')
        args.output.mkdir(parents=True, exist_ok=False)
        with zipfile.ZipFile(BASE / 'sources.zip') as archive:
            for name in manifest['sources']:
                target = args.output / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(archive.read(name))
        print('Restored planner source to', args.output.resolve())
    else:
        if args.output is None:
            ap.error('run requires --output pointing to a NEW results directory')
        run(args, manifest)


if __name__ == '__main__':
    main()
