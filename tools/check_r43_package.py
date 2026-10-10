"""Run the exported r43 contracts without driving or overwriting historical receipts."""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'candidates/r43'
sys.dont_write_bytecode = True


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--official-functions', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        parser.error('Preserve existing validation receipts; choose a new output')
    if ROOT in output.parents and any(part in output.relative_to(ROOT).parts for part in ('source', 'runs', 'baselines')):
        parser.error('Write the new receipt outside frozen source or runs')
    record = json.loads((BASE / 'verification.json').read_text(encoding='utf-8'))
    source = {path.relative_to(BASE / 'source').as_posix():digest(path)
              for path in (BASE / 'source').rglob('*.py')}
    if source != record['source_sha256']:
        raise ValueError('Exported planner differs from verified r43')
    scripts = ['test_unified.py', 'test_route_seam.py', 'test_compound_route.py', 'test_mapped_body_audit.py']
    official = args.official_functions or ROOT / 'code/onsite-structured-test-3.1.5/utils/functions.py'
    environment = dict(os.environ, PYTHONIOENCODING='utf-8', PYTHONDONTWRITEBYTECODE='1')
    if official.is_file():
        environment['ONSITE_OFFICIAL_FUNCTIONS'] = str(official.resolve())
        scripts.append('test_p0_contract.py')
    elif args.official_functions:
        parser.error('Official pure-function source does not exist')
    syntax_paths = list((BASE / 'source').rglob('*.py')) + list((BASE / 'tests').glob('*.py'))
    syntax_paths += [ROOT / 'tools/replay_r43.py', ROOT / 'tools/compare_uploaded_versions.py', Path(__file__)]
    for path in syntax_paths:
        ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    results = []
    for name in scripts:
        path = BASE / 'tests' / name
        completed = subprocess.run([sys.executable, '-B', str(path)], cwd=str(ROOT),
            env=environment, capture_output=True, text=True, encoding='utf-8')
        results.append(dict(test=name, sha256=digest(path), exit_code=completed.returncode,
            stdout=completed.stdout, stderr=completed.stderr))
        print(name, 'passed' if completed.returncode == 0 else 'FAILED', flush=True)
    passed = all(row['exit_code'] == 0 for row in results)
    receipt = dict(passed=passed, interpreter=sys.executable, source_files=len(source),
        source_sha256=source, syntax_files=len(syntax_paths), tests=results,
        p0_ran='test_p0_contract.py' in scripts,
        official_functions_sha256=digest(official) if official.is_file() else None,
        historical_verification_sha256=digest(BASE / 'verification.json'),
        scope='Export contract and syntax checks only; no scenario replay or safety certification.')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(receipt, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(passed=passed, source_files=len(source), p0_ran=receipt['p0_ran']), ensure_ascii=False))
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
