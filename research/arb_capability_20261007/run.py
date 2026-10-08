"""Freeze an independent candidate, then reuse the baseline replay protocol."""
import argparse
import hashlib
from pathlib import Path
import shutil
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'bench'))
import run_baseline as protocol  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tag', required=True)
    parser.add_argument('--entry', default='stack.adapters.onsite_replay:ArbiterPlanner')
    parser.add_argument('--source', type=Path, default=HERE / 'source')
    parser.add_argument('--scenes')
    parser.add_argument('--allow-shared-load', action='store_true')
    args = parser.parse_args()
    if not args.tag.replace('_', '').replace('-', '').isalnum():
        parser.error('tag must contain only letters, numbers, underscores or hyphens')
    baseline = protocol.verify()
    sets = protocol.read(protocol.BASE / 'sets.json')
    if args.scenes is not None:
        names = args.scenes.split(',')
        if len(names) != len(set(names)) or not set(names).issubset(sets['DEV'] + sets['REG']):
            parser.error('Select unique scene names from the frozen DEV/REG list')
    target = HERE / 'runs' / args.tag
    target.mkdir(parents=True, exist_ok=False)
    files = {}
    with zipfile.ZipFile(target / 'sources.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(args.source.rglob('*.py')):
            name = path.relative_to(args.source).as_posix()
            content = path.read_bytes()
            files[name] = hashlib.sha256(content).hexdigest()
            dest = target / 'source' / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(content)
            archive.writestr(name, content)
    shutil.copyfile(protocol.BASE / 'sets.json', target / 'sets.json')
    manifest = dict(baseline_id='ARB-CAPABILITY-' + args.tag, parent_baseline=baseline['baseline_id'],
                    parent_archive_sha256=baseline['archive_sha256'], planner=args.entry,
                    archive_sha256=protocol.digest(target / 'sources.zip'), sources=files,
                    sets_sha256=protocol.digest(target / 'sets.json'))
    protocol.save(target / 'manifest.json', manifest)
    protocol.BASE = target
    protocol.ENTRY = args.entry
    protocol.verify()
    args.output = target / 'validation'
    # The platform boot changes cwd, so capture this absolute path before it runs.
    launcher = HERE / 'run.py'
    shutil.copyfile(launcher, target / 'launcher.py')
    protocol.save(target / 'launcher_identity.json', {
        'sha256': protocol.digest(launcher), 'path': launcher.relative_to(ROOT).as_posix()})
    protocol.run(args, manifest)


if __name__ == '__main__':
    main()
