"""Explicit replay entry; requires external official platform and scene inputs."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'research/arb_capability_20261007/runs/recovery_pulse_r99_devreg130'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenes')
    parser.add_argument('--allow-shared-load', action='store_true')
    args = parser.parse_args()
    args.output = args.output.resolve()
    for dependency in ('code/onsite-structured-test-3.1.5',
                       'scenes/official_catalog/extracted',
                       'scenes/official_catalog/index.json'):
        if not (ROOT / dependency).exists():
            parser.error('Missing external dependency: ' + dependency)
    sys.path.insert(0, str(ROOT / 'bench'))
    import run_baseline as protocol
    protocol.BASE = BASE
    protocol.ENTRY = 'stack.adapters.onsite_close_guard:CloseGuardPlanner'
    manifest = protocol.verify()
    protocol.run(args, manifest)


if __name__ == '__main__':
    main()
