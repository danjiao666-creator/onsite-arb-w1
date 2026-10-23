"""Unpack the byte-preserved saved failure viewer without running a planner."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=root / 'outputs/r43_all_failures.html')
    args = parser.parse_args()
    target = args.output.resolve()
    data = gzip.decompress((root / 'replays/all_failures.html.gz').read_bytes())
    info = json.loads((root / 'review/EXPORT_PROVENANCE.json').read_text(encoding='utf-8'))
    expected = info['copies']['replays/all_failures.html.gz']['decoded_sha256']
    if hashlib.sha256(data).hexdigest() != expected:
        raise ValueError('Decoded viewer does not match the saved original')
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('xb') as stream:
        stream.write(data)
    print(str(target))


if __name__ == '__main__':
    main()
