"""Check complete candidate runs for byte-level states, actions and decisions."""
import argparse
import gzip
import json
from pathlib import Path


HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_text(encoding='utf8'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('original')
    parser.add_argument('optimized')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    folders = [HERE / 'runs' / tag / 'validation' for tag in (args.original, args.optimized)]
    documents = [read(folder / 'results.json') for folder in folders]
    names = [{row['name'] for row in document['rows']} for document in documents]
    assert names[0] == names[1] and len(names[0]) == 130, 'Both runs must cover DEV/REG130'
    rows = []
    total = 0
    for name in sorted(names[0]):
        before, after = [json.load(gzip.open(str(folder / (name + '.json.gz')), 'rt', encoding='utf8'))
                         for folder in folders]
        decisions = [[{key: value for key, value in cycle.items() if key != 'ms'}
                      for cycle in log['cycles']] for log in (before, after)]
        states_equal = before['frames'] == after['frames']
        decisions_equal = decisions[0] == decisions[1]
        rows.append(dict(name=name, states_equal=states_equal, decisions_equal=decisions_equal,
                         cycles=len(before['cycles'])))
        total += len(before['cycles'])
    report = dict(original=args.original, optimized=args.optimized, scenes=len(rows),
                  cycles=total, all_equal=all(row['states_equal'] and row['decisions_equal'] for row in rows),
                  rows=rows)
    args.output.write_text(json.dumps(report, indent=2), encoding='utf8')
    print({key: value for key, value in report.items() if key != 'rows'})
    assert report['all_equal'], [row for row in rows if not row['states_equal'] or not row['decisions_equal']]


if __name__ == '__main__':
    main()
