"""Audit actual mapped-body episodes in frozen runs without rerunning driving."""
import argparse
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import sys

from mapped_body_audit import audit_frames

SCRIPT = Path(__file__).resolve()
HERE = SCRIPT.parents[1]
PROJECT = next(parent for parent in SCRIPT.parents if (parent/'bench/sim.py').is_file())
sys.dont_write_bytecode = True


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenes', nargs='+', help='Optional named subset of the declared run scenes')
    args = parser.parse_args()
    runs, output = [path.resolve() for path in args.runs], args.output.resolve()
    if output.exists():
        parser.error('Preserve prior audit evidence')
    manifests = [read(run/'manifest.json') for run in runs]
    for run, manifest in zip(runs, manifests):
        assert all(digest(run/'source'/key) == value for key, value in manifest['sources'].items())
        assert digest(run/'sources.zip') == manifest['archive_sha256']
        assert manifest['sources'] == manifests[0]['sources'], 'Use a separate process for different source versions'
    sys.path[:0] = [str(runs[0]/'source'), str(PROJECT/'bench')]
    from stack.core.road_boundary import RoadBoundary
    import sim
    sim._boot()
    from utils.opendrive2discretenet import parse_opendrive
    boundaries, records, provenance = {}, [], []
    output.mkdir(parents=True)
    (output/'launcher.py').write_bytes(SCRIPT.read_bytes())
    (output/'mapped_body_audit.py').write_bytes((SCRIPT.parent/'mapped_body_audit.py').read_bytes())
    for run, manifest in zip(runs, manifests):
        validation_path = run/'validation/manifest.json'
        validation = read(validation_path)
        names = validation['scenes']
        if args.scenes:
            assert set(args.scenes) <= set(names), 'Requested scene is outside the run declaration'
            names = args.scenes
        results = {row['name']: row for row in read(run/'validation/results.json')['rows']}
        provenance.append(dict(run=run.name, source_archive_sha256=manifest['archive_sha256'],
            validation_manifest_sha256=digest(validation_path), manifest_sha256=digest(run/'manifest.json')))
        for name in names:
            path = run/'validation'/(name+'.json.gz')
            trace_hash = digest(path)
            with gzip.open(path, 'rt', encoding='utf-8') as stream:
                trace = json.load(stream)
            map_path = Path(sim.EXTRACTED)/name/(name+'.xodr')
            map_hash = digest(map_path)
            map_relative = map_path.relative_to(PROJECT).as_posix()
            assert validation['inputs'][map_relative] == map_hash, 'Frozen input map identity changed'
            if map_hash not in boundaries:
                boundaries[map_hash] = RoadBoundary(parse_opendrive(str(map_path)))
            boundary = boundaries[map_hash]
            body = audit_frames(trace['frames'], boundary)
            first_growth = body.get('first_exterior_growth_t')
            previous = next((cycle for cycle in reversed(trace['cycles'])
                if first_growth is not None and cycle['t'] < first_growth-1e-7), None)
            precursor = None
            if previous:
                safety = previous['safety']
                row = safety['selected']
                precursor = dict(t=previous['t'], selected={key: value for key, value in row.items()
                    if key not in ('states', 'commands', 'acceleration_proposal')},
                    feasible_witness_lower_bound=safety['module_evidence']['feasible_count'],
                    selection_policy=safety['assessment_contract']['selection_policy'])
            record = dict(run=run.name, scene=name, official_outcome=results[name]['end'],
                trace_sha256=trace_hash, map_sha256=map_hash, actual_mapped_boundary=body,
                first_growth_precursor=precursor,
                attribution='route + controller + mapped_boundary + risk_selection' if first_growth is not None else None,
                certainty='Observed mapped-body exterior; requires geometry and candidate counterfactual for causal or legal-road claim')
            records.append(record)
            (output/(run.name+'__'+name+'.json')).write_text(
                json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8')
            print(json.dumps(dict(run=run.name, scene=name, outcome=record['official_outcome'],
                first_exterior_growth_t=first_growth, max_exterior_area=body.get('max_exterior_area')),
                ensure_ascii=False), flush=True)
        assert digest(validation_path) == provenance[-1]['validation_manifest_sha256']
    summary = []
    for run in runs:
        group = [row for row in records if row['run'] == run.name]
        growing = [row for row in group if row['actual_mapped_boundary'].get('first_exterior_growth_t') is not None]
        summary.append(dict(run=run.name, scenes=len(group), actual_exterior_growth_scenes=len(growing),
            growth_outcomes=dict(Counter(row['official_outcome'] for row in growing)),
            arrival_with_exterior_growth=[row['scene'] for row in growing if row['official_outcome'] == '到达'],
            initial_exterior_unrecovered=[row['scene'] for row in group
                if row['actual_mapped_boundary'].get('initial_exterior_unrecovered')],
            initial_overhang_scenes=sum(row['actual_mapped_boundary'].get('initial_exterior_area', 0.) > 0. for row in group)))
    for run, manifest in zip(runs, manifests):
        assert all(digest(run/'source'/key) == value for key, value in manifest['sources'].items())
    result = dict(summary=summary, runs=provenance, scenes=records, source_identity_passed=True,
        tool_sha256=digest(SCRIPT), geometry_tool_sha256=digest(SCRIPT.parent/'mapped_body_audit.py'),
        unique_map_count=len(boundaries), driving_rerun=False, official_outcomes_changed=False,
        future_used_for_policy=False, limitations=[
            'Mapped-body checks are sampled, not a continuous-sweep or legal-road certificate.',
            'Supplied map geometry may itself have defects; exterior growth is not automatic proof of controller fault.'])
    (output/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    main()
