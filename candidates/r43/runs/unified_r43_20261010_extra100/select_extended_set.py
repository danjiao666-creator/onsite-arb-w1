"""Freeze an outcome-independent development extension, excluding sealed HOLD."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import re
import sys

SCRIPT = Path(__file__).resolve()
HERE = SCRIPT.parents[1]
PROJECT = HERE.parents[2]
sys.dont_write_bytecode = True


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rank(name):
    return hashlib.sha256(('onsite-dev-extension-20261010|'+name).encode()).hexdigest()


def subgroup(name):
    if name.startswith('highway_merge_'):
        return '_'.join(name.split('_')[:4])
    if 'follow' in name:
        return 'follow'
    if 'cutin' in name:
        return 'cutin'
    if 'lanechang' in name:
        return 'lanechanging'
    return re.sub(r'\d+', '#', name)


def select(catalog, base, hold, extracted, quotas):
    groups = defaultdict(lambda: defaultdict(list))
    for name, item in catalog.items():
        if name in base or name in hold or item.get('testtype') != '回放测试':
            continue
        kind = item.get('roadtype')
        if kind in quotas and (extracted/name).is_dir():
            groups[kind][subgroup(name)].append(name)
    selected, available = [], {}
    for kind, quota in quotas.items():
        buckets = {key: sorted(values, key=rank) for key, values in groups[kind].items()}
        available[kind] = sum(len(values) for values in buckets.values())
        if available[kind] < quota:
            raise ValueError('Insufficient non-HOLD extracted replay scenes: '+kind)
        keys = sorted(buckets, key=rank)
        chosen = []
        while len(chosen) < quota:
            for key in keys:
                if buckets[key] and len(chosen) < quota:
                    chosen.append(buckets[key].pop(0))
        selected.extend(chosen)
    return selected, available


def verify_extension(path):
    data = read(path)
    base = read(PROJECT/'baselines/arb_w1_v1/sets.json')
    split = read(PROJECT/'research/official_tool_patches/bench_manifest.json')
    original = set(base['DEV']+base['REG'])
    hold = set(split['sets']['HOLD'])
    assert base['DEV'] == split['sets']['DEV'] and base['REG'] == split['sets']['REG']
    assert digest(PROJECT/'scenes/official_catalog/index.json') == data['catalog_sha256']
    assert digest(PROJECT/'baselines/arb_w1_v1/sets.json') == data['original_sets_sha256']
    assert digest(PROJECT/'research/official_tool_patches/bench_manifest.json') == data['sealed_split_sha256']
    names = data['scenes']
    assert len(names) == len(set(names)) == sum(data['quotas'].values())
    assert not set(names) & (original | hold)
    for rel, value in data['selected_input_sha256'].items():
        path = PROJECT/rel
        assert path.relative_to(PROJECT/'scenes/official_catalog/extracted').parts[0] in names
        assert digest(path) == value
    return data


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise ValueError('Preserve frozen selection; use a new output path')
    catalog_path = PROJECT/'scenes/official_catalog/index.json'
    sets_path = PROJECT/'baselines/arb_w1_v1/sets.json'
    split_path = PROJECT/'research/official_tool_patches/bench_manifest.json'
    catalog, original, split = read(catalog_path), read(sets_path), read(split_path)
    base, hold = set(original['DEV']+original['REG']), set(split['sets']['HOLD'])
    assert original['DEV'] == split['sets']['DEV'] and original['REG'] == split['sets']['REG']
    quotas = {'高速公路基本段': 25, '汇入汇出': 35, '交叉口': 40}
    extracted = PROJECT/'scenes/official_catalog/extracted'
    names, available = select(catalog, base, hold, extracted, quotas)
    hashes = {p.relative_to(PROJECT).as_posix(): digest(p)
              for name in names for p in sorted((extracted/name).rglob('*')) if p.is_file()}
    data = dict(id='dev_extension100_20261010', scope='additional_development_not_holdout',
        seed_rule='sha256 onsite-dev-extension-20261010 plus scene name; subgroup round robin',
        quotas=quotas, available_non_hold_extracted=available, scenes=names,
        strata={name: dict(roadtype=catalog[name]['roadtype'], subgroup=subgroup(name)) for name in names},
        original_devreg_count=len(base), excluded_hold_count=len(hold),
        catalog_sha256=digest(catalog_path), original_sets_sha256=digest(sets_path),
        sealed_split_sha256=digest(split_path), selected_input_sha256=hashes,
        tool_sha256=digest(SCRIPT), selection_uses_outcomes=False, future_trajectories_used_for_selection=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    verify_extension(output)
    print(json.dumps(dict(selection=str(output), scenes=len(names), quotas=quotas,
                         original_devreg_preserved=True, hold_excluded=True), ensure_ascii=False))


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    main()
