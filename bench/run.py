"""并行基准：跑 DEV/REG/HOLD（或场景名列表）→ 结局 + 官方公式代理分，按题型汇总。

用法（onsite38 解释器，任意目录）：
  python bench/run.py DEV -p ARB --tag arb_v1
  python bench/run.py DEV -p UC --tag uc_iter12 --check   # 与官方闭环 bench_DEV_latest.json 对账
  python bench/run.py DEV -p ARB --tag x --vs uc_iter12   # 逐题对比
"""
import argparse
import json
import os
import sys
import time
from collections import Counter, defaultdict
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
MANIFEST = os.path.join(ROOT, 'research', 'official_tool_patches', 'bench_manifest.json')
INDEX = os.path.join(ROOT, 'scenes', 'official_catalog', 'index.json')
RESULTS = os.path.join(HERE, 'results')


def _work(args):
    name, planner = args
    from sim import run_scene
    from score import score_run
    try:
        r = run_scene(name, planner)
    except Exception as e:
        r = {'name': name, 'end': '失效', 'error': f'harness: {type(e).__name__}: {e}',
             'frames': [], 't_end': 0, 'max_t': 0, 'dt': 0.1}
    s = score_run(r)
    r.pop('frames', None)
    r['score'] = s
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('set', help='DEV/REG/HOLD 或逗号分隔场景名')
    ap.add_argument('-p', '--planner', default='ARB')
    ap.add_argument('--tag', default=None)
    ap.add_argument('-j', '--jobs', type=int, default=10)
    ap.add_argument('--vs', default=None, help='对比另一次结果的 tag')
    ap.add_argument('--check', action='store_true', help='与官方闭环 bench_<SET>_latest.json 对账')
    a = ap.parse_args()

    idx = json.load(open(INDEX, encoding='utf-8'))
    sets = json.load(open(MANIFEST, encoding='utf-8'))['sets']
    if all(x in sets for x in a.set.split('+')):
        names = [n for x in a.set.split('+') for n in sets[x]]
    else:
        names = a.set.split(',')
    tag = a.tag or f'{a.set}_{a.planner}'

    t0 = time.time()
    with Pool(a.jobs) as pool:
        rows = pool.map(_work, [(n, a.planner) for n in names], chunksize=1)
    wall = time.time() - t0
    for r in rows:
        r['roadtype'] = idx.get(r['name'], {}).get('roadtype', '未知')

    os.makedirs(RESULTS, exist_ok=True)
    out = os.path.join(RESULTS, f'{tag}.json')
    json.dump({'tag': tag, 'set': a.set, 'planner': a.planner, 'created': time.strftime('%Y-%m-%d %H:%M'),
               'rows': rows}, open(out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)

    summarize(rows, f'{tag}  ({len(rows)} 题, {wall:.0f}s)')
    fails = [r for r in rows if r['end'] == '失效']
    for r in fails[:10]:
        print(f"  失效 {r['name']}: {r.get('error')}")
    if a.check:
        check_official(rows, a.set)
    if a.vs:
        compare(rows, a.vs)
    print('明细:', out)


def summarize(rows, title):
    by = defaultdict(list)
    for r in rows:
        by[r['roadtype']].append(r)
    print(f'\n===== {title} =====')
    print(f"{'题型':10s} {'n':>3s} {'到达':>4s} {'碰撞':>4s} {'超时':>4s} {'出界':>4s} {'失效':>4s} "
          f"{'均分':>6s} {'效率':>5s} {'安全':>5s} {'舒适':>5s}")
    for rt, rs in sorted(by.items()) + [('合计', rows)]:
        c = Counter(r['end'] for r in rs)
        n = len(rs)
        m = lambda k: sum(r['score'][k] for r in rs) / n  # noqa: E731
        print(f"{rt:10s} {n:3d} {c['到达']:4d} {c['碰撞']:4d} {c['超时']:4d} {c['出界']:4d} {c['失效']:4d} "
              f"{m('total'):6.1f} {m('eff'):5.1f} {m('safety'):5.2f} {m('comfort'):5.2f}")


def check_official(rows, set_name):
    p = os.path.join(ROOT, 'research', 'official_tool_patches', f'bench_{set_name}_latest.json')
    off = json.load(open(p, encoding='utf-8'))['per_scenario']
    same = sum(1 for r in rows if off.get(r['name'], {}).get('end') == r['end'])
    print(f'\n对账官方闭环：{same}/{len(rows)} 结局一致')
    for r in rows:
        o = off.get(r['name'], {}).get('end')
        if o != r['end']:
            print(f"  {r['name']:45s} 官方={o} 本地={r['end']}")


def compare(rows, vs_tag):
    old = {r['name']: r for r in json.load(open(os.path.join(RESULTS, f'{vs_tag}.json'), encoding='utf-8'))['rows']}
    d_tot = 0.0
    gains, losses = [], []
    for r in rows:
        o = old.get(r['name'])
        if not o:
            continue
        d = r['score']['total'] - o['score']['total']
        d_tot += d
        if o['end'] != r['end']:
            (gains if d > 0 else losses).append((r['name'], o['end'], r['end'], d))
    n = len(rows)
    print(f'\n对比 {vs_tag}: 均分 Δ={d_tot / n:+.2f}')
    for title, lst in (('改善', gains), ('退化', losses)):
        print(f'  {title} {len(lst)}:')
        for nm, a, b, d in sorted(lst, key=lambda x: -abs(x[3]))[:25]:
            print(f'    {nm:45s} {a}→{b} {d:+.1f}')


if __name__ == '__main__':
    main()
