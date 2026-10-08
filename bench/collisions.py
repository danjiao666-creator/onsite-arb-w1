"""碰撞归因：重跑结果中的碰撞题，按撞击方位 / 双方速度 / 对方是否在自车路线上分类。

用法：python bench/collisions.py <results_tag> [-p ARB]
"""
import json
import math
import os
import sys
from collections import Counter
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def _one(args):
    name, planner = args
    import cv2
    import numpy as np
    from sim import run_scene
    r = run_scene(name, planner)
    if r['end'] != '碰撞':
        return name, None
    f = r['frames'][-1]
    ego = ((f['x'], -f['y']), (f['L'], f['W']), math.degrees(-f['yaw']))
    hit = None
    for k, (ox, oy, ov, oyaw, ol, ow) in f['objs'].items():
        if cv2.rotatedRectangleIntersection(ego, ((ox, -oy), (ol, ow), math.degrees(-oyaw)))[0]:
            hit = (k, ox, oy, ov, oyaw, ol, ow)
            break
    if hit is None:
        return name, {'where': '?'}
    k, ox, oy, ov, oyaw, ol, ow = hit
    c, s = math.cos(f['yaw']), math.sin(f['yaw'])
    dx = (ox - f['x']) * c + (oy - f['y']) * s
    dy = -(ox - f['x']) * s + (oy - f['y']) * c
    dh = math.degrees((oyaw - f['yaw'] + math.pi) % (2 * math.pi) - math.pi)
    where = 'rear' if dx < -f['L'] * 0.4 else 'front' if dx > f['L'] * 0.4 else 'side'
    rel = 'same' if abs(dh) < 30 else 'oncoming' if abs(dh) > 150 else 'crossing'
    # 对方速度矢量在自车纵向上的分量：追尾方向判定
    return name, {'where': where, 'rel': rel, 'v_ego': round(f['v'], 1), 'v_obj': round(ov, 1),
                  'dx': round(dx, 1), 'dy': round(dy, 1), 'dh': round(dh), 't': f['t'], 'obj': k}


if __name__ == '__main__':
    tag = sys.argv[1]
    planner = sys.argv[3] if len(sys.argv) > 3 and sys.argv[2] == '-p' else 'ARB'
    rows = json.load(open(os.path.join(HERE, 'results', f'{tag}.json'), encoding='utf-8'))['rows']
    names = [r['name'] for r in rows if r['end'] == '碰撞']
    rt = {r['name']: r['roadtype'] for r in rows}
    with Pool(10) as p:
        res = [x for x in p.map(_one, [(n, planner) for n in names]) if x[1]]
    cnt = Counter()
    for n, d in sorted(res, key=lambda x: (x[1].get('where', ''), x[1].get('rel', ''))):
        stopped = 'ego停' if d.get('v_ego', 9) < 1.0 else 'ego动'
        cnt[(d.get('where'), d.get('rel'), stopped)] += 1
        print(f"{n[:36]:36s} {rt[n][:2]} {d}")
    print()
    for k, v in cnt.most_common():
        print(v, k)
