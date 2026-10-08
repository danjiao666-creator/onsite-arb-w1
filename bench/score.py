"""本地评分器：按第四届第一赛道官方公式（2026-03 规则原文）从闭环轨迹近似出分。

  总分 = 效率^0.5 × (安全×0.5 + 舒适×0.3 + 交规×0.1 + 协调×0.1)，满分 100

规则原文给了结构与大部分阈值；缺失处的本地假设集中在 ASSUME 里，改一处全局生效。
这是"方向正确的代理分"，不是官方分——用于 A/B 比较，不用于对外宣称。
"""
import math

ASSUME = {
    'ttc_thr_s': 2.0,             # 规则未给 TTC 阈值
    'default_limit_mps': 60 / 3.6,  # xodr 无 <speed> 时的限速下限；实际取 max(本值, 主车初速)
    'coord': 10.0,                # 协调维度（SV 影响）回放下不可观测，按满分计
}

COMFORT = (  # (名称, 阈值) —— 规则原文；每项 超阈时长/总时长 × 2
    ('lon_acc', 3.0), ('lon_jerk', 6.0), ('lat_acc', 0.5), ('lat_jerk', 1.0), ('yaw_rate', 0.5),
)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _min_ttc(fr):
    ex, ey, ev, eyaw = fr['x'], fr['y'], fr['v'], fr['yaw']
    c, s = math.cos(eyaw), math.sin(eyaw)
    best = math.inf
    for ox, oy, ov, oyaw, ol, ow in fr['objs'].values():
        dx = (ox - ex) * c + (oy - ey) * s
        dy = -(ox - ex) * s + (oy - ey) * c
        if dx <= 0 or abs(dy) > (fr['W'] + ow) / 2 + 0.3:
            continue
        closing = ev - ov * math.cos(_wrap(oyaw - eyaw))
        gap = dx - (fr['L'] + ol) / 2
        if closing > 0.1:
            best = min(best, max(gap, 0.0) / closing)
    return best


def score_run(r):
    frames = r.get('frames') or []
    if r['end'] in ('失效', 'load_fail') or len(frames) < 2:
        return {'total': 0.0, 'eff': 0.0, 'safety': 0.0, 'comfort': 0.0, 'rule': 0.0,
                'coord': 0.0, 'note': 'algorithm failure'}
    dt = r['dt']
    n = len(frames)
    T = max(frames[-1]['t'], dt)
    lim = r.get('speed_limit') or max(ASSUME['default_limit_mps'], frames[0]['v'])

    # ---- 效率：完成 50 + 期望时长/实际时长 × 50（期望=按限速行驶同样距离）
    dist = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y']) for a, b in zip(frames, frames[1:]))
    eff = (50.0 if r['end'] == '到达' else 0.0) + min(50.0, (dist / lim) / T * 50.0)

    # ---- 安全：碰撞清零；TTC 低于阈值时长占比 × 10（出界直接终止，单独不计时长）
    if r['end'] == '碰撞':
        safety = 0.0
    else:
        low = sum(1 for f in frames if _min_ttc(f) < ASSUME['ttc_thr_s'])
        safety = max(0.0, 10.0 - low / n * 10.0 - (10.0 if r['end'] == '出界' else 0.0))

    # ---- 舒适：五项超阈时长占比 × 2
    L_eff = [f['L'] / 1.7 for f in frames]            # 官方自行车模型：yaw_rate = v·1.7·tan(δ)/L
    yaw_rate = [f['v'] * math.tan(f['rot'] or 0.0) / Le for f, Le in zip(frames, L_eff)]
    lat_acc = [f['v'] * w for f, w in zip(frames, yaw_rate)]
    lon_acc = [f['a'] or 0.0 for f in frames]
    series = {
        'lon_acc': lon_acc,
        'lon_jerk': [0.0] + [(b - a) / dt for a, b in zip(lon_acc, lon_acc[1:])],
        'lat_acc': lat_acc,
        'lat_jerk': [0.0] + [(b - a) / dt for a, b in zip(lat_acc, lat_acc[1:])],
        'yaw_rate': yaw_rate,
    }
    comfort_detail = {k: sum(1 for x in series[k] if abs(x) > thr) / n for k, thr in COMFORT}
    comfort = max(0.0, 10.0 - sum(v * 2.0 for v in comfort_detail.values()))

    # ---- 交规：仅超速可观测（信控/压线/让行在回放台不可判）
    vmax = max(f['v'] for f in frames)
    rule = 10.0 - (5.0 if vmax > 1.5 * lim else 2.5 if vmax > 1.2 * lim else 0.0)

    coord = ASSUME['coord']
    total = math.sqrt(max(eff, 0.0)) * (safety * 0.5 + comfort * 0.3 + rule * 0.1 + coord * 0.1)
    return {'total': round(total, 2), 'eff': round(eff, 1), 'safety': round(safety, 2),
            'comfort': round(comfort, 2), 'rule': rule, 'coord': coord,
            'comfort_detail': {k: round(v, 3) for k, v in comfort_detail.items()},
            'dist': round(dist, 1), 'limit': round(lim, 2)}
