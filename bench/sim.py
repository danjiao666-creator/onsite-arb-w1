"""单场景闭环仿真：逐行复刻官方 OnSiteReplay.run 的帧序，但不写 outputs/、不占日志。

帧序（与官方一致）：
  update_frame（背景车到 t、判终止）→ 记录 → planner.act → check_action → update_ego（t+=dt）
终止优先级也与官方一致：出界 < 超时 < 碰撞 < 到达（后写覆盖先写）。

在 onsite38 解释器下运行（官方工具锁 numpy 1.19 / py3.8）。
"""
import importlib
import logging
import math
import os
import re
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOL = os.path.join(ROOT, 'code', 'onsite-structured-test-3.1.5')
EXTRACTED = os.path.join(ROOT, 'scenes', 'official_catalog', 'extracted')

PLANNERS = {
    'UC': 'planner.UC.uc_planner:UCPlanner',
    'ARB': 'stack.adapters.onsite_replay:ArbiterPlanner',
}

_booted = False


def _boot():
    """导入官方模块（必须 chdir 到工具目录——其 logger 用相对路径）并静音其文件日志。"""
    global _booted
    if _booted:
        return
    os.chdir(TOOL)
    for p in (TOOL, ROOT):
        if p not in sys.path:
            sys.path.insert(0, p)
    import utils.logger  # noqa: F401  触发官方 logging 配置
    for name in ('file', ''):
        lg = logging.getLogger(name)
        for h in list(lg.handlers):
            lg.removeHandler(h)
        lg.addHandler(logging.NullHandler())
        lg.propagate = False
    _install_xodr_cache()
    _booted = True


def _install_xodr_cache():
    """parse_opendrive 单次 ~10s（占单场景耗时 95%+），结果确定 → 磁盘缓存。
    必须在 ReplayParser / 规划器 `from ... import parse_opendrive` 之前替换。"""
    import hashlib
    import pickle
    import utils.opendrive2discretenet as od
    raw = od.parse_opendrive
    cache_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.cache', 'xodr')
    os.makedirs(cache_dir, exist_ok=True)

    def cached(path):
        st = os.stat(path)
        key = hashlib.md5(f'{os.path.abspath(path)}|{st.st_size}|{st.st_mtime_ns}'.encode()).hexdigest()
        fp = os.path.join(cache_dir, key + '.pkl')
        if os.path.exists(fp):
            try:
                with open(fp, 'rb') as f:
                    return pickle.load(f)
            except Exception:
                pass
        net = raw(path)
        tmp = f'{fp}.{os.getpid()}.tmp'
        with open(tmp, 'wb') as f:
            pickle.dump(net, f, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(tmp, fp)
        return net

    od.parse_opendrive = cached


def load_planner(key):
    mod, cls = PLANNERS.get(key, key).split(':')
    return getattr(importlib.import_module(mod), cls)


def speed_limit_mps(xodr_path):
    """xodr <speed max unit> 的众数，换算 m/s；缺失返回 None。"""
    try:
        txt = open(xodr_path, encoding='utf-8', errors='ignore').read()
    except OSError:
        return None
    vals = []
    for m in re.finditer(r'<speed[^>]*?max="([\d.eE+-]+)"[^>]*?unit="([^"]+)"', txt):
        v, u = float(m.group(1)), m.group(2).lower()
        k = {'mph': 0.44704, 'km/h': 1 / 3.6, 'kmh': 1 / 3.6, 'm/s': 1.0}.get(u)
        if k and v > 0:
            vals.append(round(v * k, 3))
    if not vals:
        return None
    return max(set(vals), key=vals.count)


def run_scene(name, planner_key='UC', record_objects=False):
    _boot()
    from utils.ScenarioManager.ScenarioManagerForReplay import ScenarioManagerForReplay
    from OnSiteReplay.ReplayController import ReplayController
    from utils.functions import check_action

    mgr = ScenarioManagerForReplay(EXTRACTED, {'tasks': [name], 'skipExist': False})
    if not mgr.next():
        return {'name': name, 'end': 'load_fail'}
    scene = mgr.cur_scene
    ctrl = ReplayController(False)
    ctrl.init(scene)
    planner = load_planner(planner_key)()
    wall0 = time.time()
    err = None
    try:
        planner.init(scene.format())
    except Exception as e:  # 规划器初始化失败 = 算法失效
        err = f'init: {type(e).__name__}: {e}'

    tgt = scene.task_info['targetPos']
    frames = []
    action = [float('nan'), float('nan')]
    end = -1
    while err is None:
        ctrl.update_frame()
        obs = ctrl.observation
        ego = obs.ego_info
        fr = {'t': obs.test_info['t'], 'x': ego.x, 'y': ego.y, 'v': ego.v, 'a': ego.a,
              'yaw': ego.yaw, 'rot': ego.rot, 'L': ego.length, 'W': ego.width}
        if record_objects:
            fr['objs'] = {f'{c}/{k}': (o.x, o.y, o.v, o.yaw, o.length, o.width)
                          for c, d in obs.object_info.items() for k, o in d.items()}
        else:
            fr['objs'] = {k: (o.x, o.y, o.v, o.yaw, o.length, o.width)
                          for c, d in obs.object_info.items() for k, o in d.items()}
        frames.append(fr)
        end = obs.test_info['end']
        if end != -1:
            break
        try:
            action = planner.act(ctrl.get_observation())
        except Exception as e:
            err = f'act@t={obs.test_info["t"]}: {type(e).__name__}: {e}'
            break
        if action is None or any(not math.isfinite(float(a)) for a in action):
            err = f'act@t={obs.test_info["t"]}: non-finite action {action}'
            break
        ego_action = check_action(dt=scene.task_info['dt'], prev_v=ego.v,
                                  prev_action=[ego.a, ego.rot], new_action=action)
        ctrl.update_ego(ego_action)

    ends = {1: '到达', 2: '超时', 3: '碰撞', 4: '出界'}
    tcx = (tgt[0][0] + tgt[1][0]) / 2
    tcy = (tgt[0][1] + tgt[1][1]) / 2
    last = frames[-1] if frames else None
    out = {
        'name': name,
        'planner': planner_key,
        'end': ends.get(end, '失效') if err is None else '失效',
        'error': err,
        't_end': last['t'] if last else 0.0,
        'max_t': ctrl.control_info.test_setting['max_t'],
        'dt': scene.task_info['dt'],
        'goal_dist': math.hypot(last['x'] - tcx, last['y'] - tcy) if last else None,
        'v_end': last['v'] if last else None,
        'speed_limit': speed_limit_mps(scene.source_file['xodr']),
        'wall_s': round(time.time() - wall0, 2),
        'frames': frames,
    }
    diag = getattr(planner, 'diagnostics', None)
    if callable(diag):
        try:
            out['diag'] = diag()
        except Exception:
            pass
    return out


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('scenes', nargs='+')
    ap.add_argument('-p', '--planner', default='UC')
    a = ap.parse_args()
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from score import score_run
    for n in a.scenes:
        r = run_scene(n, a.planner)
        s = score_run(r)
        print(f"{n:45s} {r['end']:4s} t={r['t_end']:5.1f}/{r['max_t']:<5.1f} "
              f"v={r['v_end'] or 0:4.1f} 距盒={r['goal_dist'] or -1:6.1f} 分={s['total']:5.1f} "
              f"{r['error'] or ''}", flush=True)
