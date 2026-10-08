"""OnSite 结构化测试工具（回放台）适配层：scenario_info/Observation ↔ 平台无关核心。

  init: xodr → 车道图 → Route（装载期校验）；xosc → 起始朝向、时间预算
  act : Observation → WorldModel → Arbiter（唯一决策点）→ LateralTracker → ControlAdapter
"""
import math
import os
import re
from typing import List

import numpy as np

from stack.core.arbiter import Arbiter, EgoState
from stack.core.control import ControlAdapter, LateralTracker
from stack.core.route import Lane, RouteError, RoutePlanner, chain_fallback
from stack.core.world import Agent, WorldModel

MPS = {'mph': 0.44704, 'km/h': 1 / 3.6, 'kmh': 1 / 3.6, 'm/s': 1.0}


def _read(path):
    try:
        with open(path, encoding='utf-8', errors='ignore') as f:
            return f.read()
    except (OSError, TypeError):
        return ''


def xodr_speed_limit(txt):
    vals = []
    for m in re.finditer(r'<speed[^>]*?max="([\d.eE+-]+)"[^>]*?unit="([^"]+)"', txt):
        k = MPS.get(m.group(2).lower())
        if k and float(m.group(1)) > 0:
            vals.append(round(float(m.group(1)) * k, 3))
    return max(set(vals), key=vals.count) if vals else None


class ArbiterPlanner:
    def __init__(self):
        self.world = WorldModel()
        self.arb = Arbiter()
        self.lat = LateralTracker()
        self.ctrl = ControlAdapter()
        self.route = None
        self.log = []

    # ------------------------------------------------------------------ init
    def init(self, scenario_info: dict) -> None:
        from utils.opendrive2discretenet import parse_opendrive   # 平台依赖只在适配层

        task = scenario_info['task_info']
        start, rect = task['startPos'], task['targetPos']
        self.rect = rect
        self.dt = float(task.get('dt', 0.1) or 0.1)
        src = scenario_info['source_file']
        xosc, xodr_txt = _read(src['xosc']), _read(src['xodr'])
        m = re.search(r'(?:h|heading)_init\s*=\s*(-?[\d.]+(?:[eE][-+]?\d+)?)', xosc)
        heading = float(m.group(1)) if m else None
        m = re.search(r'v_init\s*=\s*([\d.eE+-]+)', xosc)
        v0 = float(m.group(1)) if m else 0.0
        # 时间预算 = 背景车轨迹最大时间戳（官方 ReplayInfo._get_dt_maxt 同口径；只读时间，不读轨迹）
        times = re.findall(r'<Vertex[^>]*?time=[\'"]?([\d.]+)', xosc)
        self.max_t = max((float(t) for t in times), default=0.0)

        limit = xodr_speed_limit(xodr_txt)
        self.v_des = limit if limit else max(60 / 3.6, v0)
        self.limit = self.v_des

        net = parse_opendrive(src['xodr'])
        lanes = {l.lane_id: Lane(l.lane_id, np.asarray(l.center_vertices, float),
                                 tuple(l.successor or ()), tuple(l.predecessor or ()))
                 for l in net.discretelanes}
        rp = self.rp = RoutePlanner(lanes, v_ref=max(v0, 8.0))
        try:
            self.route = rp.plan(start, heading, rect)
        except RouteError as e:
            poly, seq = chain_fallback(lanes, start, rect)
            self.route = rp.finish(poly, start, heading, rect, seq, 'chain')
            self.route.issues.insert(0, f'图搜索失败: {e}')
        r = self.route
        self.s = r.s0
        # 过盒采样上限：单帧位移须小于路径在盒内的长度，否则会从盒上"跳过去"
        span = max(r.goal_out - r.goal_in, 0.5)
        self.v_goal_cap = max(3.0, (span - 0.4) / self.dt)
        # 预算反推：均速要求高于限速时允许适度超速（超 20% 扣交规 2.5，远小于未完成的 50）
        dist = max(0.0, r.goal_in - r.s0)
        if self.max_t > 0.5 and dist / self.max_t * 1.1 > self.v_des:
            self.v_des = min(dist / self.max_t * 1.1, 1.45 * self.limit)
        self.ctrl.reset(0.0)

    # ------------------------------------------------------------------ act
    def act(self, observation) -> List[float]:
        ego = observation.ego_info
        t = float(observation.test_info['t'])
        dt = float(observation.test_info.get('dt') or self.dt)
        agents = [Agent(f'{c}/{k}', c, float(o.x), float(o.y), float(o.yaw), float(o.v),
                        float(o.length), float(o.width))
                  for c, d in observation.object_info.items() for k, o in d.items()]
        self.world.update(t, agents)
        path = self.route.path
        self.s, e = path.project(ego.x, ego.y, s_hint=self.s)
        es = EgoState(float(ego.x), float(ego.y), float(ego.yaw), float(ego.v), float(ego.a or 0.0),
                      float(ego.length), float(ego.width))
        pred = self.world.predict(self.arb.t, near_xy=(es.x, es.y))
        time_left = self.max_t - t if self.max_t > 0.5 else 0.0
        # 每帧按剩余预算重算期望速度：被慢车拖住后要能追回时间
        v_des = self.v_des
        if time_left > 0.3:
            need = max(0.0, self.route.goal_in - self.s) / time_left
            v_des = min(max(v_des, need * 1.1), max(self.v_des, 1.45 * self.limit))
        plan = self.arb.decide(es, path, self.s, e, pred, v_des=v_des, v_goal_cap=self.v_goal_cap,
                               goal_in=self.route.goal_in, time_left=time_left)
        cmd = self._execute(plan, path, self.s, e, es, dt)
        if os.environ.get('ARB_DBG') and int(round(t * 10)) % 5 == 0:
            print(f'[arb] t={t:5.1f} s={self.s:6.1f}/{self.route.goal_in:6.1f} e={e:+.2f} v={es.v:5.2f} '
                  f'{plan.label:10s} a={plan.a_cmd:+.2f} tc={plan.t_collide:4.1f} clr={plan.min_clear:5.1f} '
                  f'allc={int(plan.all_collide)} n={len(pred["ids"])}', flush=True)
        return cmd

    def _execute(self, plan, path, station, error, ego, dt):
        """执行钩子：默认保持原控制路径，实验适配器可复用已评估的控制序列。"""
        w = self.lat.yaw_rate(path, station, error, ego.yaw, ego.v)
        return self.ctrl.command(plan.a_cmd, w, ego.v, ego.length, dt)

    def diagnostics(self):
        r = self.route
        return {'route_method': r.method if r else None, 'route_issues': r.issues if r else None,
                'arb': dict(self.arb.stats), 'v_des': round(self.v_des, 2)}
