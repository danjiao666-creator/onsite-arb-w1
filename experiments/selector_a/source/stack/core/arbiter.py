"""机动仲裁：唯一决策点。

每帧生成一族候选纵向剖面（急刹 … 加速，加加速度受限），沿路线推演自车占用，
与世界模型的周车占用做同一种时空检查，按显式代价选 1 个：

  代价 = 碰撞（按首撞时刻，越早越重）+ 余隙不足 + 跟车时距不足
       + 到达耗时（含时间预算超支）+ 舒适 + 与上一决策的一致性

没有"路口让行""汇入挤入""横穿冲突"各自的 if 分支：停在冲突区里会被推演出的横穿车撞到，
挤进无间隙的车道会被推演出的后车撞到，刹停会被不减速的回放后车追尾——
这些都由同一个检查给出，不再互相污染。
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from .geometry import Path
from .native_distance import circle_distance
from .native_envelope import backwards
from .tail_eta import continuation
from .goal_contract import goal_contains


@dataclass
class EgoState:
    x: float
    y: float
    yaw: float
    v: float
    a: float
    length: float
    width: float


@dataclass
class Plan:
    a_cmd: float                 # 本帧纵向加速度指令
    label: str
    cost: float
    t_collide: float             # 该剖面首撞时刻（inf=无）
    min_clear: float
    s: np.ndarray = field(repr=False, default=None)
    v: np.ndarray = field(repr=False, default=None)
    a: np.ndarray = field(repr=False, default=None)
    all_collide: bool = False
    on_time: bool = False        # rolled arrival within final deadline horizon
    current_circle_overlap_ids: tuple = ()
    current_rectangle_overlap_ids: tuple = ()


@dataclass
class ArbiterConfig:
    horizon: float = 6.0
    dt: float = 0.1              # 剖面积分步长
    check_every: int = 2         # 每 2 步做一次碰撞检查（0.2s）
    jerk: float = 6.0            # 舒适加加速度（规则阈值）
    jerk_emerg: float = 20.0
    a_first: tuple = (-8.0, -6.0, -4.0, -3.0, -2.0, -1.0, 0.0, 0.8, 1.6, 2.5)
    a_second: tuple = (-3.0, 0.0, 1.5)
    t_switch: float = 2.0
    env_decel: float = 2.0       # 速度包络（弯道/终点）提前减速度
    lat_acc: float = 2.0         # 弯道横向加速度上限（包络）
    buf0: float = 0.15           # 碰撞判定膨胀：buf0 + buf_t·t
    buf_t: float = 0.06
    w_coll: float = 1000.0
    w_clear: float = 4.0
    clear_soft: float = 1.0
    w_headway: float = 2.0
    headway: float = 0.8
    w_budget: float = 3.0
    w_acc: float = 0.02
    w_jerk: float = 0.01
    w_cons: float = 0.05
    w_over: float = 0.5          # 超期望速度代价（软约束；硬上限只剩弯道/过盒/1.5×期望）
    v_hard: float = 1.5


def config_from_env(cfg: ArbiterConfig) -> ArbiterConfig:
    """实验钩子：ARB_CFG="clear_soft=1.0,buf_t=0.04" 覆盖字段（A/B 用，不改代码）。"""
    import os
    for kv in filter(None, os.environ.get('ARB_CFG', '').split(',')):
        k, v = kv.split('=')
        setattr(cfg, k.strip(), type(getattr(cfg, k.strip()))(float(v)))
    return cfg


class Arbiter:
    def __init__(self, cfg: Optional[ArbiterConfig] = None):
        self.cfg = config_from_env(cfg or ArbiterConfig())
        c = self.cfg
        self.n = int(round(c.horizon / c.dt))
        self.t = np.arange(1, self.n + 1) * c.dt
        self.kc = np.arange(c.check_every - 1, self.n, c.check_every)
        self.tc = self.t[self.kc]
        self.cands = [(a1, a2) for a1 in c.a_first for a2 in c.a_second
                      if not (a1 < -2.0 and a2 > 0)] + [('cruise', None)]
        self.last: Optional[Plan] = None
        self.stats = {}

    # ------------------------------------------------------------- 速度包络
    def envelope(self, path: Path, s0: float, v_des: float, v_goal_cap: float, goal_in: float):
        """沿路线的速度上限（限速 / 弯道横向加速度 / 过盒采样上限），并做提前减速回溯。"""
        c = self.cfg
        ss = np.arange(s0, s0 + 250.0, 1.0)
        k = np.abs(path.curvature(ss))
        cap = np.minimum(v_des * c.v_hard, np.sqrt(c.lat_acc / np.maximum(k, 1e-4)))
        cap = np.where(ss >= goal_in - 1.0, np.minimum(cap, v_goal_cap), cap)
        # 回溯：v(s) ≤ sqrt(cap(s')² + 2b(s'-s))
        env = backwards(cap, c.env_decel)
        return ss, env

    # ------------------------------------------------------------- 剖面推演
    def rollout(self, ego: EgoState, s0: float, env_s, env_v, v_des):
        c = self.cfg
        C, N = len(self.cands), self.n
        cruise = np.array([cd[0] == 'cruise' for cd in self.cands])
        a1 = np.array([0.0 if cd[0] == 'cruise' else cd[0] for cd in self.cands])
        a2 = np.array([0.0 if cd[0] == 'cruise' else cd[1] for cd in self.cands])
        S = np.zeros((C, N))
        V = np.zeros((C, N))
        A = np.zeros((C, N))
        s = np.full(C, s0)
        v = np.full(C, float(ego.v))
        a = np.full(C, float(ego.a))
        for k in range(N):
            tk = (k + 1) * c.dt
            tgt = np.where(cruise, np.clip(0.8 * (v_des - v), -2.0, 1.6), a1 if tk <= c.t_switch else a2)
            j = np.where(tgt < -3.5, c.jerk_emerg, c.jerk) * c.dt
            a = a + np.clip(tgt - a, -j, j)
            # 包络调速器：超过路线速度上限时必须减速
            vcap = np.interp(s, env_s, env_v)
            a = np.where(v > vcap, np.minimum(a, np.maximum(-4.0, 1.5 * (vcap - v))), a)
            a = np.where(v + a * c.dt < 0.0, -v / c.dt, a)
            s = s + v * c.dt + 0.5 * a * c.dt * c.dt
            v = np.maximum(0.0, v + a * c.dt)
            S[:, k], V[:, k], A[:, k] = s, v, a
        return S, V, A

    # ------------------------------------------------------------- 决策
    def decide(self, ego: EgoState, path: Path, s0: float, e0: float, pred, *,
               v_des: float, v_goal_cap: float, goal_in: float, time_left: float,
               offset_fn=None, commit: bool = True, extra_cost: float = 0.0,
               goal_rect=None) -> Plan:
        c = self.cfg
        env_s, env_v = self.envelope(path, s0, v_des, v_goal_cap, goal_in)
        S, V, A = self.rollout(ego, s0, env_s, env_v, v_des)
        C = len(self.cands)
        Sc = S[:, self.kc]
        tc = self.tc
        reach = S >= goal_in + 0.3
        t_goal = np.where(reach.any(axis=1), self.t[np.argmax(reach, axis=1)], np.inf)
        terminal_time = np.full(C, np.inf)
        if goal_rect is not None:
            full_off = offset_fn(self.t) if offset_fn is not None else e0 * np.exp(-self.t / 1.5)
            gx, gy, _ = path.pose(S, np.broadcast_to(full_off, S.shape))
            inside = goal_contains(goal_rect, gx, gy)
            terminal_time = np.where(inside.any(axis=1), self.t[np.argmax(inside, axis=1)], np.inf)
        # Completion ends the task. Keep one collision-check interval after
        # predicted entry to cover the discrete sampling boundary.
        valid_check = tc[None, :] <= terminal_time[:, None] + c.dt * c.check_every + 1e-9
        off = offset_fn(tc) if offset_fn is not None else e0 * np.exp(-tc / 1.5)
        ex, ey, eh = path.pose(Sc, np.broadcast_to(off, Sc.shape))
        e_circ = _circles(ex, ey, eh, ego.length)
        r_e = _radius(ego.length, ego.width)

        cost = np.zeros(C)
        t_coll = np.full(C, np.inf)
        min_clear = np.full(C, np.inf)
        n_ag = len(pred['ids'])
        circle_overlap_ids, rectangle_overlap_ids = (), ()
        if n_ag:
            P = pred['pos'][:, self.kc]
            H = pred['yaw'][:, self.kc]
            a_circ = _circles(P[..., 0], P[..., 1], H, pred['L'][:, None])
            r_a = _radius(pred['L'], pred['W'])
            now_c = _circles(pred['now'][:, 0], pred['now'][:, 1], pred['yaw_now'], pred['L'])
            ego_now = _circles(np.array([ego.x]), np.array([ego.y]), np.array([ego.yaw]), ego.length)
            d_now = np.linalg.norm(now_c[:, :, None, :] - ego_now[0][None, None, :, :], axis=-1).min(axis=(1, 2))                 - (r_a + r_e)
            # Current overlap is a diagnostic, never a reason to erase future occupancy.
            live = np.ones(n_ag, dtype=bool)
            circle_overlap_ids = tuple(key for key, gap in zip(pred['ids'], d_now) if gap <= 0.)
            now_sat = _obb_separation(ego.x, ego.y, ego.yaw, ego.length, ego.width,
                pred['now'][:, 0], pred['now'][:, 1], pred['yaw_now'], pred['L'], pred['W'])
            rectangle_overlap_ids = tuple(key for key, gap in zip(pred['ids'], now_sat) if gap <= 0.)
            # sqrt is monotone: reduce squared circle distances first, avoiding
            # 25 square roots per candidate/actor/time without changing minima.
            dist = circle_distance(e_circ, a_circ, float(r_e), r_a)
            # Refine elongated bodies whose per-circle longitudinal segment
            # exceeds their width. Five circles otherwise overinflate the side
            # clearance of buses/trucks enough to close an adjacent lane.
            elongated = (pred['L'] > NC * pred['W']) | (ego.length > NC * ego.width)
            if elongated.any():
                sat = _obb_separation(
                    ex[:, None, :], ey[:, None, :], eh[:, None, :], ego.length, ego.width,
                    P[elongated][None, :, :, 0], P[elongated][None, :, :, 1], H[elongated][None],
                    pred['L'][elongated][None, :, None], pred['W'][elongated][None, :, None])
                dist[:, elongated, :] = np.maximum(dist[:, elongated, :], sat)
            dist = np.where(live[None, :, None], dist, np.inf)
            dist = np.where(valid_check[:, None, :], dist, np.inf)
            clear = dist.min(axis=1)
            min_clear = clear.min(axis=1)
            buf = c.buf0 + c.buf_t * tc
            hit = clear < buf[None, :]
            any_hit = hit.any(axis=1)
            first = np.where(any_hit, tc[np.argmax(hit, axis=1)], np.inf)
            t_coll = first
            cost += np.where(any_hit, c.w_coll * (1.0 + (c.horizon - first) / c.horizon), 0.0)
            soft = np.clip(c.clear_soft - clear, 0.0, None)
            cost += c.w_clear * (soft ** 2).sum(axis=1) * (tc[1] - tc[0] if len(tc) > 1 else 0.2)
            cost += self._headway_cost(path, s0, Sc, V[:, self.kc], pred, ego, live,
                                       valid_check=valid_check)

        reach = S >= goal_in + 0.3
        t_goal = np.where(reach.any(axis=1), self.t[np.argmax(reach, axis=1)], np.inf)
        remaining_time = continuation(S[:, -1], V[:, -1], goal_in + .3, env_s, env_v, max(v_des, 1.))
        budget_eta = np.where(np.isfinite(t_goal), t_goal, c.horizon + remaining_time)
        rest = np.maximum(goal_in + .3 - S[:, -1], 0.) / max(v_des, 1.)
        eta = np.where(np.isfinite(t_goal), t_goal, c.horizon + rest)
        cost += eta
        cost += c.w_over * ((np.clip(V - v_des, 0.0, None) ** 2)
                          * (self.t[None, :] <= terminal_time[:, None])).sum(axis=1) * c.dt
        cost += c.w_budget * np.clip(budget_eta - time_left, 0.0, None) if time_left > 0 else 0.0
        n2 = int(round(2.0 / c.dt))
        cost += c.w_acc * (A[:, :n2] ** 2).sum(axis=1) * c.dt
        cost += c.w_jerk * ((np.diff(A[:, :n2], axis=1) / c.dt) ** 2).sum(axis=1) * c.dt
        if self.last is not None:
            cost += c.w_cons * np.abs(A[:, 0] - self.last.a_cmd)

        cost += extra_cost
        # In the final horizon, an actually rolled-out, collision-free arrival
        # before the deadline takes priority over a cheaper late arrival.
        # Outside this horizon ETA is only an extrapolation, so retain ranking.
        on_time = (terminal_time <= time_left + 1e-7) & np.isposinf(t_coll)
        if 0 < time_left <= c.horizon and on_time.any():
            i = self._select_index(np.where(on_time, cost, np.inf), t_coll)
        else:
            i = self._select_index(cost, t_coll)
        all_coll = bool(np.all(np.isfinite(t_coll)))
        cand = self.cands[i]
        label = 'cruise' if cand[0] == 'cruise' else f'{cand[0]:+.1f}/{cand[1]:+.1f}'
        plan = Plan(a_cmd=float(A[i, 0]), label=label, cost=float(cost[i]), t_collide=float(t_coll[i]),
                    min_clear=float(min_clear[i]), s=S[i], v=V[i], a=A[i], all_collide=all_coll,
                    on_time=bool(0 < time_left <= c.horizon and on_time[i]),
                    current_circle_overlap_ids=circle_overlap_ids,
                    current_rectangle_overlap_ids=rectangle_overlap_ids)
        if commit:
            self.commit(plan)
        return plan

    def _select_index(self, cost, collision_times):
        """Selection hook; default retains the frozen finite-penalty policy."""
        return int(np.argmin(cost))

    def commit(self, plan: Plan):
        """多路线变体时由调用方在比较后提交，保证一致性项只记录真正执行的决策。"""
        self.last = plan
        self.stats['frames'] = self.stats.get('frames', 0) + 1
        if plan.all_collide:
            self.stats['all_collide'] = self.stats.get('all_collide', 0) + 1

    def _headway_cost(self, path, s0, Sc, Vc, pred, ego, live, valid_check=None):
        c = self.cfg
        now = pred['now']
        s_a, e_a = path.project_many(now[:, 0], now[:, 1])
        hd = path.heading(s_a)
        vxy = pred['vxy']
        v_along = vxy[:, 0] * np.cos(hd) + vxy[:, 1] * np.sin(hd)
        v_lat = -vxy[:, 0] * np.sin(hd) + vxy[:, 1] * np.cos(hd)
        lead = live & (s_a > s0 + 0.5) & (s_a < s0 + 150.0) & (np.abs(e_a) < 1.8 + pred['W'] / 2) \
            & (v_along > -1.0) & (np.abs(v_lat) < 1.5)
        if not lead.any():
            return 0.0
        tc = self.tc
        sa = s_a[lead][:, None] + np.maximum(v_along[lead], 0.0)[:, None] * tc[None, :]      # (Al,K)
        half = (ego.length + pred['L'][lead]) / 2
        gap = sa[None, :, :] - Sc[:, None, :] - half[None, :, None]                        # (C,Al,K)
        want = c.headway * Vc[:, None, :] + 2.0
        deficit = np.clip(want - gap, 0.0, None)
        if valid_check is not None:
            deficit = np.where(valid_check[:, None, :], deficit, 0.0)
        return c.w_headway * (deficit ** 2).sum(axis=(1, 2)) * (tc[1] - tc[0] if len(tc) > 1 else 0.2)


NC = int(__import__('os').environ.get('ARB_NC', 5))   # 每车圆数：3 圆横向多估 ~0.25m/车，窄车道里并行邻车会被当成擦碰


def _circles(x, y, h, L):
    """矩形 → 沿纵轴 NC 个圆。返回形状 x.shape + (NC,2)。"""
    off = (np.arange(NC) - (NC - 1) / 2) / NC          # 圆心位于各等分段中点
    Lc = np.asarray(L, float)[..., None]
    dx = np.cos(h)[..., None] * Lc * off
    dy = np.sin(h)[..., None] * Lc * off
    return np.stack([x[..., None] + dx, y[..., None] + dy], axis=-1)


def _radius(L, W):
    return np.sqrt((np.asarray(L, float) / (2 * NC)) ** 2 + (np.asarray(W, float) / 2.0) ** 2)


def _obb_separation(ex, ey, eh, el, ew, ax, ay, ah, al, aw):
    """Maximum signed projection gap on both boxes' four axes.

    Inputs broadcast to candidate x actor x time (or scalar pairs). A positive
    value proves separation; a nonpositive value means overlap/touch. Outside
    the boxes it is a lower bound, not the exact Euclidean clearance, so using
    the existing positive safety buffer remains conservative.
    """
    dx, dy = np.asarray(ax) - ex, np.asarray(ay) - ey
    ce, se = np.cos(eh), np.sin(eh)
    ca, sa = np.cos(ah), np.sin(ah)
    parallel = np.abs(ce * ca + se * sa)
    cross = np.abs(ce * sa - se * ca)
    el, ew = np.asarray(el) * 0.5, np.asarray(ew) * 0.5
    al, aw = np.asarray(al) * 0.5, np.asarray(aw) * 0.5
    ego_long = np.abs(dx * ce + dy * se) - el - al * parallel - aw * cross
    ego_lat = np.abs(-dx * se + dy * ce) - ew - al * cross - aw * parallel
    actor_long = np.abs(dx * ca + dy * sa) - al - el * parallel - ew * cross
    actor_lat = np.abs(-dx * sa + dy * ca) - aw - el * cross - ew * parallel
    return np.maximum(np.maximum(ego_long, ego_lat), np.maximum(actor_long, actor_lat))
