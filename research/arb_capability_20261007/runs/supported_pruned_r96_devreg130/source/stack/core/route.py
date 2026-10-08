"""任务建模：起点 → 目标盒的车道级路线。

路线是本栈里唯一的"任务"表示（旧 UC 栈没有它：路口转向意图只存在于场景名里，
10 题参考线反向、10 题走错分支——R1 根因）。这里：
  - 车道图 Dijkstra（拓扑边 + 平行邻道横移边），几何门验收每条边；
  - 横移边展开为平滑过渡段（旧实现是 3 点硬跳变）；
  - 装载期校验（起点贴线、朝向一致、目标盒在线上且在前方），不合格记入 issues，
    由适配层决定报错还是降级——不再静默失败。
"""
import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .geometry import Path, dedup, wrap


@dataclass
class Lane:
    id: object
    center: np.ndarray
    successors: Sequence = ()
    predecessors: Sequence = ()


@dataclass
class Route:
    path: Path
    s0: float                 # 起点在路径上的弧长
    goal_s: float             # 盒心投影
    goal_in: float            # 路径进入目标盒的弧长
    goal_out: float           # 路径离开目标盒的弧长
    lane_seq: List[Tuple[object, int]] = field(default_factory=list)
    issues: List[str] = field(default_factory=list)
    method: str = 'graph'
    frac: Dict[int, float] = field(default_factory=dict)          # 各横移边的过渡起点（重叠区比例）
    spans: Dict[int, Tuple[float, float]] = field(default_factory=dict)  # 各横移边过渡段 [s_a, s_b]

    @property
    def ok(self):
        return not self.issues


class RouteError(RuntimeError):
    pass


def _head(p):
    d = p[-1] - p[max(0, len(p) - 4)]
    return math.atan2(d[1], d[0])


def _head0(p):
    d = p[min(len(p) - 1, 3)] - p[0]
    return math.atan2(d[1], d[0])


def _dpoly(x, y, p):
    from .native_route import nearest_distances
    return float(nearest_distances(np.array([[x, y]], dtype=float), p)[0])


def _plen(p):
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def _smoothstep5(u):
    u = np.clip(u, 0.0, 1.0)
    return u ** 3 * (10 - 15 * u + 6 * u * u)


class RoutePlanner:
    LAT_MIN, LAT_MAX = 1.5, 5.5        # 平行邻道中心距（m）
    TRANSITION_S = 4.0                 # 横移过渡段时长（按参考车速换算长度）
    DEFAULT_FRAC = 0.1

    def lateral_edges(self, seq):
        return [i for i, (a, b) in enumerate(zip(seq, seq[1:])) if self.is_lateral(a, b)]

    def __init__(self, lanes: Dict[object, Lane], v_ref: float = 15.0):
        self.v_ref = v_ref
        self.raw = {k: np.asarray(l.center, float) for k, l in lanes.items() if len(l.center) >= 2}
        self.lanes = {k: l for k, l in lanes.items() if k in self.raw}
        self._index()
        self._locked_prefix_cache = {}
        self._path_cache = {}
        self._poly_cache = {}
        self._prepare_cache = {}

    def _path(self, points, step=1.0):
        points = np.asarray(points, dtype=float)
        key = (float(step), points.shape, points.tobytes())
        cached = self._path_cache.get(key)
        if cached is None:
            cached = Path(points, step=step)
            if len(self._path_cache) >= 512:
                self._path_cache.pop(next(iter(self._path_cache)))
            self._path_cache[key] = cached
        return cached

    def _poly(self, states):
        key = tuple(states)
        cached = self._poly_cache.get(key)
        if cached is None:
            cached = dedup(np.vstack([self.orient(*state) for state in states]))
            self._poly_cache[key] = cached
        return cached

    def _index(self):
        raw = self.raw
        self.heads = {k: _head(p) for k, p in raw.items()}
        mids = {k: (p[0] + p[-1]) / 2 for k, p in raw.items()}
        ends = {k: np.vstack([p[0], p[-1]]) for k, p in raw.items()}
        self.lat_pairs = {k: set() for k in raw}
        self.near_pairs = {k: set() for k in raw}
        keys = list(raw)
        for i, a in enumerate(keys):
            for b in keys[i + 1:]:
                if np.linalg.norm(mids[a] - mids[b]) > 60.0 + 0.5 * (_plen(raw[a]) + _plen(raw[b])):
                    continue
                if np.min(np.linalg.norm(ends[a][:, None] - ends[b][None], axis=2)) < 15.0:
                    self.near_pairs[a].add(b)
                    self.near_pairs[b].add(a)
                dh = abs(wrap(self.heads[a] - self.heads[b]))
                if min(dh, abs(dh - math.pi)) > math.radians(30):
                    continue
                pa = raw[a]
                near = [_dpoly(*pa[j], raw[b]) for j in np.linspace(0, len(pa) - 1, 5).astype(int)]
                if sum(self.LAT_MIN < d < self.LAT_MAX for d in near) >= 3:
                    self.lat_pairs[a].add(b)
                    self.lat_pairs[b].add(a)

    def orient(self, k, rev):
        return self.raw[k][::-1] if rev else self.raw[k]

    # ------------------------------------------------------------------ 搜索
    def plan(self, start, heading, goal_rect) -> Route:
        tx = (goal_rect[0][0] + goal_rect[1][0]) / 2
        ty = (goal_rect[0][1] + goal_rect[1][1]) / 2
        cands = []
        for k in self.raw:
            for rev in (0, 1):
                q = self.orient(k, rev)
                d = _dpoly(start[0], start[1], q)
                if d < 8.0:
                    # 起点处的局部朝向（而非车道末端朝向）才能区分弯道上的同向/对向
                    i = int(np.argmin(np.linalg.norm(q - np.asarray(start), axis=1)))
                    j = min(i + 3, len(q) - 1)
                    i = max(0, j - 3)
                    hloc = math.atan2(q[j][1] - q[i][1], q[j][0] - q[i][0])
                    if heading is None or abs(wrap(hloc - heading)) < math.radians(60):
                        cands.append((d, (k, rev)))
        if not cands:
            raise RouteError('起点车道无候选')
        # 起点必须是自车所在车道（离中心线最近者 ±0.8m 内的并列候选），
        # 不能是 8m 内任意一条——否则路线从邻道出发，起步即横切撞车
        d_min = min(d for d, _ in cands)
        starts = [st for d, st in cands if d <= d_min + 0.8]
        goal_k = {k for k in self.raw if _dpoly(tx, ty, self.raw[k]) < 3.0}
        if not goal_k:
            raise RouteError('无车道经过目标盒')

        dist = {s: 0.0 for s in starts}
        prev = {}
        pq = [(0.0, s) for s in starts]
        heapq.heapify(pq)
        goal_state = None
        while pq:
            d, st = heapq.heappop(pq)
            if d > dist.get(st, 1e18) + 1e-6:
                continue
            if st[0] in goal_k:
                goal_state = st
                break
            for st2, w in self._edges(*st):
                nd = d + w
                if nd < dist.get(st2, 1e18):
                    dist[st2] = nd
                    prev[st2] = st
                    heapq.heappush(pq, (nd, st2))
        if goal_state is None:
            raise RouteError('起点→目标车道无连通路径')
        seq = [goal_state]
        while seq[-1] in prev:
            seq.append(prev[seq[-1]])
        seq.reverse()
        return self.variant(seq, start, heading, goal_rect, {})

    def variant(self, seq, start, heading, goal_rect, frac: Dict[int, object], anchor=None) -> Route:
        """同一车道序列、不同换道时机的路线。仲裁层在这些变体间选择 = 选"何时换道"。"""
        tx = (goal_rect[0][0] + goal_rect[1][0]) / 2
        ty = (goal_rect[0][1] + goal_rect[1][1]) / 2
        poly, spans_xy = self.assemble(seq, frac, anchor=anchor if anchor is not None else start, goal_xy=(tx, ty))
        r = self.finish(poly, start, heading, goal_rect, seq, 'graph')
        r.frac = dict(frac)
        r.span_xy = spans_xy
        for i, (pa, pb) in spans_xy.items():
            sa, _ = r.path.project(*pa)
            sb, _ = r.path.project(*pb, s_hint=sa, back=5.0, fwd=400.0)
            r.spans[i] = (sa, sb)
        return r

    def _topo_edges(self, k, rev):
        q = self.orient(k, rev)
        h0, tail = _head(q), q[-1]
        out = []
        cand = (set(self.lanes[k].successors or ()) | set(self.lanes[k].predecessors or ())
                | self.near_pairs[k])
        nrm = np.array([-math.sin(h0), math.cos(h0)])
        for sid in cand:
            if sid not in self.raw or sid == k:
                continue
            for rev2 in (0, 1):
                q2 = self.orient(sid, rev2)
                gap = q2[0] - tail
                # 首尾相接：端点近且横向错位小。旧门限 12m 会把相邻车道当成延续，
                # 路线几米内横跳一个车道以上（highway_merge_3_x 出界的根因）
                if (np.linalg.norm(gap) < 4.0 and abs(float(gap @ nrm)) < 1.5
                        and abs(wrap(_head0(q2) - h0)) < math.radians(135)):
                    out.append((sid, rev2))
        return out

    def _edges(self, k, rev):
        q = self.orient(k, rev)
        h0 = _head(q)
        out = [(st, _plen(self.orient(*st))) for st in self._topo_edges(k, rev)]
        for j in self.lat_pairs[k]:
            for rev2 in (0, 1):
                q2 = self.orient(j, rev2)
                if abs(wrap(_head(q2) - h0)) < math.radians(25):
                    out.append(((j, rev2), 12.0 + _plen(q2) * 0.1))
        return out

    def _predecessors(self, st):
        if not hasattr(self, '_pred'):
            self._pred = {}
            for k in self.raw:
                for rev in (0, 1):
                    for st2 in self._topo_edges(k, rev):
                        self._pred.setdefault(st2, []).append((k, rev))
        return self._pred.get(st, [])

    # ------------------------------------------------------------------ 组装
    def is_lateral(self, a, b):
        (k, rev), (k2, rev2) = a, b
        cur, nxt = self.orient(k, rev), self.orient(k2, rev2)
        h0 = _head(cur)
        gap = nxt[0] - cur[-1]
        topo = (np.linalg.norm(gap) < 4.0
                and abs(float(gap @ np.array([-math.sin(h0), math.cos(h0)]))) < 1.5
                and abs(wrap(_head0(nxt) - h0)) < math.radians(135))
        return (k2 in self.lat_pairs[k]) and not topo

    def assemble(self, seq, frac: Optional[Dict[int, object]] = None, anchor=None, goal_xy=None):
        """把车道序列拼成折线 → (折线, {横移边: (过渡起点xy, 终点xy)})。

        序列按横移边切成"拓扑连续段"。每条横移边：源段沿后继向前、目标段沿前驱向后
        补全与对方并行的车道（可跨多个 lane section），在整段并行区内布置过渡：
          起点 ∈ [max(并行区起点, 锚点), 并行区终点 − 最短过渡]，frac[i] 为该区间内比例；
          frac[i] 也可以是锁定后的绝对起点 xy（锁定的换道不随锚点移动）；
          长度 = min(参考车速 × 4s, 剩余并行长度)。
        旧实现只在单个 lane section 内找重叠，高速汇入常只剩 9m——35m/s 下 9m 换一个车道。"""
        frac = frac or {}
        runs, lat_idx = [[seq[0]]], []
        for i, (a, b) in enumerate(zip(seq, seq[1:])):
            if self.is_lateral(a, b):
                runs.append([b])
                lat_idx.append(i)
            else:
                runs[-1].append(b)
        poly_of = self._poly
        spans = {}
        out = []
        cur_states = list(runs[0])
        cur = poly_of(cur_states)
        v = max(self.v_ref, 5.0)
        l_full, l_min = max(25.0, v * self.TRANSITION_S), max(15.0, v * 2.0)
        for r, i in enumerate(lat_idx):
            locked = [frac.get(edge) for edge in lat_idx[:r + 1]]
            prefix_key = None
            if all(isinstance(value, (tuple, list, np.ndarray)) for value in locked):
                prefix_key = (tuple(seq), tuple(tuple(value) for value in locked),
                              None if goal_xy is None else tuple(goal_xy))
                cached = self._locked_prefix_cache.get(prefix_key)
                if cached is not None:
                    cached_out, cur, cached_states, cached_spans = cached
                    out = list(cached_out)
                    cur_states = list(cached_states)
                    spans = dict(cached_spans)
                    continue
            earlier_locked = [frac.get(edge) for edge in lat_idx[:r]]
            prepare_key = None
            if all(isinstance(value, (tuple, list, np.ndarray)) for value in earlier_locked):
                prepare_key = (tuple(seq), r, tuple(tuple(value) for value in earlier_locked),
                               None if goal_xy is None else tuple(goal_xy))
            prepared = self._prepare_cache.get(prepare_key) if prepare_key is not None else None
            if prepared is None:
                prepared = self._prepare_lateral(cur, cur_states, runs, lat_idx, r, goal_xy, l_min)
                if prepare_key is not None:
                    self._prepare_cache[prepare_key] = prepared
            cur, cached_cur_states, nxt, nxt_states, cum_cur, w0, w1, src, valid = prepared
            cur_states = list(cached_cur_states)
            if not valid:
                out.append(cur)
                cur_states = list(runs[r + 1])
                cur = poly_of(cur_states)
                continue
            f = frac.get(i, self.DEFAULT_FRAC)
            if isinstance(f, (tuple, list, np.ndarray)):
                s_a = src.project(*f)[0]
            else:
                lo = w0
                if anchor is not None:
                    lo = max(lo, src.project(*anchor)[0] + 3.0)
                hi = max(lo, w1 - l_min)
                s_a = lo + float(f) * (hi - lo)
            s_a = min(s_a, max(w0, w1 - 8.0))
            ltr = max(8.0, min(l_full, w1 - s_a))
            pa = np.array(src.xy(s_a))
            j0 = int(np.argmin(np.linalg.norm(nxt - pa, axis=1)))
            if len(nxt) - j0 < 2:
                out.append(cur)
                cur, cur_states = nxt, nxt_states
                continue
            dst = self._path(nxt[j0:], step=0.5)
            ltr = min(ltr, dst.length)
            u = np.arange(0.0, ltr + 1e-6, 1.0)
            w = _smoothstep5(u / ltr)[:, None]
            blend = (1 - w) * np.stack(src.xy(s_a + u), axis=1) + w * np.stack(dst.xy(u), axis=1)
            i_cut = int(np.searchsorted(cum_cur, s_a))
            out.append(cur[:max(i_cut, 1)])
            out.append(blend)
            spans[i] = (blend[0].copy(), blend[-1].copy())
            rest = np.stack(dst.xy(np.arange(ltr + 1.0, dst.length, 1.0)), axis=1) if dst.length > ltr + 1 else np.zeros((0, 2))
            cur = rest if len(rest) >= 2 else nxt[-2:]
            cur_states = nxt_states
            if prefix_key is not None:
                self._locked_prefix_cache[prefix_key] = (tuple(out), cur, tuple(cur_states), dict(spans))
        out.append(cur)
        return dedup(np.vstack([p for p in out if len(p)])), spans

    def _prepare_lateral(self, cur, cur_states, runs, lat_idx, r, goal_xy, l_min):
        from .native_route import nearest_distances
        last_run = r + 1 == len(runs) - 1
        target_states = runs[r + 1] if last_run else self._corridor(runs[r + 1])
        target_base = self._poly(target_states)
        extended = self._extend(cur_states, target_base, forward=True)
        forward = extended[len(cur_states):]
        if forward:
            cur = dedup(np.vstack([cur, self._poly(forward)]))
        nxt_states = self._extend(target_states, cur, forward=False)
        nxt = self._poly(nxt_states)
        distances = nearest_distances(cur, nxt)
        ok = (distances < 6.) & (distances > .5)
        cumulative = np.concatenate([[0.], np.cumsum(np.linalg.norm(np.diff(cur, axis=0), axis=1))])
        if not ok.any():
            return cur, tuple(extended), nxt, tuple(nxt_states), cumulative, 0., 0., None, False
        block, start = (0, 0), None
        for index, flag in enumerate(np.append(ok, False)):
            if flag and start is None:
                start = index
            elif not flag and start is not None:
                if index - start > block[1] - block[0]:
                    block = (start, index)
                start = None
        w0, w1 = float(cumulative[block[0]]), float(cumulative[block[1] - 1])
        src = self._path(cur, step=.5)
        if goal_xy is not None:
            goal_s, _ = src.project(*goal_xy)
            residual = np.linalg.norm(np.asarray(src.xy(goal_s)) - goal_xy)
            if residual < 8. and goal_s > w0:
                w1 = min(w1, goal_s - 5. - (len(lat_idx) - 1 - r) * l_min)
        return cur, tuple(extended), nxt, tuple(nxt_states), cumulative, w0, w1, src, True

    def _corridor(self, run, max_len=400.0):
        """沿拓扑后继（取朝向最顺者）把车道段延长成走廊。"""
        run = list(run)
        added = 0.0
        while added < max_len:
            h = _head(self.orient(*run[-1]))
            nb = [p for p in self._topo_edges(*run[-1]) if p not in run]
            if not nb:
                break
            p = min(nb, key=lambda st: abs(wrap(_head0(self.orient(*st)) - h)))
            run.append(p)
            added += _plen(self.orient(*p))
        return run

    def _extend(self, run, other_poly, forward, max_len=400.0):
        """沿拓扑后继（forward）或前驱补上与 other_poly 并行的车道。"""
        run = list(run)
        added = 0.0
        while added < max_len:
            nbrs = self._topo_edges(*run[-1]) if forward else self._predecessors(run[0])
            best = None
            for p in nbrs:
                if p in run:
                    continue
                q = self.orient(*p)
                mid = q[len(q) // 2]
                d = _dpoly(mid[0], mid[1], other_poly)
                if self.LAT_MIN < d < 6.0 and (best is None or d < best[0]):
                    best = (d, p)
            if best is None:
                break
            if forward:
                run.append(best[1])
            else:
                run.insert(0, best[1])
            added += _plen(self.orient(*best[1]))
        return run

    # ------------------------------------------------------------------ 裁剪 + 校验
    def finish(self, poly, start, heading, goal_rect, seq, method) -> Route:
        tx = (goal_rect[0][0] + goal_rect[1][0]) / 2
        ty = (goal_rect[0][1] + goal_rect[1][1]) / 2
        full = self._path(poly, step=1.0)
        s_start, _ = full.project(*start)
        s_goal, e_goal = full.project(tx, ty, s_hint=None)
        # 盒心可能被路径多次经过（环形/掉头）：取起点之后最近的一次
        if s_goal <= s_start:
            s_goal, e_goal = full.project(tx, ty, s_hint=s_start + 1e3, back=1e3 - 1.0, fwd=1e5)
        lo = max(0.0, s_start - 10.0)
        hi = min(full.length, max(s_goal, s_start) + 60.0)
        pts = np.stack(full.xy(np.arange(lo, hi, 1.0)), axis=1)
        if len(pts) < 2 or hi - lo < 2.0:
            pts = full.pts
        path = _extend(Path(pts), 60.0)
        s0, e0 = path.project(*start)
        gs, ge = path.project(tx, ty, s_hint=s0 + max(0.0, s_goal - s_start), back=30.0, fwd=30.0)
        issues = []
        if abs(e0) > 3.0:
            issues.append(f'起点离路线 {abs(e0):.1f}m')
        if heading is not None and abs(wrap(float(path.heading(s0)) - heading)) > math.radians(60):
            issues.append(f'路线起始方向与自车差 {math.degrees(abs(wrap(float(path.heading(s0)) - heading))):.0f}°')
        if abs(ge) > 2.5:
            issues.append(f'目标盒心离路线 {abs(ge):.1f}m')
        if gs <= s0 + 0.5:
            issues.append('目标盒不在起点前方')
        g_in, g_out = _box_span(path, goal_rect, gs)
        if g_in is None:
            issues.append('路线不穿过目标盒')
            g_in, g_out = gs - 0.5, gs + 0.5
        return Route(path=path, s0=s0, goal_s=gs, goal_in=g_in, goal_out=g_out,
                     lane_seq=list(seq), issues=issues, method=method)


def _extend(path: Path, runout):
    """路线末端直线延伸：盒后仍需有路可投影（旧栈 R1d"幽灵前车"坑）。"""
    h = float(path.heading(path.length))
    end = path.pts[-1]
    ext = [end + np.array([math.cos(h), math.sin(h)]) * d for d in np.arange(1.0, runout + 1.0, 1.0)]
    return Path(np.vstack([path.pts, np.array(ext)]))


def _box_span(path: Path, rect, s_c):
    s = np.arange(max(0.0, s_c - 15.0), min(path.length, s_c + 15.0), 0.05)
    x, y = path.xy(s)
    (x1, y1), (x2, y2) = rect
    inside = (x >= min(x1, x2)) & (x <= max(x1, x2)) & (y >= min(y1, y2)) & (y <= max(y1, y2))
    if not inside.any():
        return None, None
    return float(s[inside].min()), float(s[inside].max())


def chain_fallback(lanes: Dict[object, Lane], start, goal_rect):
    """图搜索失败时的降级：从最近车道沿 successor 贪心接向目标（旧栈原逻辑）。"""
    tx = (goal_rect[0][0] + goal_rect[1][0]) / 2
    ty = (goal_rect[0][1] + goal_rect[1][1]) / 2
    raw = {k: np.asarray(l.center, float) for k, l in lanes.items() if len(l.center) >= 2}
    k0 = min(raw, key=lambda k: _dpoly(start[0], start[1], raw[k]))
    chain, seen = [k0], {k0}
    while len(chain) < 50:
        best, bd = None, float('inf')
        for sid in lanes[chain[-1]].successors or ():
            if sid in seen or sid not in raw:
                continue
            d = float(np.hypot(*(raw[sid][-1] - np.array([tx, ty]))))
            if d < bd:
                best, bd = sid, d
        if best is None:
            break
        chain.append(best)
        seen.add(best)
        if bd < 15.0:
            break
    line = np.vstack([raw[k] for k in chain])
    if np.linalg.norm(line[0] - np.asarray(start)) > np.linalg.norm(line[-1] - np.asarray(start)):
        line = line[::-1]
    return dedup(line), [(k, 0) for k in chain]
