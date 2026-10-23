"""几何基元：弧长参数化折线路径（全向量化）+ 五次横向位移曲线。平台无关，仅依赖 numpy。"""
import math

import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def dedup(points, tol=0.05):
    points = np.asarray(points, dtype=float)
    # Well-separated lane samples keep every point in the greedy reference.
    # Borderline distances retain its scalar norm and last-kept-point logic.
    if points.ndim == 2 and len(points) > 1:
        distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
        guard = 16 * np.finfo(float).eps * max(1.0, abs(tol))
        separated = distances > tol + guard
        if np.all(separated):
            return points.copy()
        uncertain = np.r_[np.flatnonzero(~separated) + 1, len(points)]
        keep = [0]
        i = 1
        while i < len(points):
            if np.linalg.norm(points[i] - points[keep[-1]]) > tol:
                end = int(uncertain[np.searchsorted(uncertain, i + 1)])
                keep.extend(range(i, end))
                i = end
            else:
                i += 1
        return points[keep]
    keep = [0]
    for i in range(1, len(points)):
        if np.linalg.norm(points[i] - points[keep[-1]]) > tol:
            keep.append(i)
    return points[keep]


def resample(points, step=1.0):
    """按等弧长重采样——下游所有 s 查询都走 np.interp，等距点让曲率估计稳定。"""
    p = dedup(points)
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(2, int(math.ceil(cum[-1] / step)) + 1)
    s = np.linspace(0.0, cum[-1], n)
    return np.stack([np.interp(s, cum, p[:, 0]), np.interp(s, cum, p[:, 1])], axis=1)


class Path:
    """弧长参数化路径。s 的所有查询接受标量或数组。"""

    def __init__(self, points, step=1.0):
        self.pts = resample(points, step)
        seg = np.diff(self.pts, axis=0)
        self.cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(seg, axis=1))])
        hd = np.arctan2(seg[:, 1], seg[:, 0])
        self._hd_unwrap = np.unwrap(np.concatenate([hd, hd[-1:]]))
        # 曲率：平滑后的朝向对弧长求导（±3m 窗口）
        k = 7 if len(self._hd_unwrap) > 9 else 1
        sm = np.convolve(np.pad(self._hd_unwrap, (k // 2, k // 2), mode='edge'), np.ones(k) / k, mode='valid')
        self._curv = np.clip(np.gradient(sm, self.cum) if len(self.cum) > 1 else np.zeros(1), -0.5, 0.5)

    @property
    def length(self):
        return float(self.cum[-1])

    def xy(self, s):
        s = np.clip(s, 0.0, self.cum[-1])
        return np.interp(s, self.cum, self.pts[:, 0]), np.interp(s, self.cum, self.pts[:, 1])

    def heading(self, s):
        return wrap(np.interp(np.clip(s, 0.0, self.cum[-1]), self.cum, self._hd_unwrap))

    def curvature(self, s):
        return np.interp(np.clip(s, 0.0, self.cum[-1]), self.cum, self._curv)

    def pose(self, s, offset=0.0):
        """路径上 s 处、左偏 offset 的点与朝向。"""
        x, y = self.xy(s)
        h = self.heading(s)
        return x - np.sin(h) * offset, y + np.cos(h) * offset, h

    def project(self, x, y, s_hint=None, back=15.0, fwd=60.0):
        """→ (s, e_lat)，e_lat>0 在路径左侧。给 s_hint 时只在窗口内找，防跳到平行/交叉段。"""
        lo, hi = 0, len(self.pts) - 1
        if s_hint is not None:
            lo = max(0, int(np.searchsorted(self.cum, s_hint - back)) - 1)
            hi = min(len(self.pts) - 1, int(np.searchsorted(self.cum, s_hint + fwd)) + 1)
            if hi - lo < 1:
                lo, hi = 0, len(self.pts) - 1
        pts = self.pts[lo:hi + 1]
        a, b = pts[:-1], pts[1:]
        ab = b - a
        l2 = np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-9)
        q = np.array([x, y])
        t = np.clip(np.einsum('ij,ij->i', q - a, ab) / l2, 0.0, 1.0)
        proj = a + ab * t[:, None]
        d2 = np.einsum('ij,ij->i', q - proj, q - proj)
        i = int(np.argmin(d2))
        s = float(self.cum[lo + i] + t[i] * math.sqrt(l2[i]))
        h = math.atan2(ab[i, 1], ab[i, 0])
        e = float((q - proj[i]) @ np.array([-math.sin(h), math.cos(h)]))
        return s, e

    def project_many(self, xs, ys, segment_range=None):
        """批量全局投影（每帧对全部周车一次）→ (s[], e[])。"""
        q = np.stack([np.asarray(xs, float), np.asarray(ys, float)], axis=1)
        lo, hi = (0, len(self.pts) - 1) if segment_range is None else segment_range
        if not 0 <= lo < hi <= len(self.pts) - 1:
            raise ValueError("nonempty segment range within the path required")
        a, b = self.pts[lo:hi], self.pts[lo + 1:hi + 1]
        ab = b - a
        l2 = np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-9)
        aq = q[:, None, :] - a[None, :, :]
        t = np.clip(np.einsum('nij,ij->ni', aq, ab) / l2, 0.0, 1.0)
        proj = a[None] + ab[None] * t[..., None]
        d = q[:, None, :] - proj
        d2 = np.einsum('nij,nij->ni', d, d)
        i = np.argmin(d2, axis=1)
        n = np.arange(len(q))
        s = self.cum[lo + i] + t[n, i] * np.sqrt(l2[i])
        h = np.arctan2(ab[i, 1], ab[i, 0])
        e = d[n, i, 0] * -np.sin(h) + d[n, i, 1] * np.cos(h)
        return s, e


class LateralCurve:
    """五次横向位移曲线 offset(t)：起点位置/速度/加速度连续，终点静止。"""

    def __init__(self, start, target, duration, velocity=0.0, acceleration=0.0):
        T = self.duration = float(duration)
        self.target = float(target)
        self.c = np.zeros(6)
        self.c[:3] = [start, velocity, acceleration / 2]
        self.c[3:] = np.linalg.solve(
            [[T ** 3, T ** 4, T ** 5], [3 * T ** 2, 4 * T ** 3, 5 * T ** 4], [6 * T, 12 * T ** 2, 20 * T ** 3]],
            [target - start - velocity * T - acceleration * T * T / 2, -velocity - acceleration * T, -acceleration])

    def at(self, t):
        t = np.clip(t, 0.0, self.duration)
        c = self.c
        return c[0] + c[1] * t + c[2] * t ** 2 + c[3] * t ** 3 + c[4] * t ** 4 + c[5] * t ** 5

    def rate(self, t):
        t = np.clip(t, 0.0, self.duration)
        c = self.c
        return c[1] + 2 * c[2] * t + 3 * c[3] * t ** 2 + 4 * c[4] * t ** 3 + 5 * c[5] * t ** 4

    def accel(self, t):
        t = np.clip(t, 0.0, self.duration)
        c = self.c
        return 2 * c[2] + 6 * c[3] * t + 12 * c[4] * t ** 2 + 20 * c[5] * t ** 3
