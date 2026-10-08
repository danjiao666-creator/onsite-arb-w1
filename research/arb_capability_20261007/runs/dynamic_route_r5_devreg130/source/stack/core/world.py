"""世界模型：周车跟踪（帧间差分速度向量）+ 短时占用预测。

输出统一的时空占用张量（A 车 × K 步 × 3 圆），仲裁层对所有候选机动做同一种检查——
旧栈把让行/汇入/横穿/后扫四类约束塌成一个 obs_gap 标量，这里它们都只是"某车在某时刻占某处"。

W1（2026-09-26）：加入加速度估计（帧间差分速度的低通滤波）。
  匀速假设 → 刹车中的前车被预测为继续前进 → 间隙不会打开 → 汇入 9 碰撞。
  预测模型：匀加速 + 衰减转向率（速度衰减到 0 后不再倒退）。
"""
import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, List

import numpy as np


@dataclass
class Agent:
    id: str
    kind: str
    x: float
    y: float
    yaw: float
    v: float
    length: float
    width: float


@dataclass
class Track:
    agent: Agent
    vx: float
    vy: float
    yaw_rate: float
    accel: float          # W1: 估计的沿运动方向加速度（m/s²）
    hist: deque            # [(t, x, y)]
    vhist: deque           # W1: [(t, speed)] 用于加速度估计


class WorldModel:
    HIST_S = 0.6          # 速度差分窗口
    STALE_S = 1.0
    ACC_HIST_S = 0.8      # W1: 加速度估计窗口
    ACC_ALPHA = 0.3       # W1: 加速度低通系数（0=不改，1=即时更新）
    ACC_CLAMP = 3.5       # W1: 加速度估计限幅（物理合理范围）

    def __init__(self):
        self.tracks: Dict[str, Track] = {}
        self.t = 0.0

    def update(self, t: float, agents: List[Agent]):
        self.t = t
        seen = set()
        for a in agents:
            seen.add(a.id)
            tr = self.tracks.get(a.id)
            if tr is None:
                h = deque()
                vh = deque()
                tr = Track(a, a.v * math.cos(a.yaw), a.v * math.sin(a.yaw), 0.0, 0.0, h, vh)
                self.tracks[a.id] = tr
            tr.agent = a
            tr.hist.append((t, a.x, a.y))
            while len(tr.hist) > 2 and t - tr.hist[0][0] > self.HIST_S:
                tr.hist.popleft()
            if len(tr.hist) >= 2 and t - tr.hist[0][0] > 1e-3:
                t0, x0, y0 = tr.hist[0]
                vx, vy = (a.x - x0) / (t - t0), (a.y - y0) / (t - t0)
                sp = math.hypot(vx, vy)
                # W1: 加速度 = 速度变化率（沿运动方向投影，低通+限幅）
                tr.vhist.append((t, sp))
                while len(tr.vhist) > 2 and t - tr.vhist[0][0] > self.ACC_HIST_S:
                    tr.vhist.popleft()
                if len(tr.vhist) >= 2 and t - tr.vhist[0][0] > 1e-3:
                    raw_acc = (sp - tr.vhist[0][1]) / (t - tr.vhist[0][0])
                    raw_acc = float(np.clip(raw_acc, -self.ACC_CLAMP, self.ACC_CLAMP))
                    tr.accel = (1 - self.ACC_ALPHA) * tr.accel + self.ACC_ALPHA * raw_acc
                if sp > 1.0 and math.hypot(tr.vx, tr.vy) > 1.0:
                    dh = (math.atan2(vy, vx) - math.atan2(tr.vy, tr.vx) + math.pi) % (2 * math.pi) - math.pi
                    # 转向率：一阶低通，防差分噪声
                    tr.yaw_rate = 0.7 * tr.yaw_rate + 0.3 * float(np.clip(dh / max(t - tr.hist[-2][0], 1e-3) if len(tr.hist) >= 2 else 0.0, -0.8, 0.8))
                tr.vx, tr.vy = vx, vy
            else:
                tr.vx, tr.vy = a.v * math.cos(a.yaw), a.v * math.sin(a.yaw)
        for k in [k for k in self.tracks if k not in seen]:
            del self.tracks[k]

    def predict(self, times: np.ndarray, near_xy=None, radius=150.0):
        """→ dict: ids, pos (A,K,2), yaw (A,K), L (A,), W (A,), v (A,), vxy (A,2)。
        模型：匀加速（速度衰减到 0 后停止）+ 衰减转向率。"""
        trs = list(self.tracks.values())
        if near_xy is not None:
            trs = [tr for tr in trs if math.hypot(tr.agent.x - near_xy[0], tr.agent.y - near_xy[1]) < radius]
        A, K = len(trs), len(times)
        pos = np.zeros((A, K, 2))
        yaw = np.zeros((A, K))
        L = np.zeros(A)
        W = np.zeros(A)
        V = np.zeros(A)
        vxy = np.zeros((A, 2))
        tau = 2.0
        for i, tr in enumerate(trs):
            a = tr.agent
            sp = math.hypot(tr.vx, tr.vy)
            h0 = math.atan2(tr.vy, tr.vx) if sp > 0.5 else a.yaw
            w = tr.yaw_rate if sp > 2.0 else 0.0
            acc = tr.accel
            hs = h0 + w * tau * (1 - np.exp(-times / tau))
            dts = np.diff(np.concatenate([[0.0], times]))
            # W1: 匀加速——速度衰减到 0 后不再倒退
            sp_t = np.maximum(0.0, sp + acc * times)
            pos[i, :, 0] = a.x + np.cumsum(sp_t * np.cos(hs) * dts)
            pos[i, :, 1] = a.y + np.cumsum(sp_t * np.sin(hs) * dts)
            yaw[i] = hs if sp > 0.5 else a.yaw
            L[i], W[i], V[i] = a.length, a.width, sp
            vxy[i] = (tr.vx, tr.vy)
        return {'ids': [tr.agent.id for tr in trs], 'pos': pos, 'yaw': yaw, 'L': L, 'W': W, 'v': V,
                'vxy': vxy, 'now': np.array([[tr.agent.x, tr.agent.y] for tr in trs]).reshape(A, 2),
                'yaw_now': np.array([tr.agent.yaw for tr in trs])}
