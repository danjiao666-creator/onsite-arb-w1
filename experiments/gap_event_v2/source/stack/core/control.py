"""执行层：横向路径跟踪 + 平台执行器限幅。只执行，不决策。"""
import math
from dataclasses import dataclass

import numpy as np

from .geometry import Path, wrap
from .replay_contract import (ACC_LIMIT, STEER_LIMIT, STEER_RATE_LIMIT,
                              WHEELBASE_FACTOR, COMMAND_STEER_RATE_FACTOR)


class LateralTracker:
    """运动学 Frenet 跟踪：航向对准"预瞄点方向 − atan(k·横向误差)"，叠加曲率前馈，
    输出期望横摆角速度，再由平台适配层换算前轮转角。"""

    def __init__(self, k_e=0.9, k_psi=2.2, preview_s=0.25):
        self.k_e, self.k_psi, self.preview_s = k_e, k_psi, preview_s

    def yaw_rate(self, path: Path, s: float, e_lat: float, yaw: float, v: float, offset=0.0, offset_rate=0.0):
        s_ahead = s + max(v, 1.0) * self.preview_s
        h_path = float(path.heading(s_ahead))
        kappa = float(path.curvature(s_ahead))
        err = e_lat - offset
        # 偏移曲线的横向速度折成航向角
        h_off = math.atan2(offset_rate, max(v, 1.0))
        psi_des = h_path + h_off - math.atan(self.k_e * err / (max(v, 0.0) + 2.0))
        return v * kappa + self.k_psi * float(wrap(psi_des - yaw))


@dataclass
class ActuatorLimits:
    acc_min: float = -ACC_LIMIT
    acc_max: float = ACC_LIMIT
    steer_max: float = STEER_LIMIT          # rad（官方 check_action）
    steer_rate: float = STEER_RATE_LIMIT         # rad/s
    wheelbase_factor: float = WHEELBASE_FACTOR     # 官方运动学：yaw_rate = v·1.7·tan(δ)/车长


class ControlAdapter:
    """OnSite 回放台契约：[纵向加速度 m/s², 前轮转角 rad]。换平台 = 换这个类。"""

    def __init__(self, limits: ActuatorLimits = None):
        self.lim = limits or ActuatorLimits()
        self.prev_steer = 0.0

    def reset(self, steer=0.0):
        self.prev_steer = steer

    def command(self, acc, yaw_rate, v, length, dt):
        lim = self.lim
        L = length / lim.wheelbase_factor
        steer = math.atan(yaw_rate * L / v) if v > 0.3 else self.prev_steer
        steer = float(np.clip(steer, -lim.steer_max, lim.steer_max))
        ds = lim.steer_rate * dt * COMMAND_STEER_RATE_FACTOR
        steer = float(np.clip(steer, self.prev_steer - ds, self.prev_steer + ds))
        self.prev_steer = steer
        return [float(np.clip(acc, lim.acc_min, lim.acc_max)), steer]
