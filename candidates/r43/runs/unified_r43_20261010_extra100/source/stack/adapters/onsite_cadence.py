"""Match profile generation to replay cadence, preserving first-step increment."""
from .onsite_dynamic_route import DynamicRouteCapabilityPlanner
from ..core.arbiter import EgoState
from ..core.native_profile import NativeArbiter, NativeFeasibleArbiter
from ..core.native_distance import circle_distance
from ..core.native_envelope import backwards
from ..core.tail_eta import continuation
import numpy as np


class CadencePlanner(DynamicRouteCapabilityPlanner):
    def init(self, scenario_info):
        super().init(scenario_info)
        backwards(np.ones(2), 2.)
        continuation(np.zeros(1), np.ones(1), 2., np.arange(2, dtype=float), np.ones(2), 1.)
        sample = np.zeros((2, 3))
        continuation(sample[:, -1], sample[:, -1], 2., np.arange(2, dtype=float), np.ones(2), 1.)
        previous = self.arb
        cfg = previous.cfg
        ratio = cfg.dt / self.dt
        cfg.jerk *= ratio
        cfg.jerk_emerg *= ratio
        cfg.dt = self.dt
        cfg.check_every = max(1, int(round(.2 / self.dt)))
        self.arb = (NativeFeasibleArbiter if self._dynamic_active else NativeArbiter)(cfg)
        self.arb.last, self.arb.stats = previous.last, previous.stats
        self.arb.rollout(EgoState(0., 0., 0., 1., 0., 4.5, 1.8), 0.,
                         np.arange(2, dtype=float), np.ones(2), 1.)
        circle_distance(np.zeros((1, 1, 5, 2)), np.zeros((1, 1, 5, 2)), 1., np.ones(1))
        circle_distance(np.zeros((1, 1, 5, 2), order='C'),
                        np.zeros((1, 1, 5, 2), order='F'), 1., np.ones(1))
        circle_distance(np.zeros((2, 2, 5, 2)),
                        np.zeros((2, 4, 5, 2))[:, ::2], 1., np.ones(2))
