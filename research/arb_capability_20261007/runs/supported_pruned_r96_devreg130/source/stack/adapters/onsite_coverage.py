"""Prediction-coverage factors, isolated from the frozen production entry."""
import numpy as np

from .onsite_replay import ArbiterPlanner
from .onsite_route_capability import RouteCapabilityPlanner
from ..core.arbiter import Arbiter
from ..core.capability_arbiter import FeasibleArbiter, FeasibleCapabilityArbiter
from ..core.coverage_world import CoverageWorldModel
from ..core.current_world import CurrentStateWorldModel


class HypothesisCosts:
    """Multiple collision hypotheses must not multiply physical headway cost."""

    def _headway_cost(self, path, s0, Sc, Vc, pred, ego, live):
        mask = pred.get('headway_mask', np.ones(len(live), dtype=bool))
        return super()._headway_cost(path, s0, Sc, Vc, pred, ego, live & mask)


class CoverageArbiter(HypothesisCosts, Arbiter):
    pass


class CoverageFeasibleArbiter(HypothesisCosts, FeasibleArbiter):
    pass


class CoverageRescueArbiter(HypothesisCosts, FeasibleCapabilityArbiter):
    pass


class CoveragePlanner(ArbiterPlanner):
    def __init__(self):
        super().__init__()
        self.world = CoverageWorldModel()
        self.arb = CoverageArbiter()


class CoverageFeasiblePlanner(CoveragePlanner):
    def __init__(self):
        super().__init__()
        self.arb = CoverageFeasibleArbiter()


class CoverageRescuePlanner(CoveragePlanner):
    def __init__(self):
        super().__init__()
        self.arb = CoverageRescueArbiter()

    def diagnostics(self):
        result = super().diagnostics()
        result['capability'] = dict(self.arb.capability_stats)
        return result


class CoverageRoutePlanner(RouteCapabilityPlanner):
    def __init__(self):
        super().__init__()
        self.world = CoverageWorldModel()
        self.arb = CoverageArbiter()


class CurrentStatePlanner(ArbiterPlanner):
    def __init__(self):
        super().__init__()
        self.world = CurrentStateWorldModel()


class CurrentStateFeasiblePlanner(CurrentStatePlanner):
    def __init__(self):
        super().__init__()
        self.arb = FeasibleArbiter()
