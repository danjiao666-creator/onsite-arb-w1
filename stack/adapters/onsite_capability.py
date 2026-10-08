"""Isolated capability experiments on the immutable ARB-W1 parent."""
from .onsite_replay import ArbiterPlanner
from ..core.capability_arbiter import CapabilityArbiter, FeasibleArbiter, FeasibleCapabilityArbiter


class CapabilityPlanner(ArbiterPlanner):
    def __init__(self):
        super().__init__()
        self.arb = CapabilityArbiter()

    def diagnostics(self):
        result = super().diagnostics()
        result['capability'] = dict(self.arb.capability_stats)
        return result


class FeasibleCapabilityPlanner(CapabilityPlanner):
    def __init__(self):
        super().__init__()
        self.arb = FeasibleCapabilityArbiter()


class FeasiblePlanner(ArbiterPlanner):
    def __init__(self):
        super().__init__()
        self.arb = FeasibleArbiter()
