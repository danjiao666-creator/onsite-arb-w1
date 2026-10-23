"""Behavior-preserving instrumentation of the route-seam baseline."""
from copy import deepcopy
from .onsite_close_guard import CloseGuardPlanner
from ..core.decision_record import ExecutionDecision, feedback, next_state


class ObservedPlanner(CloseGuardPlanner):
    def init(self, scenario_info):
        super().init(scenario_info)
        self.execution_decision = None
        self.execution_feedback = {'status': 'initial'}
        self.nominal_record = None

    def act(self, observation):
        self.execution_feedback = feedback(self.execution_decision, observation.ego_info)
        return super().act(observation)

    def _execute(self, plan, path, station, error, ego, dt):
        self.nominal_record = dict(label=plan.label, action=plan.a_cmd,
            station=float(station), lateral_error=float(error), a=plan.a.tolist(),
            s=plan.s.tolist(), v=plan.v.tolist(), controller_memory=self.ctrl.prev_steer)
        action = super()._execute(plan, path, station, error, ego, dt)
        guard = self._guard_status
        checked = guard['expected_checked_action']
        self.execution_decision = ExecutionDecision(float(self.world.t),
            'legacy:' + str(guard['chosen_index']),
            'recovery' if guard['status'] == 'override' else 'main', list(action),
            list(checked), next_state(ego, checked, dt), guard['returned_action_status'],
            str(self.route.lane_seq), plan.a_cmd, source='observed_legacy')
        return action

    def safety_diagnostics(self):
        result = super().safety_diagnostics()
        result.update(execution_decision=self.execution_decision.snapshot() if self.execution_decision else None,
                      execution_feedback=deepcopy(self.execution_feedback),
                      nominal_plan=deepcopy(self.nominal_record))
        return result
