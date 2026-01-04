from src.behavior.state_machine import BehaviorDecision, PURSUE, SEARCH, STOPPED
from src.control.motion_planner import MotionCommandRequest


class BehaviorPolicies:
    def map_decision_to_motion(self, decision: BehaviorDecision) -> MotionCommandRequest:
        if decision.state == PURSUE:
            return MotionCommandRequest(linear=0.2, angular=0.0)
        if decision.state == SEARCH:
            return MotionCommandRequest(linear=0.0, angular=0.3)
        if decision.state == STOPPED:
            return MotionCommandRequest(linear=0.0, angular=0.0, stop=True)
        return MotionCommandRequest(linear=0.0, angular=0.0)
