from dataclasses import dataclass

from src.core.logging import setup_json_logger
from src.robot_api.interfaces import MotionCommand


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


@dataclass
class MotionCommandRequest:
    linear: float
    angular: float
    stop: bool = False


class MotionPlanner:
    def __init__(self, linear_limit: float, angular_limit: float) -> None:
        self.linear_limit = linear_limit
        self.angular_limit = angular_limit
        self.logger = setup_json_logger("motion_planner")

    def plan(self, request: MotionCommandRequest) -> MotionCommand:
        if request.stop:
            self.logger.info("stop_command_requested")
            return MotionCommand(linear=0.0, angular=0.0)

        linear = clamp(request.linear, -self.linear_limit, self.linear_limit)
        angular = clamp(request.angular, -self.angular_limit, self.angular_limit)
        self.logger.info(
            "motion_command", extra={"requested_linear": request.linear, "requested_angular": request.angular, "linear": linear, "angular": angular}
        )
        return MotionCommand(linear=linear, angular=angular)
