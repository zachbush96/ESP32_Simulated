import time
from typing import Any, Dict, Optional

from src.core.logging import setup_json_logger
from src.robot_api.interfaces import MotionCommand, MotionState, RobotAPI


class SimulatedRobotAPI(RobotAPI):
    def __init__(self) -> None:
        self.logger = setup_json_logger("sim_robot_api")
        self._last_command: Optional[MotionCommand] = None
        self._last_state = MotionState(linear=0.0, angular=0.0, status="idle")

    def get_camera_frame(self) -> Any:
        return None

    def get_camera_stream(self) -> Any:
        return None

    def get_camera_config(self) -> Dict[str, Any]:
        return {"resolution": "640x480", "fps": 15}

    def post_motion_drive(self, command: MotionCommand) -> None:
        self._last_command = command
        self._last_state = MotionState(
            linear=command.linear,
            angular=command.angular,
            status="driving",
        )
        self.logger.info(
            "drive_command", extra={"linear": command.linear, "angular": command.angular}
        )

    def post_motion_stop(self) -> None:
        self._last_state = MotionState(linear=0.0, angular=0.0, status="stopped")
        self.logger.info("stop_command")

    def get_motion_state(self) -> MotionState:
        return self._last_state

    def get_capabilities(self) -> Dict[str, Any]:
        return {"camera": True, "motion": True, "simulated": True}

    def get_health(self) -> Dict[str, Any]:
        return {"status": "ok", "timestamp": time.time()}

    def get_events(self) -> Dict[str, Any]:
        return {"events": []}
