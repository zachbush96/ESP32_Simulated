from src.core.logging import setup_json_logger
from src.robot_api.interfaces import RobotAPI


class SafetyController:
    def __init__(self, robot_api: RobotAPI) -> None:
        self.robot_api = robot_api
        self.logger = setup_json_logger("safety")

    def stop_robot(self) -> None:
        self.logger.info("safety_stop")
        self.robot_api.post_motion_stop()
