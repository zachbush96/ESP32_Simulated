from typing import Any, Dict, Optional

from src.core.logging import setup_json_logger
from src.robot_api.interfaces import RobotAPI, MotionCommand
from src.vision.vlm_client import VLMClient


class ExplorationBehavior:
    """
    Handles the "Exploration" behavior using a VLM.
    It sends frames to the VLM and executes the returned commands.
    """

    def __init__(self, robot_api: RobotAPI, vlm_client: Optional[VLMClient] = None) -> None:
        self.robot_api = robot_api
        self.vlm_client = vlm_client or VLMClient()
        self.logger = setup_json_logger("exploration")

        # Rate limiting or state to prevent spamming the VLM
        self.frame_counter = 0
        self.process_every_n_frames = 30  # e.g., once every 2 seconds at 15 FPS

    def update(self, frame: Any) -> None:
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        self.logger.info("requesting_vlm_exploration")
        decision = self.vlm_client.explore(frame)
        self.logger.info(f"vlm_decision: {decision}")

        self._execute_decision(decision)

    def _execute_decision(self, decision: Dict[str, Any]) -> None:
        """
        Parses the decision and calls the robot API.
        Expected format: {"action": "move_forward", "reason": "..."}
        """
        action = decision.get("action", "stop").lower()

        # Simple mapping of actions to motion commands
        # In a real MCP scenario, this would dynamically call tools.
        if action == "move_forward":
            self.robot_api.post_motion_drive(MotionCommand(linear=0.2, angular=0.0))
        elif action == "turn_left":
            self.robot_api.post_motion_drive(MotionCommand(linear=0.0, angular=0.5))
        elif action == "turn_right":
            self.robot_api.post_motion_drive(MotionCommand(linear=0.0, angular=-0.5))
        elif action == "stop":
            self.robot_api.post_motion_stop()
        else:
            self.logger.warning(f"unknown_action: {action}")
            self.robot_api.post_motion_stop()
