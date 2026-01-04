import requests
from typing import Any, Dict

from src.robot_api.interfaces import MotionCommand, MotionState, RobotAPI


class RealRobotAPI(RobotAPI):
    def __init__(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")

    def get_camera_frame(self) -> Any:
        response = requests.get(f"{self.base_url}/camera/frame", timeout=5)
        response.raise_for_status()
        return response.content

    def get_camera_stream(self) -> Any:
        return requests.get(f"{self.base_url}/camera/stream", stream=True, timeout=5)

    def get_camera_config(self) -> Dict[str, Any]:
        response = requests.get(f"{self.base_url}/camera/config", timeout=5)
        response.raise_for_status()
        return response.json()

    def post_motion_drive(self, command: MotionCommand) -> None:
        payload = {"linear": command.linear, "angular": command.angular}
        response = requests.post(f"{self.base_url}/motion/drive", json=payload, timeout=5)
        response.raise_for_status()

    def post_motion_stop(self) -> None:
        response = requests.post(f"{self.base_url}/motion/stop", timeout=5)
        response.raise_for_status()

    def get_motion_state(self) -> MotionState:
        response = requests.get(f"{self.base_url}/motion/state", timeout=5)
        response.raise_for_status()
        data = response.json()
        return MotionState(linear=data.get("linear", 0.0), angular=data.get("angular", 0.0), status=data.get("status", "unknown"))

    def get_capabilities(self) -> Dict[str, Any]:
        response = requests.get(f"{self.base_url}/capabilities", timeout=5)
        response.raise_for_status()
        return response.json()

    def get_health(self) -> Dict[str, Any]:
        response = requests.get(f"{self.base_url}/health", timeout=5)
        response.raise_for_status()
        return response.json()

    def get_events(self) -> Dict[str, Any]:
        response = requests.get(f"{self.base_url}/events", timeout=5)
        response.raise_for_status()
        return response.json()
