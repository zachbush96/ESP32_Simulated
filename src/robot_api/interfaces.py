from dataclasses import dataclass
from typing import Any, Dict, Protocol


@dataclass
class MotionCommand:
    linear: float
    angular: float


@dataclass
class MotionState:
    linear: float
    angular: float
    status: str


class RobotAPI(Protocol):
    def get_camera_frame(self) -> Any:
        ...

    def get_camera_stream(self) -> Any:
        ...

    def get_camera_config(self) -> Dict[str, Any]:
        ...

    def post_motion_drive(self, command: MotionCommand) -> None:
        ...

    def post_motion_stop(self) -> None:
        ...

    def get_motion_state(self) -> MotionState:
        ...

    def get_capabilities(self) -> Dict[str, Any]:
        ...

    def get_health(self) -> Dict[str, Any]:
        ...

    def get_events(self) -> Dict[str, Any]:
        ...
