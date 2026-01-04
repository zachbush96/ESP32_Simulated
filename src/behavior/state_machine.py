from dataclasses import dataclass
from typing import List

from src.core.logging import log_state, setup_json_logger
from src.tracking.tracker import Track


IDLE = "idle"
SEARCH = "search"
PURSUE = "pursue"
LOST = "lost"
STOPPED = "stopped"


@dataclass
class BehaviorDecision:
    state: str
    target_id: int | None


class BehaviorStateMachine:
    def __init__(self) -> None:
        self.state = IDLE
        self.logger = setup_json_logger("behavior")

    def update(self, tracks: List[Track]) -> BehaviorDecision:
        if not tracks:
            if self.state != SEARCH:
                self._transition(SEARCH)
            return BehaviorDecision(state=self.state, target_id=None)

        target_track = tracks[0]
        if self.state != PURSUE:
            self._transition(PURSUE)
        return BehaviorDecision(state=self.state, target_id=target_track.id)

    def stop(self) -> BehaviorDecision:
        self._transition(STOPPED)
        return BehaviorDecision(state=self.state, target_id=None)

    def _transition(self, new_state: str) -> None:
        log_state(self.logger, from_state=self.state, to_state=new_state)
        self.state = new_state
