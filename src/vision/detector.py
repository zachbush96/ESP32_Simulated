from dataclasses import dataclass
from typing import List


@dataclass
class Detection:
    bbox: tuple  # (x1, y1, x2, y2)
    label: str
    score: float


class Detector:
    def detect(self, frame) -> List[Detection]:
        raise NotImplementedError


class StubDetector(Detector):
    def detect(self, frame) -> List[Detection]:
        # Early prototype returns no detections; hook for later models
        return []
