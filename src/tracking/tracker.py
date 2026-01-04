from dataclasses import dataclass
from typing import List

from src.vision.detector import Detection


@dataclass
class Track:
    id: int
    detection: Detection


class Tracker:
    def __init__(self) -> None:
        self._next_id = 1

    def update(self, detections: List[Detection]) -> List[Track]:
        tracks: List[Track] = []
        for det in detections:
            tracks.append(Track(id=self._next_id, detection=det))
            self._next_id += 1
        return tracks
