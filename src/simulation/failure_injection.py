import random
from typing import List

from src.vision.detector import Detection


def maybe_drop_frame(chance: float) -> bool:
    return random.random() < chance


def maybe_remove_detections(detections: List[Detection], chance: float) -> List[Detection]:
    if random.random() < chance:
        return []
    return detections


def maybe_lose_target(detections: List[Detection], chance: float) -> List[Detection]:
    if not detections:
        return detections
    if random.random() < chance:
        return detections[1:]
    return detections
