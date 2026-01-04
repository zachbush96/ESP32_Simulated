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


class SmolVLMDetector(Detector):
    def __init__(self, mock_mode: bool = False) -> None:
        from src.vision.vlm_client import VLMClient
        import os

        # Force mock mode if requested or if env var is set
        if mock_mode:
             os.environ["VLM_MOCK_MODE"] = "true"

        self.client = VLMClient()
        self.height = 0
        self.width = 0

    def detect(self, frame) -> List[Detection]:
        self.height, self.width = frame.shape[:2]

        # In a real scenario, we might want to downsample or throttle requests here
        # For "snappy" tracking, we heavily rely on the client's ability (or our previous plan of hybrid)
        # But per user request, we use the VLM directly.

        raw_detections = self.client.detect_objects(frame)

        detections = []
        for d in raw_detections:
            # Convert normalized xywh to absolute x1y1x2y2
            norm_x, norm_y, norm_w, norm_h = d["bbox"]
            x1 = int(norm_x * self.width)
            y1 = int(norm_y * self.height)
            w = int(norm_w * self.width)
            h = int(norm_h * self.height)

            detections.append(Detection(
                bbox=(x1, y1, x1 + w, y1 + h),
                label=d["label"],
                score=d["confidence"]
            ))

        return detections
