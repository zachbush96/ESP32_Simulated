import cv2
import os
import sys

# Ensure src is in python path
sys.path.append(os.getcwd())

from src.vision.detector import SmolVLMDetector
from src.vision.vlm_client import VLMClient

def test_detector_mock():
    print("Testing SmolVLMDetector in Mock Mode...")

    # Create a dummy image (black square)
    img = cv2.imread("data/cat.jpg")
    if img is None:
        print("Could not load data/cat.jpg, using black image")
        import numpy as np
        img = np.zeros((480, 640, 3), dtype=np.uint8)

    detector = SmolVLMDetector(mock_mode=True)
    detections = detector.detect(img)

    print(f"Detections found: {len(detections)}")
    for d in detections:
        print(f" - Label: {d.label}, Score: {d.score}, BBox: {d.bbox}")

    if len(detections) > 0 and detections[0].label == "cat":
        print("SUCCESS: Mock detection works.")
    else:
        print("FAILURE: Mock detection did not return expected result.")

if __name__ == "__main__":
    test_detector_mock()
