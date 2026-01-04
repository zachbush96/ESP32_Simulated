from pathlib import Path
from typing import Iterator, Optional

import cv2

from src.core.logging import setup_json_logger


class CameraSource:
    def __iter__(self) -> Iterator:
        raise NotImplementedError


class VideoCameraSource(CameraSource):
    def __init__(self, path: str, loop: bool = True) -> None:
        self.path = path
        self.loop = loop
        self.logger = setup_json_logger("video_camera")

    def __iter__(self) -> Iterator:
        while True:
            cap = cv2.VideoCapture(str(Path(self.path)))
            if not cap.isOpened():
                self.logger.error(f"Unable to open video source: {self.path}")
                break

            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                yield frame

            cap.release()
            if not self.loop:
                break


class ImageCameraSource(CameraSource):
    def __init__(self, path: str, loop: bool = True) -> None:
        self.path = path
        self.loop = loop
        self.logger = setup_json_logger("image_camera")

    def __iter__(self) -> Iterator:
        while True:
            frame = cv2.imread(str(Path(self.path)))
            if frame is None:
                self.logger.error(f"Unable to open image source: {self.path}")
                break

            yield frame

            if not self.loop:
                break


class WebcamCameraSource(CameraSource):
    def __init__(self, index: int = 0) -> None:
        self.index = index
        self.logger = setup_json_logger("webcam_camera")

    def __iter__(self) -> Iterator:
        cap = cv2.VideoCapture(self.index)
        if not cap.isOpened():
            self.logger.error(f"Unable to open webcam index: {self.index}")
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            yield frame
        cap.release()
