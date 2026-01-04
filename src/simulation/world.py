import time
from typing import Iterable

import cv2

from src.behavior.policies import BehaviorPolicies
from src.behavior.state_machine import BehaviorStateMachine
from src.control.motion_planner import MotionPlanner
from src.control.safety import SafetyController
from src.core.config_loader import load_config
from src.core.logging import setup_json_logger
from src.robot_api.router import get_robot_api
from src.simulation.failure_injection import (
    maybe_drop_frame,
    maybe_lose_target,
    maybe_remove_detections,
)
from src.tracking.tracker import Tracker
from src.vision.camera import CameraSource, VideoCameraSource, WebcamCameraSource
from src.vision.detector import Detector, StubDetector
from src.vision.visualizer import overlay_detections


class SimulationWorld:
    def __init__(self, camera: CameraSource, detector: Detector, config: dict) -> None:
        self.camera = camera
        self.detector = detector
        self.config = config
        self.logger = setup_json_logger("simulation")
        self.tracker = Tracker()
        self.behavior = BehaviorStateMachine()
        self.policies = BehaviorPolicies()
        control_limits = config.get("settings", {}).get("control", {})
        self.motion_planner = MotionPlanner(
            linear_limit=control_limits.get("linear_limit", 0.3),
            angular_limit=control_limits.get("angular_limit", 1.0),
        )
        self.robot_api = get_robot_api(config)
        self.safety = SafetyController(self.robot_api)

    def run(self) -> None:
        failure_cfg = self.config.get("simulation", {}).get("failure_injection", {})
        visualization_cfg = self.config.get("settings", {}).get("visualization", {})
        visualize = visualization_cfg.get("enabled", False)

        try:
            for frame in self._frames():
                if maybe_drop_frame(failure_cfg.get("drop_frame_chance", 0.0)):
                    self.logger.info("frame_dropped")
                    continue

                detections = self.detector.detect(frame)
                detections = maybe_remove_detections(
                    detections, failure_cfg.get("no_detection_chance", 0.0)
                )
                detections = maybe_lose_target(
                    detections, failure_cfg.get("lost_target_chance", 0.0)
                )

                tracks = self.tracker.update(detections)
                decision = self.behavior.update(tracks)
                motion_request = self.policies.map_decision_to_motion(decision)
                command = self.motion_planner.plan(motion_request)
                self.robot_api.post_motion_drive(command)

                if visualize:
                    overlay_detections(frame, detections, decision.state)
                    cv2.imshow(
                        visualization_cfg.get("window_name", "RobotSim"), frame
                    )
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
        except KeyboardInterrupt:
            self.logger.info("simulation_interrupted")
        except Exception as exc:  # noqa: BLE001
            self.logger.error(f"simulation_error: {exc}")
            self.safety.stop_robot()
        finally:
            if visualize:
                cv2.destroyAllWindows()
            self.safety.stop_robot()

    def _frames(self) -> Iterable:
        return iter(self.camera)


def build_simulation_from_config() -> SimulationWorld:
    config = load_config()
    camera_cfg = config.get("simulation", {}).get("camera", {})
    source_type = camera_cfg.get("source_type", "video")
    if source_type == "webcam":
        camera = WebcamCameraSource(index=0)
    else:
        camera = VideoCameraSource(path=camera_cfg.get("source_path", ""), loop=camera_cfg.get("loop", True))

    detector_choice = config.get("models", {}).get("vision", {}).get("detector", "stub")
    detector: Detector
    if detector_choice == "stub":
        detector = StubDetector()
    else:
        detector = StubDetector()

    return SimulationWorld(camera=camera, detector=detector, config=config)
