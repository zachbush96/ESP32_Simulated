import base64
import json
import os
from typing import Any, Dict, List, Optional

import cv2
import requests

from src.core.logging import setup_json_logger


class VLMClient:
    """
    A client to interact with a Local LLM/VLM (like SmolVLM) via an OpenAI-compatible API
    (e.g., LM Studio).
    """

    def __init__(self, base_url: str = "http://localhost:1234/v1", model: str = "smolvlm") -> None:
        self.base_url = base_url
        self.model = model
        self.logger = setup_json_logger("vlm_client")
        # Allow overriding via env vars for testing
        self.mock_mode = os.environ.get("VLM_MOCK_MODE", "false").lower() == "true"

    def detect_objects(self, image: Any) -> List[Dict[str, Any]]:
        """
        Sends an image to the VLM and asks it to detect objects.
        Returns a list of detections (label, confidence, bbox).
        """
        if self.mock_mode:
            return self._mock_detect_objects(image)

        prompt = (
            "Detect objects in this image. Return a JSON list where each item has "
            "'label', 'confidence' (0-1), and 'bbox' [x, y, w, h] (normalized 0-1)."
        )
        response = self._send_request(image, prompt)
        return self._parse_json_response(response)

    def explore(self, image: Any) -> Dict[str, Any]:
        """
        Sends an image to the VLM and asks for an exploration decision.
        """
        if self.mock_mode:
            return self._mock_explore(image)

        prompt = (
            "You are a robot exploring an apartment. Look at this image. "
            "Decide where to go next. Return a JSON object with 'action' "
            "(e.g., 'move_forward', 'turn_left', 'turn_right', 'stop') "
            "and 'reason'."
        )
        response = self._send_request(image, prompt)
        try:
            return json.loads(response)
        except json.JSONDecodeError:
            self.logger.error(f"Failed to parse JSON from explore response: {response}")
            return {"action": "stop", "reason": "parsing_error"}

    def _send_request(self, image: Any, prompt: str) -> str:
        """
        Encodes the image and sends a request to the chat completions endpoint.
        """
        try:
            base64_image = self._encode_image(image)
            payload = {
                "model": self.model,
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {
                                "type": "image_url",
                                "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
                            },
                        ],
                    }
                ],
                "temperature": 0.7,
            }
            response = requests.post(f"{self.base_url}/chat/completions", json=payload, timeout=5)
            response.raise_for_status()
            data = response.json()
            content = data["choices"][0]["message"]["content"]
            return content
        except Exception as e:
            self.logger.error(f"VLM request failed: {e}")
            return ""

    def _encode_image(self, image: Any) -> str:
        _, buffer = cv2.imencode(".jpg", image)
        return base64.b64encode(buffer).decode("utf-8")

    def _parse_json_response(self, response_text: str) -> List[Dict[str, Any]]:
        """
        Attempts to extract and parse JSON from the VLM response.
        """
        try:
            # Simple cleanup to handle potential markdown code blocks
            clean_text = response_text.replace("```json", "").replace("```", "").strip()
            data = json.loads(clean_text)
            if isinstance(data, list):
                return data
            return []
        except json.JSONDecodeError:
            self.logger.error(f"Failed to parse JSON detection response: {response_text}")
            return []

    def _mock_detect_objects(self, image: Any) -> List[Dict[str, Any]]:
        """
        Returns mock detections for testing.
        """
        # Return a "cat" detection in the center of the frame
        return [
            {
                "label": "cat",
                "confidence": 0.95,
                "bbox": [0.3, 0.3, 0.4, 0.4],  # Normalized x, y, w, h
            }
        ]

    def _mock_explore(self, image: Any) -> Dict[str, Any]:
        """
        Returns a mock exploration decision.
        """
        return {"action": "move_forward", "reason": "path is clear"}
