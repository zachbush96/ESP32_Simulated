import base64
import time
import os
import requests
import threading
import subprocess
import cv2
import numpy as np
import gradio as gr
from mcp.server.fastmcp import FastMCP
from ultralytics import YOLO
from collections import deque

# ===========================
# CONFIGURATION
# ===========================
ROBOT_IP = "10.0.0.37"  # <--- REPLACE THIS WITH YOUR ESP32 IP
BASE_URL = f"http://{ROBOT_IP}"
ALLOWED_CLASSES = {"book", "cat"}  # limit detection to reduce post-processing
DEFAULT_DET_CONF = 0.15  # detection threshold for target search/overlay
YOLO_IMGSZ = 256  # smaller image size for faster inference
SMOLVLM_MODEL_LABEL = "SmolVLM Realtime (ESP32)"
SMOLVLM_BASE_URL = os.environ.get("SMOLVLM_BASE_URL", "http://localhost:8080")
SMOLVLM_INSTRUCTION = os.environ.get(
    "SMOLVLM_INSTRUCTION", "Your view is from the perspective of a small robotic car within an apartment. Describe what you see using simple language. Respond in only a few words."
)
SMOLVLM_REQUEST_INTERVAL_MS = int(os.environ.get("SMOLVLM_REQUEST_INTERVAL_MS", "500"))
SMOLVLM_MAX_TOKENS = int(os.environ.get("SMOLVLM_MAX_TOKENS", "80"))
SMOLVLM_TIMEOUT = float(os.environ.get("SMOLVLM_TIMEOUT", "5"))
SMOLVLM_IMAGE_MAX_WIDTH = int(os.environ.get("SMOLVLM_IMAGE_MAX_WIDTH", "480"))
SMOLVLM_JPEG_QUALITY = int(os.environ.get("SMOLVLM_JPEG_QUALITY", "80"))
MODEL_CHOICES = [
    SMOLVLM_MODEL_LABEL,
    "yolo11n.pt",
    "yolo11s.pt",
    "yolov8n.pt",
    "yolov8s.pt",
    "yolov5n.pt",
]
DEFAULT_MODEL = "yolo11n.pt"
CODEX_WORKDIR = "/Users/zachbush/Documents/code/esp_robot"
CODEX_LOG_DIR = "/tmp/codex_exec"

# PID CONSTANTS (Tune these for your specific robot motors)
TURN_P = 0.002  # Proportional gain for turning
FWD_P = 0.002   # Proportional gain for forward speed
MAX_SPEED = 0.7 # Safety cap

# ===========================
# ROBOT CLIENT (API WRAPPER)
# ===========================
class RobotClient:
    def __init__(self, base_url):
        self.base_url = base_url
        self.current_linear = 0.0
        self.current_angular = 0.0

    def drive(self, linear, angular):
        # Optimization: Don't spam requests if values haven't changed significantly
        if abs(self.current_linear - linear) < 0.05 and abs(self.current_angular - angular) < 0.05:
            return True
        
        self.current_linear = linear
        self.current_angular = angular
        
        try:
            requests.post(f"{self.base_url}/motion/drive", json={"linear": linear, "angular": angular}, timeout=0.5)
            return True
        except Exception as e:
            # print(f"Error driving: {e}") # Suppress print spam in loops
            return False

    def stop(self):
        self.current_linear = 0.0
        self.current_angular = 0.0
        try:
            requests.post(f"{self.base_url}/motion/stop", timeout=1)
            return True
        except Exception:
            return False

    def get_logs(self):
        try:
            res = requests.get(f"{self.base_url}/events", timeout=2)
            if res.status_code == 200:
                return res.json().get("events", [])
        except:
            return
        return

    def get_frame(self):
        """Fetches a single frame for MCP analysis."""
        try:
            stream = requests.get(f"{self.base_url}/camera/stream", stream=True, timeout=1)
            bytes_data = b''
            # Grab a small chunk just to get one frame
            for chunk in stream.iter_content(chunk_size=1024):
                bytes_data += chunk
                a = bytes_data.find(b'\xff\xd8')
                b = bytes_data.find(b'\xff\xd9')
                if a!= -1 and b!= -1:
                    jpg = bytes_data[a:b+2]
                    nparr = np.frombuffer(jpg, np.uint8)
                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    return img
                # Safety break to prevent hanging
                if len(bytes_data) > 400000: 
                    break
        except Exception:
            return None
        return None

robot = RobotClient(BASE_URL)

# ===========================
# SMART BRAIN (YOLO + LOGIC)
# ===========================
class SmartRobotBrain:
    def __init__(self):
        self.model = YOLO(DEFAULT_MODEL)  # Using YOLOv11 Nano for speed
        self.model_name = DEFAULT_MODEL
        self.active = False
        self.target_class = "cat"  # Default target
        self.state = "IDLE"  # IDLE, HUNT, FOLLOW
        self.latest_frame = None
        self.lock = threading.Lock()
        self.det_lock = threading.Lock()
        self.last_detections = []  # cached detections for UI overlays
        self.last_fps = 0.0
        self.last_infer_ms = 0.0
        self.stream_fps = 0.0
        self.det_conf = DEFAULT_DET_CONF
        self.model_lock = threading.Lock()
        # Build allowed class ids for filtering
        self.allowed_class_ids = [cid for cid, name in self.model.names.items() if name in ALLOWED_CLASSES]
        # Use FP16 if CUDA is available
        try:
            import torch
            self.use_half = torch.cuda.is_available()
        except Exception:
            self.use_half = False
        
        # Search variables
        self.last_seen_time = 0
        self.search_direction = 1 # 1 for right, -1 for left
        
        # Start the background logic loop
        self.thread = threading.Thread(target=self._logic_loop, daemon=True)
        self.thread.start()

    def set_mode(self, active: bool, target: str = "cat"):
        self.active = active
        self.target_class = target
        if not active:
            self.state = "IDLE"
            robot.stop()
        else:
            self.state = "HUNT"

    def update_frame(self, frame):
        with self.lock:
            self.latest_frame = frame
    
    def get_last_detections(self):
        with self.det_lock:
            return list(self.last_detections)

    def get_latest_frame(self):
        with self.lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()
    
    def set_det_conf(self, value: float):
        with self.model_lock:
            self.det_conf = max(0.01, min(0.99, float(value)))
        return self.det_conf
    
    def set_model(self, model_name: str):
        new_model = YOLO(model_name)
        allowed_class_ids = [cid for cid, name in new_model.names.items() if name in ALLOWED_CLASSES]
        with self.model_lock:
            self.model = new_model
            self.model_name = model_name
            self.allowed_class_ids = allowed_class_ids
        if not allowed_class_ids:
            return f"Loaded {model_name} (warning: 'book'/'cat' not in labels)"
        return f"Loaded {model_name}"

    def _logic_loop(self):
        """Main autonomous control loop (approx 20Hz)"""
        while True:
            if not self.active:
                time.sleep(0.1)
                continue

            # 1. Get Frame safely
            frame = None
            with self.lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()
            
            if frame is None:
                time.sleep(0.1)
                continue

            # 2. Run Inference
            start = time.time()
            with self.model_lock:
                model = self.model
                det_conf = self.det_conf
                allowed_ids = list(self.allowed_class_ids)
                use_half = self.use_half
            results = model(
                frame,
                verbose=False,
                imgsz=YOLO_IMGSZ,
                conf=det_conf,
                classes=allowed_ids if allowed_ids else None,
                half=use_half,
            )
            infer_ms = (time.time() - start) * 1000.0
            detections = results[0].boxes
            # Cache detections for UI rendering without duplicate inference
            cached = []
            for box in detections:
                cls_id = int(box.cls)
                conf = float(box.conf)
                label = model.names[cls_id]
                if conf < det_conf:
                    continue  # safety check
                if allowed_ids and label not in ALLOWED_CLASSES:
                    continue
                x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
                cached.append(
                    {
                        "cls": cls_id,
                        "label": label,
                        "conf": conf,
                        "xyxy": (x1, y1, x2, y2),
                    }
                )
            with self.det_lock:
                self.last_detections = cached
                # Approx FPS based on inference time (frame fetch adds a bit more)
                self.last_infer_ms = infer_ms
                self.last_fps = 1000.0 / infer_ms if infer_ms > 0 else 0.0
            
            target_box = None
            max_conf = 0.0
            
            # 3. Filter for our target (detections already filtered to allowed classes)
            for box in detections:
                cls_id = int(box.cls)
                conf = float(box.conf)
                label = model.names[cls_id]
                
                if conf < det_conf:
                    continue
                
                if label == self.target_class:
                    if conf > max_conf:
                        max_conf = conf
                        target_box = box

            # 4. State Machine
            current_time = time.time()
            
            if target_box:
                self.state = "FOLLOW"
                self.last_seen_time = current_time
                
                # Extract Box Geometry
                x1, y1, x2, y2 = target_box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                box_width = x2 - x1
                img_h, img_w = frame.shape[:2]
                
                # --- PID CONTROL ---
                # Error X: Distance from center of screen (steering)
                error_x = cx - (img_w / 2)
                
                # Error Y: How close are we? (Use box width as proxy for distance)
                # If box fills 30% of screen, we are close enough
                target_width = img_w * 0.30 
                error_depth = target_width - box_width
                
                # Calculate Speeds
                angular_cmd = -1 * (error_x * TURN_P) # Negative because left is usually positive angle
                linear_cmd = error_depth * FWD_P
                
                # Clamp speeds
                angular_cmd = max(min(angular_cmd, 1.0), -1.0)
                linear_cmd = max(min(linear_cmd, MAX_SPEED), -MAX_SPEED)
                
                # Deadband (stop jittering if close)
                if abs(error_x) < 20: angular_cmd = 0
                if abs(error_depth) < 20: linear_cmd = 0
                
                robot.drive(linear_cmd, angular_cmd)
                
                # Remember direction for search
                if angular_cmd > 0.1: self.search_direction = -1
                elif angular_cmd < -0.1: self.search_direction = 1

            else:
                # No target seen
                if current_time - self.last_seen_time < 2.0:
                    # Persistence: keep doing what we were doing for 2 seconds (handle occlusion)
                    pass 
                else:
                    self.state = "HUNT"
                    # Smart Search: Spin in the direction we last saw it, or just scan
                    # Spin slowly
                    robot.drive(0.0, 0.4 * self.search_direction)

            time.sleep(0.02) # faster control loop

class SmolVLMRealtimeAnalyzer:
    def __init__(
        self,
        frame_source,
        base_url: str,
        instruction: str,
        interval_ms: int,
        max_tokens: int,
        timeout: float,
        max_width: int,
        jpeg_quality: int,
    ) -> None:
        self.frame_source = frame_source
        self.base_url = base_url.rstrip("/")
        self.instruction = instruction
        self.interval_s = max(0.1, interval_ms / 1000.0)
        self.max_tokens = max_tokens
        self.timeout = timeout
        self.max_width = max_width
        self.jpeg_quality = max(10, min(95, jpeg_quality))
        self.enabled = False
        self.last_text = ""
        self.last_error = ""
        self.last_latency_ms = 0.0
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self.enabled = enabled
            if not enabled:
                self.last_text = ""
                self.last_error = ""
                self.last_latency_ms = 0.0

    def get_latest_text(self) -> str:
        with self._lock:
            if not self.enabled:
                return ""
            if self.last_error:
                return f"SmolVLM error: {self.last_error}"
            if self.last_text:
                return self.last_text
            return "SmolVLM: waiting for response..."

    def _loop(self) -> None:
        while True:
            with self._lock:
                enabled = self.enabled
            if not enabled:
                time.sleep(0.1)
                continue

            start = time.time()
            frame = self.frame_source()
            if frame is None:
                self._set_status("Waiting for camera frames...")
                time.sleep(0.1)
                continue

            try:
                text = self._send_request(frame)
                latency_ms = (time.time() - start) * 1000.0
                with self._lock:
                    self.last_text = text
                    self.last_error = ""
                    self.last_latency_ms = latency_ms
            except Exception as exc:  # noqa: BLE001
                self._set_error(str(exc))

            elapsed = time.time() - start
            time.sleep(max(0.0, self.interval_s - elapsed))

    def _set_status(self, message: str) -> None:
        with self._lock:
            self.last_text = message
            self.last_error = ""

    def _set_error(self, message: str) -> None:
        with self._lock:
            self.last_error = message

    def _send_request(self, frame) -> str:
        image_b64 = self._encode_frame(frame)
        payload = {
            "max_tokens": self.max_tokens,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": self.instruction},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"},
                        },
                    ],
                }
            ],
        }
        response = requests.post(
            f"{self.base_url}/v1/chat/completions",
            json=payload,
            timeout=self.timeout,
        )
        response.raise_for_status()
        data = response.json()
        return data["choices"][0]["message"]["content"]

    def _encode_frame(self, frame) -> str:
        resized = self._resize_frame(frame)
        ok, buffer = cv2.imencode(
            ".jpg", resized, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            raise RuntimeError("Failed to encode frame for SmolVLM.")
        return base64.b64encode(buffer).decode("utf-8")

    def _resize_frame(self, frame):
        if self.max_width <= 0:
            return frame
        height, width = frame.shape[:2]
        if width <= self.max_width:
            return frame
        scale = self.max_width / float(width)
        new_size = (int(width * scale), int(height * scale))
        return cv2.resize(frame, new_size, interpolation=cv2.INTER_AREA)

brain = SmartRobotBrain()
smolvlm_analyzer = SmolVLMRealtimeAnalyzer(
    frame_source=brain.get_latest_frame,
    base_url=SMOLVLM_BASE_URL,
    instruction=SMOLVLM_INSTRUCTION,
    interval_ms=SMOLVLM_REQUEST_INTERVAL_MS,
    max_tokens=SMOLVLM_MAX_TOKENS,
    timeout=SMOLVLM_TIMEOUT,
    max_width=SMOLVLM_IMAGE_MAX_WIDTH,
    jpeg_quality=SMOLVLM_JPEG_QUALITY,
)
os.makedirs(CODEX_LOG_DIR, exist_ok=True)

# ===========================
# MCP SERVER
# ===========================
mcp = FastMCP("ESP32 AI Robot")

@mcp.tool()
def enable_autonomous_mode(target: str) -> str:
    """
    Enables autonomous 'Hunting' mode.
    target: The object to look for (e.g., 'cat', 'person', 'chair', 'cup').
    """
    brain.set_mode(True, target)
    return f"Robot is now hunting for a {target}."

@mcp.tool()
def disable_autonomous_mode() -> str:
    """Stops the robot's AI brain and returns to manual control."""
    brain.set_mode(False)
    robot.stop()
    return "Autonomous mode disabled. Robot stopped."

@mcp.tool()
def manual_drive(linear: float, angular: float) -> str:
    """Manual drive command. Disables AI mode if active."""
    if brain.active:
        brain.set_mode(False)
    
    if robot.drive(linear, angular):
        return f"Driving L={linear} A={angular}"
    return "Failed."

@mcp.tool()
def move_robot(linear: float, angular: float) -> str:
    """
    Moves the robot. 
    linear: Speed forward (0.0 to 1.0) or backward (-1.0 to 0.0).
    angular: Turn right (0.0 to 1.0) or left (-1.0 to 0.0).
    """
    if robot.drive(linear, angular):
        return f"Driving with linear={linear}, angular={angular}"
    return "Failed to send command."

@mcp.tool()
def stop_robot() -> str:
    """Stops the robot immediately."""
    if robot.stop():
        return "Robot stopped."
    return "Failed to stop."

@mcp.tool()
def read_logs() -> str:
    """Reads the latest system logs from the robot."""
    logs = robot.get_logs()
    return "\n".join(logs or [])

# ===========================
# GRADIO INTERFACE (STREAMING)
# ===========================
def start_gradio():
    
    # Generator that yields frames with AI overlays
    def video_stream_generator():
        while True:
            # 1. Fetch Raw Frame from ESP32
            frame = robot.get_frame()
            
            if frame is not None:
                # Track stream FPS based on generator loop timing
                now = time.time()
                if not hasattr(video_stream_generator, "_last_ts"):
                    video_stream_generator._last_ts = now
                dt = now - video_stream_generator._last_ts
                video_stream_generator._last_ts = now
                if dt > 0:
                    brain.stream_fps = 1.0 / dt

                # 2. Pass to Brain (for processing logic)
                brain.update_frame(frame)
                
                # 3. Draw UI Overlays (Boxes & Status)
                if brain.active:
                    # Use cached detections from the brain to avoid duplicate inference
                    cv2.putText(frame, f"MODE: {brain.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.putText(frame, f"TARGET: {brain.target_class}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"MODEL: {brain.model_name}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
                    cv2.putText(frame, f"CONF: {brain.det_conf:.2f} INF_FPS~{brain.last_fps:.1f} STRM~{brain.stream_fps:.1f}", (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
                    
                    for det in brain.get_last_detections():
                        x1, y1, x2, y2 = map(int, det["xyxy"])
                        label = det["label"]
                        conf = det["conf"]
                        is_target = label == brain.target_class
                        color = (0, 255, 0) if is_target else (255, 0, 0)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(
                            frame,
                            f"{label} {conf:.2f}",
                            (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            color,
                            2,
                        )
                
                # Convert BGR (OpenCV) to RGB (Gradio)
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                yield frame_rgb, smolvlm_analyzer.get_latest_text()
            
            time.sleep(0.005)  # reduce UI stall

    def set_ai_btn(target_name):
        brain.set_mode(True, target_name)
        return f"Hunting: {target_name}"

    def stop_ai_btn():
        brain.set_mode(False)
        return "Manual Mode"
    
    def send_codex(prompt_text: str):
        prompt = (prompt_text or "").strip()
        if not prompt:
            return "No prompt provided."
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(CODEX_LOG_DIR, f"codex_{ts}.log")
        
        def _run():
            with open(log_path, "w") as logf:
                proc = subprocess.Popen(
                    ["codex", "exec", "--yolo", prompt],
                    cwd=CODEX_WORKDIR,
                    stdout=logf,
                    stderr=subprocess.STDOUT,
                )
                proc.wait()
        
        threading.Thread(target=_run, daemon=True).start()
        return f"Codex task started. Log: {log_path}"

    def handle_model_change(model_name: str):
        if model_name == SMOLVLM_MODEL_LABEL:
            smolvlm_analyzer.set_enabled(True)
            return (
                gr.update(visible=True),
                f"SmolVLM realtime enabled ({SMOLVLM_BASE_URL})",
            )
        smolvlm_analyzer.set_enabled(False)
        status = brain.set_model(model_name)
        return gr.update(visible=False, value=""), status

    # UI Layout
    with gr.Blocks(title="AI Robot Commander") as demo:
        gr.Markdown("# 🧠 Smart ESP32 Robot Dashboard")
        
        with gr.Row():
            # Video Column
            with gr.Column(scale=3):
                video_display = gr.Image(label="AI Vision Feed", streaming=True)
            
            # Control Column
            with gr.Column(scale=1):
                gr.Markdown("### 🤖 Autonomous Controls")
                target_input = gr.Textbox(label="Target Object", value="cat")
                model_select = gr.Dropdown(
                    label="Model",
                    choices=MODEL_CHOICES,
                    value=DEFAULT_MODEL,
                )
                conf_slider = gr.Slider(
                    label="Detection Confidence",
                    minimum=0.05,
                    maximum=0.90,
                    step=0.05,
                    value=DEFAULT_DET_CONF,
                )
                with gr.Row():
                    btn_hunt = gr.Button("🔍 START HUNTING", variant="primary")
                    btn_stop_ai = gr.Button("🛑 STOP AI", variant="stop")
                
                status_box = gr.Textbox(label="System Status", value="IDLE")

                smolvlm_output = gr.Textbox(
                    label="SmolVLM Realtime Analysis",
                    value="",
                    lines=4,
                    interactive=False,
                    visible=False,
                )
                
                gr.Markdown("---")
                gr.Markdown("### New Feature Chat")
                codex_prompt = gr.Textbox(label="New Feature Chat", placeholder="Describe the feature you want Codex to implement...")
                codex_send = gr.Button("Send to Codex")
                codex_status = gr.Textbox(label="Codex Status", value="")
                
                gr.Markdown("---")
                gr.Markdown("### 🎮 Manual Override")
                with gr.Row():
                    btn_fwd = gr.Button("⬆️")
                with gr.Row():
                    btn_left = gr.Button("⬅️")
                    btn_stop = gr.Button("⏹️")
                    btn_right = gr.Button("➡️")
                with gr.Row():
                    btn_back = gr.Button("⬇️")

        # Events
        btn_hunt.click(set_ai_btn, inputs=target_input, outputs=status_box)
        btn_stop_ai.click(stop_ai_btn, outputs=status_box)
        model_select.change(
            handle_model_change, inputs=model_select, outputs=[smolvlm_output, status_box]
        )
        conf_slider.change(lambda c: f"Confidence set to {brain.set_det_conf(c):.2f}", inputs=conf_slider, outputs=status_box)
        codex_send.click(send_codex, inputs=codex_prompt, outputs=codex_status)
        
        # Manual buttons automatically disable AI
        btn_fwd.click(lambda: (brain.set_mode(False), robot.drive(0.8, 0.0)), outputs=None)
        btn_back.click(lambda: (brain.set_mode(False), robot.drive(-0.8, 0.0)), outputs=None)
        btn_left.click(lambda: (brain.set_mode(False), robot.drive(0.0, -0.8)), outputs=None)
        btn_right.click(lambda: (brain.set_mode(False), robot.drive(0.0, 0.8)), outputs=None)
        btn_stop.click(lambda: (brain.set_mode(False), robot.stop()), outputs=None)

        # Hook up the generator after all outputs exist
        demo.load(fn=video_stream_generator, outputs=[video_display, smolvlm_output])

    demo.launch(server_name="0.0.0.0", server_port=7860, prevent_thread_lock=True)

# ===========================
# MAIN ENTRY
# ===========================
if __name__ == "__main__":
    print("Initializing System...")
    print("Loading AI Model (this may take a moment)...")
    # Trigger model load once at startup
    _ = brain.model("https://ultralytics.com/images/bus.jpg", verbose=False)
    print("Model Loaded!")
    
    start_gradio()
    
    print("Starting MCP Server on Stdio...")
    mcp.run()
