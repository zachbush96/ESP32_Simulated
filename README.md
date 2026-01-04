# Autonomous Robot Desktop Controller (Offline-First)

This repository contains the **desktop-side software** for an autonomous robot system.

The physical robot is an ESP32-CAM-based, four-wheel platform that exposes a small HTTP API for motion and camera access. The robot itself is intentionally simple. All perception, decision-making, and autonomy live on the desktop.

This project is being built **offline-first**. Once models and dependencies are installed, the system must run without internet access.

---

## High-Level Goal

Eventually, this system should be able to:

- Consume a live camera stream from a robot
- Perform **local image processing and object detection** (e.g. cats, cups, bowls)
- Track detected objects across frames
- Maintain a behavior state machine (search, pursue, lost, stop)
- Convert perception into motion commands
- Explore an apartment autonomously
- Locate and dock with a local charger
- Operate entirely on a local network with no cloud dependencies

This repository starts by building the **scaffolding and simulation environment** required to develop all of the above *before the physical robot is available*.

---

## Core Design Principles

- **Desktop brain, dumb robot body**
  - The robot exposes truth and executes commands
  - The desktop observes, plans, and decides

- **Offline-first**
  - No cloud APIs
  - Local models only
  - LAN WiFi is sufficient

- **Honest hardware assumptions**
  - No imaginary sensors
  - No fake battery or proximity data unless explicitly mocked
  - The simulated robot API must match the real robot API

- **Separation of concerns**
  - Vision
  - Tracking
  - Behavior logic
  - Control logic
  - Robot API client
  - Simulation / mocking layer

---

## Current State

- The physical robot is **not yet available**
- Development will begin using:
  - Simulated camera input (webcam, video file, or image loop)
  - A mock robot API that mirrors the real ESP32 endpoints
- Switching from simulation to real hardware requires changing **one configuration value** (`mode`)

---

## Project Layout

```
config/
  settings.yaml      # global settings (mode selection, logging, control limits)
  simulation.yaml    # simulation-specific knobs (camera source, failure injection)
  models.yaml        # detector/model selection and paths
src/
  app.py             # entrypoint to launch simulation (real mode stubbed for now)
  core/              # logging, events, configuration loading
  robot_api/         # real vs simulated API clients and router
  vision/            # camera sources, detector interfaces, visualization
  tracking/          # simple track management stubs
  behavior/          # state machine and policies mapping perception -> motion intent
  control/           # motion planning and safety stop helpers
  simulation/        # simulation loop and failure injection utilities
scripts/             # helper scripts (future)
data/                # sample media for simulation (add your own)
```

### Mode selection

- Controlled by `config/settings.yaml` (`mode: simulation` or `mode: real`).
- Environment variable `ROBOT_MODE` can override the mode at runtime.
- `src/robot_api/router.py` returns either the simulated API client or the real HTTP client based on this value.

### How components communicate

1. `CameraSource` yields frames (video file, webcam, or future image sequence adapter).
2. `Detector` processes frames into detections (stub for now).
3. `Tracker` assigns simple IDs to detections.
4. `BehaviorStateMachine` consumes tracks and emits a high-level decision (state + target).
5. `BehaviorPolicies` map the decision to a motion intent.
6. `MotionPlanner` clamps commands to configured limits and outputs a `MotionCommand`.
7. `RobotAPI` (simulated or real) receives `/motion/drive` or `/motion/stop` commands. In simulation, commands are logged.
8. Optional visualization overlays detections and state for debugging.

### Logging

- Structured JSONL logging via `src/core/logging.py` for state transitions, decisions, and commands.

### Simulation loop

- `src/simulation/world.py` ties together the camera, detector, tracker, behavior, control, and robot API.
- Failure injection supports dropped frames, missing detections, and lost targets (see `config/simulation.yaml`).
- Visualization can be toggled via `config/settings.yaml`.

### Running the simulation (early prototype)

```
python -m src.app
```

Requirements: Python 3.10+, `opencv-python`, `pyyaml`, and `requests` for real robot support. Add a sample video to `data/sample.mp4` or update `config/simulation.yaml` to point at a valid source.

---

## Next Steps

- Implement real detector integration (ONNX or other local models)
- Add image-sequence camera source for fully offline deterministic tests
- Expand tracker to maintain identities over time
- Flesh out behavior states (idle, search, pursue, lost, stopped) with timers and recovery
- Add richer safety handling and health monitoring
- Implement real-robot loop and parity tests between sim and hardware

The emphasis early on is **correct structure**, not clever behavior.
