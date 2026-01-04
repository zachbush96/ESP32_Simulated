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
- Switching from simulation to real hardware should require changing **one configuration value**

---

## What This Repo Is (and Is Not)

This repo **is**:
- A platform for building embodied autonomy
- A testbed for local vision and behavior systems
- A foundation that can evolve as hardware capabilities grow

This repo is **not**:
- A monolithic script
- A cloud-controlled robot
- A hardware firmware project
- A UI-first application

---

## Next Steps

- Define project structure
- Implement simulated robot API
- Implement vision and tracking stubs
- Build behavior state machine
- Add real robot API client
- Iterate toward real hardware integration

The emphasis early on is **correct structure**, not clever behavior.
