# 69580A — User Guide

## Table of Contents

1. [Overview](#overview)
2. [Getting Started](#getting-started)
3. [Project Structure](#project-structure)
4. [Configuration](#configuration)
5. [Autonomous Selector](#autonomous-selector)
6. [Driving](#driving)
7. [Autonomous Routines](#autonomous-routines)
8. [Localization System](#localization-system)
9. [Tuning Guide](#tuning-guide)
10. [Troubleshooting](#troubleshooting)

---

## Overview

**69580A** is a command-based PROS V5 robot framework that provides:

- **Command framework** — schedule commands that own subsystems, compose them
  sequentially / in parallel, and bind them to controller triggers.
- **Monte Carlo Localization (MCL)** — a particle filter fusing odometry, distance
  sensors, and a VEX GPS sensor for centimetre-level field positioning.
- **Motion profiling** — trapezoidal velocity profiles and cubic Bézier path
  generation for smooth autonomous movement.
- **RAMSETE & LTV path-following** — two closed-loop path-following controllers
  that drive the robot along profiled trajectories.
- **Brain-screen auton selector** — cycle through autonomous routines and flip
  alliance colour using the three LCD buttons.
- **Live pose display** — see the robot's estimated (x, y, θ) on the brain
  screen in real time.
- **GPS-based initialisation** — optionally read the VEX GPS at boot to seed
  the initial robot pose, no manual measuring required.

### Hardware Requirements

| Component | Quantity | Config Constant |
|-----------|----------|-----------------|
| V5 Motor (drive) | 6 | `LEFT_DRIVE_PORTS`, `RIGHT_DRIVE_PORTS` |
| V5 Motor (intake) | 2 | `INTAKE_PORTS` |
| V5 IMU | 1 | `IMU_PORT` |
| V5 Rotation Sensor (horizontal tracking) | 1 | `HORIZONTAL_TRACKING_PORT` |
| V5 Distance Sensor | 4 | `MCL_*_DISTANCE_PORT` |
| V5 GPS Sensor | 1 | `MCL_GPS_PORT` |
| 3-Wire Pneumatics | 4 | `SELECT1_PORT` … `WING_PORT` |

---

## Getting Started

### Prerequisites

| Tool | Version | Installation |
|------|---------|-------------|
| PROS CLI | ≥ 3.5 | `pip3 install pros-cli` |
| ARM GCC | ≥ 13 (15.2 recommended) | `brew install --cask gcc-arm-embedded` |
| VS Code + PROS extension | latest | VS Code marketplace |

### Build & Upload

```bash
# Build (use raw make — pros make has a version-detection bug)
make clean && make

# Upload to the brain
pros upload

# Or flash a specific slot
pros upload --slot 2
```

### First-Time Setup

1. **Wire the robot** according to the port table in `include/config.h`.
2. **Calibrate the IMU** — power on the brain with the robot flat and still;
   the IMU self-calibrates in ~2 seconds.
3. **Set the GPS strip** — apply the VEX GPS field strip and verify the sensor
   returns non-zero values.
4. **Verify sensors** — check the brain's Devices menu to confirm every sensor
   is detected on the correct port.

---

## Project Structure

```
├── include/
│   ├── config.h              ← all ports, PID gains, geometry, MCL config
│   ├── main.h                ← master include
│   ├── auton.h               ← active auton/alliance selection
│   ├── ui/
│   │   └── autonSelector.h   ← brain-screen selector
│   ├── command/               ← command framework (scheduler, groups, triggers)
│   ├── feedback/              ← PID controller
│   ├── subsystems/            ← drivetrain, intakes, lift, solenoids
│   ├── localization/          ← particle filter, sensor models
│   ├── motionProfiling/       ← Bézier paths, velocity profiles
│   ├── commands/              ← RAMSETE, LTV, drive/rotate commands
│   ├── autonomous/            ← auton routines, shared commands
│   └── ...                    ← Eigen, units, utils, JSON, telemetry
├── src/
│   ├── autonomous/            ← centralized auton definitions/builders
│   ├── main.cpp               ← lifecycle, GPS init, scheduler tasks
│   ├── subsystems/            ← subsystem implementations
│   ├── motionProfiling/       ← Bézier motion profiling impl
│   ├── command/               ← trigger implementation
│   ├── feedback/              ← PID implementation
│   └── ui/                    ← auton selector implementation
├── docs/
│   ├── USER_GUIDE.md          ← this file
│   └── API_REFERENCE.md       ← full API docs
└── Makefile / common.mk       ← build system
```

---

## Configuration

All tunables live in **`include/config.h`** inside `namespace CONFIG`.

### Motor Ports

```cpp
// Negative port = reversed motor
inline const std::vector<std::int8_t> LEFT_DRIVE_PORTS  = {-11, -13, -14};
inline const std::vector<std::int8_t> RIGHT_DRIVE_PORTS = { 9,  17,  20};
inline const std::vector<std::int8_t> INTAKE_PORTS      = { 3,  -5};
```

### Tracking Wheels

Set a tracking port to **0** to disable it:

| Constant | Default | Effect when 0 |
|----------|---------|---------------|
| `VERTICAL_TRACKING_PORT` | `0` | Falls back to averaged drive motor encoders |
| `HORIZONTAL_TRACKING_PORT` | `16` | Lateral displacement assumed zero |

### Startup Pose Mode

Controls how the robot's initial field pose is determined:

```cpp
enum class StartupPoseMode {
    ConfiguredStartPoseOnly,   // use START_POSE_X/Y/THETA from config
    GPSXYPlusIMUHeading,       // GPS x,y + IMU heading (recommended)
    FullGPSInit,               // GPS x,y + GPS heading
};
```

Set the active mode with:
```cpp
constexpr StartupPoseMode STARTUP_POSE_MODE = StartupPoseMode::GPSXYPlusIMUHeading;
```

`START_POSE_THETA_DEG` uses VEX compass convention for human entry:
`0° = north`, `90° = east`, clockwise positive. The localization stack converts
that to the internal radians frame automatically.

### Distance Sensor Weights

Each MCL distance sensor can be weighted 0.0–1.0 to tune its influence:

```cpp
constexpr double MCL_LEFT_DISTANCE_WEIGHT   = 0.60;
constexpr double MCL_RIGHT_DISTANCE_WEIGHT  = 0.60;
constexpr double MCL_BACK_DISTANCE_WEIGHT   = 0.80;
constexpr double MCL_FRONT_DISTANCE_WEIGHT  = 0.80;
```

Lower weight = less trust in that sensor. Sensors can also be individually
enabled/disabled with `MCL_ENABLE_*_DISTANCE_SENSOR` booleans.

### PID Gains

```cpp
inline PID::Gains TURN_PID      {kP, kI, kD, integralCap};
inline PID::Gains DISTANCE_PID  {kP, kI, kD, integralCap};
```

### RAMSETE Parameters

```cpp
constexpr float RAMSETE_ZETA = 0.4f;   // damping  (0 < ζ < 1)
constexpr float RAMSETE_BETA = 45.0f;  // aggressiveness (β > 0)
```

---

## Autonomous Selector

The brain screen provides a 3-button auton selector:

| Button | Action |
|--------|--------|
| **Left** | Previous auton |
| **Centre** | Toggle alliance (RED ↔ BLUE) |
| **Right** | Next auton |

The selected auton and alliance are shown on the screen. The selection stays
live during `initialize()` and `competition_initialize()`, and is locked in
when `autonomous()` starts.

Available routines:

| Name | Description |
|------|-------------|
| Negative 1 | Negative-side primary |
| Negative 2 | Negative-side alternate |
| Positive 1 | Positive-side primary |
| Positive 2 | Positive-side alternate |
| Skills | 60-second programming skills |
| None | Do nothing |

### Screen Layout

```
Line 0:  69580A
Line 1:  << Negative 1 >>
Line 2:  Alliance: RED
Line 3:
Line 4:  Pose: (12.3, -5.7) in
Line 5:  Heading: 45.2 deg
Line 6:  GPS locked (8 samples)
Line 7:  [<Prev]  [Alliance]  [Next>]
```

---

## Driving

### Teleop Controls

| Input | Action |
|-------|--------|
| Left stick Y | Forward / reverse |
| Right stick X | Turn |
| R1 (hold) | Intake forward |
| R2 (hold) | Intake reverse |
| L1 (press) | Toggle tongue pneumatic |
| L2 (press) | Toggle wing pneumatic |
| A (press) | Toggle select 1 pneumatic |
| B (press) | Toggle select 2 pneumatic |

The drive is **split arcade** — forward power from the left stick, turning
from the right stick.  The drivetrain will only accept manual input when no
autonomous command currently owns it.

### Skills Re-Run

During Skills, the autonomous routine auto-starts in `opcontrol()`.  The
partner controller's **Right** button acts as an abort trigger.

---

## Localization System

### How It Works

The Monte Carlo Localization system maintains a cloud of **250 particles**
(configurable) that each represent a possible robot position on the field.

Every 10 ms:

1. **Predict** — move each particle by the odometry displacement (with noise).
2. **Update** — weight each particle by the likelihood of the current sensor
   readings given that particle's position.
3. **Resample** — systematic low-variance resampling keeps the ensemble
   healthy and focused on high-probability regions.
4. **Estimate** — the weighted mean of all particles is the robot's estimated
   pose.

### Sensor Fusion

| Sensor | Role | How it helps |
|--------|------|-------------|
| IMU | Heading source | All particles share this heading |
| Odom (drive encoders or tracking wheel) | Displacement | Propagates particles each tick |
| Distance sensors (×4) | Lateral correction | Ray-cast likelihood against field walls |
| GPS | Global anchor | Gaussian likelihood on (x, y) |

### GPS Initialisation

At boot, the system optionally waits for the GPS to produce stable readings
before seeding the particle filter:

1. Polls `gps.get_position()` every 50 ms.
2. Checks if consecutive readings drift less than
   `STARTUP_GPS_READY_ERROR_M` (default 10 mm).
3. After `STARTUP_GPS_STABLE_SAMPLES` (default 8) stable readings, the GPS
   is considered locked.
4. Depending on `STARTUP_POSE_MODE`, the initial pose is set from:
   - GPS (x, y) + IMU heading (**recommended**), or
   - Full GPS (x, y, heading), or
   - Hard-coded config values only.

The wait times out after `STARTUP_GPS_MAX_WAIT_MS` (default 8 s), falling
back to config values if the GPS never locks.

---

## Tuning Guide

### Step 1 — Verify Odometry

1. Set `ODOM_DEBUG_ENABLE = true` in config.h.
2. Push the robot straight forward exactly 1 metre.
3. Check the logged forward distance on the brain screen.
4. Adjust `DRIVE_RADIUS` or `ODOM_RADIUS` until the reading matches.

### Step 2 — Verify Heading

1. Spin the robot exactly 360° by hand.
2. The IMU heading should return to ~0°.
3. If it drifts, recalibrate or check for magnetic interference.

### Step 3 — Tune Distance PID

1. Use `DriveMoveCommand` with a known distance (e.g. 0.5 m).
2. Increase `DISTANCE_PID.kP` until the robot overshoots slightly.
3. Add `DISTANCE_PID.kD` to damp oscillation.

### Step 4 — Tune Turn PID

1. Use `RotateCommand` to turn 90°.
2. Follow the same kP → kD tuning flow.

### Step 5 — Tune RAMSETE / LTV

1. Run a simple straight-line path command.
2. Increase `RAMSETE_BETA` for tighter tracking (more aggressive correction).
3. Increase `RAMSETE_ZETA` for more damping if the robot oscillates.

### Step 6 — Tune Feedforward

1. Measure the robot's free speed at full voltage → derive `FF_kV`.
2. Measure the minimum voltage to move from standstill → derive `FF_kS`.
3. `FF_kA` is usually small; increase if the robot is sluggish on acceleration.

### Step 7 — Tune MCL

1. Place the robot at a known position.
2. Check the pose readout on the brain screen.
3. If it's wrong, adjust sensor weights, offsets, or `DRIVE_NOISE`.
4. Reduce `NUM_PARTICLES` for faster updates (min ~100), increase for
   better accuracy (max ~500).

---

## Troubleshooting

### Build fails with "gnu++26 not supported"

Your ARM GCC is too old. Install GCC ≥ 15:
```bash
brew install --cask gcc-arm-embedded
```

### IntelliSense shows "unsupported option -mfloat-abi"

The `.vscode/c_cpp_properties.json` is pointing to a clang-based compiler.
Change `compilerPath` to the real GCC:
```json
"compilerPath": "/opt/homebrew/bin/arm-none-eabi-g++"
```

### GPS never locks at boot

- Verify the GPS strip is placed on the field perimeter.
- Check `MCL_GPS_PORT` matches the actual wired port.
- Increase `STARTUP_GPS_MAX_WAIT_MS` if the GPS is slow.
- The system will fall back to config start-pose after the timeout.

### Robot drives backwards

One or more motor ports need to be negated. Flip the sign in
`LEFT_DRIVE_PORTS` or `RIGHT_DRIVE_PORTS`.

### Tracking wheel reads zero

- Confirm the rotation sensor port is correct.
- Check `*_TRACKING_REVERSED` — try flipping it.
- Ensure the tracking wheel is physically touching the ground.

### Particle filter converges to wrong position

- Check distance sensor offsets (inches, robot-local frame).
- Reduce sensor weights for unreliable sensors.
- Increase `NUM_PARTICLES` temporarily to diagnose.
- Verify field walls are standard VEX competition (12′ × 12′).

### `pros make` crashes with RuntimeError

Use raw `make` instead:
```bash
make clean && make
```

---

*69580A — Team 69580A, 2025–2026 season.*
