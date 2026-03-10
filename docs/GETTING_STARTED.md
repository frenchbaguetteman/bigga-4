# Getting Started

This codebase is a PROS V5 robot project, but it is organized more like a reusable template than a one-off season dump. If you are new to the repo, start here before reading the lower-level API pages.

The structure of this guide is intentionally close to the docs style used by PROS and EZ-Template: start with the workflow, then copy working examples, then open the API when you need exact behavior.

## What You Get Out of the Box

- A command-based robot architecture built on custom `Command`, `Subsystem`, and `CommandScheduler` classes.
- A `Drivetrain` subsystem with odometry, IMU integration, driver shaping, and feedforward path control.
- A touch-screen auton selector and runtime brain screen.
- Example autonomous routines for point moves, turns, and RAMSETE paths.
- Localization that combines odometry, GPS, and particle-filter estimates.

## Start Reading Here

1. [Template Tour](TEMPLATE_TOUR.md) to understand the project layout.
2. [Autonomous Cookbook](AUTONOMOUS_COOKBOOK.md) to copy your first real code pattern.
3. [API Reference](API_REFERENCE.md) once you need exact class signatures.
4. [User Guide](USER_GUIDE.md) if you are working from the driver or field side.

## First 15 Minutes

### 1. Preview the docs site

```bash
python3 -m pip install -r requirements-docs.txt
python3 -m mkdocs serve
```

Then open `http://127.0.0.1:8000/`.

### 2. Learn the project shape

| Path | What it is for |
|---|---|
| `src/main.cpp` | Competition lifecycle, subsystem setup, and driver control loop |
| `src/autonomous/autons.cpp` | Every selectable autonomous routine |
| `include/autonomous/` | Auton enums, build context, and shared helper commands |
| `include/commands/` | Reusable commands like drive, rotate, RAMSETE, intake, and lift |
| `include/subsystems/` | Drivetrain, intake, lift, and pneumatics interfaces |
| `include/config.h` | PID gains, sensor ports, geometry, and driver tuning |
| `src/ui/` and `include/ui/` | Brain screen and auton selector plumbing |

### 3. Boot the robot once

When the robot starts:

- `initialize()` brings up the brain screen
- subsystems are constructed
- the IMU calibrates
- localization is initialized
- the selected autonomous command graph is built
- the controller rumbles once when the robot is ready

That startup sequence is implemented in [`src/main.cpp`](../src/main.cpp).

### 4. Run one of the example autons

This repo already includes four tutorial-grade routines:

- `Example Move`
- `Example Turn`
- `Example Path`
- `Example LTV`

Those are selectable through the normal auton chooser and live in [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp).

## Your First Safe Edit

The easiest first edit is changing the travel distance in `Example Move`.

Current pattern:

```cpp
const Eigen::Vector2f forward = robotRelativePoint(start, 24.0f * kInToM, 0.0f);
const Eigen::Vector2f diagonal = robotRelativePoint(start, 24.0f * kInToM, 18.0f * kInToM);
```

If you want a shorter test, reduce the `24.0f` values. If you want a longer diagonal, increase the forward or left offsets. That lets you test the command framework without touching selector plumbing or path-generation code.

## Build and Test Workflow

This is a normal PROS project. Use the PROS VS Code extension or your usual CLI workflow for compile and upload.

Common local commands:

```bash
make
pros terminal
```

Use `pros terminal` when you want to watch `printf()` output during bring-up and debugging.

## What To Learn Next

If you are coming from stock PROS projects, read [Template Tour](TEMPLATE_TOUR.md) next.

If you are coming from EZ-Template and mainly care about auton authoring, skip straight to [Autonomous Cookbook](AUTONOMOUS_COOKBOOK.md).

If you want the exact interfaces, open [API Reference](API_REFERENCE.md).
