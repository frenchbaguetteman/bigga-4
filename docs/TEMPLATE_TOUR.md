# Template Tour

This page explains how the project is put together and how it maps to the mental model you may already have from PROS or EZ-Template.

## Mental Model

Think of the robot as four layers:

1. `main.cpp` owns the competition lifecycle.
2. Subsystems wrap hardware.
3. EZ motion owns autonomous movement.
4. Direct auton functions compose full routines from EZ primitives.

If you only remember one thing, remember this: the live autonomous path is direct-function execution plus EZ motion, not the older command-graph builder path.

## Competition Lifecycle

The top-level flow lives in [`src/main.cpp`](../src/main.cpp):

```cpp
void initialize() {
    BrainScreen::initialize();
    AutonSelector::selectAuton(DEFAULT_AUTON_SELECTION);
    createRobotObjects();
    subsystemInit();
    initializeAutonMotion();
    localizationInit();
    AutonSelector::init();
}

void autonomous() {
    runSelectedAutonNow();
}

void opcontrol() {
    CommandScheduler::reset();
    // controller polling and optional Down+B auton launch live here
}
```

That is the main pattern throughout the repo:

- `initialize()` prepares long-lived state
- `autonomous()` runs the selected auton entry directly
- `opcontrol()` handles controller input and manual driving

## Autonomous Architecture

Autonomous routines live in [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp) as direct functions. The shared motion backend is the vendored EZ drive in [`include/EZ-Template/drive/drive.hpp`](../include/EZ-Template/drive/drive.hpp).

The live motion calls are:

- `pid_drive_set(...)`
- `pid_turn_set(...)`
- `pid_swing_set(...)`
- `pid_odom_set(...)`
- `pid_odom_ramsete_set(...)`
- `pid_odom_ltv_set(...)`
- `pid_wait(...)`

## How This Maps From EZ-Template

If you are used to EZ-Template, these are the closest equivalents in this repo:

| EZ-Template style idea | This repo |
|---|---|
| Global chassis object | shared EZ drive backend |
| `pid_drive_set(...)` | `pid_drive_set(...)` |
| `pid_turn_set(...)` | `pid_turn_set(...)` |
| Odom/path movement | `pid_odom_set(...)`, `pid_odom_ramsete_set(...)`, `pid_odom_ltv_set(...)` |
| Auton selector pages | `AutonSelector` + `BrainScreen` |
| Simple helper functions for auton | direct routine helpers in `src/autonomous/autons.cpp` |

The main difference from the older command-heavy branch is that the selected auton now runs inline through `runAuton(...)`.

## Coordinate System

Internal motion uses one shared convention:

- position in meters
- heading in radians
- `+X` is field-forward / east
- `+Y` is field-left / north
- positive heading is counterclockwise

This matters when you author waypoints. If a point looks mirrored or rotated, check the frame before tuning the controller.

## What To Edit For Common Tasks

| Task | Primary file |
|---|---|
| Add or change an auton | [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp) |
| Tune PID / RAMSETE / ports | [`include/config.h`](../include/config.h) |
| Change driver controls | [`src/main.cpp`](../src/main.cpp) |
| Change brain-screen behavior | [`src/ui/brainScreen.cpp`](../src/ui/brainScreen.cpp) |

## Current Selector Entries

The auton selector currently exposes these routines:

1. `Negative 1`
2. `Positive 1`
3. `Tune Drive PID`
4. `Tune Turn PID`
5. `Example Drive`
6. `Example Swing`
7. `PID Calibration`
8. `Example Ramsete`
9. `Example LTV`
10. `Skills`
11. `None`

The example routines are intentional. They make this repo much easier to learn because they isolate one motion style at a time.

## Read Next

- [Autonomous Cookbook](AUTONOMOUS_COOKBOOK.md) for copyable patterns
- [Tutorials](TUTORIALS.md) for tuning and debugging workflows
- [API Reference](API_REFERENCE.md) for exact class-level details
