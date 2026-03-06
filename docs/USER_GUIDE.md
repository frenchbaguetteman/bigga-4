# 69580A User Guide

This guide is the operator and team-facing overview for the current robot code in this repository. It explains what the robot does today, how to drive it, how autonomous selection works, how localization is fused, and which implementation caveats matter on competition day.

For deeper references, start with:

- [docs/README.md](README.md)
- [docs/MOTION_REFERENCE.md](MOTION_REFERENCE.md)
- [docs/TUTORIALS.md](TUTORIALS.md)
- [docs/API_REFERENCE.md](API_REFERENCE.md)

## What This Codebase Runs

The robot is a command-based PROS V5 project with these active systems:

- 6-motor tank drivetrain with teleop arcade control
- 2-motor intake
- 4 pneumatic outputs: `select1`, `select2`, `tongue`, `wing`
- IMU-based heading
- Odometry using a horizontal tracking wheel plus drive-encoder fallback for forward distance
- Monte Carlo localization (MCL) with GPS and four distance sensors
- Motion-profiled autonomous paths followed with RAMSETE
- Brain-screen UI for auton selection and localization diagnostics

Current non-obvious implementation facts:

- The lift is stubbed out right now. `Lift` commands compile, but the subsystem is a no-op.
- `Negative 2` currently runs the same command graph as `Negative 1`.
- `Positive 2` currently runs the same command graph as `Positive 1`.
- Alliance selection is displayed in the UI, but autonomous building does not currently branch on alliance.
- If `Skills` is selected, the skills autonomous is automatically scheduled inside `opcontrol()`.

## Competition-Day Quick Start

1. Power on the brain with the robot flat and still.
2. Wait for IMU calibration and localization startup to finish.
3. Confirm the controller rumbles once and the screen reaches `Ready`.
4. On the brain screen, select the desired autonomous and alliance.
5. Check the pose pages if localization looks suspicious.
6. Enter autonomous or driver control.

If GPS never stabilizes during boot, the robot falls back to the configured start pose in [`include/config.h`](../include/config.h).

## Operator Controls

### Driver Control

The code in [`src/main.cpp`](../src/main.cpp) binds the master controller like this:

| Control | Action | Notes |
|---|---|---|
| Left stick `Y` | Forward / reverse drive | Passed through joystick shaping |
| Right stick `X` | Turn | Passed through joystick shaping |
| `R1` held | Intake in | Runs intake at `127` |
| `R2` held | Intake out | Runs intake at `-127` |
| `L1` press | Toggle tongue pneumatic | Edge-triggered |
| `L2` press | Toggle wing pneumatic | Edge-triggered |
| `A` press | Toggle `select1` pneumatic | Edge-triggered |
| `B` press | Toggle `select2` pneumatic | Edge-triggered |
| Partner `Right` | Cancel running skills auton | Only when `Skills` is selected and active |

### Driver Feel

Teleop driving uses `Drivetrain::driverArcade()`, not raw stick passthrough. That means:

- A joystick deadband is applied.
- Forward and turn both use exponential response shaping.
- When the sticks return to center, soft active braking can hold the last wheel position briefly.

Current driver tuning values in [`include/config.h`](../include/config.h):

| Constant | Value |
|---|---|
| `DRIVER_JOYSTICK_DEADBAND` | `5.0` |
| `DRIVER_FORWARD_CURVE_T` | `5.0` |
| `DRIVER_TURN_CURVE_T` | `5.0` |
| `DRIVER_ACTIVE_BRAKE_ENABLED` | `true` |
| `DRIVER_ACTIVE_BRAKE_POWER` | `6.0` |
| `DRIVER_ACTIVE_BRAKE_KP` | `0.08` |

## Brain Screen

The brain screen is a real operator tool, not just a splash page. It is implemented by [`src/ui/brainScreen.cpp`](../src/ui/brainScreen.cpp), [`src/ui/screenManager.cpp`](../src/ui/screenManager.cpp), and related UI files.

### Startup Screen

During `initialize()` the screen shows:

- current init stage
- boot progress percentage
- GPS polling progress during localization startup

### Runtime Pages

The top bar exposes these pages:

- `SELECT`: autonomous and alliance selection, plus live fused pose
- `ODOM`: field map and pose cards for odom, MCL, GPS, and combined pose
- `PID`: live position and heading delta graph
- `PATH`: current routine summary and current fused pose
- `GPS`: raw GPS pose and fused-vs-GPS drift

### Selector Touch Controls

On the `SELECT` page:

- tap `RED` or `BLUE` to set alliance
- tap `PREV` or `NEXT` to cycle routines
- tap `QUICK: SKILLS` to jump to `Skills`

The ordered autonomous list comes from [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp):

1. `Negative 1`
2. `Negative 2`
3. `Positive 1`
4. `Positive 2`
5. `Skills`
6. `None`

## Hardware Map

These values are taken from the live config in [`include/config.h`](../include/config.h).

### Motors and Sensors

| Hardware | Ports / channels | Notes |
|---|---|---|
| Left drive | `-11`, `-15`, `-14` | Negative means reversed |
| Right drive | `18`, `17`, `20` | Blue cartridges |
| Intake | `-6`, `8` | Linked motor group |
| IMU | `13` | Used for heading |
| Vertical tracking | `0` | Disabled, so forward odom falls back to drive encoders |
| Horizontal tracking | `16` | Enabled |
| Left distance | `2` | Facing left |
| Right distance | `5` | Facing right |
| Back distance | `4` | Facing back |
| Front distance | `1` | Facing forward |
| GPS | `3` | Used in startup and runtime fusion |

### Pneumatics

| Output | ADI port |
|---|---|
| `select1` | `A` |
| `select2` | `B` |
| `tongue` | `C` |
| `wing` | `D` |

## Robot Lifecycle

The main competition flow lives in [`src/main.cpp`](../src/main.cpp).

### `initialize()`

`initialize()` performs these steps in order:

1. Initializes the brain screen
2. Applies default auton and alliance selections from [`include/auton.h`](../include/auton.h)
3. Constructs subsystems
4. Calibrates the IMU
5. Builds localization sensor models
6. Acquires a startup pose
7. Synchronizes odometry to that pose
8. Builds the currently selected autonomous command graph
9. Starts the screen task and scheduler task

When initialization is complete, the master controller rumbles once.

### `autonomous()`

`autonomous()` rebuilds the auton command from the current selector state, then schedules it.

### `opcontrol()`

`opcontrol()`:

- resets the command scheduler
- cancels any leftover autonomous command
- installs controller trigger bindings
- runs shaped arcade drive whenever the drivetrain is not owned by another command

Special case: if the selected routine is `Skills`, the auton command is scheduled immediately in driver control. This is intended for programming-skills use.

## Autonomous Selection and Slot Defaults

The file [`include/auton.h`](../include/auton.h) defines the default auton/alliance baked into the current build.

The script [`uploadAllAutons.py`](../uploadAllAutons.py) rewrites that header repeatedly and uploads:

| Slot | Default routine |
|---|---|
| `1` | `Negative 1` |
| `2` | `Negative 2` |
| `3` | `Positive 1` |
| `4` | `Positive 2` |
| `5` | `Skills` |

After it finishes, the script restores the header to `Negative 1` / `RED`.

## Localization and Pose Fusion

This project intentionally separates several pose sources:

- odometry-only pose from the drivetrain
- MCL pose from the particle filter
- GPS pose when the GPS reading is valid
- combined pose used by autonomous controllers

### Coordinate Convention

Internal motion uses one canonical frame:

- position in meters
- heading in radians
- `+X` is east / field-forward
- `+Y` is north / field-left
- heading positive is counterclockwise

Human-entered start headings stay in VEX compass convention and are converted at the config boundary.

### Startup Pose Mode

Current startup mode:

- `STARTUP_POSE_MODE = GPSXYPlusIMUHeading`

That means:

- startup `x` and `y` come from GPS once GPS stabilizes
- startup heading comes from the IMU, seeded using the configured start heading

If GPS does not stabilize in time, the code falls back to:

- `START_POSE_X_in`
- `START_POSE_Y_in`
- `START_POSE_THETA_deg`

### Runtime Fusion

The combined pose used by path followers is built like this:

- when the robot is still, bounded GPS corrections can pull odometry toward GPS
- when the robot is moving, bounded MCL corrections can pull odometry toward the particle-filter estimate
- heading remains the odometry/IMU heading

This design keeps controller behavior smooth while still letting absolute sensors correct drift.

For design rationale and regression procedures, see:

- [docs/LOCALIZATION_DESIGN_RATIONALE.md](LOCALIZATION_DESIGN_RATIONALE.md)
- [docs/LOCALIZATION_REFACTORING_SUMMARY.md](LOCALIZATION_REFACTORING_SUMMARY.md)
- [docs/LOCALIZATION_REGRESSION_TESTS.md](LOCALIZATION_REGRESSION_TESTS.md)

## Motion System Summary

The robot has three main autonomous motion primitives:

- `DriveMoveCommand`: point-to-point PID drive
- `RotateCommand`: PID turn-in-place
- `RamseteCommand`: profile follower over a `MotionProfile`

The current autonomous routines are all assembled from those primitives. Detailed step-by-step motion documentation lives in [docs/MOTION_REFERENCE.md](MOTION_REFERENCE.md).

## Build and Deploy

### Build

```bash
make clean
make
```

### Upload a Single Build

Use the PROS command your local setup supports. The repo automation script uses:

```bash
pros mu --slot 1 --name "Negative 1" --no-analytics
```

### Upload All Preassigned Slots

```bash
python3 uploadAllAutons.py
```

Dry run:

```bash
python3 uploadAllAutons.py --dry
```

## Tuning Knobs That Matter Most

The highest-impact constants for day-to-day work are:

### Driver Feel

- `DRIVER_*`

### Point Turns and Drive-to-Point

- `TURN_PID`
- `DISTANCE_PID`

### RAMSETE Tracking

- `RAMSETE_ZETA`
- `RAMSETE_BETA`

### Localization Confidence

- `MCL_*_DISTANCE_WEIGHT`
- `MCL_DISTANCE_STDDEV_in`
- `GPS_STDDEV_*`
- `GPS_ERROR_THRESHOLD_in`
- `LOC_GPS_*`
- `LOC_MCL_*`

### Startup Reliability

- `STARTUP_POSE_MODE`
- `STARTUP_GPS_MAX_WAIT_ms`
- `STARTUP_GPS_READY_ERROR_in`
- `STARTUP_GPS_STABLE_SAMPLES`

## Current Caveats and Risks

These are worth knowing before you trust the robot blindly:

- The vertical tracking wheel is disabled, so forward odometry currently uses drive encoder fallback.
- The lift is not implemented in hardware, so skills steps that call `liftCycle()` do not move a real mechanism right now.
- `Negative 2` and `Positive 2` are not unique routines yet.
- Alliance does not currently change path generation or scoring logic.
- If localization becomes non-finite, the code has several safety fallbacks, but you should still inspect the ODOM and GPS pages before a match.

## Troubleshooting

### Robot boots but pose is wrong

- Verify the GPS strip orientation and GPS error.
- Confirm the configured field frame matches the actual field.
- Check that the robot was motionless during IMU calibration.
- Review the startup-pose section in [`include/config.h`](../include/config.h).

### Robot feels too jumpy in driver control

- Reduce `DRIVER_FORWARD_CURVE_T`
- Reduce `DRIVER_TURN_CURVE_T`
- Lower `DRIVER_ACTIVE_BRAKE_POWER`

### Robot undershoots or overshoots point moves

- Re-tune `DISTANCE_PID`
- Confirm the pose source is stable on the ODOM and GPS tabs

### Robot turns the wrong amount

- Re-tune `TURN_PID`
- Verify IMU heading is sane after startup
- Confirm the heading convention is understood before changing config values

### Autonomous follows the wrong routine

- Check the selected routine on the brain screen
- Check the slot defaults in [`include/auton.h`](../include/auton.h)
- If using uploaded slots, confirm `uploadAllAutons.py` was run successfully

## Where To Go Next

- For exact motion-by-motion behavior, read [docs/MOTION_REFERENCE.md](MOTION_REFERENCE.md).
- For step-by-step team workflows, read [docs/TUTORIALS.md](TUTORIALS.md).
- For class-level interfaces, read [docs/API_REFERENCE.md](API_REFERENCE.md).
