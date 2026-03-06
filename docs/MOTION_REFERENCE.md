# Motion Reference

This document describes the robot's implemented motion behavior as it exists in code today. It covers teleop movement, command-level motion primitives, and every autonomous sequence currently built by [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp).

## Coordinate and Heading Conventions

All autonomous motion code uses the internal field frame defined in [`include/config.h`](../include/config.h):

- distance units: meters
- heading units: radians
- `+X`: east / field-forward
- `+Y`: north / field-left
- positive heading: counterclockwise

Examples:

- heading `0` means facing `+X`
- heading `pi / 2` means facing `+Y`
- heading `pi` means facing `-X`

## Teleop Motion

### Drivetrain

Teleop drive runs through `Drivetrain::driverArcade()` in [`src/subsystems/drivetrain.cpp`](../src/subsystems/drivetrain.cpp).

The behavior is:

1. Read left-stick `Y` as forward and right-stick `X` as turn.
2. Apply deadband and joystick shaping.
3. Convert the shaped values to arcade left/right voltage.
4. When the sticks go neutral, optionally hold wheel position with soft active braking.

If another command owns the drivetrain, manual drive is skipped until that command releases the subsystem.

### Intake

The intake has three operator motions:

- `R1` held: intake inward at `127`
- `R2` held: intake outward at `-127`
- neither held: stop

The intake command is requirement-owned, so it stops when released or interrupted.

### Pneumatics

Each pneumatic is a state toggle, not a timed pulse:

- `L1`: toggle `tongue`
- `L2`: toggle `wing`
- `A`: toggle `select1`
- `B`: toggle `select2`

## Motion Primitives

### `DriveMoveCommand`

Source: [`include/commands/driveMove.h`](../include/commands/driveMove.h)

Purpose:

- drive to a target `x, y` point
- continuously aim the robot at that point while moving

How it works:

- distance error feeds `DISTANCE_PID`
- heading-to-target error feeds `TURN_PID`
- outputs are clamped to `[-127, 127]`
- command finishes when the distance PID reports `atSetpoint()`

Default position tolerance:

- `0.02 m`

### `RotateCommand`

Source: [`include/commands/rotate.h`](../include/commands/rotate.h)

Purpose:

- turn in place to an absolute heading

How it works:

- heading error uses wrapped shortest-angle difference
- `TURN_PID` drives the turn output
- forward output is fixed at `0`
- finishes when the heading PID reports `atSetpoint()`

Default heading tolerance:

- `0.03 rad`

### `RamseteCommand`

Source: [`include/commands/ramsete.h`](../include/commands/ramsete.h)

Purpose:

- follow a full time-parameterized motion profile

Inputs:

- `MotionProfile`
- current fused pose source
- `RAMSETE_ZETA`
- `RAMSETE_BETA`

Behavior:

- samples the desired state at the current elapsed time
- computes pose error in robot frame
- combines profile feedforward with RAMSETE correction
- sends chassis speeds to drivetrain feedforward
- ends when profile time is complete

Current default gains:

- `RAMSETE_ZETA = 0.4`
- `RAMSETE_BETA = 45.0`

### `LtvUnicycleCommand`

Source: [`include/commands/ltvUnicycleController.h`](../include/commands/ltvUnicycleController.h)

This controller exists in the repo but is not used by the currently built autons.

## Motion Profile Construction

### Path Representation

Source: [`include/motionProfiling/path.h`](../include/motionProfiling/path.h)

A `Path` is a sequence of waypoints:

- `x`
- `y`
- `theta`

Interpolation is piecewise linear in position and heading.

### `MotionProfile`

Source: [`include/motionProfiling/motionProfile.h`](../include/motionProfiling/motionProfile.h)

`MotionProfile` combines:

- a spatial path
- a trapezoidal velocity profile over total arc length

The profile is pre-sampled into pose and velocity states, then interpolated at runtime.

### Bezier Helpers

Sources:

- [`include/motionProfiling/bezier.h`](../include/motionProfiling/bezier.h)
- [`src/motionProfiling/bezierMotionProfiling.cpp`](../src/motionProfiling/bezierMotionProfiling.cpp)

The repo also includes helpers to:

- evaluate cubic Bezier segments
- estimate curvature and arc length
- sample Bezier splines into a `Path`
- build a `MotionProfile` from Bezier data or a JSON asset

The current autonomous routines do not use the checked-in JSON assets directly, and these helper builders are not currently exposed by a public header. If you want to use them in production code, add a declaration in an appropriate header or move the helper into the file where you need it.

## Shared Autonomous Actions

Source: [`include/autonomous/sharedCommands.h`](../include/autonomous/sharedCommands.h)

Reusable autonomous snippets:

- `driveAndIntake()`: `DriveMoveCommand` in parallel with intake-in
- `outtakeTimed()`: intake-out for a fixed duration
- `liftCycle()`: move lift up, wait `0.15 s`, move lift down
- toggle helpers for each pneumatic

Important:

- `liftCycle()` currently does not move hardware because `Lift` is stubbed.

## Autonomous Routine Catalog

Available routines are declared in [`include/autonomous/autons.h`](../include/autonomous/autons.h) and ordered by [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp).

### `Negative 1`

Builder: `makeNegative1()`

Profile waypoints:

| Step | Pose |
|---|---|
| 1 | `(-1.2, -0.6, 0.0)` |
| 2 | `(-0.6, -0.6, 0.0)` |
| 3 | `(0.0, -0.3, 0.5)` |

Profile limits:

- max velocity: `1.2 m/s`
- max acceleration: `2.0 m/s^2`

Command sequence:

1. Follow the three-point RAMSETE path while intaking at `127`.
2. Wait `0.3 s`.
3. Outtake for `0.5 s`.
4. Rotate to absolute heading `pi`.
5. Drive to point `(-1.2, -0.6)`.

Interpretation:

- the robot advances along a shallow curved path toward center-left field space
- scores or releases using the outtake
- turns around
- returns toward its original negative-side area

### `Negative 2`

Builder mapping:

- `Auton::NEGATIVE_2 -> makeNegative1(ctx)`

Current behavior:

- identical to `Negative 1`

There is no unique second negative-side routine yet.

### `Positive 1`

Builder: `makePositive1()`

Profile waypoints:

| Step | Pose |
|---|---|
| 1 | `(1.2, -0.6, pi)` |
| 2 | `(0.6, -0.6, pi)` |
| 3 | `(0.0, -0.3, pi - 0.5)` |

Profile limits:

- max velocity: `1.2 m/s`
- max acceleration: `2.0 m/s^2`

Command sequence:

1. Follow the three-point RAMSETE path while intaking at `127`.
2. Wait `0.3 s`.
3. Outtake for `0.5 s`.
4. Rotate to absolute heading `0`.
5. Drive to point `(1.2, -0.6)`.

Interpretation:

- this is the mirrored positive-side counterpart to `Negative 1`
- it starts with headings facing `-X`
- it returns to a positive-side point after scoring

### `Positive 2`

Builder mapping:

- `Auton::POSITIVE_2 -> makePositive1(ctx)`

Current behavior:

- identical to `Positive 1`

There is no unique second positive-side routine yet.

### `Skills`

Builder: `makeSkills()`

The skills routine is a longer sequential command group built from short line segments plus helper actions.

Profile segment limits:

- every `makeSegment()` call uses `1.5 m/s`
- every `makeSegment()` call uses `2.5 m/s^2`

Command sequence:

1. Follow a straight RAMSETE segment from `(-1.4, -1.4)` to `(-0.3, -1.4)` while intaking.
2. Outtake for `0.4 s`.
3. Follow a straight RAMSETE segment from `(-0.3, -1.4)` to `(-0.3, 0.0)`.
4. Run `driveAndIntake()` to point `(0.3, 0.0)`.
5. Outtake for `0.4 s`.
6. Run `liftCycle(180.0, 0.0)`.
7. Follow a straight RAMSETE segment from `(0.3, 0.0)` to `(1.2, 1.0)`.
8. Outtake for `0.5 s`.
9. Drive to point `(0.0, 0.0)`.

Important skills-specific notes:

- In `opcontrol()`, if `Skills` is selected, this auton is auto-scheduled immediately.
- The partner controller `Right` button cancels it if it is still running.
- The lift step currently acts like a logical placeholder because lift hardware is stubbed.

### `None`

Builder behavior:

- returns an empty `InstantCommand`

Use this when you want a no-op autonomous slot.

## Autonomous Ownership and Interruptions

All motion commands declare drivetrain as a requirement. That means:

- only one drivetrain motion command runs at once
- driver control only resumes when the drivetrain is no longer owned
- any command ending or being canceled stops the drivetrain

This is especially relevant for:

- skills autonomous inside `opcontrol()`
- any future debugging command you schedule manually

## Pose Source Used By Motion Controllers

The path followers and point-motion commands do not read raw odometry directly. They use the shared `poseSource` lambda built in [`src/main.cpp`](../src/main.cpp).

That lambda returns:

- the fused `combinedPose` when finite
- otherwise the drivetrain odom pose

So when localization is healthy, all autonomous movement is driven against the fused pose, not pure wheel odom.

## Tolerances and Exit Behavior

| Motion | Exit rule | End behavior |
|---|---|---|
| `DriveMoveCommand` | distance PID at setpoint | drivetrain stop |
| `RotateCommand` | turn PID at setpoint | drivetrain stop |
| `RamseteCommand` | profile time elapsed | drivetrain stop |
| `IntakeSpinCommand` | never by itself | intake stop on interrupt |
| `IntakeTimedCommand` | fixed time elapsed | intake stop |

## Current Motion Gaps

These are documentation-worthy because they affect real match behavior:

- No unique `Negative 2` routine yet
- No unique `Positive 2` routine yet
- No alliance-specific branching in auton generation
- Lift motion is logical only until hardware is re-enabled
- Forward odometry uses drive-encoder fallback because the vertical tracking wheel is disabled

## Recommended Reading After This File

- [docs/USER_GUIDE.md](USER_GUIDE.md) for operations and startup
- [docs/TUTORIALS.md](TUTORIALS.md) for editing and tuning workflows
- [docs/API_REFERENCE.md](API_REFERENCE.md) for interfaces
