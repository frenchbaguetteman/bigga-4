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

### Driver-Control Autonomous Launch

Outside competition control, holding `Down + B` on the master controller launches the currently selected autonomous from `opcontrol()`.

Important limits:

- this is only for off-field testing
- it is ignored when the brain is connected to competition control
- normal competition autonomous still runs only through `autonomous()`

## Motion Primitives

### `DriveMoveCommand`

Source: [`include/commands/driveMove.h`](../include/commands/driveMove.h)

Purpose:

- drive to a target `x, y` point with forward or reverse approach
- support hold-heading distance motion for EZ-style `pid_drive_set(...)`

How it works:

- distance error feeds `DISTANCE_PID`
- heading-to-target error feeds `TURN_PID`
- hold-heading mode uses signed projection on the locked heading axis
- point mode aims the front of the robot at the target for `fwd`
- point mode aims the rear of the robot at the target for `rev`
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

Current usage note:

- this command still exists in the repo, but the currently selectable autonomous routines now use EZ-style chained point moves instead of RAMSETE paths

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

Command sequence in chassis-facing inches:

1. Spin intake at `127`.
2. Chain three forward odom points:
   - `(-47.24, -23.62)`
   - `(-23.62, -23.62)`
   - `(0.0, -11.81)`
3. Wait `0.3 s`.
4. Outtake at `-127` for `0.5 s`.
5. Turn to EZ absolute heading `180`.
6. Drive back to `(-47.24, -23.62)`.

### `Negative 2`

- identical to `Negative 1`

### `Positive 1`

Command sequence in chassis-facing inches:

1. Spin intake at `127`.
2. Chain three forward odom points:
   - `(47.24, -23.62)`
   - `(23.62, -23.62)`
   - `(0.0, -11.81)`
3. Wait `0.3 s`.
4. Outtake at `-127` for `0.5 s`.
5. Turn to EZ absolute heading `0`.
6. Drive back to `(47.24, -23.62)`.

### `Positive 2`

- identical to `Positive 1`

### `Example Move`

Command sequence:

1. Drive forward `24 in` with `pid_drive_set(24.0, 110)`.
2. Drive to a robot-relative diagonal point `24 in` forward and `18 in` left.
3. Drive back to the starting point.

This routine is the simplest place to verify:

- forward distance motion
- point-to-point odom moves
- return-to-start behavior

### `Example Turn`

Command sequence:

1. Turn to EZ absolute heading `90`.
2. Turn to EZ absolute heading `-90`.
3. Turn to EZ absolute heading `180`.
4. Turn to EZ absolute heading `0`.

### `Example Path`

Command sequence:

1. Spin intake at `96`.
2. Chain three forward odom points relative to the current pose:
   - `18 in` forward, `10 in` left
   - `36 in` forward, `8 in` right
   - `48 in` forward, `0 in` left/right
3. Outtake at `-127` for `0.3 s`.
4. Return to the starting point.

### `Skills`

Command sequence in chassis-facing inches:

1. Spin intake at `127`.
2. Drive to `(-11.81, -55.12)`.
3. Outtake at `-127` for `0.4 s`.
4. Drive to `(-11.81, 0.0)`.
5. Drive to `(11.81, 0.0)`.
6. Outtake at `-127` for `0.4 s`.
7. Run the placeholder lift cycle.
8. Drive to `(47.24, 39.37)`.
9. Outtake at `-127` for `0.5 s`.
10. Drive back to `(0.0, 0.0)`.

Important skills-specific notes:

- The lift step currently acts like a logical placeholder because lift hardware is stubbed.

### `None`

Builder behavior:

- returns an empty `InstantCommand`

Use this when you want a no-op autonomous slot.

## Autonomous Ownership and Interruptions

Official competition autonomous still runs through `autonomous()` in [`src/main.cpp`](../src/main.cpp).

Off-field testing behavior:

- holding `Down + B` in `opcontrol()` launches the selected autonomous once
- the launcher is ignored when the brain is connected to competition control
- there is no longer a special `Skills` auto-run path inside `opcontrol()`

All drivetrain motion commands declare drivetrain as a requirement. That means:

- only one drivetrain motion command runs at once
- driver control only resumes when the drivetrain is no longer owned
- any command ending or being canceled stops the drivetrain

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
| `SequentialCommandGroup` over chained odom points | final segment finishes | drivetrain stop |
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
