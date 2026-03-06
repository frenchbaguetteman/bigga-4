# Tutorials

These tutorials are task-first. Use them the way you would use EZ-Template tutorials: pick the exact thing you want to do, copy the pattern, then tune from there.

## Tutorial 1: Boot the Robot and Verify Bring-Up

**Goal:** confirm that startup, localization, and driver control all behave normally.

**Read first:** [User Guide](USER_GUIDE.md#competition-day-quick-start)

### What to do

1. Place the robot on a flat surface and do not move it during startup.
2. Power on the brain.
3. Watch the startup screen pass through subsystem init, IMU calibration, and localization init.
4. Wait for the controller rumble.
5. Open `SELECT` and confirm the runtime status is `Ready`.
6. Open `ODOM` and verify `ODOM`, `MCL`, `GPS`, and `COMBINED` are at least roughly consistent.

### Why it matters

Most autonomous issues are not autonomous issues. They are startup or localization issues that happen before your first command ever runs.

### If it fails

- If heading is wrong, reboot and keep the robot still during IMU calibration.
- If startup stalls on GPS, review the GPS startup settings in [`include/config.h`](../include/config.h).
- If GPS is wrong but odom is sane, debug the `GPS` page before retuning motion commands.

## Tutorial 2: Learn the Selector Using Example Routines

**Goal:** test the project's tutorial-grade autons before editing match routines.

### Routines to use

- `Example Move`
- `Example Turn`
- `Example Path`

These are intentionally included in [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp) so you can test one motion style at a time.

### Suggested order

1. Run `Example Move` to validate point driving.
2. Run `Example Turn` to validate heading control.
3. Run `Example Path` to validate profile following and intake parallelism.

### Why this is better than tuning on a scoring auton

You remove scoring timing, branching logic, and mechanism noise. If a simple example fails, the problem is in motion or localization, not game strategy.

## Tutorial 3: Tune `DriveMoveCommand`

**Goal:** improve point-to-point movement.

**Primary files:**

- [`include/commands/driveMove.h`](../include/commands/driveMove.h)
- [`include/config.h`](../include/config.h)

Core behavior:

```cpp
float driveOut = m_distPid.calculate(0.0f, dist);
float turnOut  = m_turnPid.calculate(
    0.0f,
    utils::angleDifference(targetAngle, pose.z()));
```

### Recommended process

1. Use `Example Move` or a temporary short `DriveMoveCommand`.
2. Start with a straight move to a nearby point.
3. Tune `DISTANCE_PID.kP` first.
4. Add or adjust `kD` only if it overshoots or rings.
5. Only touch `TURN_PID` if the robot arcs or yaws badly while driving.

### Verify

- The robot reaches the point cleanly.
- It does not oscillate near the target.
- It does not spin excessively on approach.

## Tutorial 4: Tune `RotateCommand`

**Goal:** improve turn-in-place behavior.

**Primary files:**

- [`include/commands/rotate.h`](../include/commands/rotate.h)
- [`include/config.h`](../include/config.h)

Use `Example Turn` as the test surface.

### Recommended process

1. Start with the built-in example sequence.
2. Increase `TURN_PID.kP` if the turn is lazy.
3. Increase `TURN_PID.kD` if the robot overshoots and snaps back.
4. Reboot and recheck IMU stability before making large PID changes.

## Tutorial 5: Tune `RamseteCommand`

**Goal:** improve path tracking quality.

**Primary files:**

- [`include/commands/ramsete.h`](../include/commands/ramsete.h)
- [`include/motionProfiling/motionProfile.h`](../include/motionProfiling/motionProfile.h)
- [`include/config.h`](../include/config.h)

The core control law reads the current pose from `poseSource`, samples the desired profile state, and sends `{v, omega}` to the drivetrain.

### Recommended process

1. Use `Example Path`.
2. Lower max velocity and acceleration before touching controller gains if the robot looks saturated.
3. Increase `RAMSETE_BETA` carefully if tracking is too loose.
4. Increase `RAMSETE_ZETA` if it needs more damping.
5. If the path still looks wrong, validate pose quality before doing more controller tuning.

## Tutorial 6: Create Your First Custom Auton

**Goal:** make a new routine without changing the whole architecture.

**Primary files:**

- [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp)
- [`include/autonomous/autons.h`](../include/autonomous/autons.h)
- [`include/autonomous/sharedCommands.h`](../include/autonomous/sharedCommands.h)

### Good starting point

Copy `makeExampleMove()`, `makeExampleTurn()`, or `makeExamplePath()` instead of copying `Skills`.

### Minimal workflow

1. Copy the closest `make...()` builder.
2. Rename it.
3. Edit the points, headings, or command order.
4. Add the enum entry.
5. Add the new case to `makeAutonCommand(...)`.
6. Add the new selector label through `kAvailableAutons` and `autonName()`.

For a more recipe-style version of this, read [Autonomous Cookbook](AUTONOMOUS_COOKBOOK.md).

## Tutorial 7: Build a New Motion Profile

**Goal:** create a new profiled path.

Inside [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp), reuse the local helper:

```cpp
MotionProfile profile = buildProfile({
    Eigen::Vector3f(0.0f, 0.0f, 0.0f),
    Eigen::Vector3f(0.5f, 0.0f, 0.0f),
    Eigen::Vector3f(1.0f, 0.3f, 0.4f),
}, 1.2f, 2.0f);
```

Interpretation:

- `x`, `y`: meters
- `theta`: radians
- final arguments: max velocity and max acceleration

If you need profile creation outside `autons.cpp`, move the helper into a shared header/source pair instead of duplicating it.

## Tutorial 8: Debug Localization Regressions

**Goal:** confirm whether bad auton behavior is actually coming from localization.

Use:

- [Localization Regression Tests](LOCALIZATION_REGRESSION_TESTS.md)
- [Localization Design Rationale](LOCALIZATION_DESIGN_RATIONALE.md)

Quick triage:

1. Compare pure odom, MCL, GPS, and combined pose on the brain screen.
2. Check whether the robot is still or moving when the error appears.
3. If only one pose source diverges, debug that source first.
4. Only retune motion once the combined pose is trustworthy.

## Tutorial 9: Change Defaults and Upload Slots

**Goal:** control which routine boots by default.

Manual path:

1. Edit the default auton and alliance values in [`include/auton.h`](../include/auton.h).
2. Rebuild and upload.

Batch upload path:

```bash
python3 uploadAllAutons.py
```

That script rewrites the default selections repeatedly so you can push multiple slot variants.
