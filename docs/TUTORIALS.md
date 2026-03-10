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

- `Example Drive`
- `Example Swing`
- `PID Calibration`
- `Example Ramsete`
- `Example LTV`

These are intentionally included in [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp) so you can test one motion style at a time.

### Suggested order

1. Run `Example Drive` to validate straight drive motion.
2. Run `Example Swing` to validate swing direction and side.
3. Run `PID Calibration` to validate heading control.
4. Run `Example Ramsete` to validate profile following.
5. Run `Example LTV` to compare LTV tracking on the same path shape.

### Why this is better than tuning on a scoring auton

You remove scoring timing, branching logic, and mechanism noise. If a simple example fails, the problem is in motion or localization, not game strategy.

## Tutorial 3: Tune EZ Drive Motion

**Goal:** improve point-to-point movement.

**Primary files:**

- [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp)
- [`include/EZ-Template/drive/drive.hpp`](../include/EZ-Template/drive/drive.hpp)
- [`include/config.h`](../include/config.h)

### Recommended process

1. Use `Example Drive`.
2. Start with a straight move to a nearby point.
3. Tune `DISTANCE_PID.kP` first.
4. Add or adjust `kD` only if it overshoots or rings.
5. Only touch `TURN_PID` if the robot arcs or yaws badly while driving.

### Verify

- The robot reaches the point cleanly.
- It does not oscillate near the target.
- It does not spin excessively on approach.

## Tutorial 4: Tune EZ Turning

**Goal:** improve turn-in-place behavior.

**Primary files:**

- [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp)
- [`include/EZ-Template/drive/drive.hpp`](../include/EZ-Template/drive/drive.hpp)
- [`include/config.h`](../include/config.h)

Use `PID Calibration` as the test surface.

### Recommended process

1. Start with the built-in example sequence.
2. Increase `TURN_PID.kP` if the turn is lazy.
3. Increase `TURN_PID.kD` if the robot overshoots and snaps back.
4. Reboot and recheck IMU stability before making large PID changes.

## Tutorial 5: Tune EZ RAMSETE

**Goal:** improve path tracking quality.

**Primary files:**

- [`include/EZ-Template/drive/drive.hpp`](../include/EZ-Template/drive/drive.hpp)
- [`src/EZ-Template/drive/tracked_modes.cpp`](../src/EZ-Template/drive/tracked_modes.cpp)
- [`include/config.h`](../include/config.h)

The live tracked path code now runs through `pid_odom_ramsete_set(...)`.

### Recommended process

1. Use `Example Ramsete`.
2. Lower max velocity and acceleration before touching controller gains if the robot looks saturated.
3. Increase `RAMSETE_BETA` carefully if tracking is too loose.
4. Increase `RAMSETE_ZETA` if it needs more damping.
5. If the path still looks wrong, validate pose quality before doing more controller tuning.

If you want a same-path controller comparison, run `Example LTV` after each change so you can separate path geometry issues from controller-specific issues.

## Tutorial 6: Create Your First Custom Auton

**Goal:** make a new routine without changing the whole architecture.

**Primary files:**

- [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp)
- [`include/autonomous/autons.h`](../include/autonomous/autons.h)

### Good starting point

Copy `runExampleMove()`, `runExampleTurn()`, or `runExampleRamsete()` instead of copying `Skills`.

### Minimal workflow

1. Copy the closest `run...()` routine.
2. Rename it.
3. Edit the EZ points, headings, or command order.
4. Add the enum entry.
5. Add the new selector label through `kAvailableAutons`.

For a more recipe-style version of this, read [Autonomous Cookbook](AUTONOMOUS_COOKBOOK.md).

## Tutorial 7: Build a New Tracked Path

**Goal:** create a new tracked path.

Inside [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp), follow the existing EZ tracked examples:

```cpp
ezDrive().pid_odom_ramsete_set(std::vector<ez::united_odom>{
    ezLocalMove(18.0, 0.0, ez::fwd, kDriveSpeed),
    ezLocalMove(30.0, -10.0, ez::fwd, kDriveSpeed),
    ezLocalMove(42.0, 0.0, ez::fwd, kDriveSpeed, 0 * okapi::degree),
}, true);
```

Interpretation:

- `forward`, `left`: inches in the local EZ frame
- the final heading is optional on the last point
- the final integer is the EZ speed cap

The live RAMSETE/LTV backend converts those EZ waypoints into an internal time-parameterized profile automatically.

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

That script rewrites the default selections repeatedly so you can push the three competition variants without hand-editing `include/auton.h`.
