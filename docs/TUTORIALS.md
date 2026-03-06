# Tutorials

This file is a workflow guide for students and developers working on the robot. Each tutorial is written against the current implementation in this repository.

## Tutorial 1: Boot the Robot and Verify It Is Healthy

Goal:

- confirm the robot can initialize, localize, and accept driver input

Steps:

1. Place the robot on the field or a flat surface without moving it.
2. Power on the brain.
3. Watch the startup screen progress through subsystem setup, IMU calibration, and localization.
4. Wait for the controller rumble.
5. Open the `SELECT` page and confirm the robot reports `Ready`.
6. Open the `ODOM` page and check that `ODOM`, `MCL`, `GPS`, and `COMBINED` are at least roughly consistent.
7. Slightly push the robot by hand and confirm the displayed pose changes sensibly.

What to do if it fails:

- If startup stalls on GPS, review the GPS strip and [`include/config.h`](../include/config.h) startup settings.
- If heading is obviously wrong, reboot and keep the robot still during IMU calibration.
- If only GPS looks wrong but odom is sane, inspect the `GPS` page for drift.

## Tutorial 2: Select and Dry-Run an Autonomous

Goal:

- verify you can pick the correct autonomous routine before a match

Steps:

1. Boot the robot and wait for `Ready`.
2. On the `SELECT` page, tap `PREV`, `NEXT`, or `QUICK: SKILLS` until the desired routine is highlighted.
3. Tap `RED` or `BLUE` to set the displayed alliance.
4. Confirm the large routine label is what you expect.
5. Open the `PATH` page and verify the current routine summary.

Important current caveats:

- `Negative 2` is the same code as `Negative 1`.
- `Positive 2` is the same code as `Positive 1`.
- Alliance selection is currently UI metadata only and does not change the built auton.

## Tutorial 3: Practice Driver Control

Goal:

- understand how the current teleop mappings and driver shaping feel

Steps:

1. Enter driver control.
2. Move the left stick `Y` slowly and observe the shaped forward response.
3. Move the right stick `X` slowly and observe the shaped turn response.
4. Release both sticks and feel the soft active brake behavior.
5. Hold `R1` to intake.
6. Hold `R2` to reverse the intake.
7. Press `L1`, `L2`, `A`, and `B` one at a time to verify each pneumatic toggle.

If the robot feels too aggressive:

- lower `DRIVER_FORWARD_CURVE_T`
- lower `DRIVER_TURN_CURVE_T`
- reduce `DRIVER_ACTIVE_BRAKE_POWER`

Those constants live in [`include/config.h`](../include/config.h).

## Tutorial 4: Tune `DriveMoveCommand`

Goal:

- improve point-to-point driving behavior

Relevant code:

- [`include/commands/driveMove.h`](../include/commands/driveMove.h)
- [`include/config.h`](../include/config.h)

What this command does:

- turns toward a target point
- drives toward it using `DISTANCE_PID`
- corrects heading using `TURN_PID`

Suggested process:

1. Create or temporarily schedule a simple `DriveMoveCommand` to a nearby point.
2. Start with a short straight move.
3. Watch for undershoot, overshoot, and oscillation.
4. Adjust `DISTANCE_PID.kP` first.
5. Add or adjust `kD` only if the motion overshoots or rings.
6. Re-check `TURN_PID` if the robot arcs incorrectly toward the target.

Current default gains:

- `DISTANCE_PID = {5.0, 0.0, 0.3, 0.0}`
- `TURN_PID = {2.0, 0.0, 0.15, 0.0}`

## Tutorial 5: Tune `RotateCommand`

Goal:

- improve turn-in-place accuracy

Relevant code:

- [`include/commands/rotate.h`](../include/commands/rotate.h)
- [`include/config.h`](../include/config.h)

Suggested process:

1. Run a simple turn such as `0 -> pi / 2`.
2. Check whether the robot stops short, overshoots, or oscillates.
3. Increase `TURN_PID.kP` if it is too lazy.
4. Increase `TURN_PID.kD` if it overshoots.
5. Verify IMU heading is stable before blaming the PID.

## Tutorial 6: Tune RAMSETE Path Following

Goal:

- make profiled autonomous path tracking smoother and more accurate

Relevant code:

- [`include/commands/ramsete.h`](../include/commands/ramsete.h)
- [`include/motionProfiling/motionProfile.h`](../include/motionProfiling/motionProfile.h)
- [`include/config.h`](../include/config.h)

What matters most:

- path geometry
- profile max velocity and acceleration
- `RAMSETE_ZETA`
- `RAMSETE_BETA`

Suggested process:

1. Start with a short two-point or three-point path.
2. Lower velocity and acceleration first if the robot looks saturated.
3. If tracking is loose, increase `RAMSETE_BETA` carefully.
4. If tracking becomes too twitchy, back off `RAMSETE_BETA` or increase damping with `RAMSETE_ZETA`.
5. Verify the fused pose is stable before over-tuning the controller.

## Tutorial 7: Add or Edit an Autonomous Routine

Goal:

- create a new auton or make one of the placeholder slots real

Relevant files:

- [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp)
- [`include/autonomous/autons.h`](../include/autonomous/autons.h)
- [`include/autonomous/sharedCommands.h`](../include/autonomous/sharedCommands.h)

Current pattern:

1. Build a `MotionProfile` from waypoints.
2. Wrap it in `RamseteCommand`.
3. Chain actions in a `SequentialCommandGroup`.

Example workflow:

1. Open [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp).
2. Find `makeNegative1()`, `makePositive1()`, or `makeSkills()`.
3. Copy the closest routine as a starting point.
4. Replace waypoint coordinates or command order.
5. If you want a real `Negative 2`, point `Auton::NEGATIVE_2` at a new builder instead of `makeNegative1(ctx)`.
6. Rebuild and test.

Checklist when editing:

- keep the coordinate frame consistent
- verify headings match the path direction you want
- confirm subsystem requirements do not conflict unexpectedly
- remember that lift actions are currently no-op

## Tutorial 8: Build a New Motion Profile

Goal:

- create a path with your own waypoints

If you are editing [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp), the file already contains a private helper named `buildProfile()` that you can reuse there.

Minimal example pattern inside that file:

```cpp
MotionProfile profile = buildProfile({
    Eigen::Vector3f(0.0f, 0.0f, 0.0f),
    Eigen::Vector3f(0.5f, 0.0f, 0.0f),
    Eigen::Vector3f(1.0f, 0.3f, 0.4f),
}, 1.2f, 2.0f);
```

If you want to build a profile somewhere else in the codebase, do not copy this call blindly. The helper is local to `autons.cpp`, so outside that file you need to either:

- build `Path` and `TrapezoidalVelocityProfile` manually
- or move/add a shared helper in a header/source pair

How to reason about the numbers:

- `x` and `y` are field positions in meters
- `theta` is desired heading in radians at that waypoint
- the last two arguments are maximum velocity and acceleration

If you want a straight segment, set headings to the segment direction.

## Tutorial 9: Use the Bezier / JSON Path Helpers

Goal:

- use the more advanced motion-profile helpers already present in the repo

Relevant files:

- [`include/motionProfiling/bezier.h`](../include/motionProfiling/bezier.h)
- [`src/motionProfiling/bezierMotionProfiling.cpp`](../src/motionProfiling/bezierMotionProfiling.cpp)
- [`static/n_1_6_path1.json`](../static/n_1_6_path1.json)
- [`static/p_4_path1.json`](../static/p_4_path1.json)

Capabilities already in the code:

- sample a sequence of Bezier segments into a `Path`
- build a profile from JSON with points and constraints
- set nonzero initial and end speeds

Important implementation detail:

- these helper functions currently live only in [`src/motionProfiling/bezierMotionProfiling.cpp`](../src/motionProfiling/bezierMotionProfiling.cpp)
- they are not declared in a public header yet

This is a good path if you want smoother authored curves than the current straight/interpolated waypoint profiles, but you will need to expose those helpers before other files can call them cleanly.

## Tutorial 10: Change the Default Slot Autonomous

Goal:

- control what routine a build boots into by default

Relevant file:

- [`include/auton.h`](../include/auton.h)

Manual method:

1. Edit `SELECTED_AUTON`.
2. Edit `DEFAULT_ALLIANCE` if needed.
3. Rebuild and upload.

Automated method:

```bash
python3 uploadAllAutons.py
```

That script rewrites the header for each slot before uploading.

## Tutorial 11: Upload Every Preassigned Autonomous Slot

Goal:

- quickly refresh the brain with the full slot layout used by the team

Steps:

1. Connect the robot to your machine.
2. Run:

```bash
python3 uploadAllAutons.py
```

3. Wait while the script writes `include/auton.h` repeatedly and uploads slots `1` through `5`.
4. Confirm the script exits cleanly.

Dry-run mode:

```bash
python3 uploadAllAutons.py --dry
```

Use dry-run if you want to inspect the exact slot mapping first.

## Tutorial 12: Debug Localization Drift

Goal:

- determine whether bad autonomous behavior is really a localization problem

Suggested workflow:

1. Boot the robot and open the `ODOM` page.
2. Compare `ODOM`, `MCL`, `GPS`, and `COMBINED`.
3. Open the `GPS` page and watch fused-vs-GPS drift.
4. Drive short straight lines and short turns.
5. Check whether odom diverges, GPS diverges, or only combined fusion looks wrong.

Good follow-up docs:

- [docs/LOCALIZATION_DESIGN_RATIONALE.md](LOCALIZATION_DESIGN_RATIONALE.md)
- [docs/LOCALIZATION_REGRESSION_TESTS.md](LOCALIZATION_REGRESSION_TESTS.md)

## Tutorial 13: Understand Why Skills Auto-Starts in Driver Control

Goal:

- avoid accidental confusion during skills practice

Current behavior in [`src/main.cpp`](../src/main.cpp):

- if the selected auton is `Skills`, `opcontrol()` immediately schedules the auton command
- the partner controller can cancel it with `DIGITAL_RIGHT`

Practical advice:

- select `None` or a match routine before entering ordinary driver practice
- select `Skills` only when you actually want the autonomous sequence to begin in driver control
