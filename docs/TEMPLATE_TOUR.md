# Template Tour

This page explains how the project is put together and how it maps to the mental model you may already have from PROS or EZ-Template.

## Mental Model

Think of the robot as four layers:

1. `main.cpp` owns the competition lifecycle.
2. Subsystems wrap hardware.
3. Commands express robot actions.
4. Autonomous builders compose commands into full routines.

If you only remember one thing, remember this: you usually do not write motor code directly inside `autonomous()`. You build commands and schedule them.

## Competition Lifecycle

The top-level flow lives in [`src/main.cpp`](../src/main.cpp):

```cpp
void initialize() {
    BrainScreen::initialize();
    AutonSelector::selectAuton(DEFAULT_AUTON_SELECTION);
    AutonSelector::selectAlliance(DEFAULT_ALLIANCE_SELECTION);
    subsystemInit();
    drivetrain->calibrateImu();
    localizationInit();
    buildAutonCommand();
    AutonSelector::init();
    ScreenManagerUI::init();
}

void autonomous() {
    buildAutonCommand();
    CommandScheduler::schedule(autonCommand.get());
}

void opcontrol() {
    CommandScheduler::reset();
    // direct controller polling and driverArcade live here
}
```

That is the main pattern throughout the repo:

- `initialize()` prepares long-lived state
- `autonomous()` builds and schedules a fresh command graph
- `opcontrol()` handles controller input and manual driving

## Command-Based Architecture

### A command is one robot action

Examples from this repo:

- `DriveMoveCommand`: drive to an `(x, y)` target
- `RotateCommand`: turn to a heading
- `RamseteCommand`: follow a time-parameterized path
- `LtvUnicycleCommand`: follow a time-parameterized path with velocity-varying gains
- `IntakeSpinCommand`: run the intake until cancelled

Commands declare which subsystem they own, so the scheduler can prevent conflicts.

### Commands compose into larger routines

The builder helpers on `Command` let you chain behaviors:

```cpp
new RotateCommand(&ctx.drivetrain, kPi, ctx.poseSource);

(new RamseteCommand(&ctx.drivetrain, profile, ctx.poseSource))
    ->alongWith(new IntakeSpinCommand(&ctx.intakes, 127));
```

And command groups let you build multi-step routines:

```cpp
return std::make_unique<SequentialCommandGroup>(std::vector<Command*>{
    new DriveMoveCommand(&ctx.drivetrain, forward, ctx.poseSource),
    new WaitCommand(0.2f),
    new DriveMoveCommand(&ctx.drivetrain, home, ctx.poseSource),
});
```

## How This Maps From EZ-Template

If you are used to EZ-Template, these are the closest equivalents in this repo:

| EZ-Template style idea | This repo |
|---|---|
| Global chassis object | `Drivetrain` subsystem |
| `pid_drive_set(...)` | `DriveMoveCommand` |
| `pid_turn_set(...)` | `RotateCommand` |
| Odom/path movement | `RamseteCommand` or `LtvUnicycleCommand` + `MotionProfile` |
| Auton selector pages | `AutonSelector` + `BrainScreen` |
| Simple helper functions for auton | `shared::*` helpers in `include/autonomous/sharedCommands.h` |

The big difference is composition. Instead of one big procedural auton function, this repo prefers small commands combined with `SequentialCommandGroup` and `ParallelCommandGroup`.

## Autonomous Build Context

Autonomous code receives a single context object:

```cpp
struct AutonBuildContext {
    Drivetrain& drivetrain;
    Intakes& intakes;
    Lift& lift;
    std::function<Eigen::Vector3f()> poseSource;
};
```

That keeps auton builders clean:

- the drivetrain and mechanisms are already injected
- localization is exposed through `poseSource`
- `poseSource` is already guarded against sudden fusion snaps before motion code sees it
- builders return a new top-level `Command`

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
| Add shared auton helpers | [`include/autonomous/sharedCommands.h`](../include/autonomous/sharedCommands.h) |
| Tune PID / RAMSETE / ports | [`include/config.h`](../include/config.h) |
| Change driver controls | [`src/main.cpp`](../src/main.cpp) |
| Change brain-screen behavior | [`src/ui/brainScreen.cpp`](../src/ui/brainScreen.cpp) |

## Current Selector Entries

The auton selector currently exposes these routines:

1. `Negative 1`
2. `Positive 1`
3. `Example Move`
4. `Example Turn`
5. `Example Path`
6. `Example LTV`
7. `Skills`
8. `None`

The example routines are intentional. They make this repo much easier to learn because they isolate one motion style at a time.

## Read Next

- [Autonomous Cookbook](AUTONOMOUS_COOKBOOK.md) for copyable patterns
- [Tutorials](TUTORIALS.md) for tuning and debugging workflows
- [API Reference](API_REFERENCE.md) for exact class-level details
