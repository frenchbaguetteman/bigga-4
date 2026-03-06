# Autonomous Cookbook

This page is the "show me the pattern" version of the docs. Every example here is based on code that already exists in this repo.

## Start With the Built-In Examples

The easiest way to learn the auton stack is to read these in order inside [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp):

1. `makeExampleMove()`
2. `makeExampleTurn()`
3. `makeExamplePath()`
4. `makeNegative1()` or `makePositive1()`
5. `makeSkills()`

That order goes from simplest to most composed.

## Recipe: Drive to a Point

Use `DriveMoveCommand` when you want a fast point-to-point move without building a full path.

```cpp
const Eigen::Vector3f start = basePose(ctx);
const Eigen::Vector2f forward = robotRelativePoint(start, 24.0f * kInToM, 0.0f);
const Eigen::Vector2f home = start.head<2>();

return std::make_unique<SequentialCommandGroup>(std::vector<Command*>{
    new DriveMoveCommand(ctx.drivetrain, forward, ctx.poseSource),
    new WaitCommand(0.2f),
    new DriveMoveCommand(ctx.drivetrain, home, ctx.poseSource),
});
```

Use this for:

- quick bring-up tests
- short straight moves
- simple autonomous endgame repositioning

## Recipe: Turn in Place

Use `RotateCommand` when the robot should only change heading.

```cpp
const Eigen::Vector3f start = basePose(ctx);

return std::make_unique<SequentialCommandGroup>(std::vector<Command*>{
    new RotateCommand(ctx.drivetrain, wrapRadians(start.z() + 0.5f * kPi), ctx.poseSource),
    new WaitCommand(0.2f),
    new RotateCommand(ctx.drivetrain, start.z(), ctx.poseSource),
});
```

Use this for:

- isolated turn tuning
- aligning to field elements before another move
- demonstrating heading-control behavior to new programmers

## Recipe: Follow a Path

Use `RamseteCommand` when you want a shaped path with velocity profiling.

```cpp
MotionProfile profile = buildProfile({
    robotRelativePose(start, 0.0f * kInToM, 0.0f * kInToM, 0.0f),
    robotRelativePose(start, 18.0f * kInToM, 10.0f * kInToM, 0.35f),
    robotRelativePose(start, 36.0f * kInToM, -8.0f * kInToM, -0.25f),
    robotRelativePose(start, 48.0f * kInToM, 0.0f * kInToM, 0.0f),
}, 1.0f, 1.8f);

return std::make_unique<SequentialCommandGroup>(std::vector<Command*>{
    new RamseteCommand(ctx.drivetrain, profile, ctx.poseSource),
});
```

Use this for:

- smoother autonomous paths
- controlling both translation and heading together
- motion that should be easier to scale than chained point moves

## Recipe: Run a Mechanism at the Same Time

The command builders let you run commands in parallel:

```cpp
(new RamseteCommand(ctx.drivetrain, profile, ctx.poseSource))
    ->alongWith(new IntakeSpinCommand(ctx.intakes, 127));
```

That pattern is already used in the scoring autons and in `makeExamplePath()`.

## Recipe: Reuse Shared Helpers

The shared helper file exists so common action bundles stay readable:

```cpp
shared::driveAndIntake(
    ctx.drivetrain,
    ctx.intakes,
    Eigen::Vector2f(0.3f, 0.0f),
    ctx.poseSource);

shared::outtakeTimed(ctx.intakes, 0.4f);
shared::liftCycle(ctx.lift, 180.0f, 0.0f);
```

Add a helper when:

- you copy the same two- or three-command pattern into multiple autons
- the helper name explains intent better than the raw command list
- you want consistency across mirrored or variant routines

## Recipe: Add a New Auton Slot

To add a new builder:

1. Write a `make...()` function in [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp).
2. Add the enum entry in [`include/autonomous/autons.h`](../include/autonomous/autons.h).
3. Add it to `kAvailableAutons`.
4. Add a case to `autonName()`.
5. Add a case to `autonCommands::makeAutonCommand(...)`.

Minimal switch entry pattern:

```cpp
case Auton::EXAMPLE_PATH:
    return makeExamplePath(ctx);
```

If you only want to replace a placeholder, point `NEGATIVE_2` or `POSITIVE_2` at a new builder instead of the current alias.

## Recipe: Build a Motion Profile

Inside `autons.cpp`, the local helper is:

```cpp
MotionProfile buildProfile(std::initializer_list<Eigen::Vector3f> waypoints,
                           float maxVelocityMps,
                           float maxAccelerationMps2)
```

Example:

```cpp
MotionProfile profile = buildProfile({
    Eigen::Vector3f(-1.2f, -0.6f, 0.0f),
    Eigen::Vector3f(-0.6f, -0.6f, 0.0f),
    Eigen::Vector3f(0.0f, -0.3f, 0.5f),
}, 1.2f, 2.0f);
```

Remember:

- `x` and `y` are meters
- `theta` is radians
- the final two numbers are max velocity and max acceleration

## Recipe: Use the Example Routines for Tuning

The example autons are not filler. They are the fastest tuning surfaces in the repo:

- `Example Move` for `DriveMoveCommand`
- `Example Turn` for `RotateCommand`
- `Example Path` for RAMSETE and profile tuning

That is usually better than tuning against a full match auton, because you remove game logic and mechanism timing from the test.

## Common Mistakes

- Mixing inches and meters in waypoint math.
- Forgetting that headings are radians, not degrees.
- Tuning motion before checking whether localization is sane.
- Duplicating helper code instead of promoting it into `sharedCommands.h`.
- Editing a placeholder auton and forgetting that the selector may still point at the old builder.

## Read Next

- [Tutorials](TUTORIALS.md) for tuning workflows
- [API Reference](API_REFERENCE.md) for exact constructors and behavior
- [Motion Reference](MOTION_REFERENCE.md) for the currently implemented routine-by-routine behavior
