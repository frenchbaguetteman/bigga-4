# Autonomous and Motion

## Symbol Index

| Symbol | Header | Purpose |
|---|---|---|
| `Auton`, `Alliance` | [`include/autonomous/autons.h`](../include/autonomous/autons.h) | selector-facing routine identifiers |
| `AutonBuildContext` | [`include/autonomous/autonCommands.h`](../include/autonomous/autonCommands.h) | injected dependencies for auton builders |
| `makeAutonCommand(...)` | [`include/autonomous/autonCommands.h`](../include/autonomous/autonCommands.h) | top-level auton factory |
| `DriveMoveCommand` | [`include/commands/driveMove.h`](../include/commands/driveMove.h) | PID point move |
| `RotateCommand` | [`include/commands/rotate.h`](../include/commands/rotate.h) | PID turn-in-place |
| `PathCommand` | [`include/motionProfiling/pathCommand.h`](../include/motionProfiling/pathCommand.h) | common base for profile-following commands |
| `RamseteCommand` | [`include/commands/ramsete.h`](../include/commands/ramsete.h) | RAMSETE path follower |
| `LtvUnicycleCommand` | [`include/commands/ltvUnicycleController.h`](../include/commands/ltvUnicycleController.h) | LTV unicycle path follower |
| `ProfileState`, `MotionProfile` | [`include/motionProfiling/motionProfile.h`](../include/motionProfiling/motionProfile.h) | time-parameterized path state |
| `buildProfile(...)` | [`include/motionProfiling/profileBuilder.h`](../include/motionProfiling/profileBuilder.h) | trapezoidal motion-profile builder |
| `buildBezierProfile(...)` | [`include/motionProfiling/profileBuilder.h`](../include/motionProfiling/profileBuilder.h) | Bezier-to-profile builder |
| `buildProfileFromJson(...)` | [`include/motionProfiling/profileBuilder.h`](../include/motionProfiling/profileBuilder.h) | JSON-to-profile builder |

## Selector Enums

```cpp
enum class Auton {
    NEGATIVE_1,
    POSITIVE_1,
    EXAMPLE_MOVE,
    EXAMPLE_TURN,
    EXAMPLE_PATH,
    EXAMPLE_LTV,
    SKILLS,
    NONE
};

enum class Alliance {
    RED,
    BLUE
};
```

Helper functions:

```cpp
const std::vector<Auton>& availableAutons();
const char* autonName(Auton auton);
const char* allianceName(Alliance alliance);
```

## `AutonBuildContext`

```cpp
struct AutonBuildContext {
    Drivetrain& drivetrain;
    Intakes& intakes;
    Lift& lift;
    std::function<Eigen::Vector3f()> poseSource;
};
```

| Field | Type | Meaning |
|---|---|---|
| `drivetrain` | `Drivetrain&` | motion subsystem |
| `intakes` | `Intakes&` | intake subsystem |
| `lift` | `Lift&` | lift subsystem |
| `poseSource` | `std::function<Eigen::Vector3f()>` | guarded controller pose provider |

## `autonCommands::makeAutonCommand(...)`

```cpp
std::unique_ptr<Command> makeAutonCommand(Auton auton, const AutonBuildContext& ctx);
```

Dispatch point from selector state to concrete builder functions in [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp).

## `DriveMoveCommand`

```cpp
DriveMoveCommand(Drivetrain* drivetrain,
                 Eigen::Vector2f target,
                 std::function<Eigen::Vector3f()> poseSource,
                 float tolerance = 0.02f);
```

| Parameter | Meaning |
|---|---|
| `drivetrain` | controlled drivetrain subsystem |
| `target` | field-frame `(x, y)` target in meters |
| `poseSource` | current guarded `(x, y, theta)` pose |
| `tolerance` | distance tolerance in meters |

Internal controllers:

- forward: `CONFIG::DISTANCE_PID`
- heading correction: `CONFIG::TURN_PID`

## `RotateCommand`

```cpp
RotateCommand(Drivetrain* drivetrain,
              float targetAngle,
              std::function<Eigen::Vector3f()> poseSource,
              float tolerance = 0.03f);
```

| Parameter | Meaning |
|---|---|
| `targetAngle` | final heading in radians |
| `poseSource` | current guarded pose provider |
| `tolerance` | angular tolerance in radians |

Internal controller:

- heading: `CONFIG::TURN_PID`

## `PathCommand`

```cpp
class PathCommand : public Command {
public:
    PathCommand(Drivetrain* drivetrain, MotionProfile profile);
    void initialize() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;

protected:
    float elapsedSeconds() const;
};
```

Common base for time-based profile followers.

## `RamseteCommand`

```cpp
RamseteCommand(Drivetrain* drivetrain,
               MotionProfile profile,
               std::function<Eigen::Vector3f()> poseSource,
               float zeta = CONFIG::RAMSETE_ZETA,
               float beta = CONFIG::RAMSETE_BETA);
```

| Parameter | Meaning |
|---|---|
| `profile` | time-parameterized trajectory |
| `poseSource` | current guarded pose provider |
| `zeta` | damping coefficient |
| `beta` | aggressiveness coefficient |

Execution path:

1. sample desired state from `MotionProfile`
2. compute robot-frame pose error
3. solve RAMSETE control law
4. call `Drivetrain::setDriveSpeeds({v, omega})`

## `LtvUnicycleCommand`

```cpp
LtvUnicycleCommand(Drivetrain* drivetrain,
                   MotionProfile profile,
                   std::function<Eigen::Vector3f()> poseSource,
                   Eigen::Vector3f qWeights = CONFIG::DEFAULT_DT_COST_Q);

LtvUnicycleCommand(Drivetrain* drivetrain,
                   MotionProfile profile,
                   std::function<Eigen::Vector3f()> poseSource,
                   const StateMatrix& qWeights,
                   const ControlMatrix& rWeights,
                   const StateMatrix& terminalCost);
```

| Parameter | Meaning |
|---|---|
| `profile` | time-parameterized trajectory |
| `poseSource` | current guarded pose provider |
| `qWeights` | state-cost weights; either diagonal shorthand or a full `Q` matrix |
| `rWeights` | control-effort cost matrix `R` |
| `terminalCost` | terminal Riccati cost `Qf` |

Execution path:

1. linearize and discretize the unicycle tracking-error model across the profile
2. solve the backward Riccati recursion to precompute `K[k]`
3. sample desired state and robot-frame pose error at runtime
4. apply `u = u_desired - K(t)e` and call `Drivetrain::setDriveSpeeds({v, omega})`

## `ProfileState`

```cpp
struct ProfileState {
    Eigen::Vector3f pose{0, 0, 0};
    float linearVelocity = 0.0f;
    float angularVelocity = 0.0f;
    float linearAcceleration = 0.0f;
};
```

## `MotionProfile`

```cpp
class MotionProfile {
public:
    MotionProfile(const Path& path,
                  const TrapezoidalVelocityProfile& velProfile,
                  int sampleCount = 200);

    ProfileState sample(float t) const;
    float totalTime() const;
    bool isFinished(float t) const;
    const std::vector<ProfileState>& samples() const;
};
```

| Method | Meaning |
|---|---|
| `sample(t)` | interpolate a state at time `t` |
| `totalTime()` | profile duration in seconds |
| `isFinished(t)` | completion query for elapsed time |
| `samples()` | pre-sampled lookup table |

## Profile Builders

Header: [`include/motionProfiling/profileBuilder.h`](../include/motionProfiling/profileBuilder.h)

```cpp
MotionProfile buildProfile(std::initializer_list<Eigen::Vector3f> waypoints,
                           float maxVelocityMps,
                           float maxAccelerationMps2,
                           int sampleCount = 200,
                           float initialSpeedMps = 0.0f,
                           float endSpeedMps = 0.0f);
```

Use this when you want a public, copy-pasteable way to build a trapezoidal motion profile from ordinary `{x, y, theta}` waypoints.

## Shared Auton Helpers

Header: [`include/autonomous/sharedCommands.h`](../include/autonomous/sharedCommands.h)

| Helper | Purpose |
|---|---|
| `shared::driveAndIntake(...)` | point move plus intake spin |
| `shared::outtakeTimed(...)` | fixed-duration outtake |
| `shared::liftCycle(...)` | up, wait, down lift sequence |
| `shared::toggleTongue(...)` | one-shot tongue toggle |
| `shared::toggleWing(...)` | one-shot wing toggle |
| `shared::toggleSelect1(...)` | one-shot select1 toggle |
| `shared::toggleSelect2(...)` | one-shot select2 toggle |
