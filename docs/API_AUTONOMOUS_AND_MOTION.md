# Autonomous and Motion

## Live Architecture

The live autonomous path is no longer the old command-style motion stack.

Current flow:

1. [`src/main.cpp`](../src/main.cpp) chooses the selected [`AutonEntry`](../include/autonomous/autons.h).
2. [`bindAutonRuntime(...)`](../include/autonomous/autons.h) binds the current drivetrain, intake, lift, and optional pose/cancel hooks.
3. [`runAuton(...)`](../include/autonomous/autons.h) calls the selected auton function directly in the current task.
4. The auton function drives motion through the vendored EZ backend in [`include/EZ-Template/drive/drive.hpp`](../include/EZ-Template/drive/drive.hpp).

The old `RamseteCommand`, `LtvUnicycleCommand`, and `PathCommand` headers still exist on disk, but they are not the live autonomous path.

## Selector Surface

Source: [`include/autonomous/autons.h`](../include/autonomous/autons.h)

```cpp
enum class Auton {
    NEGATIVE_1,
    POSITIVE_1,
    TUNE_DRIVE_PID,
    TUNE_TURN_PID,
    EXAMPLE_MOVE,
    EXAMPLE_SWING,
    EXAMPLE_TURN,
    EXAMPLE_RAMSETE,
    EXAMPLE_LTV,
    SKILLS,
    NONE
};

struct AutonEntry {
    Auton id;
    const char *name;
    AutonFn run;
};
```

Helpers:

```cpp
bool initializeAutonMotion();
const AutonList &availableAutons();
const AutonEntry *findAuton(Auton auton);
const char *autonName(Auton auton);
void bindAutonRuntime(Drivetrain &drivetrain,
                      Intakes &intakes,
                      Lift &lift,
                      std::function<Eigen::Vector3f()> poseSource,
                      std::function<bool()> isCancelled = {});
void resetAutonRuntime();
bool runAuton(const AutonEntry &entry);
```

## EZ Motion Surface

Source: [`include/EZ-Template/drive/drive.hpp`](../include/EZ-Template/drive/drive.hpp)

These are the live autonomous motion entrypoints used by [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp):

- `pid_drive_set(...)`
- `pid_turn_set(...)`
- `pid_swing_set(...)`
- `pid_odom_set(...)`
- `pid_odom_ramsete_set(...)`
- `pid_odom_ltv_set(...)`
- `pid_wait()`
- `pid_wait_until(...)`
- `pid_wait_until_index(...)`

### EZ RAMSETE

```cpp
void pid_ramsete_constants_set(double zeta, double beta);
void pid_odom_ramsete_set(std::vector<odom> imovements, bool slew_on);
void pid_odom_ramsete_set(std::vector<united_odom> p_imovements, bool slew_on);
```

Contract:

- uses EZ odom only
- accepts EZ waypoint lists
- forward-only in the current ship build
- rejects empty, reverse, or mixed-direction tracked paths
- runs under EZ drive-mode ownership, not the old command scheduler

### EZ LTV

```cpp
void pid_ltv_costs_set(double qx, double qy, double qtheta,
                       double rv, double romega,
                       double terminal_scale = 1.0);
void pid_odom_ltv_set(std::vector<odom> imovements, bool slew_on);
void pid_odom_ltv_set(std::vector<united_odom> p_imovements, bool slew_on);
```

Contract:

- same EZ waypoint API as RAMSETE
- same forward-only restrictions in the current ship build
- uses a precomputed finite-horizon LTV gain schedule internally
- runs through EZ motor commands in the same `[-127, 127]` space as the rest of EZ

## Example Routines

Source: [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp)

Current selector-visible examples:

- `Example Drive`
- `Example Swing`
- `PID Calibration`
- `Example Ramsete`
- `Example LTV`
- `Tune Drive PID`
- `Tune Turn PID`

`Negative 1`, `Positive 1`, and `Skills` are also direct-function EZ routines in the same file.

## Wait Semantics

Tracked EZ modes now integrate with EZ waits:

- `pid_wait()` settles on final XY and heading error
- `pid_wait_until(point)` waits on tracked odom distance
- `pid_wait_until_index(index)` uses an explicit waypoint-to-profile-sample map

`pid_wait_quick_chain()` is not special-cased for RAMSETE or LTV in this build; it falls back to normal wait behavior.

## Tuning Surface

Primary tunables in [`include/config.h`](../include/config.h):

- `TURN_PID`
- `DISTANCE_PID`
- `RAMSETE_ZETA`
- `RAMSETE_BETA`
- `defaultDtCostQ()`
- `LTV_CONTROL_COST_V`
- `LTV_CONTROL_COST_OMEGA`
- `LTV_TERMINAL_SCALE`
- `AUTON_DRIVE_SLEW_STEP`
- `AUTON_TURN_SLEW_STEP`

## Runtime Notes

- `initializeAutonMotion()` prepares one shared EZ drive backend during startup.
- `prepareEzStart()` in [`src/autonomous/autons.cpp`](../src/autonomous/autons.cpp) zeroes EZ odom before each example/competition routine.
- The tracked EZ modes do not consume the old guarded fused pose. They use EZ odom only.
- Autotune is still available, but it is a utility routine, not the primary competition motion path.
