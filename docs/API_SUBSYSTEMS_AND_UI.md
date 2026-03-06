# Subsystems and UI

## Symbol Index

| Symbol | Header | Purpose |
|---|---|---|
| `DriveSpeeds` | [`include/subsystems/drivetrain.h`](../include/subsystems/drivetrain.h) | chassis velocity pair |
| `Drivetrain` | [`include/subsystems/drivetrain.h`](../include/subsystems/drivetrain.h) | drive control, odometry, heading |
| `Intakes` | `include/subsystems/intakes.h` | intake motor group |
| `Lift` | `include/subsystems/lift.h` | lift interface, currently stubbed in practice |
| `Solenoids` | `include/subsystems/solenoids.h` | pneumatics interface |
| `AutonSelector` | [`include/ui/autonSelector.h`](../include/ui/autonSelector.h) | selector state and runtime rendering |
| `BrainScreen` | [`include/ui/brainScreen.h`](../include/ui/brainScreen.h) | init/runtime view models and render entrypoints |
| `ScreenManagerUI` | [`include/ui/screenManager.h`](../include/ui/screenManager.h) | UI render bridge |

## `DriveSpeeds`

```cpp
struct DriveSpeeds {
    float linear  = 0.0f;
    float angular = 0.0f;
};
```

Units:

- `linear`: meters per second
- `angular`: radians per second

## `Drivetrain`

```cpp
class Drivetrain : public Subsystem {
public:
    Drivetrain();
    void periodic() override;

    void setDriveSpeeds(DriveSpeeds speeds);
    void arcade(float forward, float turn);
    void driverArcade(float forwardInput, float turnInput);
    void resetDriverAssistState();
    void stop();
    void tankVoltage(float left, float right);

    Eigen::Vector3f getOdomPose() const;
    Eigen::Vector3f getPose() const;
    void setOdomPose(const Eigen::Vector3f& pose);
    void syncLocalizationReference(const Eigen::Vector3f& pose);
    void syncOdomBaselinesToCurrentSensors(float heading);
    float getHeading() const;
    void calibrateImu();
    void resetHeading(float heading = 0.0f);
    Eigen::Vector2f consumePendingDisplacement();
    Eigen::Vector2f consumePendingFwdOnlyDisplacement();
    float getForwardDistance() const;
    void resetEncoders();
    DriveSpeeds getLastSpeeds() const;
};
```

High-value entrypoints:

| Method | Use |
|---|---|
| `driverArcade(...)` | teleop stick shaping and active brake behavior |
| `setDriveSpeeds(...)` | closed-loop path followers |
| `getOdomPose()` | odometry-only pose |
| `syncLocalizationReference(...)` | align odom state to localization |
| `consumePendingFwdOnlyDisplacement()` | forward-only prediction delta for MCL |

## `Intakes`

Primary public surface used by command code:

```cpp
void spin(int voltage);
void stop();
```

Typical wrappers:

- `IntakeSpinCommand`
- `IntakeTimedCommand`
- `IntakeStopCommand`

## `Lift`

Typical command-facing surface:

```cpp
void moveTo(float targetDegrees);
void moveVoltage(int voltage);
void stop();
bool atTarget();
```

Current project note: lift command plumbing exists, but the mechanism is still not match-ready hardware behavior.

## `Solenoids`

Per-output surface:

| Output | Methods |
|---|---|
| `select1` | `setSelect1`, `getSelect1`, `toggleSelect1` |
| `select2` | `setSelect2`, `getSelect2`, `toggleSelect2` |
| `tongue` | `setTongue`, `getTongue`, `toggleTongue` |
| `wing` | `setWing`, `getWing`, `toggleWing` |

## `AutonSelector`

```cpp
class AutonSelector {
public:
    static void init();
    static void nextAuton();
    static void prevAuton();
    static void toggleAlliance();
    static void selectAlliance(Alliance alliance);
    static void selectAuton(Auton auton);
    static Auton getAuton();
    static Alliance getAlliance();
    static std::string getAutonStr();
    static std::string getAllianceStr();
    static void render(const Eigen::Vector3f& pose, const std::string& status = "");
    static void renderInit(float progress, const std::string& status = "");
};
```

Use it for:

- stored selector state
- touchscreen auton cycling
- alliance selection
- simple direct selector rendering

## `BrainScreen`

Namespace: [`include/ui/brainScreen.h`](../include/ui/brainScreen.h)

Primary types:

| Type | Purpose |
|---|---|
| `BrainScreen::InitViewModel` | startup progress screen |
| `BrainScreen::RuntimeViewModel` | runtime diagnostics and pose display |
| `DistanceSensorViewModel` | per-distance-sensor UI sample |

Primary functions:

```cpp
void initialize();
void renderInit(const InitViewModel& vm);
void renderRuntime(const RuntimeViewModel& vm);
```

## `ScreenManagerUI`

```cpp
namespace ScreenManagerUI {
    void init();
    void render(const BrainScreen::RuntimeViewModel& vm);
}
```

Thin integration layer between the runtime view model and the actual screen/page plumbing.
