# 2654E Echo — API Reference

## Table of Contents

1. [Command Framework](#command-framework)
2. [Subsystems](#subsystems)
3. [Localization](#localization)
4. [Motion Profiling](#motion-profiling)
5. [Path Following](#path-following)
6. [Feedback Controllers](#feedback-controllers)
7. [UI](#ui)
8. [Utilities](#utilities)
9. [Configuration](#configuration)

---

## Command Framework

> `include/command/`

### `Command` (`command.h`)

Abstract base class for all commands.

```cpp
class Command {
public:
    virtual void initialize();   // Called once when scheduled
    virtual void execute();      // Called every tick (10 ms)
    virtual void end(bool interrupted);  // Called on finish/cancel
    virtual bool isFinished();   // Return true to end naturally

    void cancel();               // Request cancellation
    bool isScheduled() const;    // Is this command currently running?
    Command* until(std::function<bool()> condition);  // Add end condition
    void addRequirements(std::initializer_list<Subsystem*> reqs);
};
```

### `Subsystem` (`subsystem.h`)

Base class for hardware subsystems.

```cpp
class Subsystem {
public:
    virtual void periodic();     // Called every tick when registered
    void registerThis();         // Register with the scheduler
    void setDefaultCommand(Command* cmd);
    Command* getCurrentCommand() const;
};
```

### `CommandScheduler` (`commandScheduler.h`)

Singleton scheduler — drives the command graph.

```cpp
class CommandScheduler {
public:
    static void run();                                // Tick all active commands
    static void schedule(Command* cmd);               // Start a command
    static void cancel(Command* cmd);                 // Cancel a running command
    static void addTrigger(Trigger trigger);           // Register a trigger binding
    static void registerSubsystem(Subsystem* sub);    // Register a subsystem
};
```

### `CommandGroup` (`commandGroup.h`)

Compose multiple commands.

```cpp
class SequentialCommandGroup : public Command;   // Run one after another
class ParallelCommandGroup   : public Command;   // Run all at once, end when ALL finish
class ParallelRaceGroup      : public Command;   // Run all at once, end when ANY finishes
class ParallelDeadlineGroup  : public Command;   // Run all, end when the deadline cmd finishes
```

### `Trigger` (`trigger.h`)

Bind commands to boolean conditions.

```cpp
class Trigger {
public:
    Trigger(std::function<bool()> condition);
    Trigger& onTrue(Command* cmd);     // Schedule cmd on rising edge
    Trigger& onFalse(Command* cmd);    // Schedule cmd on falling edge
    Trigger& whileTrue(Command* cmd);  // Run cmd while condition holds
    Trigger& whileFalse(Command* cmd); // Run cmd while condition is false
};
```

### Utility Commands

| Class | Header | Description |
|-------|--------|-------------|
| `WaitCommand` | `waitCommand.h` | Waits a fixed duration |
| `InstantCommand` | `instantCommand.h` | Runs a lambda once |
| `FunctionalCommand` | `functionalCommand.h` | Fully customisable (init/execute/end/isFinished lambdas) |

---

## Subsystems

### `Drivetrain` (`subsystems/drivetrain.h`)

6-motor differential drive with IMU and optional tracking wheels.

```cpp
class Drivetrain : public Subsystem {
public:
    Drivetrain();

    // Control
    void setDriveSpeeds(DriveSpeeds speeds); // Feedforward velocity control
    void arcade(float forward, float turn);  // Raw arcade drive (-127..127)
    void stop();                             // Brake all motors
    void tankVoltage(float left, float right); // Raw tank voltage

    // State
    Eigen::Vector3f getPose() const;          // (x, y, θ) in metres/radians
    void setPose(const Eigen::Vector3f& pose);
    float getHeading() const;                 // IMU heading in radians
    void resetHeading(float heading = 0.0f);
    Eigen::Vector2f getDisplacement();        // Delta since last call
    float getForwardDistance() const;          // Total forward travel (m)
    void resetEncoders();
    DriveSpeeds getLastSpeeds() const;
};
```

**`DriveSpeeds`** — linear (m/s) + angular (rad/s) velocity pair:
```cpp
struct DriveSpeeds {
    float linear  = 0.0f;
    float angular = 0.0f;
};
```

### `Intakes` (`subsystems/intakes.h`)

2-motor intake subsystem.

```cpp
class Intakes : public Subsystem {
public:
    Intakes();
    void spin(int voltage);   // -127 to 127
    void stop();
};
```

### `Solenoids` (`subsystems/solenoids.h`)

4-port pneumatic controller.

```cpp
class Solenoids : public Subsystem {
public:
    Solenoids();

    // Per-solenoid control
    void setSelect1(bool on);    bool getSelect1() const;    void toggleSelect1();
    void setSelect2(bool on);    bool getSelect2() const;    void toggleSelect2();
    void setTongue(bool on);     bool getTongue() const;     void toggleTongue();
    void setWing(bool on);       bool getWing() const;       void toggleWing();
};
```

### `Lift` (`subsystems/lift.h`)

Currently stubbed (no hardware). All methods are no-ops.

```cpp
class Lift : public Subsystem {
public:
    Lift();
    void setVoltage(int v);   // no-op
    void stop();              // no-op
    float getPosition();      // returns 0
    bool atTarget();          // returns true
};
```

---

## Localization

> `include/localization/`

### `ParticleFilter` (`particleFilter.h`)

Monte Carlo localization with systematic resampling.

```cpp
class ParticleFilter {
public:
    using PredictionFn = std::function<Eigen::Vector2f()>;  // odometry delta
    using AngleFn      = std::function<QAngle()>;           // heading source

    ParticleFilter(
        PredictionFn prediction,
        AngleFn      angle,
        std::vector<SensorModel*> sensors,
        Eigen::Vector3f initialPose = {0,0,0},
        size_t numParticles = 250);

    Eigen::Vector3f update();                 // Run one filter cycle
    Eigen::Vector3f getPrediction() const;    // Current best-estimate pose
    std::vector<Eigen::Vector3f> getParticles() const;  // Debug: all particles
};
```

### `SensorModel` (`sensor.h`)

Abstract interface for sensor likelihood models.

```cpp
class SensorModel {
public:
    virtual ~SensorModel() = default;
    virtual std::optional<float> p(const Eigen::Vector3f& particle) = 0;
    virtual void update() {}
};
```

### `DistanceSensorModel` (`distance.h`)

VEX Distance Sensor ray-cast likelihood against field walls.

```cpp
class DistanceSensorModel : public SensorModel {
public:
    DistanceSensorModel(int port,
                        const Eigen::Vector3f& offset,   // (x, y, facing)
                        float weight = 1.0f,
                        float stddev = 0.03f);
};
```

### `GpsSensorModel` (`gps.h`)

VEX GPS 2-D Gaussian likelihood model.

```cpp
class GpsSensorModel : public SensorModel {
public:
    GpsSensorModel(int port,
                   double headingOffsetDeg = 0.0,
                   float stddev = 0.05f);

    double getHeadingOffsetDeg() const;
};
```

---

## Motion Profiling

> `include/motionProfiling/`

### `BezierPath` (`bezier.h`)

Cubic Bézier curve defined by 4 control points.

```cpp
class BezierPath {
public:
    BezierPath(Eigen::Vector2f p0, Eigen::Vector2f p1,
               Eigen::Vector2f p2, Eigen::Vector2f p3);

    Eigen::Vector2f evaluate(float t) const;     // Point at parameter t ∈ [0,1]
    Eigen::Vector2f derivative(float t) const;    // Tangent vector
    float curvature(float t) const;               // Signed curvature
    float arcLength(int segments = 100) const;    // Approximate total length
};
```

### `MotionProfile` (`motionProfile.h`)

Time-parameterised trajectory along a Bézier path with a trapezoidal
velocity profile.

```cpp
class MotionProfile {
public:
    MotionProfile(const BezierPath& path,
                  float maxVel, float maxAccel,
                  int resolution = 200);

    struct State {
        Eigen::Vector2f position;
        float heading;      // radians
        float velocity;     // m/s
        float curvature;    // 1/m
        float time;         // seconds
    };

    State sample(float t) const;      // Sample at time t
    float totalTime() const;           // Duration of the profile
    bool isFinished(float t) const;
};
```

### `PathCommand` (`pathCommand.h`)

Command that drives the robot along a `MotionProfile` using a chosen
path-following controller.

```cpp
class PathCommand : public Command {
public:
    PathCommand(Drivetrain* dt,
                MotionProfile profile,
                std::function<Eigen::Vector3f()> poseSource,
                /* controller params */);
};
```

---

## Path Following

> `include/commands/`

### `RamseteController` (`ramsete.h`)

Non-linear time-varying reference tracker for differential-drive robots.

```cpp
class RamseteController {
public:
    RamseteController(float beta = CONFIG::RAMSETE_BETA,
                      float zeta = CONFIG::RAMSETE_ZETA);

    DriveSpeeds calculate(
        const Eigen::Vector3f& currentPose,
        const Eigen::Vector3f& desiredPose,
        float desiredV, float desiredOmega) const;
};
```

**Parameters:**
- `β` (beta) — aggressiveness of convergence (higher = tighter tracking)
- `ζ` (zeta) — damping ratio (0 < ζ < 1; higher = less oscillation)

### `LtvUnicycleController` (`ltvUnicycleController.h`)

Linear time-varying unicycle model controller.

```cpp
class LtvUnicycleController {
public:
    LtvUnicycleController(const Eigen::Vector3f& costQ = CONFIG::DEFAULT_DT_COST_Q);

    DriveSpeeds calculate(
        const Eigen::Vector3f& currentPose,
        const Eigen::Vector3f& desiredPose,
        float desiredV, float desiredOmega) const;
};
```

### `DriveMoveCommand` (`driveMove.h`)

Drives a fixed distance in a straight line using PID.

```cpp
class DriveMoveCommand : public Command {
public:
    DriveMoveCommand(Drivetrain* dt, float distanceMetres,
                     float maxSpeed = CONFIG::MAX_SPEED);
};
```

### `RotateCommand` (`rotate.h`)

Turns to an absolute heading using PID.

```cpp
class RotateCommand : public Command {
public:
    RotateCommand(Drivetrain* dt, float targetHeadingRad,
                  float maxAngularVel = CONFIG::MAX_ANGULAR_VEL);
};
```

### `IntakeSpinCommand` (`commands/intake/intakeCommand.h`)

Spins the intake at a given voltage while the command is active.

```cpp
class IntakeSpinCommand : public Command {
public:
    IntakeSpinCommand(Intakes* intakes, int voltage);
};
```

---

## Feedback Controllers

> `include/feedback/`

### `PID` (`pid.h`)

Standard PID controller with optional integral cap.

```cpp
class PID {
public:
    struct Gains {
        float kP, kI, kD, integralCap;
    };

    PID(Gains gains);
    float calculate(float error);   // Compute output for this tick
    void reset();                    // Zero integral and prev-error
    bool atTarget(float error, float tolerance);
};
```

---

## UI

> `include/ui/`

### `AutonSelector` (`autonSelector.h`)

Static-class brain-screen auton selector using LLEMU buttons.

```cpp
class AutonSelector {
public:
    static void init();                // Set up LCD & register button callbacks
    static void nextAuton();           // Cycle forward
    static void prevAuton();           // Cycle backward
    static void toggleAlliance();      // RED ↔ BLUE

    static Auton    getAuton();        // Current selection
    static Alliance getAlliance();
    static std::string getAutonStr();
    static std::string getAllianceStr();

    static void render(
        const Eigen::Vector3f& pose,          // Robot pose for display
        const std::string& status = "");       // Optional status line
};
```

---

## Utilities

### `units.hpp` — Compile-time unit wrappers

```cpp
using QLength = units::QLength;   // metres
using QAngle  = units::QAngle;    // radians
using QSpeed  = units::QSpeed;    // m/s
```

### `utils.h` — Math helpers

```cpp
namespace utils {
    float clamp(float value, float lo, float hi);
    float normalizeAngle(float angle);   // Wrap to [-π, π]
    float degToRad(float deg);
    float radToDeg(float rad);
}
```

### `linear.h` — Linear algebra helpers

Convenience wrappers around Eigen for 2D/3D operations.

### `motor.h` — Motor voltage utilities

```cpp
namespace motorUtil {
    int clampVoltage(float v);       // Clamp to [-127, 127]
    int mVToMove(float mV);          // Convert millivolts to PROS move()
}
```

### `json.h` — Minimal JSON serialiser

For telemetry logging.

### `telemetry.h` — SD-card data logger

```cpp
namespace Telemetry {
    void log(const std::string& key, float value);
    void flush();  // Write buffered data to SD
}
```

---

## Configuration

> `include/config.h` — `namespace CONFIG`

### Geometry

| Constant | Type | Description |
|----------|------|-------------|
| `DRIVE_RADIUS` | `float` | Drive wheel radius (m) |
| `ODOM_RADIUS` | `float` | Tracking wheel radius (m) |
| `TRACK_WIDTH` | `float` | Left-right centre distance (m) |
| `WHEEL_BASE` | `float` | Front-back axle distance (m) |

### Noise Model

| Constant | Type | Description |
|----------|------|-------------|
| `DRIVE_NOISE` | `float` | Odometry displacement noise σ (m) |
| `ANGLE_NOISE` | `float` | Heading noise σ (rad) |

### Particle Filter

| Constant | Type | Description |
|----------|------|-------------|
| `NUM_PARTICLES` | `int` | Ensemble size (default 250) |
| `FIELD_HALF_SIZE` | `float` | Half field width in metres |
| `MAX_DISTANCE_SINCE_UPDATE` | `float` | Min travel before MCL update |
| `MAX_UPDATE_INTERVAL_MS` | `int` | Max time between updates |

### Speed / Acceleration

| Constant | Type | Description |
|----------|------|-------------|
| `MAX_SPEED` | `float` | Top speed (m/s) |
| `MAX_ACCELERATION` | `float` | Max accel (m/s²) |
| `MAX_ANGULAR_VEL` | `float` | Max turn rate (rad/s) |

### Feedforward

| Constant | Type | Description |
|----------|------|-------------|
| `FF_kS` | `float` | Static friction (mV) |
| `FF_kV` | `float` | Velocity gain (mV·s/m) |
| `FF_kA` | `float` | Acceleration gain (mV·s²/m) |

### Startup

| Constant | Type | Description |
|----------|------|-------------|
| `STARTUP_POSE_MODE` | `StartupPoseMode` | How to initialise pose |
| `STARTUP_GPS_MAX_WAIT_MS` | `uint32_t` | Max wait for GPS lock |
| `STARTUP_GPS_READY_ERROR_M` | `double` | Stability threshold |
| `STARTUP_GPS_STABLE_SAMPLES` | `int` | Required consecutive stable reads |

---

*2654E Echo — API Reference v1.0*
