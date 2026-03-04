/**
 * @file main.cpp
 * 2654E Echo — PROS V5 competition lifecycle.
 *
 * Creates two periodic tasks:
 *   • command-scheduler loop  (10 ms)  — advances the command graph
 *   • screen-update loop      (50 ms)  — redraws the brain LCD with pose + auton
 *
 * autonomous() schedules the selected autonCommand.
 * opcontrol()  cancels it and runs arcade drive + trigger bindings.
 */
#include "main.h"
#include <cstdio>
#include <cmath>
#include <string>

// ═══════════════════════════════════════════════════════════════════════════
// Global subsystems & localization
// ═══════════════════════════════════════════════════════════════════════════

static Drivetrain* drivetrain;
static Intakes*    intakes;
static Lift*       lift;
static Solenoids*  solenoids;

static ParticleFilter* particleFilter;

// ═══════════════════════════════════════════════════════════════════════════
// Autonomous command — built on-demand, scheduled in autonomous()
// ═══════════════════════════════════════════════════════════════════════════

static Command* autonCommand = nullptr;

// ═══════════════════════════════════════════════════════════════════════════
// Pose source (shared lambda used by all path-followers)
// ═══════════════════════════════════════════════════════════════════════════

static std::function<Eigen::Vector3f()> poseSource;

// ═══════════════════════════════════════════════════════════════════════════
// GPS initialisation status (displayed on screen)
// ═══════════════════════════════════════════════════════════════════════════

static std::string gpsStatus = "";

// ═══════════════════════════════════════════════════════════════════════════
// Periodic task: command scheduler @ 10 ms
// ═══════════════════════════════════════════════════════════════════════════

[[noreturn]] void update_loop(void*) {
    while (true) {
        auto start_time = pros::millis();

        // Tick the localization filter
        if (particleFilter) {
            particleFilter->update();
        }

        // Advance all scheduled commands
        CommandScheduler::run();

        pros::c::task_delay_until(&start_time, 10);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Periodic task: brain-screen update @ 50 ms
// ═══════════════════════════════════════════════════════════════════════════

[[noreturn]] void screen_update_loop(void*) {
    while (true) {
        auto start_time = pros::millis();

        Eigen::Vector3f pose = poseSource ? poseSource()
                                          : Eigen::Vector3f(0, 0, 0);

        AutonSelector::render(pose, gpsStatus);

        pros::c::task_delay_until(&start_time, 50);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Subsystem construction & registration
// ═══════════════════════════════════════════════════════════════════════════

static void subsystemInit() {
    drivetrain = new Drivetrain();
    intakes    = new Intakes();
    lift       = new Lift();       // stub — no hardware until re-wired
    solenoids  = new Solenoids();

    drivetrain->registerThis();
    intakes->registerThis();
    lift->registerThis();
    solenoids->registerThis();
}

// ═══════════════════════════════════════════════════════════════════════════
// GPS initial-pose acquisition
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Wait for the GPS to produce stable readings and return the initial pose.
 *
 * Depending on CONFIG::STARTUP_POSE_MODE:
 *   ConfiguredStartPoseOnly → return hard-coded config values immediately.
 *   GPSXYPlusIMUHeading     → GPS (x,y) + IMU heading.
 *   FullGPSInit             → GPS (x,y) + GPS heading (with offset).
 *
 * Falls back to config values if GPS never stabilises.
 */
static Eigen::Vector3f acquireInitialPose() {
    // Fallback pose from config (inches → metres)
    constexpr float inToM = 0.0254f;
    Eigen::Vector3f configPose(
        static_cast<float>(CONFIG::START_POSE_X_IN) * inToM,
        static_cast<float>(CONFIG::START_POSE_Y_IN) * inToM,
        static_cast<float>(CONFIG::START_POSE_THETA_DEG * M_PI / 180.0));

    if (CONFIG::STARTUP_POSE_MODE ==
        CONFIG::StartupPoseMode::ConfiguredStartPoseOnly) {
        gpsStatus = "Pose: config only";
        return configPose;
    }

    // Open a temporary GPS handle for the init read
    pros::Gps gps(CONFIG::MCL_GPS_PORT);
    pros::delay(100);  // let the sensor boot

    gpsStatus = "Waiting for GPS...";

    uint32_t startTime = pros::millis();
    int stableCount = 0;
    float lastX = 0.0f, lastY = 0.0f;

    while (pros::millis() - startTime < CONFIG::STARTUP_GPS_MAX_WAIT_MS) {
        auto pos = gps.get_position();
        float x = static_cast<float>(pos.x);
        float y = static_cast<float>(pos.y);

        // Ignore zero readings (sensor not ready)
        if (std::abs(x) < 0.001f && std::abs(y) < 0.001f) {
            stableCount = 0;
            pros::delay(50);
            continue;
        }

        float dx = x - lastX;
        float dy = y - lastY;
        float drift = std::sqrt(dx * dx + dy * dy);

        if (drift < static_cast<float>(CONFIG::STARTUP_GPS_READY_ERROR_M)) {
            ++stableCount;
        } else {
            stableCount = 0;
        }
        lastX = x;
        lastY = y;

        char buf[40];
        std::snprintf(buf, sizeof(buf), "GPS: stable %d/%d",
                      stableCount, CONFIG::STARTUP_GPS_STABLE_SAMPLES);
        gpsStatus = buf;

        if (stableCount >= CONFIG::STARTUP_GPS_STABLE_SAMPLES) {
            break;
        }

        pros::delay(50);
    }

    // If GPS never locked, fall back
    if (stableCount < CONFIG::STARTUP_GPS_STABLE_SAMPLES) {
        gpsStatus = "GPS timeout — using config";
        return configPose;
    }

    gpsStatus = "GPS locked";

    float gpsHeadingRad = static_cast<float>(
        (gps.get_heading() - CONFIG::MCL_GPS_HEADING_OFFSET_DEG)
        * M_PI / 180.0);

    float imuHeadingRad = drivetrain->getHeading();

    switch (CONFIG::STARTUP_POSE_MODE) {
        case CONFIG::StartupPoseMode::GPSXYPlusIMUHeading:
            return Eigen::Vector3f(lastX, lastY, imuHeadingRad);

        case CONFIG::StartupPoseMode::FullGPSInit:
            return Eigen::Vector3f(lastX, lastY, gpsHeadingRad);

        default:
            return configPose;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Localization construction
// ═══════════════════════════════════════════════════════════════════════════

static void localizationInit() {
    // Sensor models — conditionally constructed based on config enables
    std::vector<SensorModel*> sensors;

    static DistanceSensorModel distLeft(
        CONFIG::MCL_LEFT_DISTANCE_PORT,  CONFIG::DIST_LEFT_OFFSET,
        static_cast<float>(CONFIG::MCL_LEFT_DISTANCE_WEIGHT));
    static DistanceSensorModel distRight(
        CONFIG::MCL_RIGHT_DISTANCE_PORT, CONFIG::DIST_RIGHT_OFFSET,
        static_cast<float>(CONFIG::MCL_RIGHT_DISTANCE_WEIGHT));
    static DistanceSensorModel distFront(
        CONFIG::MCL_FRONT_DISTANCE_PORT, CONFIG::DIST_FRONT_OFFSET,
        static_cast<float>(CONFIG::MCL_FRONT_DISTANCE_WEIGHT));
    static DistanceSensorModel distBack(
        CONFIG::MCL_BACK_DISTANCE_PORT,  CONFIG::DIST_BACK_OFFSET,
        static_cast<float>(CONFIG::MCL_BACK_DISTANCE_WEIGHT));
    static GpsSensorModel gpsSensor(
        CONFIG::MCL_GPS_PORT, CONFIG::MCL_GPS_HEADING_OFFSET_DEG);

    if (CONFIG::MCL_ENABLE_DISTANCE_SENSORS) {
        if (CONFIG::MCL_ENABLE_LEFT_DISTANCE_SENSOR)  sensors.push_back(&distLeft);
        if (CONFIG::MCL_ENABLE_RIGHT_DISTANCE_SENSOR) sensors.push_back(&distRight);
        if (CONFIG::MCL_ENABLE_FRONT_DISTANCE_SENSOR) sensors.push_back(&distFront);
        if (CONFIG::MCL_ENABLE_BACK_DISTANCE_SENSOR)  sensors.push_back(&distBack);
    }
    sensors.push_back(&gpsSensor);

    // Prediction function: odometry displacement from drivetrain
    auto predictionFn = [&]() -> Eigen::Vector2f {
        return drivetrain->getDisplacement();
    };

    // Heading function: IMU heading
    auto angleFn = [&]() -> QAngle {
        return QAngle(drivetrain->getHeading());
    };

    // ── Acquire initial pose from GPS (or config fallback) ──────────────
    Eigen::Vector3f startPose = acquireInitialPose();

    particleFilter = new ParticleFilter(
        predictionFn, angleFn, sensors,
        startPose,
        CONFIG::NUM_PARTICLES);

    // Wire up the shared pose source
    poseSource = [&]() -> Eigen::Vector3f {
        return particleFilter->getPrediction();
    };
}

// ═══════════════════════════════════════════════════════════════════════════
// Build the auton command from the current selector state
// ═══════════════════════════════════════════════════════════════════════════

static void buildAutonCommand() {
    // Sync the global AUTON / ALLIANCE variables from the selector
    AUTON    = AutonSelector::getAuton();
    ALLIANCE = AutonSelector::getAlliance();

    autonCommand = autonCommands::makeAutonCommand(
        AUTON, drivetrain, intakes, lift, solenoids, poseSource);
}

// ═══════════════════════════════════════════════════════════════════════════
// PROS lifecycle entrypoints
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Runs once when the program starts.
 */
void initialize() {
    // Start auton selector (initialises LLEMU + button callbacks)
    AutonSelector::init();

    gpsStatus = "Initialising...";
    AutonSelector::render(Eigen::Vector3f(0, 0, 0), gpsStatus);

    subsystemInit();
    localizationInit();     // blocks briefly for GPS acquisition

    // Build the auton command from current selector state
    buildAutonCommand();

    // Start periodic tasks
    pros::Task scheduler_task(update_loop, nullptr, "scheduler");
    pros::Task screen_task(screen_update_loop, nullptr, "screen");
}

/**
 * Runs while the robot is disabled.
 */
void disabled() {}

/**
 * Runs after initialize(), before autonomous (competition only).
 * The auton selector is still active here — rebuild the command so
 * the most recent selection is used.
 */
void competition_initialize() {
    buildAutonCommand();
}

/**
 * Runs the autonomous routine.
 */
void autonomous() {
    // Lock in the selector state one final time
    buildAutonCommand();

    if (autonCommand) {
        CommandScheduler::schedule(autonCommand);
    }
}

/**
 * Runs the operator-control period.
 */
void opcontrol() {
    // Cancel autonomous command if still running
    if (autonCommand && autonCommand->isScheduled()) {
        autonCommand->cancel();
    }

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Controller partner(pros::E_CONTROLLER_PARTNER);

    // ── Trigger bindings ────────────────────────────────────────────────

    // R1 → intake forward  (while held)
    CommandScheduler::addTrigger(
        Trigger([&]() { return master.get_digital(DIGITAL_R1); })
            .whileTrue(new IntakeSpinCommand(intakes, 127)));

    // R2 → intake reverse  (while held)
    CommandScheduler::addTrigger(
        Trigger([&]() { return master.get_digital(DIGITAL_R2); })
            .whileTrue(new IntakeSpinCommand(intakes, -127)));

    // L1 → toggle tongue
    CommandScheduler::addTrigger(
        Trigger([&]() { return master.get_digital(DIGITAL_L1); })
            .onTrue(shared::toggleTongue(solenoids)));

    // L2 → toggle wing
    CommandScheduler::addTrigger(
        Trigger([&]() { return master.get_digital(DIGITAL_L2); })
            .onTrue(shared::toggleWing(solenoids)));

    // A → toggle select1
    CommandScheduler::addTrigger(
        Trigger([&]() { return master.get_digital(DIGITAL_A); })
            .onTrue(shared::toggleSelect1(solenoids)));

    // B → toggle select2
    CommandScheduler::addTrigger(
        Trigger([&]() { return master.get_digital(DIGITAL_B); })
            .onTrue(shared::toggleSelect2(solenoids)));

    // ── Skills: re-schedule auton with termination predicate ────────────
    if (AUTON == Auton::SKILLS && autonCommand) {
        CommandScheduler::schedule(
            autonCommand->until([&]() {
                return partner.get_digital(DIGITAL_RIGHT);
            }));
    }

    // ── Teleop drive loop ───────────────────────────────────────────────
    while (true) {
        // Arcade drive (left stick Y = forward, right stick X = turn)
        float forward = static_cast<float>(
            master.get_analog(ANALOG_LEFT_Y));
        float turn = static_cast<float>(
            master.get_analog(ANALOG_RIGHT_X));

        // Only drive manually if no command currently owns the drivetrain
        if (drivetrain->getCurrentCommand() == nullptr) {
            drivetrain->arcade(forward, turn);
        }

        pros::delay(20);
    }
}