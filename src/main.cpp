/**
 * @file main.cpp
 * 69580A — PROS V5 competition lifecycle.
 *
 * Creates two periodic tasks:
 *   • command-scheduler loop  (10 ms)  — advances the command graph
 *   • screen-update loop      (50 ms)  — redraws the brain LCD with pose + auton
 *
 * autonomous() schedules the selected autonCommand.
 * opcontrol()  cancels it and runs arcade drive + trigger bindings.
 */
#include "main.h"
#include "ui/brainScreen.h"
#include <cstdio>
#include <cmath>
#include <string>
#include <memory>

// ═══════════════════════════════════════════════════════════════════════════
// Global subsystems & localization
// ═══════════════════════════════════════════════════════════════════════════

static Drivetrain* drivetrain = nullptr;
static Intakes*    intakes    = nullptr;
static Lift*       lift       = nullptr;
static Solenoids*  solenoids  = nullptr;

static ParticleFilter* particleFilter = nullptr;

// ═══════════════════════════════════════════════════════════════════════════
// Autonomous command — built on-demand, scheduled in autonomous()
// ═══════════════════════════════════════════════════════════════════════════

static Command* autonCommand = nullptr;

// ═══════════════════════════════════════════════════════════════════════════
// Pose source (shared lambda used by all path-followers)
// ═══════════════════════════════════════════════════════════════════════════

static std::function<Eigen::Vector3f()> poseSource;
static std::unique_ptr<pros::Gps> uiGps;

// ═══════════════════════════════════════════════════════════════════════════
// Status string for screen (only touched from screen task after init)
// ═══════════════════════════════════════════════════════════════════════════

static std::string screenStatus = "";

static void renderInitStage(float progress,
                            const std::string& stage,
                            const std::string& detail = "") {
    BrainScreen::InitViewModel vm;
    vm.progress = progress;
    vm.stageTitle = stage;
    vm.detail = detail;
    BrainScreen::renderInit(vm);
}

// ═══════════════════════════════════════════════════════════════════════════
// Periodic task: command scheduler @ 10 ms
// ═══════════════════════════════════════════════════════════════════════════

void update_loop() {
    uint32_t lastDebugMs = 0;
    while (true) {
        // Odom updates first (via drivetrain::periodic()), then PF consumes
        CommandScheduler::run();
        if (particleFilter) {
            particleFilter->update();
        }

        // Rate-limited debug instrumentation
        if constexpr (CONFIG::ODOM_DEBUG_ENABLE) {
            uint32_t now = pros::millis();
            if (now - lastDebugMs >= CONFIG::ODOM_DEBUG_LOG_EVERY_MS) {
                lastDebugMs = now;
                auto odom = drivetrain ? drivetrain->getOdomPose()
                                       : Eigen::Vector3f(0, 0, 0);
                auto pf   = particleFilter ? particleFilter->getPrediction()
                                           : Eigen::Vector3f(0, 0, 0);
                float imuH = drivetrain ? drivetrain->getHeading() : 0.0f;
                float gpsX = 0, gpsY = 0;
                bool gpsOk = false;
                if (uiGps) {
                    auto gp = uiGps->get_position();
                    Eigen::Vector2f rawGps(
                        static_cast<float>(gp.x),
                        static_cast<float>(gp.y));
                    Eigen::Vector2f gpsField = CONFIG::transformGpsToFieldFrame(rawGps);
                    gpsX = gpsField.x();
                    gpsY = gpsField.y();
                    gpsOk = std::isfinite(gpsX) && std::isfinite(gpsY);
                }
                std::printf("[LOC] gps(%c)=(%+.3f,%+.3f) imu=%.2f "
                            "odom=(%+.3f,%+.3f,%.2f) pf=(%+.3f,%+.3f,%.2f) "
                            "fin=%d%d%d\n",
                    gpsOk ? 'Y' : 'N', gpsX, gpsY, imuH,
                    odom.x(), odom.y(), odom.z(),
                    pf.x(), pf.y(), pf.z(),
                    LocMath::isFinitePose(odom) ? 1 : 0,
                    LocMath::isFinitePose(pf) ? 1 : 0,
                    LocMath::isFinite(imuH) ? 1 : 0);
            }
        }

        pros::delay(10);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Periodic task: brain-screen update @ 50 ms (started AFTER init)
// ═══════════════════════════════════════════════════════════════════════════

void screen_update_loop() {
    Eigen::Vector3f lastFiniteCombined(0.0f, 0.0f, 0.0f);
    while (true) {
        BrainScreen::RuntimeViewModel vm;

        vm.auton = AutonSelector::getAutonStr();
        vm.alliance = AutonSelector::getAllianceStr();
        vm.status = screenStatus;

        vm.pureOdomPose = drivetrain ? drivetrain->getOdomPose()
                                     : Eigen::Vector3f(0, 0, 0);
        if (!LocMath::isFinitePose(vm.pureOdomPose)) {
            std::printf("[LOC] non-finite pureOdomPose, forcing zero\n");
            vm.pureOdomPose = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        }

        vm.pureMclPose = particleFilter ? particleFilter->getPrediction()
                                        : vm.pureOdomPose;
        if (!LocMath::isFinitePose(vm.pureMclPose)) {
            std::printf("[LOC] non-finite pureMclPose, fallback to odom\n");
            vm.pureMclPose = vm.pureOdomPose;
        }

        vm.combinedPose = poseSource ? poseSource() : vm.pureMclPose;
        if (!LocMath::isFinitePose(vm.combinedPose)) {
            std::printf("[LOC] non-finite combinedPose, fallback to last finite\n");
            vm.combinedPose = lastFiniteCombined;
        }
        if (LocMath::isFinitePose(vm.combinedPose)) {
            lastFiniteCombined = vm.combinedPose;
        }
        vm.pose = vm.combinedPose;

        if (uiGps) {
            auto pos = uiGps->get_position();
            Eigen::Vector2f rawGps(
                static_cast<float>(pos.x),
                static_cast<float>(pos.y));
            Eigen::Vector2f gpsField = CONFIG::transformGpsToFieldFrame(rawGps);
            float gx = gpsField.x();
            float gy = gpsField.y();
            float gh = CONFIG::compassDegToMathRad(static_cast<float>(
                uiGps->get_heading() - CONFIG::MCL_GPS_HEADING_OFFSET_DEG));
            if (std::isfinite(gx) && std::isfinite(gy) && std::isfinite(gh)) {
                vm.gpsPose = Eigen::Vector3f(gx, gy, gh);
            }
        }

        BrainScreen::renderRuntime(vm);
        pros::delay(50);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Subsystem construction & registration
// ═══════════════════════════════════════════════════════════════════════════

static void subsystemInit() {
    drivetrain = new Drivetrain();
    intakes    = new Intakes();
    lift       = new Lift();
    solenoids  = new Solenoids();

    drivetrain->registerThis();
    intakes->registerThis();
    lift->registerThis();
    solenoids->registerThis();
}

// ═══════════════════════════════════════════════════════════════════════════
// GPS initial-pose acquisition (short poll — max ~2 s)
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Attempt to read a usable initial pose from the GPS.
 *
 * Depending on CONFIG::STARTUP_POSE_MODE:
 *   ConfiguredStartPoseOnly → return hard-coded config values immediately.
 *   GPSXYPlusIMUHeading     → GPS (x,y) + IMU heading.
 *   FullGPSInit             → GPS (x,y) + GPS heading (with offset).
 *
 * Polls GPS up to maxPolls times (50 ms apart) looking for a stable reading.
 * Falls back to config if it doesn't lock.
 */
static Eigen::Vector3f acquireInitialPose() {
    // Fallback pose from config (inches → metres, compass-deg → math-rad)
    Eigen::Vector3f configPose(
        static_cast<float>(CONFIG::START_POSE_X_IN) * CONFIG::INCH_TO_M,
        static_cast<float>(CONFIG::START_POSE_Y_IN) * CONFIG::INCH_TO_M,
        CONFIG::compassDegToMathRad(static_cast<float>(CONFIG::START_POSE_THETA_DEG)));

    if (CONFIG::STARTUP_POSE_MODE ==
        CONFIG::StartupPoseMode::ConfiguredStartPoseOnly) {
        return configPose;
    }

    pros::Gps gps(CONFIG::MCL_GPS_PORT);
    pros::delay(100);

    float lastX = 0, lastY = 0;
    bool hasLast = false;
    int stableCount = 0;
    constexpr int maxPolls = 40;   // 40 × 50 ms = 2 s max

    for (int i = 0; i < maxPolls; ++i) {
        auto pos = gps.get_position();
        Eigen::Vector2f rawGps(
            static_cast<float>(pos.x),
            static_cast<float>(pos.y));
        Eigen::Vector2f gpsField = CONFIG::transformGpsToFieldFrame(rawGps);
        float x = gpsField.x();
        float y = gpsField.y();

        if (!std::isfinite(x) || !std::isfinite(y)) {
            pros::delay(50);
            continue;
        }

        // Reject absurd magnitudes (outside plausible field area)
        if (std::fabs(x) > LocMath::GPS_ABSURD_LIMIT_M ||
            std::fabs(y) > LocMath::GPS_ABSURD_LIMIT_M) {
            pros::delay(50);
            continue;
        }

        // Skip initial zeros (GPS hasn't booted yet)
        if (!hasLast && std::abs(x) < 0.001f && std::abs(y) < 0.001f) {
            pros::delay(50);
            continue;
        }

        if (hasLast) {
            float dx = x - lastX, dy = y - lastY;
            float drift = std::sqrt(dx * dx + dy * dy);
            if (drift < static_cast<float>(CONFIG::STARTUP_GPS_READY_ERROR_M)) {
                ++stableCount;
            } else {
                stableCount = 0;
            }
        }
        lastX = x;
        lastY = y;
        hasLast = true;

        // Update loading screen inline
        float progress = 0.3f + 0.5f * (static_cast<float>(i) / maxPolls);
        char buf[40];
        std::snprintf(buf, sizeof(buf), "GPS: poll %d/%d", i + 1, maxPolls);
        renderInitStage(progress, "Init localization", buf);

        if (stableCount >= CONFIG::STARTUP_GPS_STABLE_SAMPLES) {
            break;
        }

        pros::delay(50);
    }

    if (!hasLast || stableCount < CONFIG::STARTUP_GPS_STABLE_SAMPLES) {
        renderInitStage(0.85f, "Init localization", "GPS timeout - config");
        pros::delay(200);
        std::printf("[INIT] GPS timeout, using config pose\n");
        return configPose;
    }

    float gpsHeadingRad = CONFIG::compassDegToMathRad(static_cast<float>(
        gps.get_heading() - CONFIG::MCL_GPS_HEADING_OFFSET_DEG));
    float imuHeadingRad = drivetrain->getHeading();

    // Reject non-finite headings — fall back to config
    if (!LocMath::isFinite(gpsHeadingRad) || !LocMath::isFinite(imuHeadingRad)) {
        std::printf("[INIT] heading non-finite (gps=%.2f imu=%.2f), using config\n",
                    gpsHeadingRad, imuHeadingRad);
        return configPose;
    }

    auto centerFromGps = [&](float headingRad) {
        const float cosT = std::cos(headingRad);
        const float sinT = std::sin(headingRad);
        const float ox = CONFIG::GPS_OFFSET_M.x() * cosT - CONFIG::GPS_OFFSET_M.y() * sinT;
        const float oy = CONFIG::GPS_OFFSET_M.x() * sinT + CONFIG::GPS_OFFSET_M.y() * cosT;
        return Eigen::Vector2f(lastX - ox, lastY - oy);
    };

    // Validate final pose is finite before returning
    auto safeReturn = [&](const Eigen::Vector3f& pose) -> Eigen::Vector3f {
        if (LocMath::isFinitePose(pose)) {
            std::printf("[INIT] startup pose: (%.3f, %.3f, %.2f)\n",
                        pose.x(), pose.y(), pose.z());
            return pose;
        }
        std::printf("[INIT] startup pose non-finite, using config\n");
        return configPose;
    };

    switch (CONFIG::STARTUP_POSE_MODE) {
        case CONFIG::StartupPoseMode::GPSXYPlusIMUHeading: {
            Eigen::Vector2f c = centerFromGps(imuHeadingRad);
            return safeReturn(Eigen::Vector3f(c.x(), c.y(), imuHeadingRad));
        }
        case CONFIG::StartupPoseMode::FullGPSInit: {
            Eigen::Vector2f c = centerFromGps(gpsHeadingRad);
            return safeReturn(Eigen::Vector3f(c.x(), c.y(), gpsHeadingRad));
        }
        default:
            return configPose;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Localization construction
// ═══════════════════════════════════════════════════════════════════════════

static void localizationInit() {
    std::vector<SensorModel*> sensors;

    static DistanceSensorModel distLeft(
        CONFIG::MCL_LEFT_DISTANCE_PORT,  CONFIG::DIST_LEFT_OFFSET_M,
        static_cast<float>(CONFIG::MCL_LEFT_DISTANCE_WEIGHT));
    static DistanceSensorModel distRight(
        CONFIG::MCL_RIGHT_DISTANCE_PORT, CONFIG::DIST_RIGHT_OFFSET_M,
        static_cast<float>(CONFIG::MCL_RIGHT_DISTANCE_WEIGHT));
    static DistanceSensorModel distFront(
        CONFIG::MCL_FRONT_DISTANCE_PORT, CONFIG::DIST_FRONT_OFFSET_M,
        static_cast<float>(CONFIG::MCL_FRONT_DISTANCE_WEIGHT));
    static DistanceSensorModel distBack(
        CONFIG::MCL_BACK_DISTANCE_PORT,  CONFIG::DIST_BACK_OFFSET_M,
        static_cast<float>(CONFIG::MCL_BACK_DISTANCE_WEIGHT));
    static GpsSensorModel gpsSensor(
        CONFIG::MCL_GPS_PORT,
        static_cast<float>(CONFIG::MCL_GPS_HEADING_OFFSET_DEG),
        CONFIG::MCL_GPS_OFFSET_X_M,
        CONFIG::MCL_GPS_OFFSET_Y_M);

    if (CONFIG::MCL_ENABLE_DISTANCE_SENSORS) {
        if (CONFIG::MCL_ENABLE_LEFT_DISTANCE_SENSOR)  sensors.push_back(&distLeft);
        if (CONFIG::MCL_ENABLE_RIGHT_DISTANCE_SENSOR) sensors.push_back(&distRight);
        if (CONFIG::MCL_ENABLE_FRONT_DISTANCE_SENSOR) sensors.push_back(&distFront);
        if (CONFIG::MCL_ENABLE_BACK_DISTANCE_SENSOR)  sensors.push_back(&distBack);
    }
    sensors.push_back(&gpsSensor);

    auto predictionFn = [&]() -> Eigen::Vector2f {
        return drivetrain->getDisplacement();
    };
    auto angleFn = [&]() -> QAngle {
        return QAngle(drivetrain->getHeading());
    };

    Eigen::Vector3f startPose = acquireInitialPose();
    drivetrain->syncLocalizationReference(startPose);

    renderInitStage(0.90f, "Init localization", "Starting filter...");

    particleFilter = new ParticleFilter(
        predictionFn, angleFn, sensors,
        startPose,
        CONFIG::NUM_PARTICLES);

    poseSource = [&]() -> Eigen::Vector3f {
        return particleFilter->getPrediction();
    };
}

// ═══════════════════════════════════════════════════════════════════════════
// Build the auton command from the current selector state
// ═══════════════════════════════════════════════════════════════════════════

static void buildAutonCommand() {
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
 * ALL screen rendering during init is done inline — no background tasks,
 * no mutexes, no atomics.  Tasks start only after init is fully complete.
 */
void initialize() {
    // 1. LCD on — show first frame
    BrainScreen::initialize();
    renderInitStage(0.0f, "Boot", "Booting...");

    // 2. Subsystems
    renderInitStage(0.10f, "Init subsystems", "Creating subsystem objects");
    subsystemInit();
    uiGps = std::make_unique<pros::Gps>(CONFIG::MCL_GPS_PORT);

    // 3. Localization (includes GPS poll with inline screen updates)
    renderInitStage(0.25f, "Init localization", "Preparing sensor models");
    localizationInit();

    // 4. Auton command
    renderInitStage(0.95f, "Init auton", "Building auton command graph");
    buildAutonCommand();

    // 5. Done — register selector buttons and rumble
    renderInitStage(1.0f, "Ready", "Controller rumble + start tasks");
    AutonSelector::init();
    screenStatus = "Ready";

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    master.rumble(".");

    // 6. Now start background tasks
    pros::Task::create(screen_update_loop, "screen");
    pros::Task::create(update_loop, "scheduler");
}

void disabled() {}

void competition_initialize() {
    buildAutonCommand();
}

void autonomous() {
    buildAutonCommand();
    if (autonCommand) {
        CommandScheduler::schedule(autonCommand);
    }
}

void opcontrol() {
    if (autonCommand && autonCommand->isScheduled()) {
        autonCommand->cancel();
    }

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Controller partner(pros::E_CONTROLLER_PARTNER);

    // ── Trigger bindings ────────────────────────────────────────────────

    CommandScheduler::addTrigger(
        Trigger([&]() { return master.get_digital(DIGITAL_R1); })
            .whileTrue(new IntakeSpinCommand(intakes, 127)));

    CommandScheduler::addTrigger(
        Trigger([&]() { return master.get_digital(DIGITAL_R2); })
            .whileTrue(new IntakeSpinCommand(intakes, -127)));

    CommandScheduler::addTrigger(
        Trigger([&]() { return master.get_digital(DIGITAL_L1); })
            .onTrue(shared::toggleTongue(solenoids)));

    CommandScheduler::addTrigger(
        Trigger([&]() { return master.get_digital(DIGITAL_L2); })
            .onTrue(shared::toggleWing(solenoids)));

    CommandScheduler::addTrigger(
        Trigger([&]() { return master.get_digital(DIGITAL_A); })
            .onTrue(shared::toggleSelect1(solenoids)));

    CommandScheduler::addTrigger(
        Trigger([&]() { return master.get_digital(DIGITAL_B); })
            .onTrue(shared::toggleSelect2(solenoids)));

    if (AUTON == Auton::SKILLS && autonCommand) {
        CommandScheduler::schedule(
            autonCommand->until([&]() {
                return partner.get_digital(DIGITAL_RIGHT);
            }));
    }

    while (true) {
        float forward = static_cast<float>(
            master.get_analog(ANALOG_LEFT_Y));
        float turn = static_cast<float>(
            master.get_analog(ANALOG_RIGHT_X));

        if (drivetrain->getCurrentCommand() == nullptr) {
            drivetrain->arcade(forward, turn);
        }

        pros::delay(20);
    }
}
