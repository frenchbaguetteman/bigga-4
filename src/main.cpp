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
#include <optional>
#include <utility>

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
static Eigen::Vector3f combinedPose(0.0f, 0.0f, 0.0f);
static Eigen::Vector2f combinedCorrection(0.0f, 0.0f);

// ═══════════════════════════════════════════════════════════════════════════
// Status string for screen (only touched from screen task after init)
// ═══════════════════════════════════════════════════════════════════════════

static std::string screenStatus = "";

struct GpsPoseSample {
    Eigen::Vector3f pose{0.0f, 0.0f, 0.0f};
    float errorM = -1.0f;
};

static Eigen::Vector2f clampVectorMagnitude(const Eigen::Vector2f& v, float maxMagnitude) {
    if (!LocMath::isFiniteVec2(v) || maxMagnitude <= 0.0f) {
        return Eigen::Vector2f(0.0f, 0.0f);
    }

    const float norm = v.norm();
    if (norm <= maxMagnitude || norm <= 1e-6f) return v;
    return v * (maxMagnitude / norm);
}

static Eigen::Vector2f moveTowardsVec2(const Eigen::Vector2f& current,
                                       const Eigen::Vector2f& target,
                                       float maxStep) {
    if (!LocMath::isFiniteVec2(current) || !LocMath::isFiniteVec2(target) || maxStep <= 0.0f) {
        return current;
    }

    const Eigen::Vector2f delta = target - current;
    const float dist = delta.norm();
    if (dist <= maxStep || dist <= 1e-6f) return target;
    return current + delta * (maxStep / dist);
}

static std::optional<GpsPoseSample> sampleGpsPose() {
    if (!uiGps) return std::nullopt;

    auto pos = uiGps->get_position();
    const float errorM = static_cast<float>(uiGps->get_error());
    Eigen::Vector2f rawGps(
        static_cast<float>(pos.x),
        static_cast<float>(pos.y));
    Eigen::Vector2f gpsField = CONFIG::transformGpsToFieldFrame(rawGps);
    const float heading = CONFIG::gpsSensorHeadingDegToInternalRad(static_cast<float>(
        uiGps->get_heading() - CONFIG::MCL_GPS_HEADING_OFFSET_deg));

    if (!std::isfinite(gpsField.x()) || !std::isfinite(gpsField.y()) ||
        !std::isfinite(heading) || !std::isfinite(errorM) || errorM < 0.0f) {
        return std::nullopt;
    }

    const float cosT = std::cos(heading);
    const float sinT = std::sin(heading);
    const float ox = CONFIG::GPS_OFFSET.x() * cosT - CONFIG::GPS_OFFSET.y() * sinT;
    const float oy = CONFIG::GPS_OFFSET.x() * sinT + CONFIG::GPS_OFFSET.y() * cosT;

    GpsPoseSample sample;
    sample.pose = Eigen::Vector3f(gpsField.x() - ox, gpsField.y() - oy, heading);
    sample.errorM = errorM;
    return sample;
}

static Eigen::Vector3f computeCombinedPose() {
    Eigen::Vector3f odom = drivetrain ? drivetrain->getOdomPose()
                                      : Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    if (!LocMath::isFinitePose(odom)) {
        combinedCorrection = Eigen::Vector2f(0.0f, 0.0f);
        combinedPose = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        return combinedPose;
    }

    const float motionNorm = drivetrain ? drivetrain->getLastStepDisplacementDebug().norm() : 0.0f;
    const bool still = motionNorm <= CONFIG::LOC_FUSION_STILLNESS_DEADBAND.getValue();

    bool haveTarget = false;
    Eigen::Vector2f targetCorrection = combinedCorrection;
    float maxCorrection = 0.0f;
    float maxStep = 0.0f;

    const std::optional<GpsPoseSample> gpsSample = sampleGpsPose();
    if (still && gpsSample &&
        gpsSample->errorM <= CONFIG::LOC_GPS_RUNTIME_ERROR_MAX.getValue()) {
        targetCorrection = Eigen::Vector2f(
            gpsSample->pose.x() - odom.x(),
            gpsSample->pose.y() - odom.y());
        maxCorrection = CONFIG::LOC_GPS_CORRECTION_MAX.getValue();
        maxStep = CONFIG::LOC_GPS_CORRECTION_STEP.getValue();
        haveTarget = true;
    } else if (!still && particleFilter) {
        const Eigen::Vector3f pf = particleFilter->getPrediction();
        const Eigen::Vector2f pfCorrection(
            pf.x() - odom.x(),
            pf.y() - odom.y());
        const float correctionJump = (pfCorrection - combinedCorrection).norm();

        if (LocMath::isFinitePose(pf) &&
            particleFilter->lastUpdateUsedMeasurements() &&
            particleFilter->getLastActiveSensorCount() >=
                static_cast<size_t>(CONFIG::LOC_MCL_MIN_ACTIVE_SENSORS) &&
            correctionJump <= CONFIG::LOC_MCL_CORRECTION_JUMP_REJECT.getValue()) {
            targetCorrection = pfCorrection;
            maxCorrection = CONFIG::LOC_MCL_CORRECTION_MAX.getValue();
            maxStep = CONFIG::LOC_MCL_CORRECTION_STEP.getValue();
            haveTarget = true;
        }
    }

    if (haveTarget) {
        targetCorrection = clampVectorMagnitude(targetCorrection, maxCorrection);
        combinedCorrection = moveTowardsVec2(combinedCorrection, targetCorrection, maxStep);
    } else if (!LocMath::isFiniteVec2(combinedCorrection)) {
        combinedCorrection = Eigen::Vector2f(0.0f, 0.0f);
    }

    Eigen::Vector2f fusedXY(
        odom.x() + combinedCorrection.x(),
        odom.y() + combinedCorrection.y());
    if (!LocMath::isFiniteVec2(fusedXY)) {
        combinedCorrection = Eigen::Vector2f(0.0f, 0.0f);
        fusedXY = Eigen::Vector2f(odom.x(), odom.y());
    }

    combinedPose = Eigen::Vector3f(fusedXY.x(), fusedXY.y(), odom.z());
    return combinedPose;
}

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
        // CRITICAL ORDER:
        // 1. Run CommandScheduler first — this calls drivetrain::periodic()
        //    which calls updateOdometry() and accumulates m_pendingDisplacement
        CommandScheduler::run();
        
        // 2. Then run ParticleFilter::update() which calls the predictionFn lambda
        //    EXACTLY ONCE. The filter consumes forward odometry as its motion
        //    proposal while distance sensors handle the measurement correction.
        if (particleFilter) {
            particleFilter->update();
        }
        Eigen::Vector3f fusedPose = computeCombinedPose();

        // Rate-limited debug instrumentation
        if constexpr (CONFIG::ODOM_DEBUG_ENABLE) {
            uint32_t now = pros::millis();
            if (now - lastDebugMs >= CONFIG::ODOM_DEBUG_LOG_EVERY_ms) {
                lastDebugMs = now;
                
                // === Raw IMU heading ===
                float imuH = drivetrain ? drivetrain->getHeading() : 0.0f;
                
                float gpsX = 0, gpsY = 0, gpsH = 0, gpsErr = -1.0f;
                bool gpsOk = false;
                if (auto gpsSample = sampleGpsPose()) {
                    gpsX = gpsSample->pose.x();
                    gpsY = gpsSample->pose.y();
                    gpsH = gpsSample->pose.z();
                    gpsErr = gpsSample->errorM;
                    gpsOk = true;
                }
                
                Eigen::Vector2f odomdelta = drivetrain ? drivetrain->getLastStepDisplacementDebug()
                                                       : Eigen::Vector2f(0, 0);
                float deltaLen = odomdelta.norm();
                
                auto odom = drivetrain ? drivetrain->getOdomPose()
                                       : Eigen::Vector3f(0, 0, 0);
                auto pf   = particleFilter ? particleFilter->getPrediction()
                                           : Eigen::Vector3f(0, 0, 0);
                
                std::printf("[LOC] odom_delta=%.4f imu=%.3f gps(%c)=(%.3f,%.3f,%.3f,err=%.3f) "
                            "odom=(%.3f,%.3f,%.3f) pf=(%.3f,%.3f,%.3f) "
                            "comb=(%.3f,%.3f,%.3f) corr=(%.3f,%.3f)|%.3f\n",
                    deltaLen,
                    imuH,
                    gpsOk ? 'Y' : 'N', gpsX, gpsY, gpsH, gpsErr,
                    odom.x(), odom.y(), odom.z(),
                    pf.x(), pf.y(), pf.z(),
                    fusedPose.x(), fusedPose.y(), fusedPose.z(),
                    combinedCorrection.x(), combinedCorrection.y(), combinedCorrection.norm());
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

        vm.combinedPose = combinedPose;
        if (!LocMath::isFinitePose(vm.combinedPose)) {
            vm.combinedPose = vm.pureOdomPose;
        }
        if (LocMath::isFinitePose(vm.combinedPose)) {
            lastFiniteCombined = vm.combinedPose;
        } else {
            vm.combinedPose = lastFiniteCombined;
        }
        vm.pose = vm.combinedPose;

        if (auto gpsSample = sampleGpsPose()) {
            vm.gpsPose = gpsSample->pose;
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
        CONFIG::START_POSE_X.getValue(),
        CONFIG::START_POSE_Y.getValue(),
        CONFIG::START_POSE_THETA.getValue());

    if (CONFIG::STARTUP_POSE_MODE ==
        CONFIG::StartupPoseMode::ConfiguredStartPoseOnly) {
        return configPose;
    }

    // IMU heading is only relative until we explicitly seed it into the field frame.
    // For GPSXY+IMU startup, use the configured start heading as that field reference,
    // then preserve any real rotation that happens during GPS lock acquisition.
    if (CONFIG::STARTUP_POSE_MODE == CONFIG::StartupPoseMode::GPSXYPlusIMUHeading && drivetrain) {
        drivetrain->resetHeading(configPose.z());
        pros::delay(20);
    }

    pros::Gps gps(CONFIG::MCL_GPS_PORT);
    pros::delay(100);

    float lastX = 0, lastY = 0;
    bool hasLast = false;
    int stableCount = 0;
    constexpr int pollDelayMs = 50;
    const int maxPolls = static_cast<int>(
        (CONFIG::STARTUP_GPS_MAX_WAIT_ms + pollDelayMs - 1) / pollDelayMs);

    for (int i = 0; i < maxPolls; ++i) {
        auto pos = gps.get_position();
        Eigen::Vector2f rawGps(
            static_cast<float>(pos.x),
            static_cast<float>(pos.y));
        Eigen::Vector2f gpsField = CONFIG::transformGpsToFieldFrame(rawGps);
        float x = gpsField.x();
        float y = gpsField.y();

        if (!std::isfinite(x) || !std::isfinite(y)) {
            pros::delay(pollDelayMs);
            continue;
        }

        // Reject absurd magnitudes (outside plausible field area)
        if (std::fabs(x) > LocMath::GPS_ABSURD_LIMIT_M ||
            std::fabs(y) > LocMath::GPS_ABSURD_LIMIT_M) {
            pros::delay(pollDelayMs);
            continue;
        }

        // Skip initial zeros (GPS hasn't booted yet)
        if (!hasLast && std::abs(x) < 0.001f && std::abs(y) < 0.001f) {
            pros::delay(pollDelayMs);
            continue;
        }

        if (hasLast) {
            float dx = x - lastX, dy = y - lastY;
            float drift = std::sqrt(dx * dx + dy * dy);
            if (drift < CONFIG::STARTUP_GPS_READY_ERROR.getValue()) {
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

        pros::delay(pollDelayMs);
    }

    if (!hasLast || stableCount < CONFIG::STARTUP_GPS_STABLE_SAMPLES) {
        renderInitStage(0.85f, "Init localization", "GPS timeout - config");
        pros::delay(200);
        std::printf("[INIT] GPS timeout, using config pose\n");
        return configPose;
    }

    float gpsHeadingRad = CONFIG::gpsSensorHeadingDegToInternalRad(static_cast<float>(
        gps.get_heading() - CONFIG::MCL_GPS_HEADING_OFFSET_deg));
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
        const float ox = CONFIG::GPS_OFFSET.x() * cosT - CONFIG::GPS_OFFSET.y() * sinT;
        const float oy = CONFIG::GPS_OFFSET.x() * sinT + CONFIG::GPS_OFFSET.y() * cosT;
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
        CONFIG::MCL_LEFT_DISTANCE_PORT,  CONFIG::DIST_LEFT_OFFSET,
        static_cast<float>(CONFIG::MCL_LEFT_DISTANCE_WEIGHT),
        CONFIG::MCL_DISTANCE_STDDEV.getValue(), "distLeft");
    static DistanceSensorModel distRight(
        CONFIG::MCL_RIGHT_DISTANCE_PORT, CONFIG::DIST_RIGHT_OFFSET,
        static_cast<float>(CONFIG::MCL_RIGHT_DISTANCE_WEIGHT),
        CONFIG::MCL_DISTANCE_STDDEV.getValue(), "distRight");
    static DistanceSensorModel distFront(
        CONFIG::MCL_FRONT_DISTANCE_PORT, CONFIG::DIST_FRONT_OFFSET,
        static_cast<float>(CONFIG::MCL_FRONT_DISTANCE_WEIGHT),
        CONFIG::MCL_DISTANCE_STDDEV.getValue(), "distFront");
    static DistanceSensorModel distBack(
        CONFIG::MCL_BACK_DISTANCE_PORT,  CONFIG::DIST_BACK_OFFSET,
        static_cast<float>(CONFIG::MCL_BACK_DISTANCE_WEIGHT),
        CONFIG::MCL_DISTANCE_STDDEV.getValue(), "distBack");

    if (CONFIG::MCL_ENABLE_DISTANCE_SENSORS) {
        if (CONFIG::MCL_ENABLE_LEFT_DISTANCE_SENSOR)  sensors.push_back(&distLeft);
        if (CONFIG::MCL_ENABLE_RIGHT_DISTANCE_SENSOR) sensors.push_back(&distRight);
        if (CONFIG::MCL_ENABLE_FRONT_DISTANCE_SENSOR) sensors.push_back(&distFront);
        if (CONFIG::MCL_ENABLE_BACK_DISTANCE_SENSOR)  sensors.push_back(&distBack);
    }

    auto predictionFn = [&]() -> Eigen::Vector2f {
        Eigen::Vector2f proposal = drivetrain->consumePendingFwdOnlyDisplacement();
        drivetrain->consumePendingDisplacement();
        return proposal;
    };
    auto angleFn = [&]() -> QAngle {
        return QAngle(drivetrain->getHeading());
    };

    Eigen::Vector3f startPose = acquireInitialPose();
    drivetrain->syncLocalizationReference(startPose);
    combinedCorrection = Eigen::Vector2f(0.0f, 0.0f);
    combinedPose = startPose;

    renderInitStage(0.90f, "Init localization", "Starting filter...");

    // === Localization Configuration Diagnostics ===
    std::printf("\n[LOC_CONFIG] ═════════════════════════════════════════════════\n");
    std::printf("[LOC_CONFIG] Canonical Internal Frame Convention:\n");
    std::printf("[LOC_CONFIG]   Position: metres\n");
    std::printf("[LOC_CONFIG]   Heading:  radians (0 = +X east/forward, CCW+)\n");
    std::printf("[LOC_CONFIG]   Field:    +X east, +Y north/left\n");
    std::printf("[LOC_CONFIG]\n");
    std::printf("[LOC_CONFIG] Startup Pose Mode: ");
    switch (CONFIG::STARTUP_POSE_MODE) {
        case CONFIG::StartupPoseMode::ConfiguredStartPoseOnly:
            std::printf("ConfiguredStartPoseOnly\n");
            break;
        case CONFIG::StartupPoseMode::GPSXYPlusIMUHeading:
            std::printf("GPSXYPlusIMUHeading\n");
            break;
        case CONFIG::StartupPoseMode::FullGPSInit:
            std::printf("FullGPSInit\n");
            break;
        default:
            std::printf("UNKNOWN\n");
    }
    std::printf("[LOC_CONFIG] Start Pose: (%.3f, %.3f, %.3f rad)\n",
        startPose.x(), startPose.y(), startPose.z());
    std::printf("[LOC_CONFIG] GPS Error Threshold (UI/startup only): %.3f in\n", CONFIG::GPS_ERROR_THRESHOLD_in);
    std::printf("[LOC_CONFIG] Distance Sensors Disabled: %s\n",
        CONFIG::MCL_DISABLE_DISTANCE_SENSORS_WHILE_DEBUGGING ? "YES" : "NO");
    std::printf("[LOC_CONFIG] Active MCL Sensors (distance-only): %zu\n", sensors.size());
    std::printf("[LOC_CONFIG] ═════════════════════════════════════════════════\n\n");

    particleFilter = new ParticleFilter(
        predictionFn, angleFn, sensors,
        startPose,
        CONFIG::NUM_PARTICLES);

    poseSource = [&]() -> Eigen::Vector3f {
        return LocMath::isFinitePose(combinedPose)
            ? combinedPose
            : (drivetrain ? drivetrain->getOdomPose()
                          : Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    };
}

// ═══════════════════════════════════════════════════════════════════════════
// Build the auton command from the current selector state
// ═══════════════════════════════════════════════════════════════════════════

static void buildAutonCommand() {
    if (autonCommand) {
        if (autonCommand->isScheduled()) {
            autonCommand->cancel();
        }
        delete autonCommand;
        autonCommand = nullptr;
    }

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
    renderInitStage(0.18f, "Init subsystems", "Calibrating IMU");
    if (drivetrain) {
        drivetrain->calibrateImu();
    }
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
    // Opcontrol may re-enter after disable/enable cycles; reset scheduler
    // state to avoid duplicated trigger bindings and stale commands.
    CommandScheduler::reset();

    if (autonCommand && autonCommand->isScheduled()) {
        autonCommand->cancel();
    }

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Controller partner(pros::E_CONTROLLER_PARTNER);

    // ── Trigger bindings ────────────────────────────────────────────────

    Trigger r1Trigger([&]() { return master.get_digital(DIGITAL_R1); });
    r1Trigger.whileTrue(new IntakeSpinCommand(intakes, 127));
    CommandScheduler::addTrigger(std::move(r1Trigger));

    Trigger r2Trigger([&]() { return master.get_digital(DIGITAL_R2); });
    r2Trigger.whileTrue(new IntakeSpinCommand(intakes, -127));
    CommandScheduler::addTrigger(std::move(r2Trigger));

    Trigger l1Trigger([&]() { return master.get_digital(DIGITAL_L1); });
    l1Trigger.onTrue(shared::toggleTongue(solenoids));
    CommandScheduler::addTrigger(std::move(l1Trigger));

    Trigger l2Trigger([&]() { return master.get_digital(DIGITAL_L2); });
    l2Trigger.onTrue(shared::toggleWing(solenoids));
    CommandScheduler::addTrigger(std::move(l2Trigger));

    Trigger aTrigger([&]() { return master.get_digital(DIGITAL_A); });
    aTrigger.onTrue(shared::toggleSelect1(solenoids));
    CommandScheduler::addTrigger(std::move(aTrigger));

    Trigger bTrigger([&]() { return master.get_digital(DIGITAL_B); });
    bTrigger.onTrue(shared::toggleSelect2(solenoids));
    CommandScheduler::addTrigger(std::move(bTrigger));

    if (AUTON == Auton::SKILLS && autonCommand) {
        CommandScheduler::schedule(autonCommand);
    }

    while (true) {
        if (AUTON == Auton::SKILLS && autonCommand && autonCommand->isScheduled() &&
            partner.get_digital(DIGITAL_RIGHT)) {
            autonCommand->cancel();
        }

        float forward = static_cast<float>(
            master.get_analog(ANALOG_LEFT_Y));
        float turn = static_cast<float>(
            master.get_analog(ANALOG_RIGHT_X));

        if (drivetrain->getCurrentCommand() == nullptr) {
            drivetrain->driverArcade(forward, turn);
        } else {
            drivetrain->resetDriverAssistState();
        }

        pros::delay(20);
    }
}
