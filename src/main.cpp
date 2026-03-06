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
#include "ui/screenManager.h"
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

static std::unique_ptr<Command> autonCommand;

// ═══════════════════════════════════════════════════════════════════════════
// Pose source (shared lambda used by all path-followers)
// ═══════════════════════════════════════════════════════════════════════════

static std::function<Eigen::Vector3f()> poseSource;
static std::unique_ptr<pros::Gps> uiGps;
static std::unique_ptr<pros::Distance> uiDistLeft;
static std::unique_ptr<pros::Distance> uiDistRight;
static std::unique_ptr<pros::Distance> uiDistFront;
static std::unique_ptr<pros::Distance> uiDistBack;
static Eigen::Vector3f combinedPose(0.0f, 0.0f, 0.0f);
static Eigen::Vector2f combinedCorrection(0.0f, 0.0f);
static uint32_t firstInitScreenMs = 0;

// ═══════════════════════════════════════════════════════════════════════════
// Status string for screen (only touched from screen task after init)
// ═══════════════════════════════════════════════════════════════════════════

static std::string screenStatus = "";

struct GpsPoseSample {
    Eigen::Vector3f pose{0.0f, 0.0f, 0.0f};
    float errorM = -1.0f;
};

static float headingToCompassDeg(float headingRad) {
    return CONFIG::internalRadToGpsHeadingDeg(headingRad);
}

static float wrapAngleRad(float angleRad) {
    return std::atan2(std::sin(angleRad), std::cos(angleRad));
}

static BrainScreen::RuntimeViewModel::DistanceSensorViewModel sampleDistanceSensor(
    const char* label,
    const std::unique_ptr<pros::Distance>& sensor) {
    BrainScreen::RuntimeViewModel::DistanceSensorViewModel sample;
    sample.label = label;
    if (!sensor) return sample;

    const int rawMm = sensor->get_distance();
    if (rawMm <= 0 || rawMm >= 9999) {
        sample.confidence = sensor->get_confidence();
        return sample;
    }

    sample.valid = true;
    sample.rangeM = static_cast<float>(rawMm) / 1000.0f;
    sample.confidence = sensor->get_confidence();
    return sample;
}

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

static Eigen::Vector2f applyTargetDeadband(const Eigen::Vector2f& current,
                                           const Eigen::Vector2f& target,
                                           float deadband) {
    if (!LocMath::isFiniteVec2(current) ||
        !LocMath::isFiniteVec2(target) ||
        deadband <= 0.0f) {
        return target;
    }

    const Eigen::Vector2f delta = target - current;
    if (delta.norm() <= deadband) {
        return current;
    }
    return target;
}

static std::optional<GpsPoseSample> poseSampleFromReading(
    const GpsLocalization::FieldReading& reading) {
    if (!reading.hasHeading) return std::nullopt;

    const std::optional<Eigen::Vector2f> robotCenter =
        GpsLocalization::robotCenterFromSensorPose(
            reading.sensorFieldPos,
            reading.robotHeadingRad,
            CONFIG::GPS_OFFSET);
    if (!robotCenter) {
        return std::nullopt;
    }

    GpsPoseSample sample;
    sample.pose = Eigen::Vector3f(robotCenter->x(), robotCenter->y(), reading.robotHeadingRad);
    sample.errorM = reading.errorM;
    return sample;
}

static std::optional<GpsPoseSample> sampleGpsPose() {
    if (!uiGps) return std::nullopt;

    const std::optional<GpsLocalization::FieldReading> reading =
        GpsLocalization::readFieldReading(
            *uiGps,
            CONFIG::MCL_GPS_HEADING_OFFSET_deg,
            true);
    if (!reading || reading->errorM > CONFIG::GPS_ERROR_THRESHOLD.convert(meter)) {
        return std::nullopt;
    }

    return poseSampleFromReading(*reading);
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
    const bool still = motionNorm <= CONFIG::LOC_FUSION_STILLNESS_DEADBAND.convert(meter);

    bool haveTarget = false;
    Eigen::Vector2f targetCorrection = combinedCorrection;
    float maxCorrection = 0.0f;
    float maxStep = 0.0f;
    float correctionDeadband = 0.0f;

    const std::optional<GpsPoseSample> gpsSample = sampleGpsPose();
    if (still && gpsSample &&
        gpsSample->errorM <= CONFIG::LOC_GPS_RUNTIME_ERROR_MAX.convert(meter)) {
        targetCorrection = Eigen::Vector2f(
            gpsSample->pose.x() - odom.x(),
            gpsSample->pose.y() - odom.y());
        maxCorrection = CONFIG::LOC_GPS_CORRECTION_MAX.convert(meter);
        maxStep = CONFIG::LOC_GPS_CORRECTION_STEP.convert(meter);
        correctionDeadband = CONFIG::LOC_GPS_CORRECTION_DEADBAND.convert(meter);
        haveTarget = true;
    } else if (!still && particleFilter) {
        const Eigen::Vector3f pf = particleFilter->getPrediction();
        const Eigen::Vector2f pfCorrection(
            pf.x() - odom.x(),
            pf.y() - odom.y());
        const float correctionJump = (pfCorrection - combinedCorrection).norm();

        const double minEss = static_cast<double>(CONFIG::NUM_PARTICLES) *
            static_cast<double>(CONFIG::LOC_MCL_MIN_ESS_RATIO);

        if (LocMath::isFinitePose(pf) &&
            particleFilter->lastUpdateUsedMeasurements() &&
            particleFilter->getLastEss() >= minEss &&
            (particleFilter->getLastActiveAbsoluteSensorCount() > 0 ||
             particleFilter->getLastActiveSensorCount() >=
                static_cast<size_t>(CONFIG::LOC_MCL_MIN_ACTIVE_SENSORS)) &&
            correctionJump <= CONFIG::LOC_MCL_CORRECTION_JUMP_REJECT.convert(meter)) {
            targetCorrection = pfCorrection;
            maxCorrection = CONFIG::LOC_MCL_CORRECTION_MAX.convert(meter);
            maxStep = CONFIG::LOC_MCL_CORRECTION_STEP.convert(meter);
            correctionDeadband = CONFIG::LOC_MCL_CORRECTION_DEADBAND.convert(meter);
            haveTarget = true;
        }
    }

    if (haveTarget) {
        targetCorrection = clampVectorMagnitude(targetCorrection, maxCorrection);
        targetCorrection = applyTargetDeadband(
            combinedCorrection,
            targetCorrection,
            correctionDeadband);
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
    const float clampedProgress = std::max(0.0f, std::min(1.0f, progress));
    const int pct = static_cast<int>(std::round(clampedProgress * 100.0f));
    if (firstInitScreenMs == 0) {
        firstInitScreenMs = pros::millis();
    }

    std::printf("[BOOT] %3d%% %s", pct, stage.c_str());
    if (!detail.empty()) {
        std::printf(" - %s", detail.c_str());
    }
    std::printf("\n");

    pros::screen::set_eraser(0x00000000);
    pros::screen::erase();
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_LARGE, 16, 24, "69580A BOOT");
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 64, "Stage: %s", stage.c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 92, "Detail: %s",
                        detail.empty() ? "..." : detail.c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 132, "Progress: %d%%", pct);
    pros::screen::set_pen(0x00314A5D);
    pros::screen::draw_rect(16, 168, 463, 192);
    pros::screen::set_pen(0x001FC8B0);
    const int barRight = 18 + static_cast<int>(442.0f * clampedProgress);
    if (barRight >= 18) {
        pros::screen::fill_rect(18, 170, std::min(460, barRight), 190);
    }
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
                
                std::printf("[LOC] odom_delta=%.4f imu=%.1fdeg gps(%c)=(%.3f,%.3f,%.1fdeg,err=%.3f) "
                            "odom=(%.3f,%.3f,%.1fdeg) pf=(%.3f,%.3f,%.1fdeg) "
                            "comb=(%.3f,%.3f,%.1fdeg) corr=(%.3f,%.3f)|%.3f\n",
                    deltaLen,
                    headingToCompassDeg(imuH),
                    gpsOk ? 'Y' : 'N', gpsX, gpsY, headingToCompassDeg(gpsH), gpsErr,
                    odom.x(), odom.y(), headingToCompassDeg(odom.z()),
                    pf.x(), pf.y(), headingToCompassDeg(pf.z()),
                    fusedPose.x(), fusedPose.y(), headingToCompassDeg(fusedPose.z()),
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

        vm.selectedAuton = AutonSelector::getAuton();
        vm.selectedAlliance = AutonSelector::getAlliance();
        vm.auton = std::string(autonName(vm.selectedAuton));
        vm.alliance = std::string(allianceName(vm.selectedAlliance));
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
            vm.gpsPoseValid = true;
            vm.gpsErrorM = gpsSample->errorM;
        }

        vm.pfActiveSensors = particleFilter ? particleFilter->getLastActiveSensorCount() : 0;
        vm.pfAbsoluteSensors = particleFilter ? particleFilter->getLastActiveAbsoluteSensorCount() : 0;
        vm.pfUsedMeasurements = particleFilter ? particleFilter->lastUpdateUsedMeasurements() : false;
        vm.pfDidResample = particleFilter ? particleFilter->lastUpdateDidResample() : false;
        vm.pfEss = particleFilter ? particleFilter->getLastEss() : 0.0;
        vm.pfAverageWeight = particleFilter ? particleFilter->getLastAverageWeight() : 0.0;
        vm.pfRecoveryFraction = particleFilter ? particleFilter->getLastRecoveryFraction() : 0.0f;

        vm.distanceSensors[0] = sampleDistanceSensor("LEFT", uiDistLeft);
        vm.distanceSensors[1] = sampleDistanceSensor("RIGHT", uiDistRight);
        vm.distanceSensors[2] = sampleDistanceSensor("FRONT", uiDistFront);
        vm.distanceSensors[3] = sampleDistanceSensor("BACK", uiDistBack);

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
    const float defaultImuHeadingRad = CONFIG::DEFAULT_IMU_INIT_ANGLE.convert(radian);

    // Fallback pose from config (inches → metres, compass-deg → internal-rad)
    Eigen::Vector3f configPose(
        CONFIG::START_POSE_X.convert(meter),
        CONFIG::START_POSE_Y.convert(meter),
        CONFIG::START_POSE_THETA.convert(radian));

    if (CONFIG::STARTUP_POSE_MODE ==
        CONFIG::StartupPoseMode::ConfiguredStartPoseOnly) {
        return configPose;
    }

    // IMU heading is only relative until we explicitly seed it into the field frame.
    // For GPSXY+IMU startup, use the configured IMU init heading as that field reference,
    // then preserve any real rotation that happens during GPS lock acquisition.
    if (CONFIG::STARTUP_POSE_MODE == CONFIG::StartupPoseMode::GPSXYPlusIMUHeading && drivetrain) {
        drivetrain->resetHeading(defaultImuHeadingRad);
        pros::delay(20);
    }

    pros::Gps gps(CONFIG::MCL_GPS_PORT);
    pros::delay(100);

    Eigen::Vector2f lastFieldPos(0.0f, 0.0f);
    float lastHeadingRad = 0.0f;
    bool hasLast = false;
    int stableCount = 0;
    std::optional<GpsLocalization::FieldReading> lastReading;
    constexpr int pollDelayMs = 50;
    const int maxPolls = static_cast<int>(
        (CONFIG::STARTUP_GPS_MAX_WAIT_ms + pollDelayMs - 1) / pollDelayMs);
    const bool requireGpsHeadingDuringPoll =
        CONFIG::STARTUP_POSE_MODE == CONFIG::StartupPoseMode::FullGPSInit;

    for (int i = 0; i < maxPolls; ++i) {
        const std::optional<GpsLocalization::FieldReading> reading =
            GpsLocalization::readFieldReading(
                gps,
                CONFIG::MCL_GPS_HEADING_OFFSET_deg,
                requireGpsHeadingDuringPoll);
        if (!reading || reading->errorM > CONFIG::GPS_ERROR_THRESHOLD.convert(meter)) {
            pros::delay(pollDelayMs);
            continue;
        }

        const float x = reading->sensorFieldPos.x();
        const float y = reading->sensorFieldPos.y();

        // Skip initial zeros (GPS hasn't booted yet)
        if (!hasLast && std::abs(x) < 0.001f && std::abs(y) < 0.001f) {
            pros::delay(pollDelayMs);
            continue;
        }

        if (hasLast) {
            const float positionDrift = (reading->sensorFieldPos - lastFieldPos).norm();
            bool stable = positionDrift < CONFIG::STARTUP_GPS_READY_ERROR.convert(meter);
            if (requireGpsHeadingDuringPoll) {
                const float headingDrift = std::fabs(
                    wrapAngleRad(reading->robotHeadingRad - lastHeadingRad));
                stable = stable &&
                    headingDrift < CONFIG::STARTUP_GPS_READY_HEADING.convert(radian);
            }

            if (stable) {
                ++stableCount;
            } else {
                stableCount = 0;
            }
        }
        lastFieldPos = reading->sensorFieldPos;
        if (reading->hasHeading) {
            lastHeadingRad = reading->robotHeadingRad;
        }
        hasLast = true;
        lastReading = reading;

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

    if (!lastReading || !hasLast || stableCount < CONFIG::STARTUP_GPS_STABLE_SAMPLES) {
        renderInitStage(0.85f, "Init localization", "GPS timeout - config");
        pros::delay(200);
        std::printf("[INIT] GPS timeout, using config pose\n");
        return configPose;
    }

    auto centerFromGps = [&](float headingRad) -> std::optional<Eigen::Vector2f> {
        return GpsLocalization::robotCenterFromSensorPose(
            lastReading->sensorFieldPos,
            headingRad,
            CONFIG::GPS_OFFSET);
    };

    // Validate final pose is finite before returning
    auto safeReturn = [&](const Eigen::Vector3f& pose) -> Eigen::Vector3f {
        if (LocMath::isFinitePose(pose)) {
            std::printf("[INIT] startup pose: (%.3f, %.3f, %.1fdeg compass)\n",
                        pose.x(), pose.y(), headingToCompassDeg(pose.z()));
            return pose;
        }
        std::printf("[INIT] startup pose non-finite, using config\n");
        return configPose;
    };

    switch (CONFIG::STARTUP_POSE_MODE) {
        case CONFIG::StartupPoseMode::GPSXYPlusIMUHeading: {
            const float imuHeadingRad = drivetrain ? drivetrain->getHeading() : configPose.z();
            if (!LocMath::isFinite(imuHeadingRad)) {
                std::printf("[INIT] IMU heading non-finite, using config\n");
                return configPose;
            }
            const std::optional<Eigen::Vector2f> c = centerFromGps(imuHeadingRad);
            if (!c) {
                std::printf("[INIT] GPS centre solve failed (imu heading), using config\n");
                return configPose;
            }
            return safeReturn(Eigen::Vector3f(c->x(), c->y(), imuHeadingRad));
        }
        case CONFIG::StartupPoseMode::FullGPSInit: {
            if (!lastReading->hasHeading || !LocMath::isFinite(lastReading->robotHeadingRad)) {
                std::printf("[INIT] GPS heading non-finite, using config\n");
                return configPose;
            }
            const std::optional<Eigen::Vector2f> c = centerFromGps(lastReading->robotHeadingRad);
            if (!c) {
                std::printf("[INIT] GPS centre solve failed (gps heading), using config\n");
                return configPose;
            }
            return safeReturn(Eigen::Vector3f(c->x(), c->y(), lastReading->robotHeadingRad));
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
    static GpsSensorModel gpsSensor(
        CONFIG::MCL_GPS_PORT,
        CONFIG::MCL_GPS_HEADING_OFFSET_deg,
        CONFIG::GPS_OFFSET.x(),
        CONFIG::GPS_OFFSET.y(),
        CONFIG::GPS_STDDEV_BASE.convert(meter));

    static DistanceSensorModel distLeft(
        CONFIG::MCL_LEFT_DISTANCE_PORT,  CONFIG::DIST_LEFT_OFFSET,
        static_cast<float>(CONFIG::MCL_LEFT_DISTANCE_WEIGHT),
        CONFIG::MCL_DISTANCE_STDDEV.convert(meter), "distLeft");
    static DistanceSensorModel distRight(
        CONFIG::MCL_RIGHT_DISTANCE_PORT, CONFIG::DIST_RIGHT_OFFSET,
        static_cast<float>(CONFIG::MCL_RIGHT_DISTANCE_WEIGHT),
        CONFIG::MCL_DISTANCE_STDDEV.convert(meter), "distRight");
    static DistanceSensorModel distFront(
        CONFIG::MCL_FRONT_DISTANCE_PORT, CONFIG::DIST_FRONT_OFFSET,
        static_cast<float>(CONFIG::MCL_FRONT_DISTANCE_WEIGHT),
        CONFIG::MCL_DISTANCE_STDDEV.convert(meter), "distFront");
    static DistanceSensorModel distBack(
        CONFIG::MCL_BACK_DISTANCE_PORT,  CONFIG::DIST_BACK_OFFSET,
        static_cast<float>(CONFIG::MCL_BACK_DISTANCE_WEIGHT),
        CONFIG::MCL_DISTANCE_STDDEV.convert(meter), "distBack");

    if (CONFIG::MCL_ENABLE_GPS_SENSOR) {
        sensors.push_back(&gpsSensor);
    }

    if (CONFIG::MCL_ENABLE_DISTANCE_SENSORS) {
        if (CONFIG::MCL_ENABLE_LEFT_DISTANCE_SENSOR)  sensors.push_back(&distLeft);
        if (CONFIG::MCL_ENABLE_RIGHT_DISTANCE_SENSOR) sensors.push_back(&distRight);
        if (CONFIG::MCL_ENABLE_FRONT_DISTANCE_SENSOR) sensors.push_back(&distFront);
        if (CONFIG::MCL_ENABLE_BACK_DISTANCE_SENSOR)  sensors.push_back(&distBack);
    }

    auto predictionFn = [&]() -> Eigen::Vector2f {
        if (!drivetrain) {
            return Eigen::Vector2f(0.0f, 0.0f);
        }

        Eigen::Vector2f proposal = drivetrain->consumePendingFwdOnlyDisplacement();
        drivetrain->consumePendingDisplacement();
        return LocMath::isFiniteVec2(proposal) ? proposal : Eigen::Vector2f(0.0f, 0.0f);
    };
    auto angleFn = [&]() -> QAngle {
        if (!drivetrain) {
            return QAngle(0.0f);
        }

        const float heading = drivetrain->getHeading();
        return QAngle(LocMath::isFinite(heading) ? heading : 0.0f);
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
    std::printf("[LOC_CONFIG] Default IMU Init Heading: %.1f deg compass\n",
        CONFIG::DEFAULT_IMU_INIT_ANGLE_deg);
    std::printf("[LOC_CONFIG] GPS Hard Reject Threshold: %.3f in\n", CONFIG::GPS_ERROR_THRESHOLD_in);
    std::printf("[LOC_CONFIG] GPS Sensor Enabled: %s\n",
        CONFIG::MCL_ENABLE_GPS_SENSOR ? "YES" : "NO");
    std::printf("[LOC_CONFIG] Distance Sensors Disabled: %s\n",
        CONFIG::MCL_DISABLE_DISTANCE_SENSORS_WHILE_DEBUGGING ? "YES" : "NO");
    std::printf("[LOC_CONFIG] Active MCL Sensors (gps+distance): %zu\n", sensors.size());
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
    if (autonCommand && autonCommand->isScheduled()) {
        autonCommand->cancel();
    }

    AutonBuildContext context;
    context.drivetrain = drivetrain;
    context.intakes = intakes;
    context.lift = lift;
    context.solenoids = solenoids;
    context.poseSource = poseSource;

    autonCommand = autonCommands::makeAutonCommand(
        AutonSelector::getAuton(),
        context);
}

static void scheduleSelectedAuton() {
    buildAutonCommand();
    if (autonCommand) {
        CommandScheduler::schedule(autonCommand.get());
    }
}

static void launchSelectedAutonFromDriverControl() {
    if (autonCommand && autonCommand->isScheduled()) {
        return;
    }
    scheduleSelectedAuton();
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
    firstInitScreenMs = 0;
    BrainScreen::initialize();
    renderInitStage(0.0f, "Boot", "Booting...");
    AutonSelector::selectAuton(DEFAULT_AUTON_SELECTION);
    AutonSelector::selectAlliance(DEFAULT_ALLIANCE_SELECTION);

    // 2. Subsystems
    renderInitStage(0.10f, "Init subsystems", "Creating subsystem objects");
    subsystemInit();
    renderInitStage(0.18f, "Init subsystems", "Calibrating IMU");
    if (drivetrain) {
        drivetrain->calibrateImu();
        drivetrain->resetHeading(CONFIG::DEFAULT_IMU_INIT_ANGLE.convert(radian));
    }
    uiGps = std::make_unique<pros::Gps>(CONFIG::MCL_GPS_PORT);
    uiDistLeft = std::make_unique<pros::Distance>(CONFIG::MCL_LEFT_DISTANCE_PORT);
    uiDistRight = std::make_unique<pros::Distance>(CONFIG::MCL_RIGHT_DISTANCE_PORT);
    uiDistFront = std::make_unique<pros::Distance>(CONFIG::MCL_FRONT_DISTANCE_PORT);
    uiDistBack = std::make_unique<pros::Distance>(CONFIG::MCL_BACK_DISTANCE_PORT);

    // 3. Localization (includes GPS poll with inline screen updates)
    renderInitStage(0.25f, "Init localization", "Preparing sensor models");
    localizationInit();

    // 4. Auton command
    renderInitStage(0.95f, "Init auton", "Building auton command graph");
    buildAutonCommand();

    // 5. Done — register selector buttons and rumble
    renderInitStage(1.0f, "Ready", "Controller rumble + start tasks");

    const uint32_t now = pros::millis();
    const uint32_t visibleFor = (firstInitScreenMs == 0 || now < firstInitScreenMs)
        ? 0
        : (now - firstInitScreenMs);
    if (visibleFor < CONFIG::STARTUP_SCREEN_MIN_VISIBLE_ms) {
        pros::delay(CONFIG::STARTUP_SCREEN_MIN_VISIBLE_ms - visibleFor);
    }
    pros::delay(CONFIG::STARTUP_READY_HOLD_ms);

    AutonSelector::init();
    ScreenManagerUI::init();
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
    scheduleSelectedAuton();
}

void opcontrol() {
    // Opcontrol may re-enter after disable/enable cycles; reset scheduler
    // state to avoid stale commands from the previous run.
    CommandScheduler::reset();

    if (autonCommand && autonCommand->isScheduled()) {
        autonCommand->cancel();
    }

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    IntakeSpinCommand intakeInCommand(intakes, 127);
    IntakeSpinCommand intakeOutCommand(intakes, -127);
    bool downBAutonLatch = false;

    while (true) {
        const bool r1Held = master.get_digital(DIGITAL_R1);
        const bool r2Held = master.get_digital(DIGITAL_R2);
        const bool downHeld = master.get_digital(DIGITAL_DOWN);
        const bool bHeld = master.get_digital(DIGITAL_B);
        const bool downBHeld = downHeld && bHeld;

        // Preserve command-based intake ownership while polling buttons directly.
        if (r2Held) {
            if (intakeInCommand.isScheduled()) {
                intakeInCommand.cancel();
            }
            if (!intakeOutCommand.isScheduled()) {
                CommandScheduler::schedule(&intakeOutCommand);
            }
        } else if (r1Held) {
            if (intakeOutCommand.isScheduled()) {
                intakeOutCommand.cancel();
            }
            if (!intakeInCommand.isScheduled()) {
                CommandScheduler::schedule(&intakeInCommand);
            }
        } else {
            if (intakeInCommand.isScheduled()) {
                intakeInCommand.cancel();
            }
            if (intakeOutCommand.isScheduled()) {
                intakeOutCommand.cancel();
            }
        }

        if (master.get_digital_new_press(DIGITAL_L1)) {
            solenoids->toggleTongue();
        }
        if (master.get_digital_new_press(DIGITAL_L2)) {
            solenoids->toggleWing();
        }
        if (master.get_digital_new_press(DIGITAL_A)) {
            solenoids->toggleSelect1();
        }
        if (!downHeld && master.get_digital_new_press(DIGITAL_B)) {
            solenoids->toggleSelect2();
        }

        if (downBHeld && !downBAutonLatch &&
            !pros::competition::is_connected()) {
            launchSelectedAutonFromDriverControl();
        }
        downBAutonLatch = downBHeld;

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
