/**
 * @file main.cpp
 * 69580A — PROS V5 competition lifecycle.
 *
 * Creates two periodic tasks:
 *   • command-scheduler loop  (10 ms)  — advances the command graph
 *   • screen-update loop      (50 ms)  — redraws the brain LCD with pose + auton
 *
 * autonomous() runs the selected auton directly in the current task.
 * opcontrol() runs arcade drive + trigger bindings, including DOWN+B to call
 * the selected autonomous inline.
 */
#include "main.h"
#include "localization/controllerPoseGuard.h"
#include "localization/localizationFusion.h"
#include "tuning/relayPidAutotuner.h"
#include "ui/brainScreen.h"
#include "ui/screenManager.h"
#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <string>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

namespace {

struct PoseState {
    float x;
    float y;
    float z;
};

} // namespace

static Drivetrain* drivetrain;
static Intakes* intakes;
static Lift* lift;
static pros::adi::DigitalOut* selector;
static pros::adi::DigitalOut* top;
static pros::adi::DigitalOut* tongue;
static pros::adi::DigitalOut* wing;

static ParticleFilter* particleFilter = nullptr;
static LocalizationFusion* fusion = nullptr;

// ── Shared state protected by locMutex ──────────────────────────────────────
static pros::Mutex* locMutex;
static PoseState combinedPoseState;
static PoseState controllerPoseState;
static ControllerPoseGuard* controllerPoseGuard;

static pros::Gps* uiGps;
static pros::Distance* uiDistLeft;
static pros::Distance* uiDistRight;
static pros::Distance* uiDistFront;
static pros::Distance* uiDistBack;
static pros::Task* screenTaskHandle;
static pros::Task* schedulerTaskHandle;
static uint32_t firstInitScreenMs = 0;
static std::FILE* startupLogFile = nullptr;
static bool startupLogOpened = false;
static uint32_t startupLogIndex = 0;

static char screenStatus[64];
static constexpr uint32_t SCREEN_NOTIFY_PERIOD_ms = 50;
static constexpr uint32_t SCREEN_NOTIFY_FALLBACK_ms = 250;

// GpsPoseSample is now defined in LocalizationFusion.
using GpsPoseSample = LocalizationFusion::GpsPoseSample;

// Utility aliases — canonical implementations live in LocMath::
static float headingToCompassDeg(float headingRad) {
    return LocMath::headingToCompassDeg(headingRad);
}

static float wrapAngleRad(float angleRad) {
    return LocMath::wrapAngle(angleRad);
}

static Eigen::Vector3f loadPose(const PoseState& state) {
    return Eigen::Vector3f(state.x, state.y, state.z);
}

static void storePose(PoseState* state, const Eigen::Vector3f& pose) {
    state->x = pose.x();
    state->y = pose.y();
    state->z = pose.z();
}

static const char* currentScreenStatus() {
    return screenStatus[0] == '\0' ? "Ready" : screenStatus;
}

static void setScreenStatus(const char* status) {
    if (status == nullptr || status[0] == '\0') {
        screenStatus[0] = '\0';
        return;
    }
    std::snprintf(screenStatus, sizeof(screenStatus), "%s", status);
}

static Eigen::Vector3f currentControllerPose() {
    const Eigen::Vector3f odomPose = drivetrain
        ? drivetrain->getOdomPose()
        : Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    if (locMutex) {
        std::scoped_lock guard(*locMutex);
        const Eigen::Vector3f pose = loadPose(controllerPoseState);
        return LocMath::isFinitePose(pose) ? pose : odomPose;
    }

    const Eigen::Vector3f pose = loadPose(controllerPoseState);
    return LocMath::isFinitePose(pose) ? pose : odomPose;
}

static void openStartupLog() {
    if (startupLogOpened) return;
    startupLogOpened = true;
    startupLogFile = std::fopen("/usd/startup.log", "w");
    if (startupLogFile) {
        std::setvbuf(startupLogFile, nullptr, _IONBF, 0);
    }
}

static void startupLog(const char* fmt, ...) {
    openStartupLog();

    char message[192];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(message, sizeof(message), fmt, args);
    va_end(args);

    const unsigned long now = static_cast<unsigned long>(pros::millis());
    std::printf("[TRACE %03lu @ %lums] %s\n",
                static_cast<unsigned long>(++startupLogIndex),
                now,
                message);
    if (startupLogFile) {
        std::fprintf(startupLogFile,
                     "[TRACE %03lu @ %lums] %s\n",
                     static_cast<unsigned long>(startupLogIndex),
                     now,
                     message);
        std::fflush(startupLogFile);
    }
}

static void ensureLocMutex() {
    if (!locMutex) {
        locMutex = new pros::Mutex();
    }
}

static bool subsystemsReady() {
    return drivetrain && intakes && lift && selector && top && tongue && wing;
}

static void requestScreenRefresh() {
    if (screenTaskHandle &&
        static_cast<pros::task_t>(*screenTaskHandle) != nullptr) {
        screenTaskHandle->notify();
    }
}

static std::optional<GpsPoseSample> poseSampleFromReading(
    const GpsLocalization::FieldReading& reading) {
    if (!reading.hasHeading) return std::nullopt;

    const std::optional<Eigen::Vector2f> robotCenter =
        GpsLocalization::robotCenterFromSensorPose(
            reading.sensorFieldPos,
            reading.robotHeadingRad,
            CONFIG::gpsOffset());
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

static BrainScreen::RuntimeViewModel::DistanceSensorViewModel sampleDistanceSensor(
    const char* label,
    pros::Distance* sensor) {
    BrainScreen::RuntimeViewModel::DistanceSensorViewModel sample;
    sample.label = label;
    if (!sensor) return sample;

    const int rawMm = sensor->get_distance();
    sample.confidence = sensor->get_confidence();
    if (rawMm <= 0 || rawMm >= 9999) {
        return sample;
    }

    sample.valid = true;
    sample.rangeM = static_cast<float>(rawMm) / 1000.0f;
    return sample;
}

// computeCombinedPose() logic is now in LocalizationFusion::update().

static void renderInitStage(float progress,
                            const std::string& stage,
                            const std::string& detail = "") {
    const float clampedProgress = std::max(0.0f, std::min(1.0f, progress));
    const int pct = static_cast<int>(std::round(clampedProgress * 100.0f));
    if (firstInitScreenMs == 0) {
        firstInitScreenMs = pros::millis();
    }

    startupLog("boot %3d%% %s%s%s",
               pct,
               stage.c_str(),
               detail.empty() ? "" : " - ",
               detail.empty() ? "" : detail.c_str());

    BrainScreen::InitViewModel vm;
    vm.progress = clampedProgress;
    vm.stageTitle = stage;
    vm.detail = detail.empty() ? "..." : detail;
    BrainScreen::renderInit(vm);
}

static BrainScreen::RuntimeViewModel captureRuntimeViewModel() {
    static Eigen::Vector3f lastFiniteCombined(0.0f, 0.0f, 0.0f);

    BrainScreen::RuntimeViewModel vm;
    vm.selectedAuton = AutonSelector::getAuton();
    vm.auton = std::string(autonName(vm.selectedAuton));
    vm.status = currentScreenStatus();
    char autotuneStatus[96];
    tuning::RelayPidAutotuner::copyStatus(autotuneStatus, sizeof(autotuneStatus));
    if (autotuneStatus[0] != '\0') {
        vm.status = autotuneStatus;
    }

    if (drivetrain) {
        vm.pureOdomPose = drivetrain->getOdomPose();
        if (!LocMath::isFinitePose(vm.pureOdomPose)) {
            vm.pureOdomPose = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        }
    }
    {
        if (locMutex) {
            std::scoped_lock guard(*locMutex);
            vm.combinedPose = loadPose(combinedPoseState);
        } else {
            vm.combinedPose = loadPose(combinedPoseState);
        }
    }

    if (!LocMath::isFinitePose(vm.combinedPose)) {
        vm.combinedPose = vm.pureOdomPose;
    }
    if (LocMath::isFinitePose(vm.combinedPose)) {
        lastFiniteCombined = vm.combinedPose;
    } else {
        vm.combinedPose = lastFiniteCombined;
    }

    vm.pureMclPose = particleFilter ? particleFilter->getPrediction() : vm.pureOdomPose;
    if (!LocMath::isFinitePose(vm.pureMclPose)) {
        vm.pureMclPose = vm.pureOdomPose;
    }

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

    vm.pose = vm.combinedPose;
    return vm;
}

static void renderSimpleRuntimeScreen(const BrainScreen::RuntimeViewModel& vm) {
    BrainScreen::renderRuntime(vm);
}

void update_loop() {
    startupLog("update_loop entry");
    uint32_t lastDebugMs = 0;
    uint32_t lastGuardLogMs = 0;
    uint32_t lastScreenNotifyMs = 0;

    while (true) {
        CommandScheduler::run();
        if (particleFilter) {
            particleFilter->update();
        }

        // ── Fusion ──────────────────────────────────────────────────────
        const Eigen::Vector3f odomPose = drivetrain
            ? drivetrain->getOdomPose()
            : Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        const Eigen::Vector3f fusedPose = fusion
            ? fusion->update()
            : odomPose;
        ControllerPoseGuard::UpdateResult controllerUpdate;
        controllerUpdate.pose = LocMath::finitePoseOr(fusedPose, odomPose);
        if (controllerPoseGuard) {
            controllerUpdate = controllerPoseGuard->update(fusedPose, odomPose);
        }

        // ── Publish shared state under mutex ────────────────────────────
        {
            if (locMutex) {
                std::scoped_lock guard(*locMutex);
                storePose(&combinedPoseState, fusedPose);
                storePose(&controllerPoseState, controllerUpdate.pose);
            } else {
                storePose(&combinedPoseState, fusedPose);
                storePose(&controllerPoseState, controllerUpdate.pose);
            }
        }

        if constexpr (CONFIG::ODOM_DEBUG_ENABLE) {
            uint32_t now = pros::millis();
            if (now - lastDebugMs >= CONFIG::ODOM_DEBUG_LOG_EVERY_ms) {
                lastDebugMs = now;
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
                Eigen::Vector2f odomdelta = drivetrain
                    ? drivetrain->getLastStepDisplacementDebug()
                    : Eigen::Vector2f(0.0f, 0.0f);
                float deltaLen = odomdelta.norm();
                auto odom = drivetrain
                    ? drivetrain->getOdomPose()
                    : Eigen::Vector3f(0.0f, 0.0f, 0.0f);
                auto pf   = particleFilter ? particleFilter->getPrediction()
                                           : Eigen::Vector3f(0, 0, 0);
                const Eigen::Vector2f corr = fusion
                    ? fusion->getCorrection()
                    : Eigen::Vector2f(0.0f, 0.0f);

                std::printf("[LOC] odom_delta=%.4f imu=%.1fdeg gps(%c)=(%.3f,%.3f,%.1fdeg,err=%.3f) "
                            "odom=(%.3f,%.3f,%.1fdeg) pf=(%.3f,%.3f,%.1fdeg) "
                            "comb=(%.3f,%.3f,%.1fdeg) ctrl=(%.3f,%.3f,%.1fdeg) "
                            "corr=(%.3f,%.3f)|%.3f ctrlcorr=(%.3f,%.3f)|%.3f\n",
                    deltaLen,
                    headingToCompassDeg(imuH),
                    gpsOk ? 'Y' : 'N', gpsX, gpsY, headingToCompassDeg(gpsH), gpsErr,
                    odom.x(), odom.y(), headingToCompassDeg(odom.z()),
                    pf.x(), pf.y(), headingToCompassDeg(pf.z()),
                    fusedPose.x(), fusedPose.y(), headingToCompassDeg(fusedPose.z()),
                    controllerUpdate.pose.x(), controllerUpdate.pose.y(),
                    headingToCompassDeg(controllerUpdate.pose.z()),
                    corr.x(), corr.y(), corr.norm(),
                    controllerUpdate.correction.x(),
                    controllerUpdate.correction.y(),
                    controllerUpdate.correction.norm());
            }
        }

        const uint32_t now = pros::millis();
        if ((controllerUpdate.rejectedSnap ||
             controllerUpdate.acceptedStableJump ||
             controllerUpdate.decayingToOdom ||
             controllerUpdate.usedOdomFallback) &&
            now - lastGuardLogMs >= 250) {
            lastGuardLogMs = now;
            std::printf(
                "[LOC_GUARD] fused_ok=%c snap_reject=%c stable_accept=%c stable_cycles=%d "
                "decay=%c odom_fallback=%c target=(%.3f,%.3f)|%.3f applied=(%.3f,%.3f)|%.3f\n",
                LocMath::isFinitePose(fusedPose) ? 'Y' : 'N',
                controllerUpdate.rejectedSnap ? 'Y' : 'N',
                controllerUpdate.acceptedStableJump ? 'Y' : 'N',
                controllerUpdate.stableJumpCycles,
                controllerUpdate.decayingToOdom ? 'Y' : 'N',
                controllerUpdate.usedOdomFallback ? 'Y' : 'N',
                controllerUpdate.targetCorrection.x(),
                controllerUpdate.targetCorrection.y(),
                controllerUpdate.targetCorrection.norm(),
                controllerUpdate.correction.x(),
                controllerUpdate.correction.y(),
                controllerUpdate.correction.norm());
        }

        const bool urgentScreenRefresh =
            controllerUpdate.rejectedSnap ||
            controllerUpdate.acceptedStableJump ||
            controllerUpdate.decayingToOdom ||
            controllerUpdate.usedOdomFallback;
        if (now - lastScreenNotifyMs >= SCREEN_NOTIFY_PERIOD_ms ||
            urgentScreenRefresh) {
            lastScreenNotifyMs = now;
            requestScreenRefresh();
        }

        pros::delay(10);
    }
}

void screen_update_loop() {
    startupLog("screen_update_loop entry");
    while (true) {
        pros::Task::notify_take(true, SCREEN_NOTIFY_FALLBACK_ms);
        renderSimpleRuntimeScreen(captureRuntimeViewModel());
    }
}

static void renderReadyScreen() {
    renderSimpleRuntimeScreen(captureRuntimeViewModel());
}

static void subsystemInit() {
    if (drivetrain) drivetrain->registerThis();
    if (intakes) intakes->registerThis();
}

static void createRobotObjects() {
    startupLog("construct drivetrain");
    drivetrain = new Drivetrain();
    startupLog("constructed drivetrain");

    startupLog("construct intakes");
    intakes = new Intakes();
    startupLog("constructed intakes");

    startupLog("construct lift");
    lift = new Lift();
    startupLog("constructed lift");

    startupLog("construct pneumatics");
    selector = new pros::adi::DigitalOut(CONFIG::SELECT_PORT);
    top = new pros::adi::DigitalOut(CONFIG::TOP_PORT);
    tongue = new pros::adi::DigitalOut(CONFIG::TONGUE_PORT);
    wing = new pros::adi::DigitalOut(CONFIG::WING_PORT);
    startupLog("constructed pneumatics");
}

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
    startupLog("acquireInitialPose begin");
    const float defaultImuHeadingRad = CONFIG::DEFAULT_IMU_INIT_ANGLE.convert(radian);
    Eigen::Vector3f configPose(
        CONFIG::START_POSE_X.convert(meter),
        CONFIG::START_POSE_Y.convert(meter),
        CONFIG::START_POSE_THETA.convert(radian));

    if (CONFIG::STARTUP_POSE_MODE ==
        CONFIG::StartupPoseMode::ConfiguredStartPoseOnly) {
        startupLog("acquireInitialPose using configured pose only");
        return configPose;
    }

    if (CONFIG::STARTUP_POSE_MODE == CONFIG::StartupPoseMode::GPSXYPlusIMUHeading) {
        if (drivetrain) drivetrain->resetHeading(defaultImuHeadingRad);
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
        startupLog("GPS timeout, using config pose");
        return configPose;
    }

    auto centerFromGps = [&lastReading](float headingRad) -> std::optional<Eigen::Vector2f> {
        return GpsLocalization::robotCenterFromSensorPose(
            lastReading->sensorFieldPos,
            headingRad,
            CONFIG::gpsOffset());
    };

    auto safeReturn = [&configPose](const Eigen::Vector3f& pose) -> Eigen::Vector3f {
        if (LocMath::isFinitePose(pose)) {
            startupLog("startup pose (%.3f, %.3f, %.1fdeg compass)",
                       pose.x(), pose.y(), headingToCompassDeg(pose.z()));
            return pose;
        }
        startupLog("startup pose non-finite, using config");
        return configPose;
    };

    switch (CONFIG::STARTUP_POSE_MODE) {
        case CONFIG::StartupPoseMode::GPSXYPlusIMUHeading: {
            const float imuHeadingRad = drivetrain ? drivetrain->getHeading() : defaultImuHeadingRad;
            if (!LocMath::isFinite(imuHeadingRad)) {
                startupLog("IMU heading non-finite, using config");
                return configPose;
            }
            const std::optional<Eigen::Vector2f> c = centerFromGps(imuHeadingRad);
            if (!c) {
                startupLog("GPS centre solve failed (imu heading), using config");
                return configPose;
            }
            return safeReturn(Eigen::Vector3f(c->x(), c->y(), imuHeadingRad));
        }
        case CONFIG::StartupPoseMode::FullGPSInit: {
            if (!lastReading->hasHeading || !LocMath::isFinite(lastReading->robotHeadingRad)) {
                startupLog("GPS heading non-finite, using config");
                return configPose;
            }
            const std::optional<Eigen::Vector2f> c = centerFromGps(lastReading->robotHeadingRad);
            if (!c) {
                startupLog("GPS centre solve failed (gps heading), using config");
                return configPose;
            }
            return safeReturn(Eigen::Vector3f(c->x(), c->y(), lastReading->robotHeadingRad));
        }
        default:
            startupLog("acquireInitialPose fell through to config pose");
            return configPose;
    }
}

static void localizationInit() {
    startupLog("localizationInit begin");
    std::vector<SensorModel*> sensors;
    static GpsSensorModel gpsSensor(
        CONFIG::MCL_GPS_PORT,
        CONFIG::MCL_GPS_HEADING_OFFSET_deg,
        CONFIG::gpsOffset().x(),
        CONFIG::gpsOffset().y(),
        CONFIG::GPS_STDDEV_BASE.convert(meter));

    static DistanceSensorModel distLeft(
        CONFIG::MCL_LEFT_DISTANCE_PORT,  CONFIG::distLeftOffset(),
        static_cast<float>(CONFIG::MCL_LEFT_DISTANCE_WEIGHT),
        CONFIG::MCL_DISTANCE_STDDEV.convert(meter), "distLeft");
    static DistanceSensorModel distRight(
        CONFIG::MCL_RIGHT_DISTANCE_PORT, CONFIG::distRightOffset(),
        static_cast<float>(CONFIG::MCL_RIGHT_DISTANCE_WEIGHT),
        CONFIG::MCL_DISTANCE_STDDEV.convert(meter), "distRight");
    static DistanceSensorModel distFront(
        CONFIG::MCL_FRONT_DISTANCE_PORT, CONFIG::distFrontOffset(),
        static_cast<float>(CONFIG::MCL_FRONT_DISTANCE_WEIGHT),
        CONFIG::MCL_DISTANCE_STDDEV.convert(meter), "distFront");
    static DistanceSensorModel distBack(
        CONFIG::MCL_BACK_DISTANCE_PORT,  CONFIG::distBackOffset(),
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

    // Lambdas reference only file-static objects → empty capture list is correct.
    auto predictionFn = []() -> Eigen::Vector2f {
        if (!drivetrain) return Eigen::Vector2f(0.0f, 0.0f);
        Eigen::Vector2f proposal = drivetrain->consumePendingFwdOnlyDisplacement();
        drivetrain->consumePendingDisplacement();
        return LocMath::isFiniteVec2(proposal) ? proposal : Eigen::Vector2f(0.0f, 0.0f);
    };
    auto angleFn = []() -> QAngle {
        const float heading = drivetrain ? drivetrain->getHeading() : 0.0f;
        return QAngle(LocMath::isFinite(heading) ? heading : 0.0f);
    };

    Eigen::Vector3f startPose = acquireInitialPose();
    if (drivetrain) drivetrain->syncLocalizationReference(startPose);
    storePose(&combinedPoseState, startPose);
    storePose(&controllerPoseState, startPose);

    renderInitStage(0.90f, "Init localization", "Starting filter...");

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
    std::printf("[LOC_CONFIG] Controller Guard: max=%.3f in step=%.3f in jump=%.3f in reacquire=%.3f in/%d cycles\n",
        CONFIG::LOC_CONTROLLER_CORRECTION_MAX_in,
        CONFIG::LOC_CONTROLLER_CORRECTION_STEP_in,
        CONFIG::LOC_CONTROLLER_CORRECTION_JUMP_REJECT_in,
        CONFIG::LOC_CONTROLLER_REACQUIRE_DEADBAND_in,
        CONFIG::LOC_CONTROLLER_REACQUIRE_STABLE_CYCLES);
    std::printf("[LOC_CONFIG] ═════════════════════════════════════════════════\n\n");

    particleFilter = new ParticleFilter(
        predictionFn, angleFn, sensors,
        startPose,
        CONFIG::NUM_PARTICLES);

    fusion = new LocalizationFusion(*drivetrain, particleFilter, sampleGpsPose);
    fusion->reset(startPose);
    controllerPoseGuard = new ControllerPoseGuard();
    controllerPoseGuard->reset(startPose);
    startupLog("localizationInit complete");
}

static bool runSelectedAutonNow() {
    if (!subsystemsReady()) {
        startupLog("runSelectedAutonNow aborted: subsystems not ready");
        return false;
    }

    const AutonEntry* selectedEntry = AutonSelector::getAutonEntry();
    if (selectedEntry == nullptr || selectedEntry->run == nullptr) {
        startupLog("runSelectedAutonNow aborted: no runnable auton selected");
        return false;
    }

    if (drivetrain) {
        drivetrain->resetDriverAssistState();
        drivetrain->stop();
    }
    if (intakes) {
        intakes->stop();
    }
    if (lift) {
        lift->stop();
    }
    CommandScheduler::reset();

    bindAutonRuntime(
        *drivetrain,
        *intakes,
        *lift,
        currentControllerPose);

    startupLog("run auton begin name=%s", selectedEntry->name);
    const bool ok = runAuton(*selectedEntry);
    startupLog("run auton end name=%s ok=%d", selectedEntry->name, ok ? 1 : 0);
    resetAutonRuntime();
    return ok;
}

/**
 * Runs once when the program starts.
 * Initialization renders inline, then starts background tasks once ready.
 */
void initialize() {
    openStartupLog();
    startupLog("initialize entry");
    firstInitScreenMs = 0;
    startupLog("create loc mutex");
    ensureLocMutex();
    startupLog("brain screen initialize");
    BrainScreen::initialize();
    renderInitStage(0.0f, "Boot", "Booting...");
    startupLog("selector defaults");
    AutonSelector::selectAuton(DEFAULT_AUTON_SELECTION);

    renderInitStage(0.10f, "Init subsystems", "Creating subsystem objects");
    createRobotObjects();
    startupLog("register subsystems");
    subsystemInit();
    renderInitStage(0.18f, "Init subsystems", "Calibrating IMU");
    startupLog("calibrate imu begin");
    drivetrain->calibrateImu();
    startupLog("calibrate imu done");
    drivetrain->resetHeading(CONFIG::DEFAULT_IMU_INIT_ANGLE.convert(radian));
    startupLog("construct ui sensors");
    uiGps = new pros::Gps(CONFIG::MCL_GPS_PORT);
    uiDistLeft = new pros::Distance(CONFIG::MCL_LEFT_DISTANCE_PORT);
    uiDistRight = new pros::Distance(CONFIG::MCL_RIGHT_DISTANCE_PORT);
    uiDistFront = new pros::Distance(CONFIG::MCL_FRONT_DISTANCE_PORT);
    uiDistBack = new pros::Distance(CONFIG::MCL_BACK_DISTANCE_PORT);
    startupLog("constructed ui sensors");

    renderInitStage(0.25f, "Init localization", "Preparing sensor models");
    startupLog("localization init begin");
    localizationInit();
    startupLog("localization init done");

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
    setScreenStatus("Ready");
    renderReadyScreen();

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    startupLog("controller rumble");
    master.rumble(".");

    startupLog("create screen task");
    screenTaskHandle = new pros::Task(
        screen_update_loop,
        TASK_PRIORITY_DEFAULT,
        TASK_STACK_DEPTH_DEFAULT * 2,
        "screen");
    startupLog("create scheduler task");
    schedulerTaskHandle = new pros::Task(
        update_loop,
        TASK_PRIORITY_DEFAULT,
        TASK_STACK_DEPTH_DEFAULT * 2,
        "scheduler");

    const pros::task_t screenTask =
        screenTaskHandle ? static_cast<pros::task_t>(*screenTaskHandle) : nullptr;
    const pros::task_t schedulerTask =
        schedulerTaskHandle ? static_cast<pros::task_t>(*schedulerTaskHandle) : nullptr;

    if (screenTask == nullptr || schedulerTask == nullptr) {
        setScreenStatus("Background task start failed");
        startupLog("task create failure screen=%p scheduler=%p", screenTask, schedulerTask);
        renderReadyScreen();
        requestScreenRefresh();
        return;
    }

    startupLog("initialize complete");
    requestScreenRefresh();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    startupLog("autonomous entry");
    runSelectedAutonNow();
    startupLog("autonomous exit");
}

void opcontrol() {
    CommandScheduler::reset();

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    bool downBAutonLatch = false;
    enum class IntakeMode {
        Off,
        Loading,
        ScoreHigh,
        ScoreMid,
        ScoreLow,
    };
    IntakeMode intakeMode = IntakeMode::Off;
    IntakeMode previousIntakeMode = IntakeMode::Off;
    bool wingState = false;
    bool tongueState = false;
    int intakeSpeed = 0;
    bool selectMode = false;
    bool upMode = false;
    uint32_t scoreMidEnteredMs = 0;
    constexpr uint32_t SCORE_MID_INTAKE_DELAY_ms = 500;

    while (true) {
        const bool downHeld = master.get_digital(DIGITAL_DOWN);
        const bool bHeld = master.get_digital(DIGITAL_B);
        const bool downBHeld = downHeld && bHeld;
        const float forward = static_cast<float>(master.get_analog(ANALOG_LEFT_Y));
        const float turn = static_cast<float>(master.get_analog(ANALOG_RIGHT_X));

        if (master.get_digital_new_press(DIGITAL_R2)) {
            intakeMode = (intakeMode == IntakeMode::Loading)
                ? IntakeMode::Off
                : IntakeMode::Loading;
        }
        if (master.get_digital_new_press(DIGITAL_L2)) {
            intakeMode = (intakeMode == IntakeMode::ScoreHigh)
                ? IntakeMode::Off
                : IntakeMode::ScoreHigh;
        }
        if (master.get_digital_new_press(DIGITAL_R1)) {
            intakeMode = (intakeMode == IntakeMode::ScoreMid)
                ? IntakeMode::Off
                : IntakeMode::ScoreMid;
        }
        if (master.get_digital_new_press(DIGITAL_L1)) {
            intakeMode = (intakeMode == IntakeMode::ScoreLow)
                ? IntakeMode::Off
                : IntakeMode::ScoreLow;
        }

        if (intakeMode != previousIntakeMode) {
            if (intakeMode == IntakeMode::ScoreMid) {
                scoreMidEnteredMs = pros::millis();
            }
            previousIntakeMode = intakeMode;
        }

        switch (intakeMode) {
            case IntakeMode::Loading:
                intakeSpeed = 127;
                selectMode = true;
                upMode = false;
                break;
            case IntakeMode::ScoreHigh:
                intakeSpeed = 127;
                selectMode = true;
                upMode = true;
                break;
            case IntakeMode::ScoreMid:
                intakeSpeed =
                    (pros::millis() - scoreMidEnteredMs >= SCORE_MID_INTAKE_DELAY_ms)
                    ? 127
                    : 0;
                selectMode = false;
                break;
            case IntakeMode::ScoreLow:
                intakeSpeed = -80;
                break;
            case IntakeMode::Off:
            default:
                intakeSpeed = 0;
                break;
        }

        if (intakeSpeed == 0) {
            intakes->stop();
        } else {
            intakes->spin(intakeSpeed);
        }
        selector->set_value(selectMode);
        top->set_value(upMode);

        if (downBHeld && !downBAutonLatch) {
            autonomous();
            if (AutonSelector::getAuton() != Auton::NONE) {
                master.rumble(".");
            }
            intakeMode = IntakeMode::Off;
            previousIntakeMode = IntakeMode::Off;
            intakeSpeed = 0;
            selectMode = false;
            upMode = false;
            if (intakes) {
                intakes->stop();
            }
            if (drivetrain) {
                drivetrain->resetDriverAssistState();
            }
        }
        downBAutonLatch = downBHeld;

        if (drivetrain->getCurrentCommand() == nullptr) {
            drivetrain->driverArcade(forward, turn);
        } else {
            drivetrain->resetDriverAssistState();
        }
        // Only toggle wing/tongue when NOT in the DOWN+B auton-launch combo
        if (!downHeld && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            wingState = !wingState;
        }
        if (!bHeld && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            tongueState = !tongueState;
        }
        
        wing->set_value(wingState);
        tongue->set_value(tongueState);

        pros::delay(20);
    }
}
