#include "autonomous/autons.h"

#include "config.h"
#include "tuning/relayPidAutotuner.h"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/tracking_wheel.hpp"
#include "Eigen/Core"
#include "pros/rtos.hpp"
#include "subsystems/drivetrain.h"
#include "subsystems/intakes.h"
#include "subsystems/lift.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <utility>
#include <vector>

namespace {

constexpr float kInToM = 0.0254f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr double kDriveWheelDiameter_in = CONFIG::DRIVE_RADIUS_in * 2.0;
constexpr double kDriveCartridgeRpm = 600.0;
constexpr double kTrackingWheelDiameter_in = 2.0;
constexpr int kDriveSpeed = 110;
constexpr int kTurnSpeed = 90;
constexpr int kSwingSpeed = 110;
constexpr int kCalibrationTurnSpeed = 70;

struct RuntimeState {
    Drivetrain* drivetrain = nullptr;
    Intakes* intakes = nullptr;
    Lift* lift = nullptr;
    std::function<Eigen::Vector3f()> poseSource;
    std::function<bool()> isCancelled;
    const AutonEntry* currentEntry = nullptr;
};

RuntimeState g_runtime;
std::unique_ptr<ez::Drive> g_ezDrive;
std::unique_ptr<ez::tracking_wheel> g_horizontalTracker;
bool g_ezMotionReady = false;

void autonLog(const char* fmt, ...) {
    char message[192];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(message, sizeof(message), fmt, args);
    va_end(args);

    std::printf("[AUTON @ %lums] %s\n",
                static_cast<unsigned long>(pros::millis()),
                message);
}

bool runtimeReady() {
    return g_runtime.drivetrain != nullptr &&
           g_runtime.intakes != nullptr &&
           g_runtime.lift != nullptr;
}

Drivetrain& drivetrain() {
    return *g_runtime.drivetrain;
}

Intakes& intakes() {
    return *g_runtime.intakes;
}

Lift& lift() {
    return *g_runtime.lift;
}

Eigen::Vector3f currentPose() {
    if (g_runtime.poseSource) {
        return g_runtime.poseSource();
    }
    if (g_runtime.drivetrain) {
        return g_runtime.drivetrain->getOdomPose();
    }
    return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
}

bool routineCancelled() {
    return g_runtime.isCancelled && g_runtime.isCancelled();
}

float metersToInches(float meters) {
    return meters / kInToM;
}

float radiansToDegrees(float radians) {
    return radians * 180.0f / kPi;
}

std::vector<int> asPortVector(const std::array<std::int8_t, 3>& ports) {
    return {ports[0], ports[1], ports[2]};
}

void configureEzDrive(ez::Drive& drive) {
    drive.pid_print_toggle(false);
    drive.opcontrol_curve_buttons_toggle(false);
    drive.drive_width_set(CONFIG::TRACK_WIDTH_in * okapi::inch);

    drive.pid_drive_constants_set(20.0, 0.0, 100.0);
    drive.pid_heading_constants_set(11.0, 0.0, 20.0);
    drive.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);
    drive.pid_swing_constants_set(6.0, 0.0, 65.0);
    drive.pid_odom_angular_constants_set(6.5, 0.0, 52.5);
    drive.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);

    drive.pid_turn_exit_condition_set(
        90 * okapi::millisecond, 3 * okapi::degree,
        250 * okapi::millisecond, 7 * okapi::degree,
        500 * okapi::millisecond, 500 * okapi::millisecond);
    drive.pid_swing_exit_condition_set(
        90 * okapi::millisecond, 3 * okapi::degree,
        250 * okapi::millisecond, 7 * okapi::degree,
        500 * okapi::millisecond, 500 * okapi::millisecond);
    drive.pid_drive_exit_condition_set(
        90 * okapi::millisecond, 1 * okapi::inch,
        250 * okapi::millisecond, 3 * okapi::inch,
        500 * okapi::millisecond, 500 * okapi::millisecond);
    drive.pid_odom_turn_exit_condition_set(
        90 * okapi::millisecond, 3 * okapi::degree,
        250 * okapi::millisecond, 7 * okapi::degree,
        500 * okapi::millisecond, 750 * okapi::millisecond);
    drive.pid_odom_drive_exit_condition_set(
        90 * okapi::millisecond, 1 * okapi::inch,
        250 * okapi::millisecond, 3 * okapi::inch,
        500 * okapi::millisecond, 750 * okapi::millisecond);

    drive.pid_turn_chain_constant_set(3 * okapi::degree);
    drive.pid_swing_chain_constant_set(5 * okapi::degree);
    drive.pid_drive_chain_constant_set(3 * okapi::inch);

    drive.slew_turn_constants_set(3 * okapi::degree, 70);
    drive.slew_drive_constants_set(3 * okapi::inch, 70);
    drive.slew_swing_constants_set(3 * okapi::inch, 80);

    drive.odom_turn_bias_set(0.9);
    drive.odom_look_ahead_set(7 * okapi::inch);
    drive.odom_boomerang_distance_set(16 * okapi::inch);
    drive.odom_boomerang_dlead_set(0.625);
    drive.pid_angle_behavior_set(ez::shortest);
    drive.pid_ramsete_constants_set(CONFIG::RAMSETE_ZETA, CONFIG::RAMSETE_BETA);
    drive.pid_ltv_costs_set(
        CONFIG::defaultDtCostQ().x(),
        CONFIG::defaultDtCostQ().y(),
        CONFIG::defaultDtCostQ().z(),
        CONFIG::LTV_CONTROL_COST_V,
        CONFIG::LTV_CONTROL_COST_OMEGA,
        CONFIG::LTV_TERMINAL_SCALE);
    drive.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
}

bool attachEzTrackers(ez::Drive& drive) {
    if (CONFIG::HORIZONTAL_TRACKING_PORT == 0) {
        return false;
    }

    const int trackerPort = CONFIG::HORIZONTAL_TRACKING_REVERSED
        ? -CONFIG::HORIZONTAL_TRACKING_PORT
        : CONFIG::HORIZONTAL_TRACKING_PORT;
    g_horizontalTracker = std::make_unique<ez::tracking_wheel>(
        trackerPort,
        kTrackingWheelDiameter_in,
        std::fabs(CONFIG::LATERAL_WHEEL_OFFSET_in));

    if (CONFIG::LATERAL_WHEEL_OFFSET_in <= 0.0f) {
        drive.odom_tracker_back_set(g_horizontalTracker.get());
    } else {
        drive.odom_tracker_front_set(g_horizontalTracker.get());
    }
    return true;
}

bool ezDriveReady() {
    return g_ezDrive != nullptr && g_ezMotionReady;
}

ez::Drive& ezDrive() {
    return *g_ezDrive;
}

void stopEzMotion() {
    if (!ezDriveReady()) {
        return;
    }
    ezDrive().drive_mode_set(ez::DISABLE, true);
    ezDrive().pid_targets_reset();
    ezDrive().drive_set(0, 0);
}

void prepareEzStart() {
    stopEzMotion();
    ezDrive().drive_sensor_reset();
    ezDrive().odom_xyt_set(0 * okapi::inch, 0 * okapi::inch, 0 * okapi::degree);
    ezDrive().drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
}

ez::united_pose ezLocalPose(double forwardIn,
                            double leftIn,
                            okapi::QAngle theta = ez::p_ANGLE_NOT_SET) {
    return {
        -leftIn * okapi::inch,
        forwardIn * okapi::inch,
        theta,
    };
}

ez::united_odom ezLocalMove(double forwardIn,
                            double leftIn,
                            ez::drive_directions dir,
                            int speed,
                            okapi::QAngle theta = ez::p_ANGLE_NOT_SET) {
    return {ezLocalPose(forwardIn, leftIn, theta), dir, speed};
}

bool cancellableDelay(int milliseconds) {
    constexpr int kPollMs = 10;

    for (int elapsedMs = 0; elapsedMs < milliseconds; elapsedMs += kPollMs) {
        if (routineCancelled()) {
            return false;
        }

        const int remainingMs = milliseconds - elapsedMs;
        pros::delay(std::min(kPollMs, remainingMs));
    }

    return !routineCancelled();
}

bool intakeTimed(int voltage, int milliseconds) {
    if (routineCancelled()) {
        intakes().stop();
        return false;
    }

    intakes().spin(voltage);
    const bool completed = cancellableDelay(milliseconds);
    intakes().stop();
    return completed;
}

void printTuneResult(const char* configLabel,
                     tuning::RelayPidAutotuner::Mode mode,
                     const tuning::RelayPidAutotuner::Result& result) {
    if (!result.success) {
        std::printf("[PID_TUNE] %s failed: %s\n", configLabel, result.message);
        return;
    }

    const float amplitudeHuman = mode == tuning::RelayPidAutotuner::Mode::DriveDistance
        ? metersToInches(result.oscillationAmplitude)
        : radiansToDegrees(result.oscillationAmplitude);
    const char* unit = mode == tuning::RelayPidAutotuner::Mode::DriveDistance ? "in" : "deg";

    std::printf(
        "[PID_TUNE] %s summary Ku=%.5f Pu=%.4fs amp=%.3f %s switches=%zu\n"
        "[PID_TUNE] Suggested CONFIG::%s (safe) -> {%.5ff, %.5ff, %.5ff, 0.0f}\n"
        "[PID_TUNE] Suggested CONFIG::%s (ZN)   -> {%.5ff, %.5ff, %.5ff, 0.0f}\n",
        configLabel,
        result.ku,
        result.puSec,
        amplitudeHuman,
        unit,
        result.switchCount,
        configLabel,
        result.gains.noOvershoot.kP,
        result.gains.noOvershoot.kI,
        result.gains.noOvershoot.kD,
        configLabel,
        result.gains.zieglerNichols.kP,
        result.gains.zieglerNichols.kI,
        result.gains.zieglerNichols.kD);
}

void runRelayPidTune(const char* configLabel,
                     const tuning::RelayPidAutotuner::Config& config) {
    tuning::RelayPidAutotuner::clearStatus();

    stopEzMotion();
    drivetrain().stop();
    pros::delay(250);

    tuning::RelayPidAutotuner tuner(
        drivetrain(),
        []() {
            return currentPose();
        },
        []() {
            return routineCancelled();
        });

    const auto result = tuner.run(config);
    drivetrain().stop();

    if (routineCancelled()) {
        std::printf("[PID_TUNE] %s cancelled\n", configLabel);
        return;
    }

    printTuneResult(configLabel, config.mode, result);
}

void runNegative1() {
    autonLog("Negative 1 start");
    prepareEzStart();

    intakes().spin(127);
    autonLog("Negative 1 pure pursuit -> goal");
    ezDrive().pid_odom_set(std::vector<ez::united_odom>{
        ezLocalMove(23.62, 0.0, ez::fwd, 102),
        ezLocalMove(47.24, 11.81, ez::fwd, 102),
    }, true);
    ezDrive().pid_wait();
    if (routineCancelled()) {
        intakes().stop();
        autonLog("Negative 1 cancelled after path");
        return;
    }

    if (!cancellableDelay(250)) {
        intakes().stop();
        autonLog("Negative 1 cancelled during settle");
        return;
    }
    if (!intakeTimed(-127, 450)) {
        autonLog("Negative 1 cancelled during outtake");
        return;
    }

    intakes().spin(127);
    autonLog("Negative 1 boomerang -> home");
    ezDrive().pid_odom_set(
        ezLocalMove(0.0, 0.0, ez::rev, 96, 0 * okapi::degree),
        true);
    ezDrive().pid_wait();
}

void runPositive1() {
    autonLog("Positive 1 start");
    prepareEzStart();

    intakes().spin(127);
    autonLog("Positive 1 pure pursuit -> goal");
    ezDrive().pid_odom_set(std::vector<ez::united_odom>{
        ezLocalMove(23.62, 0.0, ez::fwd, 102),
        ezLocalMove(47.24, -11.81, ez::fwd, 102),
    }, true);
    ezDrive().pid_wait();
    if (routineCancelled()) {
        intakes().stop();
        autonLog("Positive 1 cancelled after path");
        return;
    }

    if (!cancellableDelay(250)) {
        intakes().stop();
        autonLog("Positive 1 cancelled during settle");
        return;
    }
    if (!intakeTimed(-127, 450)) {
        autonLog("Positive 1 cancelled during outtake");
        return;
    }

    intakes().spin(127);
    autonLog("Positive 1 boomerang -> home");
    ezDrive().pid_odom_set(
        ezLocalMove(0.0, 0.0, ez::rev, 96, 0 * okapi::degree),
        true);
    ezDrive().pid_wait();
}

void runSkills() {
    autonLog("Skills start");
    prepareEzStart();

    intakes().spin(127);
    ezDrive().pid_odom_set(ezLocalMove(55.12, 0.0, ez::fwd, 110), true);
    ezDrive().pid_wait();
    if (!intakeTimed(-127, 400)) {
        autonLog("Skills cancelled on first score");
        return;
    }

    intakes().spin(127);
    ezDrive().pid_swing_set(ez::LEFT_SWING, 35 * okapi::degree, 90, 35);
    ezDrive().pid_wait();
    if (routineCancelled()) {
        autonLog("Skills cancelled after swing");
        return;
    }

    ezDrive().pid_odom_set(std::vector<ez::united_odom>{
        ezLocalMove(36.0, -18.0, ez::fwd, 100),
        ezLocalMove(56.0, -18.0, ez::fwd, 100),
    }, true);
    ezDrive().pid_wait();
    if (!intakeTimed(-127, 400)) {
        autonLog("Skills cancelled on second score");
        return;
    }

    lift().moveTo(180.0f);
    if (!cancellableDelay(150)) {
        lift().stop();
        autonLog("Skills cancelled during lift");
        return;
    }
    lift().moveTo(0.0f);

    ezDrive().pid_odom_set(
        ezLocalMove(0.0, 0.0, ez::rev, 100, 0 * okapi::degree),
        true);
    ezDrive().pid_wait();
}

void runTuneDrivePid() {
    autonLog("Tune Drive PID start");
    tuning::RelayPidAutotuner::Config config;
    config.mode = tuning::RelayPidAutotuner::Mode::DriveDistance;
    config.target = CONFIG::PID_AUTOTUNE_DRIVE_TARGET_in * kInToM;
    config.relayAmplitude = CONFIG::PID_AUTOTUNE_DRIVE_RELAY;
    config.noiseBand = CONFIG::PID_AUTOTUNE_DRIVE_NOISE_in * kInToM;
    config.controllerPeriodMs = 10;
    config.timeoutMs = CONFIG::PID_AUTOTUNE_TIMEOUT_ms;
    config.requiredPeakPairs = CONFIG::PID_AUTOTUNE_REQUIRED_PEAK_PAIRS;
    config.analysisPeakPairs = CONFIG::PID_AUTOTUNE_ANALYSIS_PEAK_PAIRS;
    config.minOscillationAmplitude = 0.25f * config.noiseBand;

    runRelayPidTune("DISTANCE_PID", config);
}

void runTuneTurnPid() {
    autonLog("Tune Turn PID start");
    tuning::RelayPidAutotuner::Config config;
    config.mode = tuning::RelayPidAutotuner::Mode::TurnAngle;
    config.target = CONFIG::PID_AUTOTUNE_TURN_TARGET_deg * kDegToRad;
    config.relayAmplitude = CONFIG::PID_AUTOTUNE_TURN_RELAY;
    config.noiseBand = CONFIG::PID_AUTOTUNE_TURN_NOISE_deg * kDegToRad;
    config.controllerPeriodMs = 10;
    config.timeoutMs = CONFIG::PID_AUTOTUNE_TIMEOUT_ms;
    config.requiredPeakPairs = CONFIG::PID_AUTOTUNE_REQUIRED_PEAK_PAIRS;
    config.analysisPeakPairs = CONFIG::PID_AUTOTUNE_ANALYSIS_PEAK_PAIRS;
    config.minOscillationAmplitude = 0.25f * config.noiseBand;

    runRelayPidTune("TURN_PID", config);
}

void runExampleMove() {
    autonLog("Example Drive start");
    prepareEzStart();

    ezDrive().pid_drive_set(24 * okapi::inch, kDriveSpeed, true);
    ezDrive().pid_wait();
    if (routineCancelled()) {
        autonLog("Example Drive cancelled after first leg");
        return;
    }

    ezDrive().pid_drive_set(-12 * okapi::inch, kDriveSpeed);
    ezDrive().pid_wait();
}

void runExampleSwing() {
    autonLog("Example Swing start");
    prepareEzStart();

    ezDrive().pid_swing_set(ez::LEFT_SWING, 45 * okapi::degree, kSwingSpeed, 45);
    ezDrive().pid_wait();
    if (routineCancelled()) {
        autonLog("Example Swing cancelled after first arc");
        return;
    }

    ezDrive().pid_swing_set(ez::RIGHT_SWING, 0 * okapi::degree, kSwingSpeed, 45);
    ezDrive().pid_wait();
}

void runExampleTurn() {
    autonLog("PID Calibration start");
    prepareEzStart();

    ezDrive().pid_turn_set(90 * okapi::degree, kCalibrationTurnSpeed);
    ezDrive().pid_wait();
    if (routineCancelled()) {
        autonLog("PID Calibration cancelled at 90");
        return;
    }

    if (!cancellableDelay(250)) {
        autonLog("PID Calibration cancelled during settle");
        return;
    }

    ezDrive().pid_turn_set(0 * okapi::degree, kCalibrationTurnSpeed);
    ezDrive().pid_wait();
}

void runExampleRamsete() {
    autonLog("Example Ramsete start");
    prepareEzStart();

    ezDrive().pid_odom_ramsete_set(std::vector<ez::united_odom>{
        ezLocalMove(18.0, 0.0, ez::fwd, kDriveSpeed),
        ezLocalMove(30.0, -10.0, ez::fwd, kDriveSpeed),
        ezLocalMove(42.0, 0.0, ez::fwd, kDriveSpeed, 0 * okapi::degree),
    }, true);
    ezDrive().pid_wait();
}

void runExampleLtv() {
    autonLog("Example LTV start");
    prepareEzStart();

    ezDrive().pid_odom_ltv_set(std::vector<ez::united_odom>{
        ezLocalMove(18.0, 0.0, ez::fwd, kDriveSpeed),
        ezLocalMove(30.0, 10.0, ez::fwd, kDriveSpeed),
        ezLocalMove(42.0, 0.0, ez::fwd, kDriveSpeed, 0 * okapi::degree),
    }, true);
    ezDrive().pid_wait();
}

const AutonList kAvailableAutons = {{
    {Auton::NEGATIVE_1, "Negative 1", &runNegative1},
    {Auton::POSITIVE_1, "Positive 1", &runPositive1},
    {Auton::TUNE_DRIVE_PID, "Tune Drive PID", &runTuneDrivePid},
    {Auton::TUNE_TURN_PID, "Tune Turn PID", &runTuneTurnPid},
    {Auton::EXAMPLE_MOVE, "Example Drive", &runExampleMove},
    {Auton::EXAMPLE_SWING, "Example Swing", &runExampleSwing},
    {Auton::EXAMPLE_TURN, "PID Calibration", &runExampleTurn},
    {Auton::EXAMPLE_RAMSETE, "Example Ramsete", &runExampleRamsete},
    {Auton::EXAMPLE_LTV, "Example LTV", &runExampleLtv},
    {Auton::SKILLS, "Skills", &runSkills},
    {Auton::NONE, "None", nullptr},
}};

} // namespace

bool initializeAutonMotion() {
    if (g_ezDrive) {
        return g_ezMotionReady;
    }

    autonLog("EZ motion init begin");
    g_ezDrive = std::make_unique<ez::Drive>(
        asPortVector(CONFIG::LEFT_DRIVE_PORTS),
        asPortVector(CONFIG::RIGHT_DRIVE_PORTS),
        CONFIG::IMU_PORT,
        kDriveWheelDiameter_in,
        kDriveCartridgeRpm);

    configureEzDrive(*g_ezDrive);
    const bool usingHorizontalTracker = attachEzTrackers(*g_ezDrive);
    g_ezMotionReady = g_ezDrive->drive_imu_calibrate(false);

    if (!g_ezMotionReady) {
        autonLog("EZ motion init failed");
        return false;
    }

    g_ezDrive->drive_sensor_reset();
    g_ezDrive->odom_xyt_set(0 * okapi::inch, 0 * okapi::inch, 0 * okapi::degree);
    g_ezDrive->drive_mode_set(ez::DISABLE, true);

    autonLog("EZ motion ready tracker=%s track=%.2f wheel=%.2f",
             usingHorizontalTracker ? "horizontal" : "integrated",
             CONFIG::TRACK_WIDTH_in,
             kDriveWheelDiameter_in);
    return true;
}

const AutonList& availableAutons() {
    return kAvailableAutons;
}

const AutonEntry* findAuton(Auton auton) {
    for (const auto& entry : kAvailableAutons) {
        if (entry.id == auton) {
            return &entry;
        }
    }
    return nullptr;
}

const char* autonName(Auton auton) {
    const AutonEntry* entry = findAuton(auton);
    return entry ? entry->name : "Unknown";
}

const char* allianceName(Alliance alliance) {
    switch (alliance) {
        case Alliance::RED:  return "RED";
        case Alliance::BLUE: return "BLUE";
    }
    return "UNKNOWN";
}

void bindAutonRuntime(Drivetrain& drivetrainRef,
                      Intakes& intakesRef,
                      Lift& liftRef,
                      std::function<Eigen::Vector3f()> poseSource,
                      std::function<bool()> isCancelled) {
    resetAutonRuntime();

    g_runtime.drivetrain = &drivetrainRef;
    g_runtime.intakes = &intakesRef;
    g_runtime.lift = &liftRef;
    g_runtime.poseSource = std::move(poseSource);
    g_runtime.isCancelled = std::move(isCancelled);

    const Eigen::Vector3f pose = currentPose();
    autonLog("runtime bound pose=(%.2f, %.2f, %.1fdeg) ez=%s",
             metersToInches(pose.x()),
             metersToInches(pose.y()),
             radiansToDegrees(pose.z()),
             ezDriveReady() ? "ready" : "not-ready");
}

void resetAutonRuntime() {
    stopEzMotion();
    if (g_runtime.intakes) {
        g_runtime.intakes->stop();
    }
    if (g_runtime.lift) {
        g_runtime.lift->stop();
    }
    if (g_runtime.drivetrain) {
        g_runtime.drivetrain->stop();
    }

    g_runtime = RuntimeState{};
}

bool runAuton(const AutonEntry& entry) {
    if (!runtimeReady()) {
        autonLog("run rejected: runtime not bound");
        return false;
    }
    if (entry.run == nullptr) {
        autonLog("run rejected: %s has no function", entry.name);
        return false;
    }

    const bool needsEzMotion =
        entry.id != Auton::TUNE_DRIVE_PID &&
        entry.id != Auton::TUNE_TURN_PID &&
        entry.id != Auton::NONE;
    if (needsEzMotion && !ezDriveReady()) {
        autonLog("run rejected: EZ motion not ready");
        return false;
    }

    g_runtime.currentEntry = &entry;
    const Eigen::Vector3f startPose = currentPose();
    autonLog("start %s pose=(%.2f, %.2f, %.1fdeg)",
             entry.name,
             metersToInches(startPose.x()),
             metersToInches(startPose.y()),
             radiansToDegrees(startPose.z()));

    entry.run();

    intakes().stop();
    lift().stop();
    stopEzMotion();
    drivetrain().stop();

    const bool wasCancelled = routineCancelled();
    const Eigen::Vector3f endPose = currentPose();
    autonLog("end %s cancelled=%c pose=(%.2f, %.2f, %.1fdeg)",
             entry.name,
             wasCancelled ? 'Y' : 'N',
             metersToInches(endPose.x()),
             metersToInches(endPose.y()),
             radiansToDegrees(endPose.z()));
    g_runtime.currentEntry = nullptr;
    return !wasCancelled;
}
