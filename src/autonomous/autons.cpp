#include "autonomous/autons.h"

#include "autonomous/chassis.h"
#include "command/command.h"
#include "commands/ltvUnicycleController.h"
#include "commands/ramsete.h"
#include "motionProfiling/profileBuilder.h"
#include "pros/rtos.hpp"
#include "subsystems/intakes.h"
#include "subsystems/lift.h"
#include "tuning/relayPidAutotuner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <utility>

namespace {

constexpr float kInToM = 0.0254f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;

struct RuntimeState {
    Drivetrain* drivetrain = nullptr;
    Intakes* intakes = nullptr;
    Lift* lift = nullptr;
    std::function<Eigen::Vector3f()> poseSource;
    std::function<bool()> isCancelled;
    std::unique_ptr<Chassis> chassis;
    const AutonEntry* currentEntry = nullptr;
};

RuntimeState g_runtime;

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
           g_runtime.lift != nullptr &&
           g_runtime.chassis != nullptr;
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

Chassis& chassis() {
    return *g_runtime.chassis;
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

ez::pose fieldPoseInches(double xInches, double yInches) {
    return ez::pose{xInches, yInches};
}

float metersToInches(float meters) {
    return meters / kInToM;
}

float radiansToDegrees(float radians) {
    return radians * 180.0f / kPi;
}

bool runInlineCommand(Command& command, int pollMs = 10) {
    command.initialize();

    bool interrupted = false;
    while (!command.isFinished()) {
        if (routineCancelled()) {
            interrupted = true;
            break;
        }

        command.execute();
        if (!command.isFinished()) {
            pros::delay(pollMs);
        }
    }

    if (!interrupted && routineCancelled()) {
        interrupted = true;
    }

    command.end(interrupted);
    return !interrupted;
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

    chassis().cancel_motion();
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

void runNegative1();
void runPositive1();
void runTuneDrivePid();
void runTuneTurnPid();
void runExampleMove();
void runExampleTurn();
void runExamplePath();
void runExampleLtv();
void runSkills();

void runNegative1() {
    autonLog("Negative 1 start");
    intakes().spin(127);
    autonLog("Negative 1 path -> center");
    chassis().pid_odom_set({
        {{-47.24, -23.62}, ez::fwd, 102},
        {{-23.62, -23.62}, ez::fwd, 102},
        {{0.0, -11.81}, ez::fwd, 102},
    }, true);
    chassis().pid_wait();
    if (routineCancelled()) {
        intakes().stop();
        autonLog("Negative 1 cancelled after path");
        return;
    }

    if (!cancellableDelay(300)) {
        intakes().stop();
        autonLog("Negative 1 cancelled during settle delay");
        return;
    }
    if (!intakeTimed(-127, 500)) {
        autonLog("Negative 1 cancelled during outtake");
        return;
    }

    autonLog("Negative 1 turn -> 180");
    chassis().pid_turn_set(180.0, 96);
    chassis().pid_wait();
    if (routineCancelled()) {
        autonLog("Negative 1 cancelled after turn");
        return;
    }

    autonLog("Negative 1 return -> corner");
    chassis().pid_odom_set(fieldPoseInches(-47.24, -23.62), ez::fwd, 96);
    chassis().pid_wait();
}

void runPositive1() {
    autonLog("Positive 1 start");
    intakes().spin(127);
    autonLog("Positive 1 path -> center");
    chassis().pid_odom_set({
        {{47.24, -23.62}, ez::fwd, 102},
        {{23.62, -23.62}, ez::fwd, 102},
        {{0.0, -11.81}, ez::fwd, 102},
    }, true);
    chassis().pid_wait();
    if (routineCancelled()) {
        intakes().stop();
        autonLog("Positive 1 cancelled after path");
        return;
    }

    if (!cancellableDelay(300)) {
        intakes().stop();
        autonLog("Positive 1 cancelled during settle delay");
        return;
    }
    if (!intakeTimed(-127, 500)) {
        autonLog("Positive 1 cancelled during outtake");
        return;
    }

    autonLog("Positive 1 turn -> 0");
    chassis().pid_turn_set(0.0, 96);
    chassis().pid_wait();
    if (routineCancelled()) {
        autonLog("Positive 1 cancelled after turn");
        return;
    }

    autonLog("Positive 1 return -> corner");
    chassis().pid_odom_set(fieldPoseInches(47.24, -23.62), ez::fwd, 96);
    chassis().pid_wait();
}

void runSkills() {
    autonLog("Skills start");
    intakes().spin(127);

    chassis().pid_odom_set(fieldPoseInches(-11.81, -55.12), ez::fwd, 127);
    chassis().pid_wait();
    if (!intakeTimed(-127, 400)) {
        autonLog("Skills cancelled on first score");
        return;
    }

    intakes().spin(127);
    chassis().pid_odom_set(fieldPoseInches(-11.81, 0.0), ez::fwd, 110);
    chassis().pid_wait();
    if (routineCancelled()) {
        intakes().stop();
        autonLog("Skills cancelled after first traverse");
        return;
    }
    chassis().pid_odom_set(fieldPoseInches(11.81, 0.0), ez::fwd, 96);
    chassis().pid_wait();
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
    if (routineCancelled()) {
        lift().stop();
        autonLog("Skills cancelled after lift");
        return;
    }

    chassis().pid_odom_set(fieldPoseInches(47.24, 39.37), ez::fwd, 110);
    chassis().pid_wait();
    if (!intakeTimed(-127, 500)) {
        autonLog("Skills cancelled on third score");
        return;
    }

    chassis().pid_odom_set(fieldPoseInches(0.0, 0.0), ez::fwd, 96);
    chassis().pid_wait();
}

void runTuneDrivePid() {
    autonLog("Tune Drive PID start");
    tuning::RelayPidAutotuner::Config config;
    config.mode = tuning::RelayPidAutotuner::Mode::DriveDistance;
    config.target = CONFIG::PID_AUTOTUNE_DRIVE_TARGET_in * kInToM;
    config.relayAmplitude = CONFIG::PID_AUTOTUNE_DRIVE_RELAY;
    config.noiseBand = CONFIG::PID_AUTOTUNE_DRIVE_NOISE_in * kInToM;
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
    config.timeoutMs = CONFIG::PID_AUTOTUNE_TIMEOUT_ms;
    config.requiredPeakPairs = CONFIG::PID_AUTOTUNE_REQUIRED_PEAK_PAIRS;
    config.analysisPeakPairs = CONFIG::PID_AUTOTUNE_ANALYSIS_PEAK_PAIRS;
    config.minOscillationAmplitude = 0.25f * config.noiseBand;

    runRelayPidTune("TURN_PID", config);
}

void runExampleMove() {
    autonLog("Example Move start");
    chassis().set_local_frame();

    autonLog("Example Move drive -> 24in");
    chassis().pid_drive_set(24.0, 110);
    chassis().pid_wait();
    if (routineCancelled()) {
        autonLog("Example Move cancelled after drive");
        return;
    }

    autonLog("Example Move point -> (24,18)");
    chassis().pid_odom_set({24.0, 18.0}, ez::fwd, 110);
    chassis().pid_wait();
    if (routineCancelled()) {
        autonLog("Example Move cancelled after point");
        return;
    }

    autonLog("Example Move point -> home");
    chassis().pid_odom_set({0.0, 0.0}, ez::fwd, 110);
    chassis().pid_wait();
}

void runExampleTurn() {
    autonLog("Example Turn start");

    autonLog("Example Turn -> 90");
    chassis().pid_turn_set(90.0, 90);
    chassis().pid_wait();
    if (routineCancelled()) {
        autonLog("Example Turn cancelled at 90");
        return;
    }

    autonLog("Example Turn -> -90");
    chassis().pid_turn_set(-90.0, 90);
    chassis().pid_wait();
    if (routineCancelled()) {
        autonLog("Example Turn cancelled at -90");
        return;
    }

    autonLog("Example Turn -> 180");
    chassis().pid_turn_set(180.0, 90);
    chassis().pid_wait();
    if (routineCancelled()) {
        autonLog("Example Turn cancelled at 180");
        return;
    }

    autonLog("Example Turn -> 0");
    chassis().pid_turn_set(0.0, 90);
    chassis().pid_wait();
}

MotionProfile buildExampleProfile(const Chassis& activeChassis) {
    return buildProfile({
        activeChassis.pose_to_global(
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Chassis::CoordinateFrame::Local),
        activeChassis.pose_to_global(
            Eigen::Vector3f(18.0f * kInToM, 10.0f * kInToM, 0.35f),
            Chassis::CoordinateFrame::Local),
        activeChassis.pose_to_global(
            Eigen::Vector3f(36.0f * kInToM, -8.0f * kInToM, -0.25f),
            Chassis::CoordinateFrame::Local),
        activeChassis.pose_to_global(
            Eigen::Vector3f(48.0f * kInToM, 0.0f, 0.0f),
            Chassis::CoordinateFrame::Local),
    }, 1.0f, 1.8f);
}

void runExamplePath() {
    autonLog("Example Path start");
    chassis().set_local_frame();
    RamseteCommand command(
        &drivetrain(),
        buildExampleProfile(chassis()),
        []() {
            return currentPose();
        });

    intakes().spin(96);
    if (!runInlineCommand(command)) {
        intakes().stop();
        autonLog("Example Path cancelled during ramsete");
        return;
    }
    if (!intakeTimed(-127, 300)) {
        autonLog("Example Path cancelled during outtake");
        return;
    }
    chassis().pid_odom_set({0.0, 0.0}, ez::fwd, 96);
    chassis().pid_wait();
}

void runExampleLtv() {
    autonLog("Example LTV start");
    chassis().set_local_frame();
    LtvUnicycleCommand command(
        &drivetrain(),
        buildExampleProfile(chassis()),
        []() {
            return currentPose();
        });

    intakes().spin(96);
    if (!runInlineCommand(command)) {
        intakes().stop();
        autonLog("Example LTV cancelled during track");
        return;
    }
    if (!intakeTimed(-127, 300)) {
        autonLog("Example LTV cancelled during outtake");
        return;
    }
    chassis().pid_odom_set({0.0, 0.0}, ez::fwd, 96);
    chassis().pid_wait();
}

const AutonList kAvailableAutons = {{
    {Auton::NEGATIVE_1, "Negative 1", &runNegative1},
    {Auton::POSITIVE_1, "Positive 1", &runPositive1},
    {Auton::TUNE_DRIVE_PID, "Tune Drive PID", &runTuneDrivePid},
    {Auton::TUNE_TURN_PID, "Tune Turn PID", &runTuneTurnPid},
    {Auton::EXAMPLE_MOVE, "Example Move", &runExampleMove},
    {Auton::EXAMPLE_TURN, "Example Turn", &runExampleTurn},
    {Auton::EXAMPLE_PATH, "Example Path", &runExamplePath},
    {Auton::EXAMPLE_LTV, "Example LTV", &runExampleLtv},
    {Auton::SKILLS, "Skills", &runSkills},
    {Auton::NONE, "None", nullptr},
}};

} // namespace

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
    g_runtime.chassis =
        std::make_unique<Chassis>(&drivetrainRef, g_runtime.poseSource, g_runtime.isCancelled);
    g_runtime.chassis->pid_targets_reset();
    g_runtime.chassis->set_local_origin_here();
    g_runtime.chassis->set_global_frame();

    const Eigen::Vector3f pose = currentPose();
    autonLog("runtime bound pose=(%.2f, %.2f, %.1fdeg)",
             metersToInches(pose.x()),
             metersToInches(pose.y()),
             radiansToDegrees(pose.z()));
}

void resetAutonRuntime() {
    if (g_runtime.chassis) {
        g_runtime.chassis->cancel_motion();
    }
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
    chassis().cancel_motion();
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
