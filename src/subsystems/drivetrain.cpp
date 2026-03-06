/**
 * @file drivetrain.cpp
 * Drivetrain subsystem implementation.
 *
 * Uses rotation sensors for tracking wheels when their port ≠ 0.
 * Falls back to averaged drive motor encoders when vertical tracking is disabled.
 */
#include "subsystems/drivetrain.h"
#include "config.h"
#include "utils/utils.h"
#include "utils/motor.h"
#include "utils/localization_math.h"
#include <algorithm>
#include <cmath>

namespace {
inline float normalizeAngleRad(float a) {
    return std::atan2(std::sin(a), std::cos(a));
}
}

float Drivetrain::joystickCurve(float input, float t, float deadzone) {
    if (std::fabs(input) < deadzone) return 0.0f;
    if (t == 0.0f) return input;
    const float expNeg = std::exp(-t / 10.0f);
    const float expTerm = std::exp((std::fabs(input) - 127.0f) / 10.0f);
    return (expNeg + expTerm * (1.0f - expNeg)) * input;
}

// ── Construction ────────────────────────────────────────────────────────────

Drivetrain::Drivetrain()
    : m_left(CONFIG::LEFT_DRIVE_PORTS)
    , m_right(CONFIG::RIGHT_DRIVE_PORTS)
    , m_imu(CONFIG::IMU_PORT)
{
    m_left.set_gearing(pros::E_MOTOR_GEAR_BLUE);   // 600 RPM cartridge
    m_right.set_gearing(pros::E_MOTOR_GEAR_BLUE);
    m_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    m_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // Initialise optional tracking wheels
    if constexpr (CONFIG::VERTICAL_TRACKING_PORT != 0) {
        m_verticalTracking = std::make_unique<pros::Rotation>(CONFIG::VERTICAL_TRACKING_PORT);
        m_verticalTracking->set_reversed(CONFIG::VERTICAL_TRACKING_REVERSED);
        m_verticalTracking->reset_position();
    }
    if constexpr (CONFIG::HORIZONTAL_TRACKING_PORT != 0) {
        m_horizontalTracking = std::make_unique<pros::Rotation>(CONFIG::HORIZONTAL_TRACKING_PORT);
        m_horizontalTracking->set_reversed(CONFIG::HORIZONTAL_TRACKING_REVERSED);
        m_horizontalTracking->reset_position();
    }

    if constexpr (CONFIG::VERTICAL_TRACKING_PORT == 0) {
        std::printf("[ODOM] WARN: vertical tracking wheel disabled; using drive-encoder fallback\n");
    }
    if constexpr (CONFIG::HORIZONTAL_TRACKING_PORT == 0) {
        std::printf("[ODOM] WARN: horizontal tracking wheel disabled; lateral odom = 0\n");
    }

    syncOdometryState();
    resetDriverAssistState();
}

// ── Periodic ────────────────────────────────────────────────────────────────

void Drivetrain::periodic() {
    updateOdometry();
}

// ── Control ─────────────────────────────────────────────────────────────────

void Drivetrain::setDriveSpeeds(DriveSpeeds speeds) {
    m_lastSpeeds = speeds;
    auto [leftMV, rightMV] = CONFIG::DRIVETRAIN_FEEDFORWARD(
        speeds.linear, speeds.angular);
    m_left.move(motorUtil::mVToMove(leftMV));
    m_right.move(motorUtil::mVToMove(rightMV));
}

void Drivetrain::arcade(float forward, float turn) {
    float left  = forward + turn;
    float right = forward - turn;
    m_left.move(motorUtil::clampVoltage(left));
    m_right.move(motorUtil::clampVoltage(right));
}

void Drivetrain::driverArcade(float forwardInput, float turnInput) {
    const bool sticksActive = std::fabs(forwardInput) >= m_activeBrakeStickDeadband ||
                              std::fabs(turnInput) >= m_activeBrakeStickDeadband;

    if (sticksActive) {
        m_activeBrakeWasDriving = true;
        m_activeBrakeLeftTargetDeg = leftMotorPositionDeg();
        m_activeBrakeRightTargetDeg = rightMotorPositionDeg();

        const float curvedForward = joystickCurve(forwardInput, m_driverForwardCurve, m_driverDeadband);
        const float curvedTurn = joystickCurve(turnInput, m_driverTurnCurve, m_driverDeadband);
        arcade(curvedForward, curvedTurn);
        return;
    }

    if (m_activeBrakeEnabled && m_activeBrakeWasDriving) {
        applyActiveBrake();
        return;
    }

    tankVoltage(0.0f, 0.0f);
}

void Drivetrain::resetDriverAssistState() {
    m_activeBrakeWasDriving = false;
    m_activeBrakeLeftTargetDeg = leftMotorPositionDeg();
    m_activeBrakeRightTargetDeg = rightMotorPositionDeg();
}

void Drivetrain::stop() {
    m_left.brake();
    m_right.brake();
    m_lastSpeeds = {};
    resetDriverAssistState();
}

void Drivetrain::tankVoltage(float left, float right) {
    m_left.move(motorUtil::clampVoltage(left));
    m_right.move(motorUtil::clampVoltage(right));
}

void Drivetrain::applyActiveBrake() {
    const float leftError = m_activeBrakeLeftTargetDeg - leftMotorPositionDeg();
    const float rightError = m_activeBrakeRightTargetDeg - rightMotorPositionDeg();

    if (std::fabs(leftError) <= m_activeBrakePosDeadbandDeg &&
        std::fabs(rightError) <= m_activeBrakePosDeadbandDeg) {
        tankVoltage(0.0f, 0.0f);
        return;
    }

    float leftOut = std::clamp(leftError * m_activeBrakeKp,
                               -m_activeBrakePower, m_activeBrakePower);
    float rightOut = std::clamp(rightError * m_activeBrakeKp,
                                -m_activeBrakePower, m_activeBrakePower);

    if (std::fabs(leftOut) < m_activeBrakeOutputDeadband) leftOut = 0.0f;
    if (std::fabs(rightOut) < m_activeBrakeOutputDeadband) rightOut = 0.0f;

    tankVoltage(leftOut, rightOut);
}

float Drivetrain::leftMotorPositionDeg() const {
    return static_cast<float>(m_left.get_position());
}

float Drivetrain::rightMotorPositionDeg() const {
    return static_cast<float>(m_right.get_position());
}

// ── Odometry ────────────────────────────────────────────────────────────────

float Drivetrain::rawForwardDistance() const {
    if (m_verticalTracking) {
        // Rotation sensor: centidegrees → radians → metres
        float centideg = static_cast<float>(m_verticalTracking->get_position());
        float radians  = centideg / 100.0f * static_cast<float>(M_PI) / 180.0f;
        return radians * CONFIG::ODOM_RADIUS.getValue();
    }
    // Fallback: average of left/right drive motor encoder positions
    // MotorGroup::get_position() returns degrees (average of all motors)
    float leftDeg  = static_cast<float>(m_left.get_position());
    float rightDeg = static_cast<float>(m_right.get_position());
    float avgDeg   = (leftDeg + rightDeg) / 2.0f;
    return (avgDeg / 360.0f) * 2.0f * static_cast<float>(M_PI) * CONFIG::DRIVE_RADIUS.getValue();
}

float Drivetrain::rawLateralDistance() const {
    if (m_horizontalTracking) {
        float centideg = static_cast<float>(m_horizontalTracking->get_position());
        float radians  = centideg / 100.0f * static_cast<float>(M_PI) / 180.0f;
        return radians * CONFIG::ODOM_RADIUS.getValue();
    }
    return 0.0f;  // no horizontal tracking → no lateral correction
}

void Drivetrain::updateOdometry() {
    float fwd = rawForwardDistance();
    float lat = rawLateralDistance();
    float heading = getHeading();

    if (!LocMath::isFinite(fwd) || !LocMath::isFinite(lat) || !LocMath::isFinite(heading) ||
        !LocMath::isFinite(m_prevForwardDist) || !LocMath::isFinite(m_prevLateralDist) ||
        !LocMath::isFinite(m_prevHeading)) {
        std::printf("[ODOM] non-finite sensor/baseline, skipping tick\n");
        m_lastStepDisplacement = Eigen::Vector2f(0.0f, 0.0f);
        m_lastStepFwdOnlyDisplacement = Eigen::Vector2f(0.0f, 0.0f);
        syncOdometryState();
        return;
    }

    float dFwd = fwd - m_prevForwardDist;
    float dLat = lat - m_prevLateralDist;
    float dTheta = normalizeAngleRad(heading - m_prevHeading);

    // Correct lateral for tracking-wheel arc during turns
    float dLatCorrected = dLat - CONFIG::LATERAL_WHEEL_OFFSET.getValue() * dTheta;

    m_prevForwardDist = fwd;
    m_prevLateralDist = lat;
    m_prevHeading     = heading;

    // Mid-angle for arc approximation
    float midTheta = heading - dTheta / 2.0f;

    float dx = dFwd * std::cos(midTheta) - dLatCorrected * std::sin(midTheta);
    float dy = dFwd * std::sin(midTheta) + dLatCorrected * std::cos(midTheta);

    // Forward-only displacement (no lateral wheel) for MCL prediction
    float dxFwdOnly = dFwd * std::cos(midTheta);
    float dyFwdOnly = dFwd * std::sin(midTheta);

    m_lastStepDisplacement = Eigen::Vector2f(dx, dy);
    m_lastStepFwdOnlyDisplacement = Eigen::Vector2f(dxFwdOnly, dyFwdOnly);

    m_pose.x() += dx;
    m_pose.y() += dy;
    m_pose.z()  = heading;

    // Accumulate displacement for PF consumption
    ++m_odomGeneration;
    m_pendingDisplacement.x() += dx;
    m_pendingDisplacement.y() += dy;
    m_pendingFwdOnlyDisplacement.x() += dxFwdOnly;
    m_pendingFwdOnlyDisplacement.y() += dyFwdOnly;
}

Eigen::Vector3f Drivetrain::getOdomPose() const { return m_pose; }

Eigen::Vector3f Drivetrain::getPose() const { return getOdomPose(); }

void Drivetrain::setOdomPose(const Eigen::Vector3f& pose) {
    if (!LocMath::isFinitePose(pose)) {
        std::printf("[ODOM] setOdomPose non-finite, forcing origin\n");
        m_pose = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    } else {
        m_pose = pose;
    }
    syncOdometryState();
    m_prevHeading = m_pose.z();
}

void Drivetrain::syncLocalizationReference(const Eigen::Vector3f& pose) {
    Eigen::Vector3f safePose = LocMath::isFinitePose(pose)
        ? pose : Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    if (!LocMath::isFinitePose(pose)) {
        std::printf("[ODOM] syncLocalizationReference non-finite input, forcing origin\n");
    }
    // 1. IMU heading (convert internal radians → VEX GPS compass degrees)
    m_imu.set_heading(CONFIG::internalRadToGpsHeadingDeg(safePose.z()));
    // 2. Odom pose
    m_pose = safePose;
    // 3. Zero encoder baselines
    resetEncoders();
    // 4. Explicit heading baseline (avoids IMU read-back lag)
    syncOdomBaselinesToCurrentSensors(safePose.z());
    // 5. Clear pending displacement
    m_lastStepDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    m_lastStepFwdOnlyDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    m_pendingDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    m_pendingFwdOnlyDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    m_lastConsumedGeneration = m_odomGeneration;
    m_lastConsumedFwdOnlyGeneration = m_odomGeneration;
}

void Drivetrain::setPose(const Eigen::Vector3f& pose) { setOdomPose(pose); }

float Drivetrain::getHeading() const {
    const float rawHeadingDeg = static_cast<float>(m_imu.get_heading());
    if (!LocMath::isFinite(rawHeadingDeg)) {
        if (LocMath::isFinite(m_prevHeading)) return m_prevHeading;
        if (LocMath::isFinite(m_pose.z())) return m_pose.z();
        return 0.0f;
    }

    const float h = CONFIG::gpsHeadingDegToInternalRad(rawHeadingDeg);
    if (!LocMath::isFinite(h)) {
        if (LocMath::isFinite(m_prevHeading)) return m_prevHeading;
        if (LocMath::isFinite(m_pose.z())) return m_pose.z();
        return 0.0f;
    }
    return h;
}

void Drivetrain::calibrateImu() {
    std::printf("[IMU] Calibrating...\n");
    m_imu.reset(true);
    while (m_imu.is_calibrating()) {
        pros::delay(10);
    }
    syncOdometryState();
    m_lastStepDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    m_lastStepFwdOnlyDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    m_pendingDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    m_pendingFwdOnlyDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    m_lastConsumedGeneration = m_odomGeneration;
    m_lastConsumedFwdOnlyGeneration = m_odomGeneration;
    std::printf("[IMU] Ready\n");
}

void Drivetrain::resetHeading(float heading) {
    m_imu.set_heading(CONFIG::internalRadToGpsHeadingDeg(heading));
    syncOdometryState();
    m_prevHeading = heading;
    m_pose.z() = heading;
}

Eigen::Vector2f Drivetrain::consumePendingDisplacement() {
    if (m_lastConsumedGeneration == m_odomGeneration) {
        if (std::fabs(m_pendingDisplacement.x()) > 1e-6f ||
            std::fabs(m_pendingDisplacement.y()) > 1e-6f) {
            std::printf("[ODOM] WARN: displacement consumed without new odom generation\n");
        }
    }

    // Return displacement accumulated by updateOdometry() since last call
    Eigen::Vector2f d = m_pendingDisplacement;
    if (!LocMath::isFiniteVec2(d)) {
        std::printf("[ODOM] non-finite pending displacement, zeroing\n");
        d = Eigen::Vector2f(0.0f, 0.0f);
    }
    m_pendingDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    m_lastConsumedGeneration = m_odomGeneration;
    return d;
}

Eigen::Vector2f Drivetrain::consumePendingFwdOnlyDisplacement() {
    Eigen::Vector2f d = m_pendingFwdOnlyDisplacement;
    if (!LocMath::isFiniteVec2(d)) {
        std::printf("[ODOM] non-finite pending fwd-only displacement, zeroing\n");
        d = Eigen::Vector2f(0.0f, 0.0f);
    }
    m_pendingFwdOnlyDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    m_lastConsumedFwdOnlyGeneration = m_odomGeneration;
    return d;
}

Eigen::Vector2f Drivetrain::getDisplacement() {
    // Legacy alias for backward compatibility
    return consumePendingDisplacement();
}

float Drivetrain::getForwardDistance() const {
    return rawForwardDistance();
}

void Drivetrain::resetEncoders() {
    if (m_verticalTracking)   m_verticalTracking->reset_position();
    if (m_horizontalTracking) m_horizontalTracking->reset_position();
    m_left.tare_position();
    m_right.tare_position();
    m_lastStepDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    m_lastStepFwdOnlyDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    syncOdometryState();
}

void Drivetrain::syncOdometryState() {
    float fwd = rawForwardDistance();
    float lat = rawLateralDistance();
    float h = getHeading();

    m_prevForwardDist = LocMath::isFinite(fwd) ? fwd : 0.0f;
    m_prevLateralDist = LocMath::isFinite(lat) ? lat : 0.0f;
    m_prevHeading = LocMath::isFinite(h) ? h : 0.0f;
}

void Drivetrain::syncOdomBaselinesToCurrentSensors(float heading) {
    m_prevForwardDist = rawForwardDistance();
    m_prevLateralDist = rawLateralDistance();
    m_prevHeading = heading;
}
