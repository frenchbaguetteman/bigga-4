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
#include <cmath>

namespace {
inline float normalizeAngleRad(float a) {
    return std::atan2(std::sin(a), std::cos(a));
}
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

    syncOdometryState();
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

void Drivetrain::stop() {
    m_left.brake();
    m_right.brake();
    m_lastSpeeds = {};
}

void Drivetrain::tankVoltage(float left, float right) {
    m_left.move(motorUtil::clampVoltage(left));
    m_right.move(motorUtil::clampVoltage(right));
}

// ── Odometry ────────────────────────────────────────────────────────────────

float Drivetrain::rawForwardDistance() const {
    if (m_verticalTracking) {
        // Rotation sensor: centidegrees → radians → metres
        float centideg = static_cast<float>(m_verticalTracking->get_position());
        float radians  = centideg / 100.0f * static_cast<float>(M_PI) / 180.0f;
        return radians * CONFIG::ODOM_RADIUS;
    }
    // Fallback: average of left/right drive motor encoder positions
    // MotorGroup::get_position() returns degrees (average of all motors)
    float leftDeg  = static_cast<float>(m_left.get_position());
    float rightDeg = static_cast<float>(m_right.get_position());
    float avgDeg   = (leftDeg + rightDeg) / 2.0f;
    return (avgDeg / 360.0f) * 2.0f * static_cast<float>(M_PI) * CONFIG::DRIVE_RADIUS;
}

float Drivetrain::rawLateralDistance() const {
    if (m_horizontalTracking) {
        float centideg = static_cast<float>(m_horizontalTracking->get_position());
        float radians  = centideg / 100.0f * static_cast<float>(M_PI) / 180.0f;
        return radians * CONFIG::ODOM_RADIUS;
    }
    return 0.0f;  // no horizontal tracking → no lateral correction
}

void Drivetrain::updateOdometry() {
    float fwd = rawForwardDistance();
    float lat = rawLateralDistance();
    float heading = getHeading();

    float dFwd = fwd - m_prevForwardDist;
    float dLat = lat - m_prevLateralDist;
    float dTheta = normalizeAngleRad(heading - m_prevHeading);

    // Correct lateral for tracking-wheel arc during turns
    float dLatCorrected = dLat - CONFIG::LATERAL_WHEEL_OFFSET_M * dTheta;

    m_prevForwardDist = fwd;
    m_prevLateralDist = lat;
    m_prevHeading     = heading;

    // Mid-angle for arc approximation
    float midTheta = heading - dTheta / 2.0f;

    float dx = dFwd * std::cos(midTheta) - dLatCorrected * std::sin(midTheta);
    float dy = dFwd * std::sin(midTheta) + dLatCorrected * std::cos(midTheta);

    m_pose.x() += dx;
    m_pose.y() += dy;
    m_pose.z()  = heading;

    // Accumulate displacement for PF consumption
    m_pendingDisplacement.x() += dx;
    m_pendingDisplacement.y() += dy;
}

Eigen::Vector3f Drivetrain::getOdomPose() const { return m_pose; }

Eigen::Vector3f Drivetrain::getPose() const { return getOdomPose(); }

void Drivetrain::setOdomPose(const Eigen::Vector3f& pose) {
    m_pose = pose;
    syncOdometryState();
    m_prevHeading = pose.z();
}

void Drivetrain::syncLocalizationReference(const Eigen::Vector3f& pose) {
    // 1. IMU heading
    m_imu.set_heading(CONFIG::mathRadToCompassDeg(pose.z()));
    // 2. Odom pose
    m_pose = pose;
    // 3. Zero encoder baselines
    resetEncoders();
    // 4. Explicit heading baseline (avoids IMU read-back lag)
    syncOdomBaselinesToCurrentSensors(pose.z());
    // 5. Clear pending displacement
    m_pendingDisplacement = Eigen::Vector2f(0.0f, 0.0f);
}

void Drivetrain::setPose(const Eigen::Vector3f& pose) { setOdomPose(pose); }

float Drivetrain::getHeading() const {
    return CONFIG::compassDegToMathRad(static_cast<float>(m_imu.get_heading()));
}

void Drivetrain::resetHeading(float heading) {
    m_imu.set_heading(CONFIG::mathRadToCompassDeg(heading));
    syncOdometryState();
    m_prevHeading = heading;
    m_pose.z() = heading;
}

Eigen::Vector2f Drivetrain::getDisplacement() {
    // Return displacement accumulated by updateOdometry() since last call
    Eigen::Vector2f d = m_pendingDisplacement;
    m_pendingDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    return d;
}

float Drivetrain::getForwardDistance() const {
    return rawForwardDistance();
}

void Drivetrain::resetEncoders() {
    if (m_verticalTracking)   m_verticalTracking->reset_position();
    if (m_horizontalTracking) m_horizontalTracking->reset_position();
    m_left.tare_position();
    m_right.tare_position();
    syncOdometryState();
}

void Drivetrain::syncOdometryState() {
    m_prevForwardDist = rawForwardDistance();
    m_prevLateralDist = rawLateralDistance();
    m_prevHeading = getHeading();
}

void Drivetrain::syncOdomBaselinesToCurrentSensors(float heading) {
    m_prevForwardDist = rawForwardDistance();
    m_prevLateralDist = rawLateralDistance();
    m_prevHeading = heading;
}
