/**
 * @file drivetrain.cpp
 * 69580A — OkapiLib-based drivetrain implementation.
 */
#include "subsystems/drivetrain.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

Drivetrain::Drivetrain()
    : m_imu(CONFIG::IMU_PORT) {

    // Build the OkapiLib chassis controller with odometry.
    // Uses the built-in motor encoders for tracking (the project's vertical
    // tracking wheel port is 0 / disabled).  The horizontal tracking wheel
    // is useful for localization but not needed for OkapiLib's 2-encoder odom.

    // Wheel diameter in inches, track width in inches
    constexpr double wheelDiameterIn = CONFIG::DRIVE_RADIUS_in * 2.0;
    constexpr double trackWidthIn    = CONFIG::TRACK_WIDTH_in;

    m_chassis = okapi::ChassisControllerBuilder()
        .withMotors(
            {CONFIG::LEFT_DRIVE_PORTS[0], CONFIG::LEFT_DRIVE_PORTS[1], CONFIG::LEFT_DRIVE_PORTS[2]},
            {CONFIG::RIGHT_DRIVE_PORTS[0], CONFIG::RIGHT_DRIVE_PORTS[1], CONFIG::RIGHT_DRIVE_PORTS[2]})
        .withDimensions(
            {okapi::AbstractMotor::gearset::blue, (1.0)},
            {{wheelDiameterIn * okapi::inch, trackWidthIn * okapi::inch},
             static_cast<int>(okapi::imev5BlueTPR)})
        .withGains(
            {CONFIG::DISTANCE_PID.kP, CONFIG::DISTANCE_PID.kI, CONFIG::DISTANCE_PID.kD},
            {CONFIG::TURN_PID.kP, CONFIG::TURN_PID.kI, CONFIG::TURN_PID.kD})
        .withOdometry()
        .buildOdometry();

    if (auto model = getModel()) {
        model->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    }
    resetDriverAssistState();

    std::printf("[DRIVE] OkapiLib chassis built: blue 600RPM, wheel=%.3f\" track=%.3f\"\n",
        wheelDiameterIn, trackWidthIn);
}

void Drivetrain::periodic() {
    // Update displacement tracking for the particle filter.
    Eigen::Vector3f curPose = getOdomPose();
    Eigen::Vector2f delta(curPose.x() - m_prevOdomPose.x(),
                          curPose.y() - m_prevOdomPose.y());
    m_lastStepDisplacement = delta;
    m_pendingDisplacement += delta;

    // Forward-only component (for MCL prediction)
    float heading = curPose.z();
    Eigen::Vector2f fwdAxis(std::cos(heading), std::sin(heading));
    float fwdDist = delta.dot(fwdAxis);
    if (fwdDist > 0.0f) {
        m_pendingFwdOnly += Eigen::Vector2f(
            fwdDist * std::cos(heading),
            fwdDist * std::sin(heading));
    }

    m_prevOdomPose = curPose;
}

std::shared_ptr<okapi::ChassisModel> Drivetrain::getModel() const {
    return m_chassis ? m_chassis->getModel() : nullptr;
}

// ── Motor control ───────────────────────────────────────────────────────────

void Drivetrain::arcade(float forward, float turn) {
    if (!m_chassis) return;
    // OkapiLib arcade takes [-1, 1] range; our inputs are [-127, 127]
    m_chassis->getModel()->arcade(forward / 127.0, turn / 127.0);
}

void Drivetrain::stop() {
    if (!m_chassis) return;
    m_chassis->stop();
}

void Drivetrain::setDriveSpeeds(DriveSpeeds speeds) {
    if (!m_chassis) return;
    m_lastSpeeds = speeds;
    // Use CONFIG feedforward model: (v, ω) → (leftMV, rightMV)
    auto [leftMV, rightMV] = CONFIG::DRIVETRAIN_FEEDFORWARD(
        speeds.linear, speeds.angular);
    // OkapiLib tank() takes [-1, 1]; feedforward gives millivolts, scale to [-1, 1]
    constexpr float kMaxMV = 12000.0f;
    m_chassis->getModel()->tank(
        std::clamp(leftMV / kMaxMV, -1.0f, 1.0f),
        std::clamp(rightMV / kMaxMV, -1.0f, 1.0f));
}

float Drivetrain::joystickCurve(float input, float t, float deadzone) {
    if (std::fabs(input) < deadzone) return 0.0f;
    if (t == 0.0f) return input;
    const float expNeg = std::exp(-t / 10.0f);
    const float expTerm = std::exp((std::fabs(input) - 127.0f) / 10.0f);
    return (expNeg + expTerm * (1.0f - expNeg)) * input;
}

void Drivetrain::driverArcade(float forwardStick, float turnStick) {
    const bool sticksActive = std::fabs(forwardStick) >= m_activeBrakeStickDeadband ||
                              std::fabs(turnStick) >= m_activeBrakeStickDeadband;

    if (sticksActive) {
        m_activeBrakeWasDriving = true;
        m_activeBrakeLeftTargetDeg = leftMotorPositionDeg();
        m_activeBrakeRightTargetDeg = rightMotorPositionDeg();

        const float curvedForward =
            joystickCurve(forwardStick, m_driverForwardCurve, m_driverDeadband);
        const float curvedTurn =
            joystickCurve(turnStick, m_driverTurnCurve, m_driverDeadband);
        arcade(curvedForward, curvedTurn);
        return;
    }

    if (m_activeBrakeEnabled && m_activeBrakeWasDriving) {
        applyActiveBrake();
        return;
    }

    if (auto model = getModel()) {
        model->tank(0.0, 0.0);
    }
}

void Drivetrain::resetDriverAssistState() {
    m_activeBrakeWasDriving = false;
    m_activeBrakeLeftTargetDeg = leftMotorPositionDeg();
    m_activeBrakeRightTargetDeg = rightMotorPositionDeg();
}

void Drivetrain::applyActiveBrake() {
    auto model = getModel();
    if (!model) return;

    const float leftError = m_activeBrakeLeftTargetDeg - leftMotorPositionDeg();
    const float rightError = m_activeBrakeRightTargetDeg - rightMotorPositionDeg();

    if (std::fabs(leftError) <= m_activeBrakePosDeadbandDeg &&
        std::fabs(rightError) <= m_activeBrakePosDeadbandDeg) {
        model->tank(0.0, 0.0);
        return;
    }

    float leftOut = std::clamp(leftError * m_activeBrakeKp,
                               -m_activeBrakePower, m_activeBrakePower);
    float rightOut = std::clamp(rightError * m_activeBrakeKp,
                                -m_activeBrakePower, m_activeBrakePower);

    if (std::fabs(leftOut) < m_activeBrakeOutputDeadband) leftOut = 0.0f;
    if (std::fabs(rightOut) < m_activeBrakeOutputDeadband) rightOut = 0.0f;

    model->tank(leftOut / 127.0f, rightOut / 127.0f);
}

float Drivetrain::leftMotorPositionDeg() const {
    auto model = getModel();
    if (!model) return 0.0f;
    const auto sensors = model->getSensorVals();
    if (sensors.size() < 2) return 0.0f;
    return static_cast<float>(sensors[0]) * 360.0f / static_cast<float>(okapi::imev5BlueTPR);
}

float Drivetrain::rightMotorPositionDeg() const {
    auto model = getModel();
    if (!model) return 0.0f;
    const auto sensors = model->getSensorVals();
    if (sensors.size() < 2) return 0.0f;
    return static_cast<float>(sensors[1]) * 360.0f / static_cast<float>(okapi::imev5BlueTPR);
}

// ── Heading / IMU ───────────────────────────────────────────────────────────

float Drivetrain::getHeading() const {
    // Convert VEX compass heading to internal radians (0 = +X, CCW+)
    float compassDeg = m_imu.get_heading();
    if (!std::isfinite(compassDeg)) return 0.0f;
    return CONFIG::gpsHeadingDegToInternalRad(compassDeg);
}

void Drivetrain::calibrateImu() {
    m_imu.reset(true);  // blocking calibration
    while (m_imu.is_calibrating()) {
        pros::delay(10);
    }
    std::printf("[DRIVE] IMU calibrated\n");
}

void Drivetrain::resetHeading(float headingRad) {
    float compassDeg = CONFIG::internalRadToGpsHeadingDeg(headingRad);
    m_imu.set_heading(compassDeg);
}

// ── Odometry ────────────────────────────────────────────────────────────────

Eigen::Vector3f Drivetrain::getOdomPose() const {
    if (!m_chassis) return Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    auto state = m_chassis->getState();
    return Eigen::Vector3f(
        static_cast<float>(state.x.convert(okapi::meter)),
        static_cast<float>(state.y.convert(okapi::meter)),
        static_cast<float>(state.theta.convert(okapi::radian)));
}

float Drivetrain::rawForwardDistance() const {
    if (!m_chassis) return 0.0f;
    auto model = m_chassis->getModel();
    auto sensors = model->getSensorVals();
    if (sensors.size() < 2) return 0.0f;
    // Average of left and right encoder, convert from ticks to metres
    double avgTicks = (sensors[0] + sensors[1]) / 2.0;
    double circumference = CONFIG::DRIVE_RADIUS_in * 2.0 * CONFIG::IN_TO_M * M_PI;
    return static_cast<float>(avgTicks / okapi::imev5BlueTPR * circumference);
}

void Drivetrain::resetEncoders() {
    if (!m_chassis) return;
    m_chassis->getModel()->resetSensors();
}

// ── Localization helpers ────────────────────────────────────────────────────

void Drivetrain::syncLocalizationReference(const Eigen::Vector3f& pose) {
    if (m_chassis) {
        m_chassis->setState({
            pose.x() * okapi::meter,
            pose.y() * okapi::meter,
            pose.z() * okapi::radian});
    }
    resetHeading(pose.z());
    m_prevOdomPose = pose;
    m_pendingDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    m_pendingFwdOnly = Eigen::Vector2f(0.0f, 0.0f);
    m_lastStepDisplacement = Eigen::Vector2f(0.0f, 0.0f);
}

Eigen::Vector2f Drivetrain::consumePendingDisplacement() {
    Eigen::Vector2f d = m_pendingDisplacement;
    m_pendingDisplacement = Eigen::Vector2f(0.0f, 0.0f);
    return d;
}

Eigen::Vector2f Drivetrain::consumePendingFwdOnlyDisplacement() {
    Eigen::Vector2f d = m_pendingFwdOnly;
    m_pendingFwdOnly = Eigen::Vector2f(0.0f, 0.0f);
    return d;
}

Eigen::Vector2f Drivetrain::getLastStepDisplacementDebug() const {
    return m_lastStepDisplacement;
}

// ── OkapiLib motion ─────────────────────────────────────────────────────────

void Drivetrain::moveDistance(okapi::QLength distance) {
    if (!m_chassis) return;
    m_chassis->moveDistance(distance);
}

void Drivetrain::turnAngle(okapi::QAngle angle) {
    if (!m_chassis) return;
    m_chassis->turnAngle(angle);
}

void Drivetrain::waitUntilSettled() {
    if (!m_chassis) return;
    m_chassis->waitUntilSettled();
}

void Drivetrain::setMaxVelocity(int velocity) {
    if (!m_chassis) return;
    m_chassis->setMaxVelocity(velocity);
}
