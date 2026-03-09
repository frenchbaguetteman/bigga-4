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

float Drivetrain::joystickCurve(float input, float t) {
    if (std::fabs(input) < CONFIG::DRIVER_JOYSTICK_DEADBAND) return 0.0f;
    float sign = (input > 0.0f) ? 1.0f : -1.0f;
    float norm = std::fabs(input) / 127.0f;
    float curved = std::pow(norm, t);
    return sign * curved * 127.0f;
}

void Drivetrain::driverArcade(float forwardStick, float turnStick) {
    float fwd  = joystickCurve(forwardStick, CONFIG::DRIVER_FORWARD_CURVE_T);
    float turn = joystickCurve(turnStick, CONFIG::DRIVER_TURN_CURVE_T);

    bool stickIdle = (std::fabs(fwd) < CONFIG::DRIVER_ACTIVE_BRAKE_STICK_DEADBAND &&
                      std::fabs(turn) < CONFIG::DRIVER_ACTIVE_BRAKE_STICK_DEADBAND);

    if (CONFIG::DRIVER_ACTIVE_BRAKE_ENABLED && stickIdle) {
        if (m_wasStopped) {
            // Active brake: hold current heading
            float heading = getHeading();
            float headingDeg = heading * 180.0f / static_cast<float>(M_PI);
            float err = m_activeBrakeTargetDeg - headingDeg;
            while (err >  180.0f) err -= 360.0f;
            while (err < -180.0f) err += 360.0f;

            float brakePower = CONFIG::DRIVER_ACTIVE_BRAKE_KP * err;
            if (std::fabs(err) < CONFIG::DRIVER_ACTIVE_BRAKE_POS_DEADBAND_deg) {
                brakePower = 0.0f;
            }
            if (std::fabs(brakePower) > CONFIG::DRIVER_ACTIVE_BRAKE_POWER) {
                brakePower = (brakePower > 0.0f) ? CONFIG::DRIVER_ACTIVE_BRAKE_POWER
                                                  : -CONFIG::DRIVER_ACTIVE_BRAKE_POWER;
            }
            if (std::fabs(brakePower) < CONFIG::DRIVER_ACTIVE_BRAKE_OUTPUT_DEADBAND) {
                brakePower = 0.0f;
            }
            arcade(0.0f, brakePower);
        } else {
            m_activeBrakeTargetDeg = getHeading() * 180.0f / static_cast<float>(M_PI);
            m_wasStopped = true;
            arcade(0.0f, 0.0f);
        }
    } else {
        m_wasStopped = false;
        arcade(fwd, turn);
    }
}

void Drivetrain::resetDriverAssistState() {
    m_wasStopped = true;
    m_activeBrakeTargetDeg = getHeading() * 180.0f / static_cast<float>(M_PI);
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
