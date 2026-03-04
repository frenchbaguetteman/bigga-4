/**
 * @file drivetrain.h
 * Drivetrain subsystem — 6-motor differential drive with IMU, optional
 * tracking wheels, feedforward model, and pose estimation hookup.
 *
 * Vertical tracking port = 0  →  falls back to averaged drive motor encoders.
 * Horizontal tracking port = 0  →  lateral displacement assumed zero.
 */
#pragma once

#include "command/subsystem.h"
#include "Eigen/Dense"
#include "units/units.hpp"
#include "config.h"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include <utility>
#include <functional>
#include <memory>

struct DriveSpeeds {
    float linear  = 0.0f;  // m/s
    float angular = 0.0f;  // rad/s
};

class Drivetrain : public Subsystem {
public:
    Drivetrain();

    // ── Subsystem interface ─────────────────────────────────────────────
    void periodic() override;

    // ── Control ─────────────────────────────────────────────────────────

    /** Set desired chassis speeds (m/s, rad/s). Uses feedforward. */
    void setDriveSpeeds(DriveSpeeds speeds);

    /** Direct voltage arcade drive (for teleop). */
    void arcade(float forward, float turn);

    /** Stop all motors and set brake mode. */
    void stop();

    /** Move motors with raw voltage (−127 … 127). */
    void tankVoltage(float left, float right);

    // ── Odometry / pose ─────────────────────────────────────────────────

    /** Current odometry-only pose (x, y, θ) in metres / radians. */
    Eigen::Vector3f getOdomPose() const;

    /** Backward-compatible alias for getOdomPose(). */
    Eigen::Vector3f getPose() const;

    /** Set odometry-only pose. */
    void setOdomPose(const Eigen::Vector3f& pose);

    /**
     * Synchronize IMU + odometry reference to a localization pose.
     * Ensures next odom delta is not polluted by stale heading/encoder baselines.
     */
    void syncLocalizationReference(const Eigen::Vector3f& pose);

    /**
     * Set odom baselines from current sensor values with an explicit heading.
     * Use at startup so the first delta from the chosen pose is zero.
     */
    void syncOdomBaselinesToCurrentSensors(float heading);

    /** Backward-compatible alias for setOdomPose(). */
    void setPose(const Eigen::Vector3f& pose);

    /** Heading from IMU (radians). */
    float getHeading() const;

    /** Reset IMU heading to a given value. */
    void resetHeading(float heading = 0.0f);

    /** Delta displacement since last call (for localization prediction). */
    Eigen::Vector2f getDisplacement();

    /** Forward distance in metres (from tracking wheel or drive encoders). */
    float getForwardDistance() const;

    /** Reset tracking-wheel / encoder state. */
    void resetEncoders();

    // ── Accessors ───────────────────────────────────────────────────────
    DriveSpeeds getLastSpeeds() const { return m_lastSpeeds; }

private:
    // Hardware
    pros::MotorGroup m_left;
    pros::MotorGroup m_right;
    pros::Imu        m_imu;

    // Tracking wheels (rotation sensors) — nullptr when port == 0
    std::unique_ptr<pros::Rotation> m_verticalTracking;
    std::unique_ptr<pros::Rotation> m_horizontalTracking;

    // State
    Eigen::Vector3f m_pose{0.0f, 0.0f, 0.0f};
    float m_prevForwardDist = 0.0f;
    float m_prevLateralDist = 0.0f;
    float m_prevHeading     = 0.0f;
    Eigen::Vector2f m_pendingDisplacement{0.0f, 0.0f};
    DriveSpeeds m_lastSpeeds{};

    // Internal helpers
    void updateOdometry();

    /** Raw forward distance from best available source (metres). */
    float rawForwardDistance() const;

    /** Raw lateral distance from horizontal tracking wheel (metres, 0 if disabled). */
    float rawLateralDistance() const;

    /** Align odometry previous-state caches with live sensor values. */
    void syncOdometryState();
};
