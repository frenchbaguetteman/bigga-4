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

    /** Driver-control arcade with joystick curves and soft active braking. */
    void driverArcade(float forwardInput, float turnInput);

    /** Clear teleop-specific state when another command takes over. */
    void resetDriverAssistState();

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

    /** Calibrate IMU and refresh odometry baselines. */
    void calibrateImu();

    /** Reset IMU heading to a given value. */
    void resetHeading(float heading = 0.0f);

    /**
     * Consume the pending displacement delta accumulated by updateOdometry()
     * since the last call to getDisplacement() or consumePendingDisplacement().
     * Includes BOTH forward and lateral wheel contributions.
     *
     * Returns the field-frame delta (dx, dy) and clears the internal buffer.
     * Called exactly once per tick by ParticleFilter::update() after
     * CommandScheduler::run() has executed drivetrain::periodic().
     */
    Eigen::Vector2f consumePendingDisplacement();

    /**
     * Consume the forward-encoder-only displacement (no lateral wheel).
     * This is the authoritative interface for MCL prediction so that
     * MCL stays "pure" — only IMU heading + distance sensors, no lateral
     * tracking wheel leaking in through the prediction step.
     */
    Eigen::Vector2f consumePendingFwdOnlyDisplacement();

    /** Get latest pending displacement WITHOUT consuming it (debug only). */
    Eigen::Vector2f getPendingDisplacementDebug() const { return m_pendingDisplacement; }

    /** Get the last single odometry step without touching pending buffers. */
    Eigen::Vector2f getLastStepDisplacementDebug() const { return m_lastStepDisplacement; }

    /** Legacy alias; use consumePendingDisplacement() for MCL. */
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
    Eigen::Vector2f m_lastStepDisplacement{0.0f, 0.0f};
    Eigen::Vector2f m_lastStepFwdOnlyDisplacement{0.0f, 0.0f};
    Eigen::Vector2f m_pendingDisplacement{0.0f, 0.0f};
    Eigen::Vector2f m_pendingFwdOnlyDisplacement{0.0f, 0.0f};
    uint32_t m_odomGeneration = 0;
    uint32_t m_lastConsumedGeneration = 0;
    uint32_t m_lastConsumedFwdOnlyGeneration = 0;
    DriveSpeeds m_lastSpeeds{};
    float m_driverForwardCurve = CONFIG::DRIVER_FORWARD_CURVE_T;
    float m_driverTurnCurve = CONFIG::DRIVER_TURN_CURVE_T;
    float m_driverDeadband = CONFIG::DRIVER_JOYSTICK_DEADBAND;
    bool m_activeBrakeEnabled = CONFIG::DRIVER_ACTIVE_BRAKE_ENABLED;
    float m_activeBrakePower = CONFIG::DRIVER_ACTIVE_BRAKE_POWER;
    float m_activeBrakeKp = CONFIG::DRIVER_ACTIVE_BRAKE_KP;
    float m_activeBrakeStickDeadband = CONFIG::DRIVER_ACTIVE_BRAKE_STICK_DEADBAND;
    float m_activeBrakePosDeadbandDeg = CONFIG::DRIVER_ACTIVE_BRAKE_POS_DEADBAND_deg;
    float m_activeBrakeOutputDeadband = CONFIG::DRIVER_ACTIVE_BRAKE_OUTPUT_DEADBAND;
    bool m_activeBrakeWasDriving = false;
    float m_activeBrakeLeftTargetDeg = 0.0f;
    float m_activeBrakeRightTargetDeg = 0.0f;

    // Internal helpers
    void updateOdometry();
    static float joystickCurve(float input, float t, float deadzone);
    void applyActiveBrake();
    float leftMotorPositionDeg() const;
    float rightMotorPositionDeg() const;

    /** Raw forward distance from best available source (metres). */
    float rawForwardDistance() const;

    /** Raw lateral distance from horizontal tracking wheel (metres, 0 if disabled). */
    float rawLateralDistance() const;

    /** Align odometry previous-state caches with live sensor values. */
    void syncOdometryState();
};
