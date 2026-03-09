/**
 * @file drivetrain.h
 * 69580A — OkapiLib-based drivetrain subsystem.
 *
 * Wraps an OkapiLib OdomChassisController built via ChassisControllerBuilder.
 * Provides the same Subsystem interface for the command framework, plus
 * arcade drive for opcontrol and odometry access for localization.
 */
#pragma once

#include "command/subsystem.h"
#include "config.h"
#include "okapi/api.hpp"
#include "Eigen/Core"
#include "pros/imu.hpp"
#include <memory>

struct DriveSpeeds {
    float linear  = 0.0f;  // m/s
    float angular = 0.0f;  // rad/s
};

class Drivetrain : public Subsystem {
public:
    Drivetrain();

    /* ── Subsystem hooks ─────────────────────────────────────────────── */
    void periodic() override;

    /* ── Chassis access ──────────────────────────────────────────────── */
    std::shared_ptr<okapi::OdomChassisController> getOdomChassis() const {
        return m_chassis;
    }
    std::shared_ptr<okapi::ChassisModel> getModel() const;

    /* ── Motor control ───────────────────────────────────────────────── */
    void arcade(float forward, float turn);
    void stop();
    void driverArcade(float forwardStick, float turnStick);
    void resetDriverAssistState();

    /** Set drive speeds using feedforward (for RAMSETE / LTV controllers). */
    void setDriveSpeeds(DriveSpeeds speeds);

    /* ── Heading / IMU ───────────────────────────────────────────────── */
    float getHeading() const;
    void calibrateImu();
    void resetHeading(float headingRad);

    /* ── Odometry ────────────────────────────────────────────────────── */
    Eigen::Vector3f getOdomPose() const;
    float rawForwardDistance() const;
    void resetEncoders();

    /* ── Localization helpers ─────────────────────────────────────────── */
    void syncLocalizationReference(const Eigen::Vector3f& pose);
    Eigen::Vector2f consumePendingDisplacement();
    Eigen::Vector2f consumePendingFwdOnlyDisplacement();
    Eigen::Vector2f getLastStepDisplacementDebug() const;

    /* ── OkapiLib motion (for use in commands) ───────────────────────── */
    void moveDistance(okapi::QLength distance);
    void turnAngle(okapi::QAngle angle);
    void waitUntilSettled();
    void setMaxVelocity(int velocity);

    /** Last commanded speeds (for telemetry). */
    DriveSpeeds getLastSpeeds() const { return m_lastSpeeds; }

private:
    std::shared_ptr<okapi::OdomChassisController> m_chassis;
    pros::Imu m_imu;

    /* Localization tracking */
    Eigen::Vector3f m_prevOdomPose{0.0f, 0.0f, 0.0f};
    Eigen::Vector2f m_pendingDisplacement{0.0f, 0.0f};
    Eigen::Vector2f m_pendingFwdOnly{0.0f, 0.0f};
    Eigen::Vector2f m_lastStepDisplacement{0.0f, 0.0f};

    /* Driver-assist state */
    float m_activeBrakeTargetDeg = 0.0f;
    bool  m_wasStopped = true;
    DriveSpeeds m_lastSpeeds{};

    static float joystickCurve(float input, float t);
};
