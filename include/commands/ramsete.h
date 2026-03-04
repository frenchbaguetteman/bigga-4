/**
 * @file ramsete.h
 * RAMSETE nonlinear path-following controller.
 *
 * Follows a MotionProfile by computing commanded chassis velocities from
 * pose error in the robot frame and feedforward velocities from the profile.
 *
 * Canonical RAMSETE equations:
 *     k  = 2ζ √(ω_d² + β v_d²)
 *     v  = v_d cos(e_θ) + k · e_x
 *     ω  = ω_d + k · e_θ + β · v_d · sinc(e_θ) · e_y
 *
 * Gains ζ (zeta, 0 < ζ < 1) and β (beta, > 0) are read from CONFIG.
 */
#pragma once

#include "motionProfiling/pathCommand.h"
#include "motionProfiling/motionProfile.h"
#include "subsystems/drivetrain.h"
#include "telemetry/telemetry.h"
#include "config.h"
#include "utils/utils.h"
#include "Eigen/Dense"
#include <cmath>
#include <vector>
#include <functional>

class RamseteCommand : public PathCommand {
public:
    /**
     * @param drivetrain  robot drivetrain subsystem
     * @param profile     time-parameterised trajectory
     * @param poseSource  returns current (x,y,θ) — e.g. from localization
     * @param zeta        damping coefficient (default CONFIG)
     * @param beta        aggressiveness     (default CONFIG)
     */
    RamseteCommand(Drivetrain* drivetrain,
                   MotionProfile profile,
                   std::function<Eigen::Vector3f()> poseSource,
                   float zeta = CONFIG::RAMSETE_ZETA,
                   float beta = CONFIG::RAMSETE_BETA)
        : PathCommand(drivetrain, std::move(profile))
        , m_poseSource(std::move(poseSource))
        , m_zeta(zeta)
        , m_beta(beta) {}

    void initialize() override {
        PathCommand::initialize();
    }

    void execute() override {
        float t = elapsedSeconds();
        ProfileState desired = m_profile.sample(t);

        // Current state
        Eigen::Vector3f currentPose = m_poseSource();
        Eigen::Vector3f desiredPose = desired.pose;

        float desiredVelocity        = desired.linearVelocity;   // v_d
        float desiredAngularVelocity = desired.angularVelocity;  // ω_d

        // ── Error in robot frame ────────────────────────────────────────
        // Rotate (desired - current) by -θ_current
        Eigen::Vector2f xyError =
            Eigen::Rotation2Df(-currentPose.z()) *
            (desiredPose - currentPose).head<2>();

        float errorAngle = utils::angleDifference(
            desiredPose.z(), currentPose.z());

        // ── Gain k ──────────────────────────────────────────────────────
        float k = 2.0f * m_zeta *
            std::sqrt(desiredAngularVelocity * desiredAngularVelocity
                      + m_beta * desiredVelocity * desiredVelocity);

        // ── Commanded velocities ────────────────────────────────────────
        float v = desiredVelocity * std::cos(errorAngle)
                + k * xyError.x();

        float omega = desiredAngularVelocity
                    + k * errorAngle
                    + m_beta * desiredVelocity
                      * utils::sinc(errorAngle) * xyError.y();

        // ── Send to drivetrain ──────────────────────────────────────────
        m_drivetrain->setDriveSpeeds({v, omega});
        m_lastSpeeds = {v, omega};

        // ── Telemetry ───────────────────────────────────────────────────
        Telemetry::send(t, currentPose, desiredPose);
    }

    void end(bool interrupted) override {
        PathCommand::end(interrupted);
    }

    DriveSpeeds getLastSpeeds() const { return m_lastSpeeds; }

private:
    std::function<Eigen::Vector3f()> m_poseSource;
    float m_zeta;
    float m_beta;
    DriveSpeeds m_lastSpeeds{};
};
