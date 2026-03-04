/**
 * @file ltvUnicycleController.h
 * LTV (Linear Time-Varying) unicycle path-tracking controller.
 *
 * Uses a linearised unicycle model and an LQR-style feedback gain that
 * varies with the desired velocity.  At runtime the gain is either
 * pre-computed or approximated analytically.
 *
 * State error:  e = [x_err, y_err, θ_err]ᵀ  (robot frame)
 * Control:      u = [v, ω]ᵀ
 *
 * The LQR cost matrix Q is configured via CONFIG::DEFAULT_DT_COST_Q.
 * R is identity (unit cost on control effort).
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
#include <functional>

class LtvUnicycleCommand : public PathCommand {
public:
    LtvUnicycleCommand(Drivetrain* drivetrain,
                       MotionProfile profile,
                       std::function<Eigen::Vector3f()> poseSource,
                       Eigen::Vector3f qWeights = CONFIG::DEFAULT_DT_COST_Q)
        : PathCommand(drivetrain, std::move(profile))
        , m_poseSource(std::move(poseSource))
        , m_q(qWeights) {}

    void initialize() override {
        PathCommand::initialize();
    }

    void execute() override {
        float t = elapsedSeconds();
        ProfileState desired = m_profile.sample(t);

        Eigen::Vector3f currentPose = m_poseSource();
        Eigen::Vector3f desiredPose = desired.pose;

        float vd = desired.linearVelocity;
        float wd = desired.angularVelocity;

        // Error in robot frame
        Eigen::Vector2f xyError =
            Eigen::Rotation2Df(-currentPose.z()) *
            (desiredPose - currentPose).head<2>();

        float eTheta = utils::angleDifference(desiredPose.z(), currentPose.z());

        // ── Approximate LTV gain computation ────────────────────────────
        // For a unicycle linearised about (v_d, ω_d), the continuous-time
        // system matrix is:
        //   A = [[0, 0, 0],
        //        [0, 0, v_d],
        //        [0, 0, 0]]
        //   B = [[1, 0],
        //        [0, 0],
        //        [0, 1]]
        //
        // Analytic LQR solution for this structure gives decoupled gains:
        //   k1 = √(q1)         → longitudinal gain
        //   k3 = √(q3)         → heading gain
        //   k2 = √(q2 * |v_d|) → lateral gain (velocity-varying)
        float k1 = std::sqrt(m_q.x());
        float k3 = std::sqrt(m_q.z());
        float k2 = std::sqrt(m_q.y() * std::fabs(vd) + 1e-6f);

        // Control law
        float v = vd * std::cos(eTheta) + k1 * xyError.x();
        float omega = wd + k3 * eTheta + k2 * utils::sinc(eTheta) * xyError.y();

        m_drivetrain->setDriveSpeeds({v, omega});

        Telemetry::send(t, currentPose, desiredPose);
    }

private:
    std::function<Eigen::Vector3f()> m_poseSource;
    Eigen::Vector3f m_q;   // diagonal of Q cost matrix
};
