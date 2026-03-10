/**
 * @file driveMove.h
 * PID drive-to-point command using OkapiLib PID controllers.
 */
#pragma once

#include "command/command.h"
#include "config.h"
#include "feedback/pid.h"
#include "subsystems/drivetrain.h"
#include "utils/localization_math.h"
#include "utils/utils.h"
#include "Eigen/Core"
#include <cmath>
#include <functional>
#include <vector>

class DriveMoveCommand : public Command {
public:
    static constexpr float kMetersToInches = 39.3701f;
    static constexpr float kRadiansToDegrees = 180.0f / static_cast<float>(M_PI);
    static constexpr float kPointTurnEnterDeg = 25.0f;
    static constexpr float kPointTurnExitDeg = 8.0f;

    enum class MotionMode {
        PointToPoint,
        HoldHeading,
    };

    /**
     * @param drivetrain   robot drivetrain
     * @param target       target (x, y) in metres
     * @param poseSource   returns current (x, y, θ)
     * @param tolerance    finish threshold (metres)
     */
    DriveMoveCommand(Drivetrain* drivetrain,
                     Eigen::Vector2f target,
                     std::function<Eigen::Vector3f()> poseSource,
                     float tolerance = 0.02f,
                     float maxOutput = 127.0f)
        : m_drivetrain(drivetrain)
        , m_target(target)
        , m_poseSource(std::move(poseSource))
        , m_tolerance(tolerance)
        , m_motionMode(MotionMode::PointToPoint)
        , m_headingTarget(0.0f)
        , m_driveSign(1)
        , m_maxOutput(std::fabs(maxOutput))
        , m_distPid(CONFIG::DISTANCE_PID)
        , m_turnPid(CONFIG::TURN_PID) {}

    DriveMoveCommand(Drivetrain* drivetrain,
                     Eigen::Vector2f target,
                     std::function<Eigen::Vector3f()> poseSource,
                     MotionMode motionMode,
                     float headingTarget,
                     int driveSign,
                     float tolerance = 0.02f,
                     float maxOutput = 127.0f)
        : m_drivetrain(drivetrain)
        , m_target(target)
        , m_poseSource(std::move(poseSource))
        , m_tolerance(tolerance)
        , m_motionMode(motionMode)
        , m_headingTarget(headingTarget)
        , m_driveSign(driveSign >= 0 ? 1 : -1)
        , m_maxOutput(std::fabs(maxOutput))
        , m_distPid(CONFIG::DISTANCE_PID)
        , m_turnPid(CONFIG::TURN_PID) {}

    void initialize() override {
        m_distPid.reset();
        m_turnPid.reset();
        m_aligningToTarget = false;
    }

    void execute() override {
        const Eigen::Vector3f pose = samplePose();
        const Eigen::Vector2f error(m_target.x() - pose.x(), m_target.y() - pose.y());

        float driveError = 0.0f;
        float targetAngle = m_headingTarget;

        if (m_motionMode == MotionMode::HoldHeading) {
            const Eigen::Vector2f axis(std::cos(m_headingTarget), std::sin(m_headingTarget));
            driveError = error.dot(axis);
        } else {
            const float dist = error.norm();
            targetAngle = std::atan2(error.y(), error.x());
            if (m_driveSign < 0) {
                targetAngle = utils::angleWrap(targetAngle + static_cast<float>(M_PI));
            }
            driveError = dist * static_cast<float>(m_driveSign);
        }

        const float driveErrorInches = driveError * kMetersToInches;
        const float turnErrorRadians = utils::angleDifference(targetAngle, pose.z());
        const float turnErrorDegrees = turnErrorRadians * kRadiansToDegrees;

        float driveOut = m_distPid.calculate(0.0f, driveErrorInches);
        const float turnOut = m_turnPid.calculate(0.0f, turnErrorDegrees);
        const float absTurnErrorDegrees = std::fabs(turnErrorDegrees);
        const float alignmentScale = std::max(0.0f, std::cos(std::fabs(turnErrorRadians)));

        if (m_motionMode == MotionMode::PointToPoint) {
            if (absTurnErrorDegrees >= kPointTurnEnterDeg) {
                m_aligningToTarget = true;
            } else if (absTurnErrorDegrees <= kPointTurnExitDeg) {
                m_aligningToTarget = false;
            }

            if (m_aligningToTarget) {
                driveOut = 0.0f;
            }
        }

        driveOut *= alignmentScale;

        // Okapi yaw is clockwise-positive, while our internal heading error is
        // counterclockwise-positive.
        m_drivetrain->arcade(
            utils::clamp(driveOut, -m_maxOutput, m_maxOutput),
            -utils::clamp(turnOut, -m_maxOutput, m_maxOutput));
    }

    void end(bool /*interrupted*/) override {
        m_drivetrain->stop();
    }

    bool isFinished() override {
        const Eigen::Vector3f pose = samplePose();
        const Eigen::Vector2f error(m_target.x() - pose.x(), m_target.y() - pose.y());
        return error.norm() < m_tolerance;
    }

    std::vector<Subsystem*> getRequirements() override {
        return {m_drivetrain};
    }

private:
    Eigen::Vector3f samplePose() const {
        const Eigen::Vector3f rawPose =
            m_poseSource ? m_poseSource() : Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        const Eigen::Vector3f odomPose =
            m_drivetrain ? m_drivetrain->getOdomPose() : Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        return LocMath::finitePoseOr(rawPose, odomPose);
    }

    Drivetrain* m_drivetrain;
    Eigen::Vector2f m_target;
    std::function<Eigen::Vector3f()> m_poseSource;
    float m_tolerance;
    MotionMode m_motionMode;
    float m_headingTarget;
    int m_driveSign;
    float m_maxOutput;
    bool m_aligningToTarget = false;
    PID m_distPid;
    PID m_turnPid;
};
