/**
 * @file driveMove.h
 * PID drive-to-point command using OkapiLib PID controllers.
 */
#pragma once

#include "command/command.h"
#include "subsystems/drivetrain.h"
#include "okapi/impl/control/iterative/iterativeControllerFactory.hpp"
#include "config.h"
#include "utils/utils.h"
#include "Eigen/Core"
#include <cmath>
#include <vector>
#include <functional>

class DriveMoveCommand : public Command {
public:
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
        , m_distPid(okapi::IterativeControllerFactory::posPID(
              CONFIG::DISTANCE_PID.kP, CONFIG::DISTANCE_PID.kI, CONFIG::DISTANCE_PID.kD))
        , m_turnPid(okapi::IterativeControllerFactory::posPID(
              CONFIG::TURN_PID.kP, CONFIG::TURN_PID.kI, CONFIG::TURN_PID.kD)) {}

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
        , m_distPid(okapi::IterativeControllerFactory::posPID(
              CONFIG::DISTANCE_PID.kP, CONFIG::DISTANCE_PID.kI, CONFIG::DISTANCE_PID.kD))
        , m_turnPid(okapi::IterativeControllerFactory::posPID(
              CONFIG::TURN_PID.kP, CONFIG::TURN_PID.kI, CONFIG::TURN_PID.kD)) {}

    void initialize() override {
        m_distPid.reset();
        m_turnPid.reset();
        m_distPid.setTarget(0.0);
        m_turnPid.setTarget(0.0);
    }

    void execute() override {
        Eigen::Vector3f pose = m_poseSource();
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

        // OkapiLib PID: target is 0, pass -error as reading so output drives toward 0
        double driveOut = m_distPid.step(static_cast<double>(-driveError));
        double turnOut  = m_turnPid.step(
            static_cast<double>(-utils::angleDifference(targetAngle, pose.z())));

        // OkapiLib PID outputs [-1, 1], scale to [-maxOutput, maxOutput]
        m_drivetrain->arcade(
            utils::clamp(static_cast<float>(driveOut * m_maxOutput), -m_maxOutput, m_maxOutput),
            utils::clamp(static_cast<float>(turnOut * m_maxOutput), -m_maxOutput, m_maxOutput));
    }

    void end(bool /*interrupted*/) override {
        m_drivetrain->stop();
    }

    bool isFinished() override {
        Eigen::Vector3f pose = m_poseSource();
        const Eigen::Vector2f error(m_target.x() - pose.x(), m_target.y() - pose.y());
        return error.norm() < m_tolerance;
    }

    std::vector<Subsystem*> getRequirements() override {
        return {m_drivetrain};
    }

private:
    Drivetrain* m_drivetrain;
    Eigen::Vector2f m_target;
    std::function<Eigen::Vector3f()> m_poseSource;
    float m_tolerance;
    MotionMode m_motionMode;
    float m_headingTarget;
    int m_driveSign;
    float m_maxOutput;
    okapi::IterativePosPIDController m_distPid;
    okapi::IterativePosPIDController m_turnPid;
};
