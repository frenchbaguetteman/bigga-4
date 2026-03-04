/**
 * @file driveMove.h
 * Simple PID drive-to-point command.
 */
#pragma once

#include "command/command.h"
#include "subsystems/drivetrain.h"
#include "feedback/pid.h"
#include "config.h"
#include "utils/utils.h"
#include "Eigen/Core"
#include <cmath>
#include <vector>
#include <functional>

class DriveMoveCommand : public Command {
public:
    /**
     * @param drivetrain   robot drivetrain
     * @param target       target (x, y) in metres
     * @param poseSource   returns current (x, y, θ)
     * @param tolerance    finish threshold (metres)
     */
    DriveMoveCommand(Drivetrain* drivetrain,
                     Eigen::Vector2f target,
                     std::function<Eigen::Vector3f()> poseSource,
                     float tolerance = 0.02f)
        : m_drivetrain(drivetrain)
        , m_target(target)
        , m_poseSource(std::move(poseSource))
        , m_tolerance(tolerance)
        , m_distPid(CONFIG::DISTANCE_PID, tolerance)
        , m_turnPid(CONFIG::TURN_PID, 0.05f) {}

    void initialize() override {
        m_distPid.reset();
        m_turnPid.reset();
    }

    void execute() override {
        Eigen::Vector3f pose = m_poseSource();
        float dx = m_target.x() - pose.x();
        float dy = m_target.y() - pose.y();
        float dist = std::sqrt(dx * dx + dy * dy);
        float targetAngle = std::atan2(dy, dx);

        float driveOut = m_distPid.calculate(0.0f, dist);
        float turnOut  = m_turnPid.calculate(0.0f,
            utils::angleDifference(targetAngle, pose.z()));

        m_drivetrain->arcade(
            utils::clamp(driveOut, -127.0f, 127.0f),
            utils::clamp(turnOut, -127.0f, 127.0f));
    }

    void end(bool /*interrupted*/) override {
        m_drivetrain->stop();
    }

    bool isFinished() override {
        return m_distPid.atSetpoint();
    }

    std::vector<Subsystem*> getRequirements() override {
        return {m_drivetrain};
    }

private:
    Drivetrain* m_drivetrain;
    Eigen::Vector2f m_target;
    std::function<Eigen::Vector3f()> m_poseSource;
    float m_tolerance;
    PID m_distPid;
    PID m_turnPid;
};
