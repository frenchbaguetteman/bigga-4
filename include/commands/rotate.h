/**
 * @file rotate.h
 * Turn-in-place command using PID on heading error.
 */
#pragma once

#include "command/command.h"
#include "subsystems/drivetrain.h"
#include "feedback/pid.h"
#include "config.h"
#include "utils/utils.h"
#include <cmath>
#include <vector>
#include <functional>

class RotateCommand : public Command {
public:
    /**
     * @param drivetrain   robot drivetrain
     * @param targetAngle  target heading (radians)
     * @param poseSource   returns current (x, y, θ)
     * @param tolerance    finish threshold (radians)
     */
    RotateCommand(Drivetrain* drivetrain,
                  float targetAngle,
                  std::function<Eigen::Vector3f()> poseSource,
                  float tolerance = 0.03f)
        : m_drivetrain(drivetrain)
        , m_targetAngle(targetAngle)
        , m_poseSource(std::move(poseSource))
        , m_pid(CONFIG::TURN_PID, tolerance) {}

    void initialize() override {
        m_pid.reset();
    }

    void execute() override {
        Eigen::Vector3f pose = m_poseSource();
        float error = utils::angleDifference(m_targetAngle, pose.z());
        float output = m_pid.calculate(0.0f, error);

        m_drivetrain->arcade(0.0f, utils::clamp(output, -127.0f, 127.0f));
    }

    void end(bool /*interrupted*/) override {
        m_drivetrain->stop();
    }

    bool isFinished() override {
        return m_pid.atSetpoint();
    }

    std::vector<Subsystem*> getRequirements() override {
        return {m_drivetrain};
    }

private:
    Drivetrain* m_drivetrain;
    float m_targetAngle;
    std::function<Eigen::Vector3f()> m_poseSource;
    PID m_pid;
};
