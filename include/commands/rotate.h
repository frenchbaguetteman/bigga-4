/**
 * @file rotate.h
 * Turn-in-place command using OkapiLib PID on heading error.
 */
#pragma once

#include "command/command.h"
#include "subsystems/drivetrain.h"
#include "okapi/impl/control/iterative/iterativeControllerFactory.hpp"
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
                  float tolerance = 0.03f,
                  float maxOutput = 127.0f)
        : m_drivetrain(drivetrain)
        , m_targetAngle(targetAngle)
        , m_poseSource(std::move(poseSource))
        , m_tolerance(tolerance)
        , m_maxOutput(std::fabs(maxOutput))
        , m_pid(okapi::IterativeControllerFactory::posPID(
              CONFIG::TURN_PID.kP, CONFIG::TURN_PID.kI, CONFIG::TURN_PID.kD)) {}

    void initialize() override {
        m_pid.reset();
        m_pid.setTarget(0.0);
    }

    void execute() override {
        Eigen::Vector3f pose = m_poseSource();
        float error = utils::angleDifference(m_targetAngle, pose.z());
        double output = m_pid.step(static_cast<double>(-error));

        m_drivetrain->arcade(0.0f,
            utils::clamp(static_cast<float>(output * m_maxOutput), -m_maxOutput, m_maxOutput));
    }

    void end(bool /*interrupted*/) override {
        m_drivetrain->stop();
    }

    bool isFinished() override {
        Eigen::Vector3f pose = m_poseSource();
        float error = std::fabs(utils::angleDifference(m_targetAngle, pose.z()));
        return error < m_tolerance;
    }

    std::vector<Subsystem*> getRequirements() override {
        return {m_drivetrain};
    }

private:
    Drivetrain* m_drivetrain;
    float m_targetAngle;
    std::function<Eigen::Vector3f()> m_poseSource;
    float m_tolerance;
    float m_maxOutput;
    okapi::IterativePosPIDController m_pid;
};
