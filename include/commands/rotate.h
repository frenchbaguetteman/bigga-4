/**
 * @file rotate.h
 * Turn-in-place command using OkapiLib PID on heading error.
 */
#pragma once

#include "command/command.h"
#include "config.h"
#include "feedback/pid.h"
#include "subsystems/drivetrain.h"
#include "utils/localization_math.h"
#include "utils/utils.h"
#include <cmath>
#include <functional>
#include <vector>

class RotateCommand : public Command {
public:
    static constexpr float kRadiansToDegrees = 180.0f / static_cast<float>(M_PI);

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
        , m_pid(CONFIG::TURN_PID) {}

    void initialize() override {
        m_pid.reset();
    }

    void execute() override {
        const Eigen::Vector3f pose = samplePose();
        const float errorDegrees =
            utils::angleDifference(m_targetAngle, pose.z()) * kRadiansToDegrees;
        const float output = m_pid.calculate(0.0f, errorDegrees);

        // Okapi yaw is clockwise-positive, while our internal heading error is
        // counterclockwise-positive.
        m_drivetrain->arcade(0.0f,
            -utils::clamp(output, -m_maxOutput, m_maxOutput));
    }

    void end(bool /*interrupted*/) override {
        m_drivetrain->stop();
    }

    bool isFinished() override {
        const Eigen::Vector3f pose = samplePose();
        float error = std::fabs(utils::angleDifference(m_targetAngle, pose.z()));
        return error < m_tolerance;
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
    float m_targetAngle;
    std::function<Eigen::Vector3f()> m_poseSource;
    float m_tolerance;
    float m_maxOutput;
    PID m_pid;
};
