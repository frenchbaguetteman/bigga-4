/**
 * @file rotate.h
 * Turn-in-place command using the simpler radian-based PID loop from the
 * working non-Okapi branch, adapted only for Okapi yaw sign.
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
        , m_maxOutput(std::clamp(std::fabs(maxOutput) / 127.0f, 0.0f, 1.0f))
        , m_tolerance(tolerance)
        , m_pid(CONFIG::TURN_PID, tolerance) {}

    void initialize() override {
        m_pid.reset();
    }

    void execute() override {
        const float heading = sampleHeading();
        const float error = utils::angleDifference(m_targetAngle, heading);
        const float output = m_pid.calculate(0.0f, error);
        const float normalizedTurn =
            utils::clamp(output, -m_maxOutput, m_maxOutput);

        // Okapi yaw is clockwise-positive, while our internal heading error is
        // counterclockwise-positive.
        m_drivetrain->arcade(0.0f,
            -normalizedTurn * 127.0f);
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
    Eigen::Vector3f samplePose() const {
        const Eigen::Vector3f rawPose =
            m_poseSource ? m_poseSource() : Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        const Eigen::Vector3f odomPose =
            m_drivetrain ? m_drivetrain->getOdomPose() : Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        return LocMath::finitePoseOr(rawPose, odomPose);
    }

    float sampleHeading() const {
        const float poseHeading = samplePose().z();
        if (LocMath::isFinite(poseHeading)) {
            return poseHeading;
        }
        if (m_drivetrain) {
            const float imuHeading = m_drivetrain->getHeading();
            if (LocMath::isFinite(imuHeading)) {
                return imuHeading;
            }
        }
        return 0.0f;
    }

    Drivetrain* m_drivetrain;
    float m_targetAngle;
    std::function<Eigen::Vector3f()> m_poseSource;
    float m_maxOutput;
    float m_tolerance;
    PID m_pid;
};
