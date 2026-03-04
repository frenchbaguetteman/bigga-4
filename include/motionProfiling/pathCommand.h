/**
 * @file pathCommand.h
 * Abstract base for commands that follow a MotionProfile.
 */
#pragma once

#include "command/command.h"
#include "motionProfiling/motionProfile.h"
#include "subsystems/drivetrain.h"
#include "pros/rtos.hpp"
#include <vector>

class PathCommand : public Command {
public:
    PathCommand(Drivetrain* drivetrain, MotionProfile profile)
        : m_drivetrain(drivetrain), m_profile(std::move(profile)) {}

    void initialize() override {
        m_startTime = pros::millis();
    }

    bool isFinished() override {
        return m_profile.isFinished(elapsedSeconds());
    }

    void end(bool interrupted) override {
        m_drivetrain->stop();
    }

    std::vector<Subsystem*> getRequirements() override {
        return {m_drivetrain};
    }

protected:
    float elapsedSeconds() const {
        return (pros::millis() - m_startTime) / 1000.0f;
    }

    Drivetrain*   m_drivetrain;
    MotionProfile m_profile;
    uint32_t      m_startTime = 0;
};
