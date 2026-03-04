/**
 * @file waitCommand.h
 * A command that simply waits for a specified duration.
 */
#pragma once

#include "command/command.h"
#include "pros/rtos.hpp"

class WaitCommand : public Command {
public:
    /** @param seconds  how long to wait */
    explicit WaitCommand(float seconds) : m_duration(seconds) {}

    void initialize() override { m_startTime = pros::millis(); }
    void execute() override {}
    bool isFinished() override {
        return (pros::millis() - m_startTime) / 1000.0f >= m_duration;
    }

private:
    float    m_duration;
    uint32_t m_startTime = 0;
};
