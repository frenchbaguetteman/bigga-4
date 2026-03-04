/**
 * @file liftCommand.h
 * Commands for the lift subsystem.
 */
#pragma once

#include "command/command.h"
#include "subsystems/lift.h"
#include "pros/rtos.hpp"
#include <vector>

// ── Move lift to a position and hold ────────────────────────────────────────

class LiftMoveCommand : public Command {
public:
    LiftMoveCommand(Lift* lift, float targetDegrees)
        : m_lift(lift), m_target(targetDegrees) {}

    void initialize() override { m_lift->moveTo(m_target); }
    void execute() override {}   // PID runs in Lift::periodic()
    void end(bool interrupted) override {
        if (interrupted) m_lift->stop();
    }
    bool isFinished() override { return m_lift->atTarget(); }
    std::vector<Subsystem*> getRequirements() override { return {m_lift}; }

private:
    Lift* m_lift;
    float m_target;
};

// ── Manual voltage control (hold while active) ─────────────────────────────

class LiftManualCommand : public Command {
public:
    LiftManualCommand(Lift* lift, int voltage)
        : m_lift(lift), m_voltage(voltage) {}

    void initialize() override { m_lift->moveVoltage(m_voltage); }
    void execute() override {}
    void end(bool /*interrupted*/) override { m_lift->stop(); }
    bool isFinished() override { return false; }
    std::vector<Subsystem*> getRequirements() override { return {m_lift}; }

private:
    Lift* m_lift;
    int m_voltage;
};
