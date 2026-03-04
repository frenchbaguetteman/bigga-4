/**
 * @file intakeCommand.h
 * Commands for the intake subsystem.
 */
#pragma once

#include "command/command.h"
#include "subsystems/intakes.h"
#include <vector>

// ── Spin intake at a given voltage until cancelled ──────────────────────────

class IntakeSpinCommand : public Command {
public:
    IntakeSpinCommand(Intakes* intakes, int voltage)
        : m_intakes(intakes), m_voltage(voltage) {}

    void initialize() override { m_intakes->spin(m_voltage); }
    void execute() override {}
    void end(bool /*interrupted*/) override { m_intakes->stop(); }
    bool isFinished() override { return false; }   // runs until cancelled
    std::vector<Subsystem*> getRequirements() override { return {m_intakes}; }

private:
    Intakes* m_intakes;
    int m_voltage;
};

// ── Spin intake for a fixed duration ────────────────────────────────────────

class IntakeTimedCommand : public Command {
public:
    IntakeTimedCommand(Intakes* intakes, int voltage, float seconds)
        : m_intakes(intakes), m_voltage(voltage), m_duration(seconds) {}

    void initialize() override {
        m_start = pros::millis();
        m_intakes->spin(m_voltage);
    }
    void execute() override {}
    void end(bool /*interrupted*/) override { m_intakes->stop(); }
    bool isFinished() override {
        return (pros::millis() - m_start) / 1000.0f >= m_duration;
    }
    std::vector<Subsystem*> getRequirements() override { return {m_intakes}; }

private:
    Intakes* m_intakes;
    int m_voltage;
    float m_duration;
    uint32_t m_start = 0;
};

// ── Stop intake instantly ───────────────────────────────────────────────────

class IntakeStopCommand : public Command {
public:
    explicit IntakeStopCommand(Intakes* intakes) : m_intakes(intakes) {}

    void initialize() override { m_intakes->stop(); }
    bool isFinished() override { return true; }
    std::vector<Subsystem*> getRequirements() override { return {m_intakes}; }

private:
    Intakes* m_intakes;
};
