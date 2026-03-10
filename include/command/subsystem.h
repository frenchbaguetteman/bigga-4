/**
 * @file subsystem.h
 * Base Subsystem class.
 *
 * A subsystem represents a hardware component (e.g. drivetrain, intake).
 * The CommandScheduler calls periodic() every tick and ensures that at most
 * one command requiring a given subsystem runs at a time.
 */
#pragma once

#include <atomic>
#include "command/command.h"
class Subsystem {
public:
    virtual ~Subsystem() = default;

    /** Called every scheduler tick, regardless of which command is active. */
    virtual void periodic() {}

    /** Set the command that runs when no other command requires this subsystem. */
    void setDefaultCommand(Command* cmd) { m_defaultCommand = cmd; }
    Command* getDefaultCommand() const { return m_defaultCommand; }

    /** Which command currently owns this subsystem (nullptr if none). */
    Command* getCurrentCommand() const { return m_currentCommand.load(); }

    /** Register this subsystem with the global CommandScheduler. */
    void registerThis();

private:
    friend class CommandScheduler;
    Command* m_defaultCommand  = nullptr;
    std::atomic<Command*> m_currentCommand{nullptr};
};
