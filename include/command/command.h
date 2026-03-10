/**
 * @file command.h
 * Base Command class — the unit of robot behaviour.
 *
 * Every command declares which subsystems it requires, and provides
 * initialize / execute / end / isFinished hooks that the
 * CommandScheduler drives at ~100 Hz.
 */
#pragma once

#include <atomic>
#include <vector>
#include <functional>
#include <memory>

// Forward declarations
class Subsystem;
class CommandScheduler;

class Command {
public:
    virtual ~Command() = default;

    /** Called once when the command is first scheduled. */
    virtual void initialize() {}

    /** Called every scheduler tick while the command is running. */
    virtual void execute() {}

    /**
     * Called when the command ends.
     * @param interrupted  true if the command was cancelled rather than
     *                     finishing on its own.
     */
    virtual void end(bool interrupted) {}

    /** Return true when the command has completed its objective. */
    virtual bool isFinished() { return false; }

    /** The subsystems this command uses (prevents conflicts). */
    virtual std::vector<Subsystem*> getRequirements() { return {}; }

    // ── Builder helpers (return heap-allocated groups) ───────────────────

    /** Run this command, then the given command. */
    Command* andThen(Command* next);

    /** Run this command in parallel with the given command. */
    Command* alongWith(Command* other);

    /** Run in parallel; end when the first command finishes. */
    Command* raceWith(Command* other);

    /** Wrap with a timeout (seconds). */
    Command* withTimeout(float seconds);

    /** Wrap with an external finish condition. */
    Command* until(std::function<bool()> condition);

    // ── Scheduler integration ───────────────────────────────────────────

    void cancel();
    bool isScheduled() const;

protected:
    friend class CommandScheduler;
    std::atomic<bool> m_scheduled{false};
};
