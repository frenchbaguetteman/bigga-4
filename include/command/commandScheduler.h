/**
 * @file commandScheduler.h
 * Singleton command scheduler.
 *
 * Drives the entire command lifecycle: scheduling, requirements checking,
 * execution, cancellation, and default-command fallback.
 */
#pragma once

#include "command/command.h"
#include "command/subsystem.h"
#include "pros/rtos.hpp"
#include <mutex>
#include <vector>
#include <algorithm>

class CommandScheduler {
public:
    // ── Core API (all static — singleton) ───────────────────────────────

    /** Advance one scheduler tick. Called from the 10 ms periodic task. */
    static void run() {
        std::scoped_lock guard(mutex());

        // 1. Execute scheduled commands; retire finished ones
        auto& cmds = commands();
        for (auto it = cmds.begin(); it != cmds.end(); ) {
            Command* cmd = *it;
            cmd->execute();
            if (cmd->isFinished()) {
                cmd->end(false);
                unregisterRequirements(cmd);
                cmd->m_scheduled.store(false);
                it = cmds.erase(it);
            } else {
                ++it;
            }
        }

        // 2. Run default commands for idle subsystems
        for (Subsystem* sub : subsystems()) {
            if (sub->m_currentCommand.load() == nullptr && sub->m_defaultCommand != nullptr) {
                schedule(sub->m_defaultCommand);
            }
        }

        // 3. Subsystem periodic hooks
        for (Subsystem* sub : subsystems()) {
            sub->periodic();
        }
    }

    /** Schedule a command for execution. */
    static void schedule(Command* cmd) {
        std::scoped_lock guard(mutex());
        if (!cmd || cmd->m_scheduled.load()) return;

        // Cancel any currently-running commands that conflict on requirements
        for (Subsystem* req : cmd->getRequirements()) {
            Command* current = req->m_currentCommand.load();
            if (current) {
                cancel(current);
            }
        }

        // Register requirement ownership
        for (Subsystem* req : cmd->getRequirements()) {
            req->m_currentCommand.store(cmd);
        }

        cmd->m_scheduled.store(true);
        cmd->initialize();
        commands().push_back(cmd);
    }

    /** Cancel a running command (calls end(true)). */
    static void cancel(Command* cmd) {
        std::scoped_lock guard(mutex());
        if (!cmd || !cmd->m_scheduled.load()) return;

        cmd->end(true);
        unregisterRequirements(cmd);
        cmd->m_scheduled.store(false);

        auto& cmds = commands();
        cmds.erase(std::remove(cmds.begin(), cmds.end(), cmd), cmds.end());
    }

    /** Cancel every running command. */
    static void cancelAll() {
        std::scoped_lock guard(mutex());
        auto& cmds = commands();
        for (Command* cmd : cmds) {
            cmd->end(true);
            unregisterRequirements(cmd);
            cmd->m_scheduled.store(false);
        }
        cmds.clear();
    }

    /** Register a subsystem so it receives periodic() calls. */
    static void registerSubsystem(Subsystem* sub) {
        std::scoped_lock guard(mutex());
        auto& subs = subsystems();
        if (std::find(subs.begin(), subs.end(), sub) == subs.end()) {
            subs.push_back(sub);
        }
    }

    /** Full scheduler reset: cancel every running command. */
    static void reset() {
        std::scoped_lock guard(mutex());
        cancelAll();
    }

    /** True if the given command is currently scheduled. */
    static bool isScheduled(const Command* cmd) {
        std::scoped_lock guard(mutex());
        auto& cmds = commands();
        return std::find(cmds.begin(), cmds.end(), cmd) != cmds.end();
    }

private:
    static void unregisterRequirements(Command* cmd) {
        for (Subsystem* req : cmd->getRequirements()) {
            if (req->m_currentCommand.load() == cmd) {
                req->m_currentCommand.store(nullptr);
            }
        }
    }

    // Meyer's singletons so we avoid static-init-order issues
    static std::vector<Command*>& commands() {
        static std::vector<Command*> v;
        return v;
    }
    static std::vector<Subsystem*>& subsystems() {
        static std::vector<Subsystem*> v;
        return v;
    }

    static pros::RecursiveMutex& mutex() {
        static pros::RecursiveMutex* m = nullptr;
        if (!m) {
            m = new pros::RecursiveMutex();
        }
        return *m;
    }
};

// ── Inline implementations that depend on CommandScheduler ──────────────────

inline void Command::cancel()        { CommandScheduler::cancel(this); }
inline bool Command::isScheduled() const { return m_scheduled.load(); }
inline void Subsystem::registerThis() { CommandScheduler::registerSubsystem(this); }
