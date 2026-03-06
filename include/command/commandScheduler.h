/**
 * @file commandScheduler.h
 * Singleton command scheduler.
 *
 * Drives the entire command lifecycle: scheduling, requirements checking,
 * execution, cancellation, default-command fallback, and trigger polling.
 */
#pragma once

#include "command/command.h"
#include "command/subsystem.h"
#include "command/trigger.h"
#include <vector>
#include <algorithm>
#include <map>

class CommandScheduler {
public:
    // ── Core API (all static — singleton) ───────────────────────────────

    /** Advance one scheduler tick. Called from the 10 ms periodic task. */
    static void run() {
        // 1. Poll triggers
        for (auto& trig : triggers()) {
            trig.poll();
        }

        // 2. Execute scheduled commands; retire finished ones
        auto& cmds = commands();
        for (auto it = cmds.begin(); it != cmds.end(); ) {
            Command* cmd = *it;
            cmd->execute();
            if (cmd->isFinished()) {
                cmd->end(false);
                unregisterRequirements(cmd);
                cmd->m_scheduled = false;
                it = cmds.erase(it);
            } else {
                ++it;
            }
        }

        // 3. Run default commands for idle subsystems
        for (Subsystem* sub : subsystems()) {
            if (sub->m_currentCommand == nullptr && sub->m_defaultCommand != nullptr) {
                schedule(sub->m_defaultCommand);
            }
        }

        // 4. Subsystem periodic hooks
        for (Subsystem* sub : subsystems()) {
            sub->periodic();
        }
    }

    /** Schedule a command for execution. */
    static void schedule(Command* cmd) {
        if (!cmd || cmd->m_scheduled) return;

        // Cancel any currently-running commands that conflict on requirements
        for (Subsystem* req : cmd->getRequirements()) {
            if (req->m_currentCommand) {
                cancel(req->m_currentCommand);
            }
        }

        // Register requirement ownership
        for (Subsystem* req : cmd->getRequirements()) {
            req->m_currentCommand = cmd;
        }

        cmd->m_scheduled = true;
        cmd->initialize();
        commands().push_back(cmd);
    }

    /** Cancel a running command (calls end(true)). */
    static void cancel(Command* cmd) {
        if (!cmd || !cmd->m_scheduled) return;

        cmd->end(true);
        unregisterRequirements(cmd);
        cmd->m_scheduled = false;

        auto& cmds = commands();
        cmds.erase(std::remove(cmds.begin(), cmds.end(), cmd), cmds.end());
    }

    /** Cancel every running command. */
    static void cancelAll() {
        auto& cmds = commands();
        for (Command* cmd : cmds) {
            cmd->end(true);
            unregisterRequirements(cmd);
            cmd->m_scheduled = false;
        }
        cmds.clear();
    }

    /** Register a subsystem so it receives periodic() calls. */
    static void registerSubsystem(Subsystem* sub) {
        auto& subs = subsystems();
        if (std::find(subs.begin(), subs.end(), sub) == subs.end()) {
            subs.push_back(sub);
        }
    }

    /** Add a trigger to be polled each tick. */
    static void addTrigger(Trigger trigger) {
        triggers().push_back(std::move(trigger));
    }

    /** Remove all triggers and free any trigger-owned command objects. */
    static void clearTriggers() {
        triggers().clear();
    }

    /** Full scheduler reset: cancel commands and clear all triggers. */
    static void reset() {
        cancelAll();
        clearTriggers();
    }

    /** True if the given command is currently scheduled. */
    static bool isScheduled(const Command* cmd) {
        auto& cmds = commands();
        return std::find(cmds.begin(), cmds.end(), cmd) != cmds.end();
    }

private:
    static void unregisterRequirements(Command* cmd) {
        for (Subsystem* req : cmd->getRequirements()) {
            if (req->m_currentCommand == cmd) {
                req->m_currentCommand = nullptr;
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
    static std::vector<Trigger>& triggers() {
        static std::vector<Trigger> v;
        return v;
    }
};

// ── Inline implementations that depend on CommandScheduler ──────────────────

inline void Command::cancel()        { CommandScheduler::cancel(this); }
inline bool Command::isScheduled() const { return m_scheduled; }
inline void Subsystem::registerThis() { CommandScheduler::registerSubsystem(this); }
