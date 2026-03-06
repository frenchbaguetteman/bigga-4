/**
 * @file trigger.h
 * Button / boolean-condition → command binding.
 */
#pragma once

#include "command/command.h"
#include <functional>
#include <vector>
#include <memory>
#include <algorithm>

class Trigger {
public:
    explicit Trigger(std::function<bool()> condition)
        : m_condition(std::move(condition)) {}

    /** Bind: schedule command on rising edge. */
    Trigger& onTrue(Command* cmd) { m_onTrue = adopt(cmd); return *this; }

    /** Bind: schedule command on falling edge. */
    Trigger& onFalse(Command* cmd) { m_onFalse = adopt(cmd); return *this; }

    /** Bind: schedule while held, cancel on release. */
    Trigger& whileTrue(Command* cmd) { m_whileTrue = adopt(cmd); return *this; }

    /** Bind: schedule while NOT held, cancel on press. */
    Trigger& whileFalse(Command* cmd) { m_whileFalse = adopt(cmd); return *this; }

    /** Bind: toggle command on rising edge. */
    Trigger& toggleOnTrue(Command* cmd) { m_toggleOnTrue = adopt(cmd); return *this; }

    /** Called by the scheduler each tick. */
    void poll();

private:
    Command* adopt(Command* cmd) {
        if (!cmd) return nullptr;
        auto it = std::find_if(m_ownedCommands.begin(), m_ownedCommands.end(),
            [cmd](const std::unique_ptr<Command>& c) { return c.get() == cmd; });
        if (it == m_ownedCommands.end()) {
            m_ownedCommands.emplace_back(cmd);
        }
        return cmd;
    }

    std::function<bool()> m_condition;
    bool m_lastState = false;

    // Commands bound into this trigger are owned by the trigger and will be
    // cleaned up when the trigger is removed/reset by the scheduler.
    std::vector<std::unique_ptr<Command>> m_ownedCommands;

    Command* m_onTrue       = nullptr;
    Command* m_onFalse      = nullptr;
    Command* m_whileTrue    = nullptr;
    Command* m_whileFalse   = nullptr;
    Command* m_toggleOnTrue = nullptr;
};
