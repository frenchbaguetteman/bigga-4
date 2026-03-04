/**
 * @file trigger.h
 * Button / boolean-condition → command binding.
 */
#pragma once

#include <functional>

// Forward declaration
class Command;

class Trigger {
public:
    explicit Trigger(std::function<bool()> condition)
        : m_condition(std::move(condition)) {}

    /** Bind: schedule command on rising edge. */
    Trigger& onTrue(Command* cmd) { m_onTrue = cmd; return *this; }

    /** Bind: schedule command on falling edge. */
    Trigger& onFalse(Command* cmd) { m_onFalse = cmd; return *this; }

    /** Bind: schedule while held, cancel on release. */
    Trigger& whileTrue(Command* cmd) { m_whileTrue = cmd; return *this; }

    /** Bind: schedule while NOT held, cancel on press. */
    Trigger& whileFalse(Command* cmd) { m_whileFalse = cmd; return *this; }

    /** Bind: toggle command on rising edge. */
    Trigger& toggleOnTrue(Command* cmd) { m_toggleOnTrue = cmd; return *this; }

    /** Called by the scheduler each tick. */
    void poll();

private:
    std::function<bool()> m_condition;
    bool m_lastState = false;

    Command* m_onTrue       = nullptr;
    Command* m_onFalse      = nullptr;
    Command* m_whileTrue    = nullptr;
    Command* m_whileFalse   = nullptr;
    Command* m_toggleOnTrue = nullptr;
};
