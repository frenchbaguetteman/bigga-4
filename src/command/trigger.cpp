/**
 * @file trigger.cpp
 * Trigger::poll() implementation — separated because it needs
 * CommandScheduler which in turn includes trigger.h.
 *
 * Compiled as part of the command library.
 */
#include "command/trigger.h"
#include "command/commandScheduler.h"

void Trigger::poll() {
    bool current = m_condition ? m_condition() : false;
    bool rising  = current && !m_lastState;
    bool falling = !current && m_lastState;

    // Rising edge
    if (rising) {
        if (m_onTrue) CommandScheduler::schedule(m_onTrue);
        if (m_toggleOnTrue) {
            if (m_toggleOnTrue->isScheduled())
                CommandScheduler::cancel(m_toggleOnTrue);
            else
                CommandScheduler::schedule(m_toggleOnTrue);
        }
    }

    // Falling edge
    if (falling) {
        if (m_onFalse) CommandScheduler::schedule(m_onFalse);
    }

    // While held
    if (current && m_whileTrue && !m_whileTrue->isScheduled()) {
        CommandScheduler::schedule(m_whileTrue);
    }
    if (!current && m_whileTrue && m_whileTrue->isScheduled()) {
        CommandScheduler::cancel(m_whileTrue);
    }

    // While NOT held
    if (!current && m_whileFalse && !m_whileFalse->isScheduled()) {
        CommandScheduler::schedule(m_whileFalse);
    }
    if (current && m_whileFalse && m_whileFalse->isScheduled()) {
        CommandScheduler::cancel(m_whileFalse);
    }

    m_lastState = current;
}
