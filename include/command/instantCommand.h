/**
 * @file instantCommand.h
 * A command that runs a single action and immediately finishes.
 */
#pragma once

#include "command/command.h"
#include <functional>
#include <vector>

class InstantCommand : public Command {
public:
    InstantCommand(std::function<void()> action,
                   std::vector<Subsystem*> requirements = {})
        : m_action(std::move(action)),
          m_requirements(std::move(requirements)) {}

    void initialize() override { m_action(); }
    bool isFinished() override { return true; }
    std::vector<Subsystem*> getRequirements() override { return m_requirements; }

private:
    std::function<void()>   m_action;
    std::vector<Subsystem*> m_requirements;
};
