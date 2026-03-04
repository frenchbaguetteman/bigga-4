/**
 * @file functionalCommand.h
 * A command defined entirely by lambdas.
 */
#pragma once

#include "command/command.h"
#include <functional>
#include <vector>

class FunctionalCommand : public Command {
public:
    FunctionalCommand(
        std::function<void()>       onInit,
        std::function<void()>       onExec,
        std::function<void(bool)>   onEnd,
        std::function<bool()>       finished,
        std::vector<Subsystem*>     requirements = {})
        : m_init(std::move(onInit)),
          m_exec(std::move(onExec)),
          m_end(std::move(onEnd)),
          m_finished(std::move(finished)),
          m_requirements(std::move(requirements)) {}

    void initialize() override       { if (m_init)     m_init(); }
    void execute() override          { if (m_exec)     m_exec(); }
    void end(bool interrupted) override { if (m_end)   m_end(interrupted); }
    bool isFinished() override       { return m_finished ? m_finished() : false; }
    std::vector<Subsystem*> getRequirements() override { return m_requirements; }

private:
    std::function<void()>       m_init;
    std::function<void()>       m_exec;
    std::function<void(bool)>   m_end;
    std::function<bool()>       m_finished;
    std::vector<Subsystem*>     m_requirements;
};
