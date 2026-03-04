/**
 * @file commandGroup.h
 * Composite commands: Sequential, Parallel, and Race groups.
 */
#pragma once

#include "command/command.h"
#include "command/subsystem.h"
#include <vector>
#include <algorithm>
#include <set>
#include <functional>

// ── SequentialCommandGroup ──────────────────────────────────────────────────

class SequentialCommandGroup : public Command {
public:
    SequentialCommandGroup(std::initializer_list<Command*> cmds)
        : m_commands(cmds) {}
    SequentialCommandGroup(std::vector<Command*> cmds)
        : m_commands(std::move(cmds)) {}

    void initialize() override {
        m_index = 0;
        if (!m_commands.empty()) m_commands[0]->initialize();
    }

    void execute() override {
        if (m_index >= m_commands.size()) return;
        Command* cur = m_commands[m_index];
        cur->execute();
        if (cur->isFinished()) {
            cur->end(false);
            ++m_index;
            if (m_index < m_commands.size()) {
                m_commands[m_index]->initialize();
            }
        }
    }

    void end(bool interrupted) override {
        if (interrupted && m_index < m_commands.size()) {
            m_commands[m_index]->end(true);
        }
    }

    bool isFinished() override { return m_index >= m_commands.size(); }

    std::vector<Subsystem*> getRequirements() override {
        std::set<Subsystem*> reqs;
        for (auto* c : m_commands)
            for (auto* s : c->getRequirements())
                reqs.insert(s);
        return {reqs.begin(), reqs.end()};
    }

private:
    std::vector<Command*> m_commands;
    size_t m_index = 0;
};

// ── ParallelCommandGroup ────────────────────────────────────────────────────

class ParallelCommandGroup : public Command {
public:
    ParallelCommandGroup(std::initializer_list<Command*> cmds)
        : m_commands(cmds) {}
    ParallelCommandGroup(std::vector<Command*> cmds)
        : m_commands(std::move(cmds)) {}

    void initialize() override {
        m_finished.assign(m_commands.size(), false);
        for (auto* c : m_commands) c->initialize();
    }

    void execute() override {
        for (size_t i = 0; i < m_commands.size(); ++i) {
            if (m_finished[i]) continue;
            m_commands[i]->execute();
            if (m_commands[i]->isFinished()) {
                m_commands[i]->end(false);
                m_finished[i] = true;
            }
        }
    }

    void end(bool interrupted) override {
        if (interrupted) {
            for (size_t i = 0; i < m_commands.size(); ++i)
                if (!m_finished[i]) m_commands[i]->end(true);
        }
    }

    bool isFinished() override {
        for (bool f : m_finished) if (!f) return false;
        return true;
    }

    std::vector<Subsystem*> getRequirements() override {
        std::set<Subsystem*> reqs;
        for (auto* c : m_commands)
            for (auto* s : c->getRequirements())
                reqs.insert(s);
        return {reqs.begin(), reqs.end()};
    }

private:
    std::vector<Command*>  m_commands;
    std::vector<bool>      m_finished;
};

// ── ParallelRaceGroup ───────────────────────────────────────────────────────

class ParallelRaceGroup : public Command {
public:
    ParallelRaceGroup(std::initializer_list<Command*> cmds)
        : m_commands(cmds) {}
    ParallelRaceGroup(std::vector<Command*> cmds)
        : m_commands(std::move(cmds)) {}

    void initialize() override {
        m_done = false;
        for (auto* c : m_commands) c->initialize();
    }

    void execute() override {
        for (auto* c : m_commands) {
            c->execute();
            if (c->isFinished()) { m_done = true; }
        }
    }

    void end(bool interrupted) override {
        for (auto* c : m_commands) c->end(true);
    }

    bool isFinished() override { return m_done; }

    std::vector<Subsystem*> getRequirements() override {
        std::set<Subsystem*> reqs;
        for (auto* c : m_commands)
            for (auto* s : c->getRequirements())
                reqs.insert(s);
        return {reqs.begin(), reqs.end()};
    }

private:
    std::vector<Command*> m_commands;
    bool m_done = false;
};

// ── Deadline (timeout) wrapper ──────────────────────────────────────────────

class DeadlineCommand : public Command {
public:
    DeadlineCommand(Command* inner, float timeoutSec)
        : m_inner(inner), m_timeout(timeoutSec) {}

    void initialize() override { m_elapsed = 0; m_inner->initialize(); }
    void execute() override    { m_inner->execute(); m_elapsed += 0.01f; /* ~10 ms tick */ }
    void end(bool i) override  { m_inner->end(i); }
    bool isFinished() override { return m_inner->isFinished() || m_elapsed >= m_timeout; }
    std::vector<Subsystem*> getRequirements() override { return m_inner->getRequirements(); }

private:
    Command* m_inner;
    float m_timeout;
    float m_elapsed = 0;
};

// ── ConditionalFinish wrapper ───────────────────────────────────────────────

class ConditionalFinishCommand : public Command {
public:
    ConditionalFinishCommand(Command* inner, std::function<bool()> cond)
        : m_inner(inner), m_cond(std::move(cond)) {}

    void initialize() override { m_inner->initialize(); }
    void execute() override    { m_inner->execute(); }
    void end(bool i) override  { m_inner->end(i); }
    bool isFinished() override { return m_inner->isFinished() || m_cond(); }
    std::vector<Subsystem*> getRequirements() override { return m_inner->getRequirements(); }

private:
    Command* m_inner;
    std::function<bool()> m_cond;
};

// ── Builder method implementations ──────────────────────────────────────────

inline Command* Command::andThen(Command* next) {
    return new SequentialCommandGroup({this, next});
}
inline Command* Command::alongWith(Command* other) {
    return new ParallelCommandGroup({this, other});
}
inline Command* Command::raceWith(Command* other) {
    return new ParallelRaceGroup({this, other});
}
inline Command* Command::withTimeout(float seconds) {
    return new DeadlineCommand(this, seconds);
}
inline Command* Command::until(std::function<bool()> condition) {
    return new ConditionalFinishCommand(this, std::move(condition));
}
