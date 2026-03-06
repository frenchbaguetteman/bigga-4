# Command Framework

## Symbol Index

| Symbol | Header | Purpose |
|---|---|---|
| `Command` | [`include/command/command.h`](../include/command/command.h) | base unit of robot behavior |
| `Subsystem` | [`include/command/subsystem.h`](../include/command/subsystem.h) | hardware ownership boundary |
| `CommandScheduler` | [`include/command/commandScheduler.h`](../include/command/commandScheduler.h) | lifecycle, execution, conflict handling |
| `SequentialCommandGroup` | [`include/command/commandGroup.h`](../include/command/commandGroup.h) | run commands one after another |
| `ParallelCommandGroup` | [`include/command/commandGroup.h`](../include/command/commandGroup.h) | run commands together until all finish |
| `ParallelRaceGroup` | [`include/command/commandGroup.h`](../include/command/commandGroup.h) | run commands together until one finishes |
| `DeadlineCommand` | [`include/command/commandGroup.h`](../include/command/commandGroup.h) | timeout wrapper |
| `ConditionalFinishCommand` | [`include/command/commandGroup.h`](../include/command/commandGroup.h) | stop when external condition becomes true |

## `Command`

```cpp
class Command {
public:
    virtual void initialize();
    virtual void execute();
    virtual void end(bool interrupted);
    virtual bool isFinished();
    virtual std::vector<Subsystem*> getRequirements();

    Command* andThen(Command* next);
    Command* alongWith(Command* other);
    Command* raceWith(Command* other);
    Command* withTimeout(float seconds);
    Command* until(std::function<bool()> condition);

    void cancel();
    bool isScheduled() const;
};
```

| Method | Meaning |
|---|---|
| `initialize()` | one-time setup when scheduled |
| `execute()` | called every scheduler tick while active |
| `end(bool)` | cleanup on finish or cancellation |
| `isFinished()` | natural completion condition |
| `getRequirements()` | subsystem ownership declaration |
| `cancel()` | request cancellation through the scheduler |
| `isScheduled()` | current scheduled state |

## `Subsystem`

```cpp
class Subsystem {
public:
    virtual void periodic();
    void setDefaultCommand(Command* cmd);
    Command* getDefaultCommand() const;
    Command* getCurrentCommand() const;
    void registerThis();
};
```

| Method | Meaning |
|---|---|
| `periodic()` | always-on per-tick hook |
| `setDefaultCommand(...)` | fallback command when subsystem is idle |
| `getCurrentCommand()` | active owner or `nullptr` |
| `registerThis()` | register with the global scheduler |

## `CommandScheduler`

```cpp
class CommandScheduler {
public:
    static void run();
    static void schedule(Command* cmd);
    static void cancel(Command* cmd);
    static void cancelAll();
    static void registerSubsystem(Subsystem* sub);
    static void reset();
    static bool isScheduled(const Command* cmd);
};
```

| Method | Effect |
|---|---|
| `run()` | execute active commands, install default commands, call subsystem `periodic()` |
| `schedule(...)` | claim requirements, cancel conflicting commands, call `initialize()` |
| `cancel(...)` | call `end(true)` and release requirements |
| `cancelAll()` | stop every running command |
| `reset()` | full scheduler reset, currently equivalent to `cancelAll()` |
| `isScheduled(...)` | query current scheduled state |

## Composition Helpers

The builder-style helpers on `Command` allocate wrappers on the heap:

```cpp
auto cmd = (new RamseteCommand(ctx.drivetrain, profile, ctx.poseSource))
    ->alongWith(new IntakeSpinCommand(ctx.intakes, 127))
    ->withTimeout(1.8f);
```

| Helper | Returns |
|---|---|
| `andThen(...)` | `SequentialCommandGroup` |
| `alongWith(...)` | `ParallelCommandGroup` |
| `raceWith(...)` | `ParallelRaceGroup` |
| `withTimeout(...)` | `DeadlineCommand` |
| `until(...)` | `ConditionalFinishCommand` |

## Utility Commands

| Symbol | Header | Notes |
|---|---|---|
| `WaitCommand` | [`include/command/waitCommand.h`](../include/command/waitCommand.h) | fixed-duration delay |
| `InstantCommand` | [`include/command/instantCommand.h`](../include/command/instantCommand.h) | one-shot lambda |
| `FunctionalCommand` | [`include/command/functionalCommand.h`](../include/command/functionalCommand.h) | inline custom command |

## Repo-Specific Notes

- There is no local `Trigger` reference surface in the current tree; controller input is polled directly in `opcontrol()`.
- Requirement conflicts are resolved by cancelling the currently running owner.
- Subsystem default commands are scheduled automatically when the subsystem becomes idle.
