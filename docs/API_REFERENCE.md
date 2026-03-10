# API Reference

<div class="page-intro">
  <strong>Scope:</strong> project-local symbols and integration points for the
  69580A codebase. This is the repo reference layer, not a replacement for the
  official PROS API.
</div>

## Modules

<div class="api-grid" markdown>

<div class="api-card">
  <span class="api-label">Core</span>
  <h3><a href="API_COMMANDS/">Command Framework</a></h3>
  <p>`Command`, `Subsystem`, scheduler behavior, command groups, and utility commands.</p>
</div>

<div class="api-card">
  <span class="api-label">Motion</span>
  <h3><a href="API_AUTONOMOUS_AND_MOTION/">Autonomous and Motion</a></h3>
  <p>Auton selection enums, build context, point moves, turns, RAMSETE, and profiles.</p>
</div>

<div class="api-card">
  <span class="api-label">Hardware</span>
  <h3><a href="API_SUBSYSTEMS_AND_UI/">Subsystems and UI</a></h3>
  <p>Drivetrain, intake, pneumatics, selector state, and brain-screen interfaces.</p>
</div>

<div class="api-card">
  <span class="api-label">Config</span>
  <h3><a href="API_CONFIGURATION/">Configuration Surface</a></h3>
  <p>Ports, gains, driver tuning, and the constants most likely to be edited.</p>
</div>

</div>

## Global Conventions

| Topic | Rule |
|---|---|
| Pose units | meters for position, radians for heading |
| Internal frame | `+X` forward/east, `+Y` left/north, positive heading is CCW |
| Teleop driving | handled in `opcontrol()` in [`src/main.cpp`](../src/main.cpp) |
| Autonomous entrypoint | `autonCommands::makeAutonCommand(...)` |
| Shared mechanism bundles | `include/autonomous/sharedCommands.h` |
| Main tuning surface | [`include/config.h`](../include/config.h) |

## Runtime Entry Points

| Symbol | File | Purpose |
|---|---|---|
| `initialize()` | [`src/main.cpp`](../src/main.cpp) | startup, subsystem construction, localization init, first auton build |
| `autonomous()` | [`src/main.cpp`](../src/main.cpp) | rebuild and schedule the selected auton |
| `opcontrol()` | [`src/main.cpp`](../src/main.cpp) | controller polling, manual drive, skills special-case handling |
| `makeAutonCommand(...)` | [`include/autonomous/autonCommands.h`](../include/autonomous/autonCommands.h) | convert selector state into a runnable top-level command graph |

## Most Used Symbols

| Symbol | Page |
|---|---|
| `Command`, `Subsystem`, `CommandScheduler` | [Command Framework](API_COMMANDS.md) |
| `DriveMoveCommand`, `RotateCommand`, `RamseteCommand`, `LtvUnicycleCommand` | [Autonomous and Motion](API_AUTONOMOUS_AND_MOTION.md) |
| `Drivetrain`, `AutonSelector`, `BrainScreen` | [Subsystems and UI](API_SUBSYSTEMS_AND_UI.md) |
| `DISTANCE_PID`, `TURN_PID`, `RAMSETE_*` | [Configuration Surface](API_CONFIGURATION.md) |

## File Index

| Area | Primary headers |
|---|---|
| Command framework | `include/command/*.h` |
| Autonomous selection/building | `include/autonomous/*.h` |
| Motion commands | `include/commands/*.h`, `include/motionProfiling/*.h` |
| Subsystems | `include/subsystems/*.h` |
| UI | `include/ui/*.h` |
| Tunables | `include/config.h` |
