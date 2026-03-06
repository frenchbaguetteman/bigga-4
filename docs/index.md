# 69580A Robot Docs

<div class="hero-block">
  <div class="hero-kicker">Competition Brain</div>
  <h1>69580A documentation, organized like a real template.</h1>
  <p>
    Start from zero, learn the command model, copy working autonomous examples,
    and then use the API reference when you need exact interfaces. The docs are
    grounded in the robot code that is actually in this repository right now.
  </p>
  <div class="hero-actions">
    <a class="md-button md-button--primary" href="GETTING_STARTED/">Start Here</a>
    <a class="md-button" href="AUTONOMOUS_COOKBOOK/">Copy an Auton Example</a>
    <a class="md-button" href="API_REFERENCE/">Open API Reference</a>
  </div>
</div>

<div class="stat-grid" markdown>

<div class="stat-card">
  <span class="stat-label">New users</span>
  <strong>Learn the repo like a template</strong>
  <p>Start with setup, folder layout, competition lifecycle, and the first edits worth making.</p>
</div>

<div class="stat-card">
  <span class="stat-label">Autonomous</span>
  <strong>Working snippets you can copy</strong>
  <p>Example move, turn, and path routines are already in the codebase and documented step by step.</p>
</div>

<div class="stat-card">
  <span class="stat-label">Engineering</span>
  <strong>Task-first tutorials plus real API docs</strong>
  <p>The site splits onboarding, recipes, and low-level interfaces so you can read at the right depth.</p>
</div>

</div>

## Fast Paths

<div class="grid cards" markdown>

-   :material-school-outline:{ .lg .middle } **Start Here**

    ---

    Learn the template structure, where code lives, what runs in `initialize()`, `autonomous()`, and `opcontrol()`, and what to edit first.

    [Open Getting Started](GETTING_STARTED.md)

-   :material-book-open-page-variant-outline:{ .lg .middle } **Template Tour**

    ---

    Map common PROS and EZ-Template ideas onto this repo's command-based architecture and folder layout.

    [Open Template Tour](TEMPLATE_TOUR.md)

-   :material-vector-polyline:{ .lg .middle } **Autonomous**

    ---

    Copy real autonomous patterns from this project: point moves, turns, RAMSETE paths, and shared helper commands.

    [Open Autonomous Cookbook](AUTONOMOUS_COOKBOOK.md)

-   :material-file-code-outline:{ .lg .middle } **API Reference**

    ---

    Jump into subsystem and command interfaces, localization rationale, regression procedures, and implementation checklists.

    [Open Engineering Docs](API_REFERENCE.md)

</div>

## Match-Day Loop

<div class="quick-flow" markdown>

1. Boot the robot flat and still.
2. Wait for IMU and localization startup to settle.
3. Confirm the selected autonomous and alliance on the brain screen.
4. Use the quick links below for the exact workflow you need.

</div>

<div class="grid cards compact-cards" markdown>

-   :material-clipboard-check-outline:{ .lg .middle } **Pre-Match Checklist**

    ---

    Use the practical operator walkthrough before queueing, connecting controllers, or selecting a routine.

    [Open the User Guide](USER_GUIDE.md#competition-day-quick-start)

-   :material-map-marker-path:{ .lg .middle } **Motion Reference**

    ---

    Read the exact motion behavior of the current commands and autonomous routines, including placeholder mappings.

    [Open Motion Reference](MOTION_REFERENCE.md)

-   :material-source-branch:{ .lg .middle } **Tutorial Workflows**

    ---

    Use the task-oriented tutorials for tuning, localization debugging, custom autons, and upload workflows.

    [Open Tutorials](TUTORIALS.md)

</div>

!!! warning "Current implementation caveats"
    The docs are grounded in the current code, including some unfinished areas:

    - `Negative 2` currently runs the same graph as `Negative 1`
    - `Positive 2` currently runs the same graph as `Positive 1`
    - alliance selection is UI-visible but does not yet change auton generation
    - the lift is still stubbed and does not move real hardware
    - forward odometry currently uses drive-encoder fallback because the vertical tracking wheel is disabled

## What This Site Covers

| Area | Best page |
|---|---|
| First-time setup and repo tour | [Getting Started](GETTING_STARTED.md) |
| How this project maps to a template | [Template Tour](TEMPLATE_TOUR.md) |
| Copy-paste autonomous patterns | [Autonomous Cookbook](AUTONOMOUS_COOKBOOK.md) |
| Driver controls and startup | [User Guide](USER_GUIDE.md) |
| Every current robot motion | [Motion Reference](MOTION_REFERENCE.md) |
| Team workflows and tuning | [Tutorials](TUTORIALS.md) |
| Class-level interfaces | [API Reference](API_REFERENCE.md) |
| Localization architecture | [Localization Design Rationale](LOCALIZATION_DESIGN_RATIONALE.md) |

## Build and Preview

<div class="command-strip" markdown>

```bash
python3 -m pip install -r requirements-docs.txt
python3 -m mkdocs serve
python3 -m mkdocs build
```

</div>

## Source Layout

The site is built directly from the markdown files in `docs/`. The robot code remains the source of truth; these docs are a structured layer on top of the implementation in `src/` and `include/`.
