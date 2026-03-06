# 69580A Robot Docs

<div class="hero-block">
  <div class="hero-kicker">Competition Brain</div>
  <h1>69580A documentation, organized for real match use.</h1>
  <p>
    This site turns the repo docs into an operator-facing manual, a programmer
    reference, and a motion handbook for the robot code that is actually in this
    repository right now.
  </p>
  <div class="hero-actions">
    <a class="md-button md-button--primary" href="USER_GUIDE/">Open User Guide</a>
    <a class="md-button" href="MOTION_REFERENCE/">Browse Motion Reference</a>
    <a class="md-button" href="TUTORIALS/">Run the Tutorials</a>
  </div>
</div>

## Fast Paths

<div class="grid cards" markdown>

-   :material-controller-classic:{ .lg .middle } **Drivers**

    ---

    Start with the operator view of the robot: controls, startup flow, brain-screen pages, slot defaults, and competition-day caveats.

    [Open the User Guide](USER_GUIDE.md)

-   :material-vector-polyline:{ .lg .middle } **Autonomous**

    ---

    Read the exact motion behavior of every command and every current autonomous sequence, including the placeholder routine mappings.

    [Open the Motion Reference](MOTION_REFERENCE.md)

-   :material-wrench-cog:{ .lg .middle } **Programmers**

    ---

    Follow step-by-step workflows for tuning, localization debugging, editing routines, building profiles, and uploading slot layouts.

    [Open Tutorials](TUTORIALS.md)

-   :material-file-code-outline:{ .lg .middle } **Engineers**

    ---

    Jump into subsystem and command interfaces, localization rationale, regression procedures, and implementation checklists.

    [Open Engineering Docs](API_REFERENCE.md)

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
| Driver controls and startup | [User Guide](USER_GUIDE.md) |
| Every current robot motion | [Motion Reference](MOTION_REFERENCE.md) |
| Team workflows and tuning | [Tutorials](TUTORIALS.md) |
| Class-level interfaces | [API Reference](API_REFERENCE.md) |
| Localization architecture | [Localization Design Rationale](LOCALIZATION_DESIGN_RATIONALE.md) |

## Local Preview

Install the docs dependencies:

```bash
python3 -m pip install -r requirements-docs.txt
```

Run the site locally:

```bash
python3 -m mkdocs serve
```

Build the static site:

```bash
python3 -m mkdocs build
```

## Source Layout

The site is built directly from the markdown files in `docs/`. The robot code remains the source of truth; these docs are a structured layer on top of the implementation in `src/` and `include/`.
