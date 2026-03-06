# Documentation Map

This directory now has separate docs for onboarding, template usage, autonomous examples, operator workflows, and low-level references.

If you want the pretty docs site instead of raw markdown, run:

```bash
python3 -m pip install -r ../requirements-docs.txt
python3 -m mkdocs serve
```

## Start Here

- [GETTING_STARTED.md](GETTING_STARTED.md): first-time setup, repo map, and first safe edits
- [TEMPLATE_TOUR.md](TEMPLATE_TOUR.md): how the repo is structured and how it maps to PROS and EZ-Template concepts
- [AUTONOMOUS_COOKBOOK.md](AUTONOMOUS_COOKBOOK.md): copyable autonomous patterns, examples, and snippets
- [TUTORIALS.md](TUTORIALS.md): task-first workflows for tuning, localization debugging, editing autons, and uploads
- [USER_GUIDE.md](USER_GUIDE.md): competition-day operations, controls, hardware map, startup, and major caveats
- [LOCALIZATION_SCREEN_MOCKUP.md](LOCALIZATION_SCREEN_MOCKUP.md): docs-side visual mockup of the brain-screen localization page
- [MOTION_REFERENCE.md](MOTION_REFERENCE.md): every implemented teleop motion and autonomous routine, step by step

## Engineering References

- [API_REFERENCE.md](API_REFERENCE.md): API landing page and module index
- [API_COMMANDS.md](API_COMMANDS.md): command framework symbols
- [API_AUTONOMOUS_AND_MOTION.md](API_AUTONOMOUS_AND_MOTION.md): auton builders, point moves, path following, and profiles
- [API_SUBSYSTEMS_AND_UI.md](API_SUBSYSTEMS_AND_UI.md): subsystem and UI symbols
- [API_CONFIGURATION.md](API_CONFIGURATION.md): ports, gains, and tuning constants
- [LOCALIZATION_DESIGN_RATIONALE.md](LOCALIZATION_DESIGN_RATIONALE.md): why the localization architecture works the way it does
- [LOCALIZATION_REFACTORING_SUMMARY.md](LOCALIZATION_REFACTORING_SUMMARY.md): summary of the refactor and behavior guarantees
- [LOCALIZATION_REGRESSION_TESTS.md](LOCALIZATION_REGRESSION_TESTS.md): manual test plan for localization regressions
- [IMPLEMENTATION_CHECKLIST.md](IMPLEMENTATION_CHECKLIST.md): engineering checklist and follow-up items

## Suggested Reading Order

For a new driver:

1. `USER_GUIDE.md`
2. `MOTION_REFERENCE.md`

For a new programmer:

1. `GETTING_STARTED.md`
2. `TEMPLATE_TOUR.md`
3. `AUTONOMOUS_COOKBOOK.md`
4. `TUTORIALS.md`
5. `API_REFERENCE.md`
6. specific `API_*` pages as needed

For autonomous work:

1. `AUTONOMOUS_COOKBOOK.md`
2. `MOTION_REFERENCE.md`
3. `TUTORIALS.md`
4. [`../src/autonomous/autons.cpp`](../src/autonomous/autons.cpp)
