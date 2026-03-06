# Documentation Map

This directory now has separate docs for operators, motion behavior, tutorials, and low-level references.

## Start Here

- [USER_GUIDE.md](USER_GUIDE.md): competition-day operations, controls, hardware map, startup, and major caveats
- [MOTION_REFERENCE.md](MOTION_REFERENCE.md): every implemented teleop motion and autonomous routine, step by step
- [TUTORIALS.md](TUTORIALS.md): workflow guides for tuning, editing autons, localization debugging, and uploads

## Engineering References

- [API_REFERENCE.md](API_REFERENCE.md): class and subsystem API overview
- [LOCALIZATION_DESIGN_RATIONALE.md](LOCALIZATION_DESIGN_RATIONALE.md): why the localization architecture works the way it does
- [LOCALIZATION_REFACTORING_SUMMARY.md](LOCALIZATION_REFACTORING_SUMMARY.md): summary of the refactor and behavior guarantees
- [LOCALIZATION_REGRESSION_TESTS.md](LOCALIZATION_REGRESSION_TESTS.md): manual test plan for localization regressions
- [IMPLEMENTATION_CHECKLIST.md](IMPLEMENTATION_CHECKLIST.md): engineering checklist and follow-up items

## Suggested Reading Order

For a new driver:

1. `USER_GUIDE.md`
2. `MOTION_REFERENCE.md`

For a new programmer:

1. `USER_GUIDE.md`
2. `TUTORIALS.md`
3. `API_REFERENCE.md`
4. localization docs as needed

For autonomous work:

1. `MOTION_REFERENCE.md`
2. `TUTORIALS.md`
3. [`../src/autonomous/autons.cpp`](../src/autonomous/autons.cpp)
