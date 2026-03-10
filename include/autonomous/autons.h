/**
 * @file autons.h
 * Autonomous routine identifiers and shared selector metadata.
 */
#pragma once

#include "Eigen/Core"

#include <array>
#include <functional>

enum class Auton {
    NEGATIVE_1,
    POSITIVE_1,
    TUNE_DRIVE_PID,
    TUNE_TURN_PID,
    EXAMPLE_MOVE,
    EXAMPLE_SWING,
    EXAMPLE_TURN,
    EXAMPLE_RAMSETE,
    EXAMPLE_LTV,
    SKILLS,
    NONE
};

enum class Alliance {
    RED,
    BLUE
};

class Drivetrain;
class Intakes;
class Lift;

using AutonFn = void (*)();

struct AutonEntry {
    Auton id = Auton::NONE;
    const char* name = "None";
    AutonFn run = nullptr;
};

using AutonList = std::array<AutonEntry, 11>;

/** Prepare the vendored EZ-Template motion backend during startup. */
bool initializeAutonMotion();

/** Ordered list used by the UI and build plumbing. */
const AutonList& availableAutons();

/** Lookup a selector entry by enum id. */
const AutonEntry* findAuton(Auton auton);

/** Convert an Auton enum to a human-readable name. */
const char* autonName(Auton auton);

/** Convert an Alliance enum to a human-readable name. */
const char* allianceName(Alliance alliance);

/** Bind the current robot runtime so auton functions can run directly. */
void bindAutonRuntime(Drivetrain& drivetrain,
                      Intakes& intakes,
                      Lift& lift,
                      std::function<Eigen::Vector3f()> poseSource,
                      std::function<bool()> isCancelled = {});

/** Stop and clear the currently bound auton runtime. */
void resetAutonRuntime();

/** Run a selector entry immediately in the current task. */
bool runAuton(const AutonEntry& entry);
