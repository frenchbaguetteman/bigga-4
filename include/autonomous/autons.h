/**
 * @file autons.h
 * Autonomous routine identifiers and shared selector metadata.
 */
#pragma once

#include <vector>

enum class Auton {
    NEGATIVE_1,
    NEGATIVE_2,
    POSITIVE_1,
    POSITIVE_2,
    SKILLS,
    NONE
};

enum class Alliance {
    RED,
    BLUE
};

/** Ordered list used by the UI and build plumbing. */
const std::vector<Auton>& availableAutons();

/** Convert an Auton enum to a human-readable name. */
const char* autonName(Auton auton);

/** Convert an Alliance enum to a human-readable name. */
const char* allianceName(Alliance alliance);
