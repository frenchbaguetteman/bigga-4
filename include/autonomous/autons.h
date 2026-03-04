/**
 * @file autons.h
 * Autonomous routine enumeration and selector.
 */
#pragma once

#include <string>

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

/** Convert Auton enum to a human-readable name. */
inline std::string autonName(Auton a) {
    switch (a) {
        case Auton::NEGATIVE_1: return "Negative 1";
        case Auton::NEGATIVE_2: return "Negative 2";
        case Auton::POSITIVE_1: return "Positive 1";
        case Auton::POSITIVE_2: return "Positive 2";
        case Auton::SKILLS:     return "Skills";
        case Auton::NONE:       return "None";
    }
    return "Unknown";
}
