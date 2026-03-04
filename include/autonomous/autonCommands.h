/**
 * @file autonCommands.h
 * Top-level factory that returns the selected autonomous Command*.
 */
#pragma once

#include "autonomous/autons.h"
#include "autonomous/negativeAutons.h"
#include "autonomous/positiveAutons.h"
#include "autonomous/skillsAuton.h"
#include "command/command.h"
#include "command/instantCommand.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intakes.h"
#include "subsystems/lift.h"
#include "subsystems/solenoids.h"
#include <functional>

namespace autonCommands {

/**
 * Build the top-level autonCommand for the given Auton selection.
 */
inline Command* makeAutonCommand(
        Auton auton,
        Drivetrain* dt, Intakes* intakes, Lift* lift, Solenoids* sol,
        std::function<Eigen::Vector3f()> poseSource) {

    switch (auton) {
        case Auton::NEGATIVE_1:
            return autons::makeNegative1(dt, intakes, lift, sol, poseSource);
        case Auton::NEGATIVE_2:
            return autons::makeNegative2(dt, intakes, lift, sol, poseSource);
        case Auton::POSITIVE_1:
            return autons::makePositive1(dt, intakes, lift, sol, poseSource);
        case Auton::POSITIVE_2:
            return autons::makePositive2(dt, intakes, lift, sol, poseSource);
        case Auton::SKILLS:
            return autons::makeSkills(dt, intakes, lift, sol, poseSource);
        case Auton::NONE:
        default:
            return new InstantCommand([]() {}); // no-op
    }
}

} // namespace autonCommands
