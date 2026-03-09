/**
 * @file sharedCommands.h
 * Shared autonomous helper builders.
 */
#pragma once

#include "command/commandGroup.h"
#include "command/waitCommand.h"
#include "commands/driveMove.h"
#include "commands/intake/intakeCommand.h"
#include "commands/lift/liftCommand.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intakes.h"
#include "subsystems/lift.h"
#include "Eigen/Core"
#include <functional>

namespace shared {

inline Command* driveAndIntake(
        Drivetrain* dt, Intakes* intakes,
        Eigen::Vector2f target,
        std::function<Eigen::Vector3f()> poseSource,
        int intakeVoltage = 127) {
    return (new DriveMoveCommand(dt, target, poseSource))
        ->alongWith(new IntakeSpinCommand(intakes, intakeVoltage));
}

inline Command* outtakeTimed(Intakes* intakes, float seconds) {
    return new IntakeTimedCommand(intakes, -127, seconds);
}

inline Command* liftCycle(Lift* lift, float upDeg, float downDeg) {
    return new SequentialCommandGroup({
        new LiftMoveCommand(lift, upDeg),
        new WaitCommand(0.15f),
        new LiftMoveCommand(lift, downDeg)
    });
}

} // namespace shared
