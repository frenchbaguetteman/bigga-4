/**
 * @file sharedCommands.h
 * Reusable command snippets shared across autonomous routines.
 */
#pragma once

#include "command/command.h"
#include "command/commandGroup.h"
#include "command/instantCommand.h"
#include "command/waitCommand.h"
#include "commands/intake/intakeCommand.h"
#include "commands/lift/liftCommand.h"
#include "commands/driveMove.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intakes.h"
#include "subsystems/lift.h"
#include "subsystems/solenoids.h"
#include "Eigen/Core"
#include <functional>

namespace shared {

/** Drive forward to a point and intake simultaneously. */
inline Command* driveAndIntake(
        Drivetrain* dt, Intakes* intakes,
        Eigen::Vector2f target,
        std::function<Eigen::Vector3f()> poseSource,
        int intakeVoltage = 127) {
    return (new DriveMoveCommand(dt, target, poseSource))
        ->alongWith(new IntakeSpinCommand(intakes, intakeVoltage));
}

/** Outtake for a fixed duration. */
inline Command* outtakeTimed(Intakes* intakes, float seconds) {
    return new IntakeTimedCommand(intakes, -127, seconds);
}

/** Raise lift, wait, then lower. */
inline Command* liftCycle(Lift* lift, float upDeg, float downDeg) {
    return new SequentialCommandGroup({
        new LiftMoveCommand(lift, upDeg),
        new WaitCommand(0.15f),
        new LiftMoveCommand(lift, downDeg)
    });
}

/** Toggle tongue pneumatic. */
inline Command* toggleTongue(Solenoids* sol) {
    return new InstantCommand([sol]() { sol->toggleTongue(); });
}

/** Toggle wing pneumatic. */
inline Command* toggleWing(Solenoids* sol) {
    return new InstantCommand([sol]() { sol->toggleWing(); });
}

/** Toggle select1 pneumatic. */
inline Command* toggleSelect1(Solenoids* sol) {
    return new InstantCommand([sol]() { sol->toggleSelect1(); });
}

/** Toggle select2 pneumatic. */
inline Command* toggleSelect2(Solenoids* sol) {
    return new InstantCommand([sol]() { sol->toggleSelect2(); });
}

} // namespace shared
