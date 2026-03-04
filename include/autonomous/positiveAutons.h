/**
 * @file positiveAutons.h
 * Autonomous routines that start on the "positive" (right/near) side.
 */
#pragma once

#include "command/command.h"
#include "command/commandGroup.h"
#include "command/waitCommand.h"
#include "commands/ramsete.h"
#include "commands/driveMove.h"
#include "commands/rotate.h"
#include "commands/intake/intakeCommand.h"
#include "autonomous/sharedCommands.h"
#include "motionProfiling/motionProfile.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intakes.h"
#include "subsystems/lift.h"
#include "subsystems/solenoids.h"
#include "Eigen/Core"
#include <functional>

namespace autons {

/**
 * Positive-1 autonomous routine.
 */
inline Command* makePositive1(
        Drivetrain* dt, Intakes* intakes, Lift* lift, Solenoids* sol,
        std::function<Eigen::Vector3f()> poseSource) {

    Path p1;
    p1.addWaypoint({1.2f, -0.6f, static_cast<float>(M_PI)});
    p1.addWaypoint({0.6f, -0.6f, static_cast<float>(M_PI)});
    p1.addWaypoint({0.0f, -0.3f, static_cast<float>(M_PI) - 0.5f});
    ProfileConstraints c{QVelocity(1.2f), QAcceleration(2.0f)};
    TrapezoidalVelocityProfile vp(QLength(p1.totalLength()), c);
    MotionProfile mp1(p1, vp);

    return new SequentialCommandGroup({
        (new RamseteCommand(dt, mp1, poseSource))
            ->alongWith(new IntakeSpinCommand(intakes, 127)),
        new WaitCommand(0.3f),
        shared::outtakeTimed(intakes, 0.5f),
        new RotateCommand(dt, 0.0f, poseSource),
        new DriveMoveCommand(dt, Eigen::Vector2f(1.2f, -0.6f), poseSource)
    });
}

/** Positive-2 routine (stub). */
inline Command* makePositive2(
        Drivetrain* dt, Intakes* intakes, Lift* lift, Solenoids* sol,
        std::function<Eigen::Vector3f()> poseSource) {
    return makePositive1(dt, intakes, lift, sol, poseSource);
}

} // namespace autons
