/**
 * @file negativeAutons.h
 * Autonomous routines that start on the "negative" (left/far) side.
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
 * Build the Negative-1 autonomous command graph.
 *
 * Outline: drive out → collect ring → score → return.
 * Paths are defined programmatically here; for production use,
 * load pre-computed paths from the static JSON path files via buildProfileFromJson().
 */
inline Command* makeNegative1(
        Drivetrain* dt, Intakes* intakes, Lift* lift, Solenoids* sol,
        std::function<Eigen::Vector3f()> poseSource) {

    // Example waypoints — replace with real field coordinates
    Path p1;
    p1.addWaypoint({-1.2f, -0.6f, 0.0f});
    p1.addWaypoint({-0.6f, -0.6f, 0.0f});
    p1.addWaypoint({ 0.0f, -0.3f, 0.5f});
    ProfileConstraints c{QVelocity(1.2f), QAcceleration(2.0f)};
    TrapezoidalVelocityProfile vp(QLength(p1.totalLength()), c);
    MotionProfile mp1(p1, vp);

    return new SequentialCommandGroup({
        // 1. Follow path to first ring while intaking
        (new RamseteCommand(dt, mp1, poseSource))
            ->alongWith(new IntakeSpinCommand(intakes, 127)),
        // 2. Score
        new WaitCommand(0.3f),
        shared::outtakeTimed(intakes, 0.5f),
        // 3. Rotate and drive back
        new RotateCommand(dt, static_cast<float>(M_PI), poseSource),
        new DriveMoveCommand(dt, Eigen::Vector2f(-1.2f, -0.6f), poseSource)
    });
}

/** Negative-2 routine (stub). */
inline Command* makeNegative2(
        Drivetrain* dt, Intakes* intakes, Lift* lift, Solenoids* sol,
        std::function<Eigen::Vector3f()> poseSource) {
    // TODO: fill with real negative-2 path
    return makeNegative1(dt, intakes, lift, sol, poseSource);
}

} // namespace autons
