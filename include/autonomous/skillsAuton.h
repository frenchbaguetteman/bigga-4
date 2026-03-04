/**
 * @file skillsAuton.h
 * Programming-skills autonomous routine (60 seconds).
 */
#pragma once

#include "command/command.h"
#include "command/commandGroup.h"
#include "commands/ramsete.h"
#include "commands/driveMove.h"
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
 * Full skills autonomous — scores as many game elements as possible
 * in 60 seconds.
 *
 * This is a skeleton; replace waypoints/paths with real field data.
 */
inline Command* makeSkills(
        Drivetrain* dt, Intakes* intakes, Lift* lift, Solenoids* sol,
        std::function<Eigen::Vector3f()> poseSource) {

    // Example multi-segment skills path
    auto makeSegment = [&](float x1, float y1, float x2, float y2) -> Command* {
        Path p;
        float theta = std::atan2(y2 - y1, x2 - x1);
        p.addWaypoint({x1, y1, theta});
        p.addWaypoint({x2, y2, theta});
        ProfileConstraints c{QVelocity(1.5f), QAcceleration(2.5f)};
        TrapezoidalVelocityProfile vp(QLength(p.totalLength()), c);
        MotionProfile mp(p, vp);
        return new RamseteCommand(dt, mp, poseSource);
    };

    return new SequentialCommandGroup({
        // Segment 1: drive across field while intaking
        makeSegment(-1.4f, -1.4f, -0.3f, -1.4f)
            ->alongWith(new IntakeSpinCommand(intakes, 127)),

        // Score
        shared::outtakeTimed(intakes, 0.4f),

        // Segment 2
        makeSegment(-0.3f, -1.4f, -0.3f, 0.0f),

        // Collect more
        shared::driveAndIntake(dt, intakes,
            Eigen::Vector2f(0.3f, 0.0f), poseSource),

        // Score again
        shared::outtakeTimed(intakes, 0.4f),

        // Lift cycle
        shared::liftCycle(lift, 180.0f, 0.0f),

        // Continue across field ...
        makeSegment(0.3f, 0.0f, 1.2f, 1.0f),

        // Final score
        shared::outtakeTimed(intakes, 0.5f),

        // Drive to finishing position
        new DriveMoveCommand(dt, Eigen::Vector2f(0.0f, 0.0f), poseSource)
    });
}

} // namespace autons
