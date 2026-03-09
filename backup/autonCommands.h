/**
 * @file autonCommands.h
 * Central autonomous build entrypoint.
 */
#pragma once

#include "Eigen/Core"
#include "autonomous/autons.h"
#include "command/command.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intakes.h"
#include "subsystems/lift.h"

#include <functional>
#include <memory>

struct AutonBuildContext {
    Drivetrain& drivetrain;
    Intakes& intakes;
    Lift& lift;
    std::function<Eigen::Vector3f()> poseSource;
};

namespace autonCommands {

/** Build a fresh top-level command graph for the selected autonomous routine. */
std::unique_ptr<Command> makeAutonCommand(Auton auton, const AutonBuildContext& ctx);

} // namespace autonCommands
