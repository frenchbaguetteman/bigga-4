/**
 * @file main.h
 * 69580A — master include header.
 *
 * \copyright Copyright (c) 2017-2024, Purdue University ACM SIGBots.
 */
#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"
#include "okapi/api.hpp"

#include "Eigen/Dense"
#include "units/units.hpp"
#include "utils/utils.h"
#include "utils/linear.h"
#include "utils/motor.h"
#include "json/json.h"
#include "telemetry/telemetry.h"
#include "config.h"
#include "auton.h"

// Command framework
#include "command/command.h"
#include "command/subsystem.h"
#include "command/commandScheduler.h"
#include "command/commandGroup.h"
#include "command/waitCommand.h"
#include "command/instantCommand.h"
#include "command/functionalCommand.h"

// Feedback
#include "feedback/feedback.h"
#include "feedback/pid.h"

// Velocity profiles
#include "velocityProfile/trapezoidalVelocityProfile.hpp"
#include "velocityProfile/trapProfile.h"

// Motion profiling
#include "motionProfiling/path.h"
#include "motionProfiling/bezier.h"

// Localization
#include "localization/sensor.h"
#include "localization/distance.h"
#include "localization/gps.h"
#include "localization/particleFilter.h"

// Subsystems
#include "subsystems/drivetrain.h"
#include "subsystems/intakes.h"
#include "subsystems/lift.h"

// Robot commands
#include "commands/driveMove.h"
#include "commands/rotate.h"
#include "commands/intake/intakeCommand.h"
#include "commands/lift/liftCommand.h"

// UI
#include "ui/autonSelector.h"

// Autonomous
#include "autonomous/autons.h"

#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#endif // _PROS_MAIN_H_
