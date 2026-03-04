/**
 * @file motor.h
 * Motor / actuator utility helpers.
 */
#pragma once

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include <algorithm>

namespace motorUtil {

/** Clamp a voltage command to the V5 motor range [−127, 127]. */
inline int clampVoltage(float voltage) {
    return static_cast<int>(std::clamp(voltage, -127.0f, 127.0f));
}

/** Clamp a velocity command to [−maxRPM, maxRPM]. */
inline int clampVelocity(float vel, float maxRPM = 600.0f) {
    return static_cast<int>(std::clamp(vel, -maxRPM, maxRPM));
}

/** Convert voltage (−12 000..12 000 mV) to the move() scale (−127..127). */
inline int mVToMove(float mV) {
    return clampVoltage(mV * 127.0f / 12000.0f);
}

/** Set a motor group to brake mode. */
inline void setBrake(pros::MotorGroup& mg) {
    mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

/** Set a motor group to coast mode. */
inline void setCoast(pros::MotorGroup& mg) {
    mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

/** Set a motor group to hold mode. */
inline void setHold(pros::MotorGroup& mg) {
    mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

} // namespace motorUtil
