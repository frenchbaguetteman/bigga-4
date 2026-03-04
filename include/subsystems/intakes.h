/**
 * @file intakes.h
 * Intake subsystem — linked motor pair, simple spin/stop interface.
 */
#pragma once

#include "command/subsystem.h"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

class Intakes : public Subsystem {
public:
    Intakes();

    void periodic() override;

    /** Spin at the given voltage (−127..127). */
    void spin(int voltage);

    /** Spin at a given velocity (RPM). */
    void spinVelocity(int rpm);

    /** Stop and brake. */
    void stop();

    /** Is something jammed? (current spike detection). */
    bool isStalled() const;

    /** Average current draw in mA. */
    float getCurrent() const;

private:
    pros::MotorGroup m_motors;
};
