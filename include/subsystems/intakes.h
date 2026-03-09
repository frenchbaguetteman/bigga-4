/**
 * @file intakes.h
 * Intake subsystem — OkapiLib MotorGroup wrapper with spin/stop interface.
 */
#pragma once

#include "command/subsystem.h"
#include "okapi/api.hpp"

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
    okapi::MotorGroup m_motors;
};
