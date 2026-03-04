/**
 * @file lift.h
 * Lift subsystem — CURRENTLY DISABLED (no lift hardware wired).
 *
 * Provides a no-op stub so the rest of the codebase compiles.
 * Uncomment hardware in config.h and fill in the constructor
 * when the lift is re-wired.
 */
#pragma once

#include "command/subsystem.h"

class Lift : public Subsystem {
public:
    Lift() = default;

    void periodic() override {}

    /** Move to a target position (degrees). */
    void moveTo(float /*targetDegrees*/) {}

    /** Direct voltage control (−127..127). */
    void moveVoltage(int /*voltage*/) {}

    /** Stop and hold. */
    void stop() {}

    /** Current position in degrees. */
    float getPosition() const { return 0.0f; }

    /** Whether the lift is at its target within tolerance. */
    bool atTarget() const { return true; }
};
