/**
 * @file lift.h
 * Disabled lift stub.
 */
#pragma once

struct Lift {
    Lift() = default;

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
