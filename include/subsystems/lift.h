/**
 * @file lift.h
 * Lift stub — CURRENTLY DISABLED (no lift hardware wired).
 *
 * This is a plain no-op surface so the rest of the codebase compiles
 * without dragging a fake scheduler subsystem around.
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
