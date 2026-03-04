/**
 * @file feedback.h
 * Abstract feedback-controller interface.
 */
#pragma once

class FeedbackController {
public:
    virtual ~FeedbackController() = default;

    /** Compute controller output given current measurement and setpoint. */
    virtual float calculate(float measurement, float setpoint) = 0;

    /** Reset internal state (integral term, previous error, etc.). */
    virtual void reset() = 0;

    /** Whether the controller is "at setpoint" within tolerance. */
    virtual bool atSetpoint() const = 0;
};
