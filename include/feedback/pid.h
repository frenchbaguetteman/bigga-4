/**
 * @file pid.h
 * PID feedback controller.
 */
#pragma once

#include "feedback.h"

class PID : public FeedbackController {
public:
    struct Gains {
        float kP       = 0.0f;
        float kI       = 0.0f;
        float kD       = 0.0f;
        float integralCap = 0.0f;   // 0 = no cap
    };

    explicit PID(Gains gains, float tolerance = 0.01f);

    float calculate(float measurement, float setpoint) override;
    void  reset() override;
    bool  atSetpoint() const override;

    void  setGains(Gains gains);
    void  setTolerance(float tol);
    float getError() const { return m_error; }

private:
    Gains m_gains;
    float m_tolerance;

    float m_error      = 0.0f;
    float m_prevError  = 0.0f;
    float m_integral   = 0.0f;
    float m_derivative = 0.0f;
    float m_output     = 0.0f;
    bool  m_firstRun   = true;
};
