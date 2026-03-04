/**
 * @file pid.cpp
 * PID feedback controller implementation.
 */
#include "feedback/pid.h"
#include <cmath>
#include <algorithm>

PID::PID(Gains gains, float tolerance)
    : m_gains(gains), m_tolerance(tolerance) {}

float PID::calculate(float measurement, float setpoint) {
    m_error = setpoint - measurement;

    // Integral accumulation with optional capping
    m_integral += m_error;
    if (m_gains.integralCap > 0.0f) {
        m_integral = std::clamp(m_integral, -m_gains.integralCap, m_gains.integralCap);
    }

    // Derivative (skip first cycle to avoid kick)
    if (m_firstRun) {
        m_derivative = 0.0f;
        m_firstRun   = false;
    } else {
        m_derivative = m_error - m_prevError;
    }

    m_output    = m_gains.kP * m_error
                + m_gains.kI * m_integral
                + m_gains.kD * m_derivative;

    m_prevError = m_error;
    return m_output;
}

void PID::reset() {
    m_error      = 0.0f;
    m_prevError  = 0.0f;
    m_integral   = 0.0f;
    m_derivative = 0.0f;
    m_output     = 0.0f;
    m_firstRun   = true;
}

bool PID::atSetpoint() const {
    return std::fabs(m_error) < m_tolerance;
}

void PID::setGains(Gains gains) { m_gains = gains; }
void PID::setTolerance(float tol) { m_tolerance = tol; }
