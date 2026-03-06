/**
 * @file trapezoidalVelocityProfile.hpp
 * Simple trapezoidal velocity profile with precomputed segment times.
 *
 * Three phases: accelerate → cruise → decelerate.
 * If the distance is too short for full cruise, becomes triangular.
 *
 * Stores ta (accel time), ts (cruise time), td (decel time),
 * cruiseSpeed, and segment accelerations aa / ad.
 */
#pragma once

#include "units/units.hpp"
#include <cmath>
#include <algorithm>

struct ProfileConstraints {
    QSpeed         maxVelocity;
    QAcceleration  maxAcceleration;
};

class TrapezoidalVelocityProfile {
public:
    TrapezoidalVelocityProfile() = default;

    TrapezoidalVelocityProfile(QLength distance,
                                ProfileConstraints constraints,
                                QSpeed initialSpeed = QSpeed(0),
                                QSpeed endSpeed     = QSpeed(0))
        : m_distance(distance), m_constraints(constraints)
        , m_initialSpeed(initialSpeed), m_endSpeed(endSpeed)
    {
        calculate();
    }

    // ── Query ───────────────────────────────────────────────────────────

    /** Distance travelled at time t. */
    QLength distanceAt(QTime t) const {
        float ts = t.convert(second);
        if (ts < 0.0f) return QLength(0);

        // Phase 1: acceleration
        if (ts <= m_ta) {
            float v0 = m_initialSpeed.convert(mps);
            return QLength(v0 * ts + 0.5f * m_aa * ts * ts);
        }
        QLength d1(m_initialSpeed.convert(mps) * m_ta + 0.5f * m_aa * m_ta * m_ta);

        // Phase 2: cruise
        float t2 = ts - m_ta;
        if (t2 <= m_ts) {
            return d1 + QLength(m_cruiseSpeed * t2);
        }
        QLength d2 = d1 + QLength(m_cruiseSpeed * m_ts);

        // Phase 3: deceleration
        float t3 = t2 - m_ts;
        t3 = std::min(t3, m_td);
        return d2 + QLength(m_cruiseSpeed * t3 + 0.5f * m_ad * t3 * t3);
    }

    /** Velocity at time t. */
    QSpeed velocityAt(QTime t) const {
        float ts = t.convert(second);
        if (ts <= 0.0f)    return m_initialSpeed;
        if (ts <= m_ta)    return QSpeed(m_initialSpeed.convert(mps) + m_aa * ts);
        if (ts <= m_ta + m_ts) return QSpeed(m_cruiseSpeed);
        float t3 = ts - m_ta - m_ts;
        if (t3 >= m_td)    return m_endSpeed;
        return QSpeed(m_cruiseSpeed + m_ad * t3);
    }

    /** Acceleration at time t. */
    QAcceleration accelerationAt(QTime t) const {
        float ts = t.convert(second);
        if (ts < m_ta)          return QAcceleration(m_aa);
        if (ts < m_ta + m_ts)   return QAcceleration(0);
        if (ts < m_ta + m_ts + m_td) return QAcceleration(m_ad);
        return QAcceleration(0);
    }

    /** Total profile duration. */
    QTime totalTime() const { return QTime(m_ta + m_ts + m_td); }

    /** Is time t past the profile end? */
    bool isFinished(QTime t) const { return t >= totalTime(); }

    /** Total distance (should match input). */
    QLength totalDistance() const { return m_distance; }

private:
    QLength      m_distance{};
    ProfileConstraints m_constraints{};
    QSpeed       m_initialSpeed{};
    QSpeed       m_endSpeed{};

    float m_ta = 0;            // accel time
    float m_ts = 0;            // cruise time
    float m_td = 0;            // decel time
    float m_cruiseSpeed = 0;   // m/s
    float m_aa = 0;            // accel (m/s²)
    float m_ad = 0;            // decel (m/s², negative)

    void calculate() {
        float d   = std::fabs(m_distance.convert(meter));
        float v0  = m_initialSpeed.convert(mps);
        float v1  = m_endSpeed.convert(mps);
        float vMax = m_constraints.maxVelocity.convert(mps);
        float aMax = m_constraints.maxAcceleration.convert(mps2);

        if (aMax <= 0.0f || vMax <= 0.0f) return;

        // Time to ramp up / down
        m_ta = (vMax - v0) / aMax;
        m_td = (vMax - v1) / aMax;

        // Distance consumed by accel + decel ramps
        float dAccel = (v0 + vMax) * m_ta / 2.0f;
        float dDecel = (v1 + vMax) * m_td / 2.0f;
        float dCruise = d - dAccel - dDecel;

        if (dCruise < 0.0f) {
            // Triangular profile — solve for reachable peak speed
            // v_c² = v0² + 2*a*(d * a + (v0²-v1²)/2) / (2*a)
            // Simplified: v_c = sqrt( (2*a*d + v0² + v1²) / 2 )
            float vcSq = (2.0f * aMax * d + v0 * v0 + v1 * v1) / 2.0f;
            if (vcSq < 0.0f) vcSq = 0.0f;
            float vc = std::sqrt(vcSq);
            vc = std::min(vc, vMax);

            m_ta = (vc - v0) / aMax;
            m_td = (vc - v1) / aMax;
            m_ts = 0.0f;
            m_cruiseSpeed = vc;
        } else {
            m_ts = dCruise / vMax;
            m_cruiseSpeed = vMax;
        }

        m_aa =  aMax;          // always positive during accel
        m_ad = -aMax;          // always negative during decel

        // Clamp negative times
        if (m_ta < 0) m_ta = 0;
        if (m_ts < 0) m_ts = 0;
        if (m_td < 0) m_td = 0;
    }
};
