/**
 * @file trapProfile.h
 * General trapezoidal motion-profile solver with direction handling,
 * velocity clamping, and support for arbitrary initial/goal states.
 *
 * Modelled after WPILib's TrapezoidProfile — solves for the state
 * (position, velocity) at time t given constraints and initial/goal states.
 */
#pragma once

#include "units/units.hpp"
#include <cmath>
#include <algorithm>

class TrapProfile {
public:
    struct Constraints {
        float maxVelocity     = 0.0f;   // m/s  (positive)
        float maxAcceleration = 0.0f;   // m/s² (positive)
    };

    struct State {
        float position = 0.0f;   // metres
        float velocity = 0.0f;   // m/s
    };

    TrapProfile() = default;
    explicit TrapProfile(Constraints c) : m_constraints(c) {}

    /**
     * Calculate the profile state at time t, given initial and goal states.
     */
    State calculate(float t, State initial, State goal) const {
        float direction = shouldFlipAcceleration(initial, goal) ? -1.0f : 1.0f;

        // Work in "positive direction" frame
        State ini = direct(initial, direction);
        State gol = direct(goal, direction);

        // Clamp current velocity to max
        ini.velocity = std::min(ini.velocity, m_constraints.maxVelocity);
        gol.velocity = std::min(gol.velocity, m_constraints.maxVelocity);

        float cutoffBegin = ini.velocity / m_constraints.maxAcceleration;
        float cutoffEnd   = gol.velocity / m_constraints.maxAcceleration;

        // Distance to reach zero velocity from initial / goal speed
        float cutoffDistBegin = cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0f;
        float cutoffDistEnd   = cutoffEnd   * cutoffEnd   * m_constraints.maxAcceleration / 2.0f;

        float fullTrapezoidDist = cutoffDistBegin + (gol.position - ini.position) + cutoffDistEnd;
        float accelerationTime  = m_constraints.maxVelocity / m_constraints.maxAcceleration;
        float fullSpeedDist     = fullTrapezoidDist - accelerationTime * accelerationTime * m_constraints.maxAcceleration;

        float accelerationDt, decelerationDt, cruiseDt;

        if (fullSpeedDist < 0.0f) {
            // Triangle profile
            accelerationDt = std::sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
            decelerationDt = accelerationDt;
            cruiseDt = 0.0f;
        } else {
            accelerationDt = accelerationTime;
            decelerationDt = accelerationTime;
            cruiseDt = fullSpeedDist / m_constraints.maxVelocity;
        }

        accelerationDt -= cutoffBegin;
        decelerationDt -= cutoffEnd;

        float totalDt = accelerationDt + cruiseDt + decelerationDt;
        t = std::clamp(t, 0.0f, totalDt);

        State result;

        if (t < accelerationDt) {
            // Accelerating
            result.velocity = ini.velocity + t * m_constraints.maxAcceleration;
            result.position = ini.position + (ini.velocity + t * m_constraints.maxAcceleration / 2.0f) * t;
        } else if (t < accelerationDt + cruiseDt) {
            // Cruising
            float cruiseVel = ini.velocity + accelerationDt * m_constraints.maxAcceleration;
            result.velocity = cruiseVel;
            result.position = ini.position
                + (ini.velocity + accelerationDt * m_constraints.maxAcceleration / 2.0f) * accelerationDt
                + cruiseVel * (t - accelerationDt);
        } else {
            // Decelerating
            float cruiseVel = ini.velocity + accelerationDt * m_constraints.maxAcceleration;
            float dt = t - accelerationDt - cruiseDt;
            result.velocity = cruiseVel - dt * m_constraints.maxAcceleration;
            result.position = ini.position
                + (ini.velocity + accelerationDt * m_constraints.maxAcceleration / 2.0f) * accelerationDt
                + cruiseVel * cruiseDt
                + (cruiseVel - dt * m_constraints.maxAcceleration / 2.0f) * dt;
        }

        // Un-flip direction
        result.position *= direction;
        result.velocity *= direction;

        return result;
    }

    /** Total duration to go from initial to goal. */
    float totalTime(State initial, State goal) const {
        float direction = shouldFlipAcceleration(initial, goal) ? -1.0f : 1.0f;
        State ini = direct(initial, direction);
        State gol = direct(goal, direction);
        ini.velocity = std::min(ini.velocity, m_constraints.maxVelocity);
        gol.velocity = std::min(gol.velocity, m_constraints.maxVelocity);

        float cutoffBegin = ini.velocity / m_constraints.maxAcceleration;
        float cutoffEnd   = gol.velocity / m_constraints.maxAcceleration;
        float cutoffDistBegin = cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0f;
        float cutoffDistEnd   = cutoffEnd   * cutoffEnd   * m_constraints.maxAcceleration / 2.0f;
        float fullTrapezoidDist = cutoffDistBegin + (gol.position - ini.position) + cutoffDistEnd;
        float accelerationTime  = m_constraints.maxVelocity / m_constraints.maxAcceleration;
        float fullSpeedDist     = fullTrapezoidDist - accelerationTime * accelerationTime * m_constraints.maxAcceleration;

        float accelDt, decelDt, cruiseDt;
        if (fullSpeedDist < 0.0f) {
            accelDt = std::sqrt(fullTrapezoidDist / m_constraints.maxAcceleration) - cutoffBegin;
            decelDt = accelDt;
            cruiseDt = 0.0f;
        } else {
            accelDt = accelerationTime - cutoffBegin;
            decelDt = accelerationTime - cutoffEnd;
            cruiseDt = fullSpeedDist / m_constraints.maxVelocity;
        }
        return std::max(0.0f, accelDt + cruiseDt + decelDt);
    }

private:
    Constraints m_constraints;

    bool shouldFlipAcceleration(State initial, State goal) const {
        return initial.position > goal.position;
    }

    static State direct(State s, float direction) {
        return {s.position * direction, s.velocity * direction};
    }
};
