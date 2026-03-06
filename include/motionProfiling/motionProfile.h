/**
 * @file motionProfile.h
 * MotionProfile — combines a spatial path with a velocity profile to produce
 * time-parameterised trajectory states (pose + velocities).
 *
 * Used by the RAMSETE and LTV tracking commands.
 */
#pragma once

#include "Eigen/Core"
#include "motionProfiling/path.h"
#include "velocityProfile/trapezoidalVelocityProfile.hpp"
#include "units/units.hpp"
#include "utils/utils.h"
#include <cmath>

struct ProfileState {
    Eigen::Vector3f pose{0, 0, 0};      // x, y, θ
    float linearVelocity     = 0.0f;    // m/s
    float angularVelocity    = 0.0f;    // rad/s
    float linearAcceleration = 0.0f;    // m/s²
};

class MotionProfile {
public:
    MotionProfile() = default;

    /**
     * Construct from a path + velocity profile.
     * Pre-samples the path to build a look-up table of (time → pose + vel).
     */
    MotionProfile(const Path& path, const TrapezoidalVelocityProfile& velProfile,
                  int sampleCount = 200)
        : m_path(path), m_velProfile(velProfile)
    {
        float totalT = velProfile.totalTime().convert(second);
        float dt = totalT / sampleCount;
        float totalLen = path.totalLength();

        for (int i = 0; i <= sampleCount; ++i) {
            float t = i * dt;
            QTime qt(t);
            float dist = velProfile.distanceAt(qt).convert(meter);
            float frac = (totalLen > 0) ? dist / totalLen : 0.0f;
            frac = std::clamp(frac, 0.0f, 1.0f);

            Waypoint wp = path.interpolate(frac);
            float vel   = velProfile.velocityAt(qt).convert(mps);
            float accel = velProfile.accelerationAt(qt).convert(mps2);

            ProfileState s;
            s.pose = Eigen::Vector3f(wp.x, wp.y, wp.theta);
            s.linearVelocity     = vel;
            s.linearAcceleration = accel;

            // Estimate angular velocity from heading change
            if (i > 0) {
                float dTheta = utils::angleDifference(wp.theta, m_samples.back().pose.z());
                s.angularVelocity = (dt > 0) ? dTheta / dt : 0.0f;
            }

            m_samples.push_back(s);
        }
        m_totalTime = totalT;
    }

    /** Sample the profile at time t (seconds). Linear interpolation. */
    ProfileState sample(float t) const {
        if (m_samples.empty()) return {};
        if (t <= 0) return m_samples.front();
        if (t >= m_totalTime) return m_samples.back();

        float frac = t / m_totalTime * (m_samples.size() - 1);
        int idx = static_cast<int>(frac);
        float alpha = frac - idx;

        if (idx + 1 >= static_cast<int>(m_samples.size())) return m_samples.back();

        const auto& a = m_samples[idx];
        const auto& b = m_samples[idx + 1];

        ProfileState result;
        result.pose = a.pose + (b.pose - a.pose) * alpha;
        result.pose.z() = a.pose.z() + utils::angleDifference(b.pose.z(), a.pose.z()) * alpha;
        result.linearVelocity     = a.linearVelocity     + alpha * (b.linearVelocity     - a.linearVelocity);
        result.angularVelocity    = a.angularVelocity    + alpha * (b.angularVelocity    - a.angularVelocity);
        result.linearAcceleration = a.linearAcceleration  + alpha * (b.linearAcceleration - a.linearAcceleration);
        return result;
    }

    float totalTime() const { return m_totalTime; }
    bool  isFinished(float t) const { return t >= m_totalTime; }

    const std::vector<ProfileState>& samples() const { return m_samples; }

private:
    Path m_path;
    TrapezoidalVelocityProfile m_velProfile;
    std::vector<ProfileState> m_samples;
    float m_totalTime = 0;
};
