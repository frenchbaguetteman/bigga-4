/**
 * @file path.h
 * Path abstraction — a sequence of waypoints that define a spatial trajectory.
 */
#pragma once

#include "Eigen/Core"
#include <vector>
#include <cmath>

struct Waypoint {
    float x     = 0;   // metres
    float y     = 0;   // metres
    float theta = 0;   // radians (heading)
};

class Path {
public:
    Path() = default;
    explicit Path(std::vector<Waypoint> waypoints) : m_waypoints(std::move(waypoints)) {}

    /** Number of waypoints. */
    size_t size() const { return m_waypoints.size(); }
    bool   empty() const { return m_waypoints.empty(); }

    const Waypoint& operator[](size_t i) const { return m_waypoints[i]; }
    Waypoint&       operator[](size_t i)       { return m_waypoints[i]; }

    /** Append a waypoint. */
    void addWaypoint(Waypoint wp) { m_waypoints.push_back(wp); }

    /** Total arc length (piecewise linear). */
    float totalLength() const {
        float len = 0;
        for (size_t i = 1; i < m_waypoints.size(); ++i) {
            float dx = m_waypoints[i].x - m_waypoints[i - 1].x;
            float dy = m_waypoints[i].y - m_waypoints[i - 1].y;
            len += std::sqrt(dx * dx + dy * dy);
        }
        return len;
    }

    /** Interpolate position at a given fraction t ∈ [0, 1]. */
    Waypoint interpolate(float t) const {
        if (m_waypoints.empty()) return {};
        if (t <= 0.0f) return m_waypoints.front();
        if (t >= 1.0f) return m_waypoints.back();

        float targetDist = t * totalLength();
        float cumDist = 0;
        for (size_t i = 1; i < m_waypoints.size(); ++i) {
            float dx = m_waypoints[i].x - m_waypoints[i - 1].x;
            float dy = m_waypoints[i].y - m_waypoints[i - 1].y;
            float seg = std::sqrt(dx * dx + dy * dy);
            if (cumDist + seg >= targetDist && seg > 0) {
                float frac = (targetDist - cumDist) / seg;
                return {
                    m_waypoints[i - 1].x + frac * dx,
                    m_waypoints[i - 1].y + frac * dy,
                    m_waypoints[i - 1].theta + frac * (m_waypoints[i].theta - m_waypoints[i - 1].theta)
                };
            }
            cumDist += seg;
        }
        return m_waypoints.back();
    }

    const std::vector<Waypoint>& waypoints() const { return m_waypoints; }

private:
    std::vector<Waypoint> m_waypoints;
};
