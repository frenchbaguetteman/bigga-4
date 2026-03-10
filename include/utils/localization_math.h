/**
 * @file localization_math.h
 * Finite-value guards, angle utilities, conversion constants, and shared
 * helper functions used across the localization, fusion, and UI stacks.
 *
 * All functions live in the LocMath namespace so call-sites are explicit.
 */
#pragma once

#include "Eigen/Core"
#include "config.h"
#include <cmath>

namespace LocMath {

/// Unit conversion
constexpr float IN_TO_M = 0.0254f;
constexpr float M_TO_IN = 1.0f / IN_TO_M;
constexpr float PI_F = 3.14159265358979323846f;

/// Likelihood floor — prevents weight collapse in particle filter
constexpr float LIKELIHOOD_EPS = 1e-10f;

/// Maximum plausible GPS coordinate magnitude (metres from field centre)
constexpr float GPS_ABSURD_LIMIT_M = 5.0f;

// ── Finite-value guards ─────────────────────────────────────────────────────

inline bool isFinite(float v) {
    return std::isfinite(v);
}

inline bool isFiniteVec2(const Eigen::Vector2f& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y());
}

inline bool isFinitePose(const Eigen::Vector3f& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

inline Eigen::Vector3f finitePoseOr(const Eigen::Vector3f& preferred,
                                    const Eigen::Vector3f& fallback) {
    if (isFinitePose(preferred)) return preferred;
    if (isFinitePose(fallback)) return fallback;
    return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
}

// ── Angle utilities ─────────────────────────────────────────────────────────

/// Wrap angle to (−π, π]
inline float wrapAngle(float a) {
    return std::atan2(std::sin(a), std::cos(a));
}

/// Radians → degrees
inline float radToDeg(float r) {
    return r * 180.0f / PI_F;
}

/// Degrees → radians
inline float degToRad(float d) {
    return d * PI_F / 180.0f;
}

/// Wrap degrees to (−180, 180]
inline float wrapDeg(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

// ── Unit conversion helpers ─────────────────────────────────────────────────

/// Metres → inches
inline float mToIn(float m) {
    return m * M_TO_IN;
}

/// Inches → metres
inline float inToM(float in) {
    return in * IN_TO_M;
}

// ── Heading conversion helpers ──────────────────────────────────────────────

/// Internal heading (radians) → VEX compass degrees (0° = north, CW+)
inline float headingToCompassDeg(float headingRad) {
    return CONFIG::internalRadToGpsHeadingDeg(headingRad);
}

/// Compute signed compass-degree difference from a heading-delta in radians.
/// Convention: positive = CW heading change.
inline float headingDeltaCompassDeg(float deltaRad) {
    return wrapDeg(-radToDeg(deltaRad));
}

// ── Vector math helpers ─────────────────────────────────────────────────────

/// Clamp a 2-D vector's magnitude to at most maxMagnitude.
inline Eigen::Vector2f clampVectorMagnitude(const Eigen::Vector2f& v,
                                            float maxMagnitude) {
    if (!isFiniteVec2(v) || maxMagnitude <= 0.0f) {
        return Eigen::Vector2f(0.0f, 0.0f);
    }
    const float norm = v.norm();
    if (norm <= maxMagnitude || norm <= 1e-6f) return v;
    return v * (maxMagnitude / norm);
}

/// Move `current` towards `target` by at most `maxStep` per call.
inline Eigen::Vector2f moveTowardsVec2(const Eigen::Vector2f& current,
                                       const Eigen::Vector2f& target,
                                       float maxStep) {
    if (!isFiniteVec2(current) || !isFiniteVec2(target) || maxStep <= 0.0f) {
        return current;
    }
    const Eigen::Vector2f delta = target - current;
    const float dist = delta.norm();
    if (dist <= maxStep || dist <= 1e-6f) return target;
    return current + delta * (maxStep / dist);
}

/// Return `target` unless the delta from `current` is within `deadband`.
inline Eigen::Vector2f applyTargetDeadband(const Eigen::Vector2f& current,
                                           const Eigen::Vector2f& target,
                                           float deadband) {
    if (!isFiniteVec2(current) || !isFiniteVec2(target) || deadband <= 0.0f) {
        return target;
    }
    if ((target - current).norm() <= deadband) {
        return current;
    }
    return target;
}

} // namespace LocMath
