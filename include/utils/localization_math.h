/**
 * @file localization_math.h
 * Finite-value guards, angle utilities, and conversion constants
 * shared across the localization stack.
 */
#pragma once

#include "Eigen/Core"
#include <cmath>

namespace LocMath {

/// Unit conversion
constexpr float IN_TO_M = 0.0254f;

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

// ── Angle utilities ─────────────────────────────────────────────────────────

/// Wrap angle to (−π, π]
inline float wrapAngle(float a) {
    return std::atan2(std::sin(a), std::cos(a));
}

} // namespace LocMath
