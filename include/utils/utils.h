/**
 * @file utils.h
 * Math and general utility helpers used throughout the project.
 */
#pragma once

#include <cmath>
#include <algorithm>
#include "units/units.hpp"

namespace utils {

// ── Angle helpers ───────────────────────────────────────────────────────────

/** Wrap angle to (-π, π]. */
inline float angleWrap(float angle) {
    angle = std::fmod(angle, 2.0f * static_cast<float>(M_PI));
    if (angle > static_cast<float>(M_PI))  angle -= 2.0f * static_cast<float>(M_PI);
    if (angle <= -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

/** Signed shortest-path difference: target − current, wrapped to (-π, π]. */
inline float angleDifference(float target, float current) {
    return angleWrap(target - current);
}

/** QAngle overload. */
inline QAngle angleDifference(QAngle target, QAngle current) {
    return QAngle(angleDifference(target.convert(radian), current.convert(radian)));
}

// ── Numeric helpers ─────────────────────────────────────────────────────────

/** sinc(x) = sin(x)/x, with sinc(0) = 1. */
inline float sinc(float x) {
    if (std::fabs(x) < 1e-6f) return 1.0f;
    return std::sin(x) / x;
}

/** signum: returns −1, 0, or +1. */
inline float sgn(float x) {
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

/** Clamp value to [lo, hi]. */
inline float clamp(float v, float lo, float hi) {
    return std::max(lo, std::min(v, hi));
}

/** Linear interpolation. */
inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

/** Map a value from one range to another. */
inline float map(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
    return toLow + (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow);
}

/** Deadband: returns 0 if |x| < threshold, else x. */
inline float deadband(float x, float threshold) {
    return (std::fabs(x) < threshold) ? 0.0f : x;
}

/** Convert RPM to m/s given wheel radius (metres). */
inline float rpmToMps(float rpm, float radius) {
    return rpm * 2.0f * static_cast<float>(M_PI) * radius / 60.0f;
}

/** Convert m/s to RPM given wheel radius (metres). */
inline float mpsToRpm(float mps, float radius) {
    return mps * 60.0f / (2.0f * static_cast<float>(M_PI) * radius);
}

} // namespace utils
