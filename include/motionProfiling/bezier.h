/**
 * @file bezier.h
 * Cubic Bézier spline helpers for path generation.
 */
#pragma once

#include "Eigen/Core"
#include <vector>
#include <cmath>

namespace bezier {

/** Evaluate a cubic Bézier at parameter t ∈ [0, 1]. */
inline Eigen::Vector2f cubicBezier(
        const Eigen::Vector2f& p0, const Eigen::Vector2f& p1,
        const Eigen::Vector2f& p2, const Eigen::Vector2f& p3,
        float t) {
    float u  = 1.0f - t;
    float u2 = u * u;
    float u3 = u2 * u;
    float t2 = t * t;
    float t3 = t2 * t;
    return u3 * p0 + 3.0f * u2 * t * p1 + 3.0f * u * t2 * p2 + t3 * p3;
}

/** First derivative of a cubic Bézier at t. */
inline Eigen::Vector2f cubicBezierDerivative(
        const Eigen::Vector2f& p0, const Eigen::Vector2f& p1,
        const Eigen::Vector2f& p2, const Eigen::Vector2f& p3,
        float t) {
    float u = 1.0f - t;
    return 3.0f * u * u * (p1 - p0)
         + 6.0f * u * t * (p2 - p1)
         + 3.0f * t * t * (p3 - p2);
}

/** Second derivative of a cubic Bézier at t. */
inline Eigen::Vector2f cubicBezierSecondDerivative(
        const Eigen::Vector2f& p0, const Eigen::Vector2f& p1,
        const Eigen::Vector2f& p2, const Eigen::Vector2f& p3,
        float t) {
    float u = 1.0f - t;
    return 6.0f * u * (p2 - 2.0f * p1 + p0)
         + 6.0f * t * (p3 - 2.0f * p2 + p1);
}

/** Signed curvature of a cubic Bézier at t. */
inline float cubicBezierCurvature(
        const Eigen::Vector2f& p0, const Eigen::Vector2f& p1,
        const Eigen::Vector2f& p2, const Eigen::Vector2f& p3,
        float t) {
    Eigen::Vector2f d1 = cubicBezierDerivative(p0, p1, p2, p3, t);
    Eigen::Vector2f d2 = cubicBezierSecondDerivative(p0, p1, p2, p3, t);
    float cross = d1.x() * d2.y() - d1.y() * d2.x();
    float normCubed = std::pow(d1.norm(), 3.0f);
    return (normCubed > 1e-9f) ? (cross / normCubed) : 0.0f;
}

/** Approximate arc length of a cubic Bézier using N linear segments. */
inline float cubicBezierArcLength(
        const Eigen::Vector2f& p0, const Eigen::Vector2f& p1,
        const Eigen::Vector2f& p2, const Eigen::Vector2f& p3,
        int N = 64) {
    float len = 0;
    Eigen::Vector2f prev = p0;
    for (int i = 1; i <= N; ++i) {
        float t = static_cast<float>(i) / N;
        Eigen::Vector2f pt = cubicBezier(p0, p1, p2, p3, t);
        len += (pt - prev).norm();
        prev = pt;
    }
    return len;
}

/**
 * A Bézier spline: a sequence of cubic Bézier segments joined end-to-end.
 * Each segment is defined by four control points.
 */
struct BezierSegment {
    Eigen::Vector2f p0, p1, p2, p3;
};

/** Evaluate a composite spline at arc-length fraction t ∈ [0, 1]. */
inline Eigen::Vector2f evaluateSpline(
        const std::vector<BezierSegment>& segments, float t) {
    if (segments.empty()) return Eigen::Vector2f(0, 0);
    // Uniform-parameter approximation (not arc-length parametrised)
    float total = static_cast<float>(segments.size());
    float scaled = t * total;
    int idx = static_cast<int>(scaled);
    if (idx >= static_cast<int>(segments.size())) idx = static_cast<int>(segments.size()) - 1;
    float local = scaled - idx;
    const auto& s = segments[idx];
    return cubicBezier(s.p0, s.p1, s.p2, s.p3, local);
}

} // namespace bezier
