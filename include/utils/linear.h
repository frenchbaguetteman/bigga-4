/**
 * @file linear.h
 * Pose type alias and small geometric utilities.
 */
#pragma once

#include "Eigen/Dense"

// A pose is stored as (x, y, θ) in an Eigen::Vector3f.
using Pose = Eigen::Vector3f;

namespace linear {

/** Build a Pose from x, y, θ. */
inline Pose makePose(float x, float y, float theta) {
    return Pose(x, y, theta);
}

/** 2-D Euclidean distance between two poses (ignores heading). */
inline float poseDistance(const Pose& a, const Pose& b) {
    return (a.head<2>() - b.head<2>()).norm();
}

/** Rotate a 2-D vector by an angle. */
inline Eigen::Vector2f rotate2D(const Eigen::Vector2f& v, float angle) {
    return Eigen::Rotation2Df(angle) * v;
}

} // namespace linear
