/**
 * @file profileBuilder.h
 * Public builders for trapezoidal motion profiles.
 */
#pragma once

#include "Eigen/Core"
#include "motionProfiling/bezier.h"
#include "motionProfiling/motionProfile.h"
#include "motionProfiling/path.h"
#include "velocityProfile/trapezoidalVelocityProfile.hpp"

#include <initializer_list>
#include <string>
#include <vector>

/**
 * Build a trapezoidal motion profile over an existing path.
 */
MotionProfile buildProfile(const Path& path,
                           ProfileConstraints constraints,
                           int sampleCount = 200,
                           QSpeed initialSpeed = QSpeed(0),
                           QSpeed endSpeed = QSpeed(0));

/**
 * Build a trapezoidal motion profile from explicit waypoints.
 */
MotionProfile buildProfile(const std::vector<Waypoint>& waypoints,
                           ProfileConstraints constraints,
                           int sampleCount = 200,
                           QSpeed initialSpeed = QSpeed(0),
                           QSpeed endSpeed = QSpeed(0));

/**
 * Build a trapezoidal motion profile from `{x, y, theta}` waypoints in metres/radians.
 */
MotionProfile buildProfile(const std::vector<Eigen::Vector3f>& waypoints,
                           float maxVelocityMps,
                           float maxAccelerationMps2,
                           int sampleCount = 200,
                           float initialSpeedMps = 0.0f,
                           float endSpeedMps = 0.0f);

/**
 * Convenience overload for inline waypoint lists.
 */
MotionProfile buildProfile(std::initializer_list<Eigen::Vector3f> waypoints,
                           float maxVelocityMps,
                           float maxAccelerationMps2,
                           int sampleCount = 200,
                           float initialSpeedMps = 0.0f,
                           float endSpeedMps = 0.0f);

/**
 * Build a motion profile from cubic Bezier segments with trapezoidal speed limits.
 */
MotionProfile buildBezierProfile(const std::vector<bezier::BezierSegment>& segments,
                                 ProfileConstraints constraints,
                                 int sampleCount = 200,
                                 float initialSpeed = 0.0f,
                                 float endSpeed = 0.0f);

/**
 * Build a motion profile from a JSON asset string.
 */
MotionProfile buildProfileFromJson(const std::string& jsonStr);
