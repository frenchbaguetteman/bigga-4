/**
 * @file profileBuilder.cpp
 * Public trapezoidal motion-profile builders.
 */
#include "motionProfiling/profileBuilder.h"

#include <algorithm>
#include <cmath>

namespace {

std::vector<Waypoint> vector3fToWaypoints(const std::vector<Eigen::Vector3f>& waypoints) {
    std::vector<Waypoint> pathPoints;
    pathPoints.reserve(waypoints.size());

    for (const auto& waypoint : waypoints) {
        pathPoints.push_back({waypoint.x(), waypoint.y(), waypoint.z()});
    }

    return pathPoints;
}

} // namespace

MotionProfile buildProfile(const Path& path,
                           ProfileConstraints constraints,
                           int sampleCount,
                           QSpeed initialSpeed,
                           QSpeed endSpeed) {
    const ProfileConstraints sanitizedConstraints{
        QSpeed(std::fabs(constraints.maxVelocity.convert(mps))),
        QAcceleration(std::fabs(constraints.maxAcceleration.convert(mps2))),
    };

    const TrapezoidalVelocityProfile velocityProfile(
        QLength(path.totalLength()),
        sanitizedConstraints,
        QSpeed(std::fabs(initialSpeed.convert(mps))),
        QSpeed(std::fabs(endSpeed.convert(mps))));

    return MotionProfile(path, velocityProfile, std::max(1, sampleCount));
}

MotionProfile buildProfile(const std::vector<Waypoint>& waypoints,
                           ProfileConstraints constraints,
                           int sampleCount,
                           QSpeed initialSpeed,
                           QSpeed endSpeed) {
    return buildProfile(
        Path(waypoints),
        constraints,
        sampleCount,
        initialSpeed,
        endSpeed);
}

MotionProfile buildProfile(const std::vector<Eigen::Vector3f>& waypoints,
                           float maxVelocityMps,
                           float maxAccelerationMps2,
                           int sampleCount,
                           float initialSpeedMps,
                           float endSpeedMps) {
    return buildProfile(
        vector3fToWaypoints(waypoints),
        ProfileConstraints{
            QSpeed(std::fabs(maxVelocityMps)),
            QAcceleration(std::fabs(maxAccelerationMps2)),
        },
        sampleCount,
        QSpeed(std::fabs(initialSpeedMps)),
        QSpeed(std::fabs(endSpeedMps)));
}

MotionProfile buildProfile(std::initializer_list<Eigen::Vector3f> waypoints,
                           float maxVelocityMps,
                           float maxAccelerationMps2,
                           int sampleCount,
                           float initialSpeedMps,
                           float endSpeedMps) {
    return buildProfile(
        std::vector<Eigen::Vector3f>(waypoints),
        maxVelocityMps,
        maxAccelerationMps2,
        sampleCount,
        initialSpeedMps,
        endSpeedMps);
}
