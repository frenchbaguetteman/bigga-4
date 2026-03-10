/**
 * @file bezierMotionProfiling.cpp
 * Bézier-based motion-profile construction.
 *
 * Builds a MotionProfile from a set of Bézier control points and
 * velocity constraints.
 */
#include "json/json.h"
#include "motionProfiling/profileBuilder.h"
#include "utils/utils.h"

#include <cmath>
#include <vector>

/**
 * Build a MotionProfile from an array of Bézier segments.
 *
 * @param segments         cubic Bézier segments
 * @param constraints      velocity / acceleration limits
 * @param sampleCount      number of points to sample along the spline
 * @param initialSpeed     starting velocity (m/s)
 * @param endSpeed         ending velocity  (m/s)
 */
MotionProfile buildBezierProfile(
        const std::vector<bezier::BezierSegment>& segments,
        ProfileConstraints constraints,
        int sampleCount,
        float initialSpeed,
        float endSpeed) {

    // 1. Sample the spline into a Path
    Path path;
    for (int i = 0; i <= sampleCount; ++i) {
        float t = static_cast<float>(i) / sampleCount;
        Eigen::Vector2f pt = bezier::evaluateSpline(segments, t);

        // Heading = tangent direction
        float theta = 0.0f;
        if (i < sampleCount) {
            float t2 = static_cast<float>(i + 1) / sampleCount;
            Eigen::Vector2f next = bezier::evaluateSpline(segments, t2);
            Eigen::Vector2f diff = next - pt;
            theta = std::atan2(diff.y(), diff.x());
        } else if (!path.empty()) {
            theta = path[path.size() - 1].theta;
        }

        path.addWaypoint({pt.x(), pt.y(), theta});
    }

    // 2. Build velocity profile over total arc length
    QLength totalLen(path.totalLength());
    TrapezoidalVelocityProfile velProfile(
        totalLen, constraints, QSpeed(initialSpeed), QSpeed(endSpeed));

    // 3. Assemble
    return MotionProfile(path, velProfile, sampleCount);
}

/**
 * Build a MotionProfile from a JSON asset string.
 *
 * Expected JSON format:
 * {
 *   "points": [
 *     {"x": 0.0, "y": 0.0, "theta": 0.0},
 *     ...
 *   ],
 *   "constraints": { "maxVelocity": 1.5, "maxAcceleration": 2.5 }
 * }
 */
MotionProfile buildProfileFromJson(const std::string& jsonStr) {
    JsonValue root = parseJson(jsonStr);

    // Parse waypoints
    Path path;
    if (root.has("points")) {
        for (size_t i = 0; i < root["points"].size(); ++i) {
            const auto& pt = root["points"][i];
            path.addWaypoint({
                pt["x"].asFloat(),
                pt["y"].asFloat(),
                pt.has("theta") ? pt["theta"].asFloat() : 0.0f
            });
        }
    }

    // Parse constraints
    float maxV = 1.5f, maxA = 2.5f;
    if (root.has("constraints")) {
        const auto& c = root["constraints"];
        if (c.has("maxVelocity"))     maxV = c["maxVelocity"].asFloat();
        if (c.has("maxAcceleration")) maxA = c["maxAcceleration"].asFloat();
    }

    float initSpeed = root.has("initialSpeed") ? root["initialSpeed"].asFloat() : 0.0f;
    float endSpeed  = root.has("endSpeed")     ? root["endSpeed"].asFloat()     : 0.0f;

    ProfileConstraints constraints{QSpeed(maxV), QAcceleration(maxA)};
    QLength totalLen(path.totalLength());
    TrapezoidalVelocityProfile velProfile(totalLen, constraints,
                                           QSpeed(initSpeed), QSpeed(endSpeed));

    return MotionProfile(path, velProfile);
}
