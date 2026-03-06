/**
 * @file gps.h
 * VEX GPS sensor likelihood model.
 *
 * Evaluates a 2-D Gaussian on (x,y) between each particle's robot-center
 * and a robot-center estimate inferred from the raw GPS sensor position plus
 * configured GPS mount offset.
 *
 * GPS readings are converted from VEX convention (x, y in metres; heading in
 * compass degrees with 0° = north, CW positive) to internal convention
 * (x, y in metres; heading in radians with 0° = east/forward, CCW positive).
 */
#pragma once

#include "localization/sensor.h"
#include "config.h"
#include "utils/localization_math.h"
#include "Eigen/Core"
#include "pros/gps.hpp"
#include <cmath>
#include <limits>
#include <optional>

namespace GpsLocalization {

struct FieldReading {
    Eigen::Vector2f sensorFieldPos{0.0f, 0.0f};
    float robotHeadingRad = 0.0f;
    float errorM = -1.0f;
    bool hasHeading = false;
};

inline bool isPlausibleFieldPosition(const Eigen::Vector2f& position) {
    return LocMath::isFiniteVec2(position) &&
        std::fabs(position.x()) <= LocMath::GPS_ABSURD_LIMIT_M &&
        std::fabs(position.y()) <= LocMath::GPS_ABSURD_LIMIT_M;
}

inline std::optional<FieldReading> readFieldReading(pros::Gps& gps,
                                                    float headingOffsetDeg = 0.0f,
                                                    bool requireHeading = true) {
    auto pos = gps.get_position();
    const Eigen::Vector2f rawGps(
        static_cast<float>(pos.x),
        static_cast<float>(pos.y));
    const Eigen::Vector2f sensorFieldPos = CONFIG::transformGpsToFieldFrame(rawGps);
    if (!isPlausibleFieldPosition(sensorFieldPos)) {
        return std::nullopt;
    }

    const float gpsErrorM = static_cast<float>(gps.get_error());
    if (!LocMath::isFinite(gpsErrorM) || gpsErrorM < 0.0f) {
        return std::nullopt;
    }

    FieldReading reading;
    reading.sensorFieldPos = sensorFieldPos;
    reading.errorM = gpsErrorM;

    if (!requireHeading) {
        return reading;
    }

    const float rawHeadingDeg = static_cast<float>(gps.get_heading());
    if (!LocMath::isFinite(rawHeadingDeg)) {
        return std::nullopt;
    }

    const float robotHeading = CONFIG::gpsSensorHeadingDegToInternalRad(
        rawHeadingDeg - headingOffsetDeg);
    if (!LocMath::isFinite(robotHeading)) {
        return std::nullopt;
    }

    reading.robotHeadingRad = robotHeading;
    reading.hasHeading = true;
    return reading;
}

inline std::optional<Eigen::Vector2f> robotCenterFromSensorPose(const Eigen::Vector2f& sensorFieldPos,
                                                                float headingRad,
                                                                const Eigen::Vector2f& offset) {
    if (!LocMath::isFiniteVec2(sensorFieldPos) || !LocMath::isFinite(headingRad)) {
        return std::nullopt;
    }

    const float cosT = std::cos(headingRad);
    const float sinT = std::sin(headingRad);
    const Eigen::Vector2f offsetWorld(
        offset.x() * cosT - offset.y() * sinT,
        offset.x() * sinT + offset.y() * cosT);
    const Eigen::Vector2f robotCenter = sensorFieldPos - offsetWorld;
    if (!LocMath::isFiniteVec2(robotCenter)) {
        return std::nullopt;
    }

    return robotCenter;
}

inline std::optional<Eigen::Vector3f> robotPoseFromSensor(pros::Gps& gps,
                                                          float headingOffsetDeg,
                                                          const Eigen::Vector2f& offset,
                                                          float maxErrorM = std::numeric_limits<float>::infinity()) {
    const std::optional<FieldReading> reading = readFieldReading(gps, headingOffsetDeg, true);
    if (!reading || reading->errorM > maxErrorM) {
        return std::nullopt;
    }

    const std::optional<Eigen::Vector2f> robotCenter = robotCenterFromSensorPose(
        reading->sensorFieldPos,
        reading->robotHeadingRad,
        offset);
    if (!robotCenter) {
        return std::nullopt;
    }

    return Eigen::Vector3f(robotCenter->x(), robotCenter->y(), reading->robotHeadingRad);
}

} // namespace GpsLocalization

class GpsSensorModel : public SensorModel {
public:
    /**
     * @param port              PROS smart-port
     * @param headingOffsetDeg  GPS mounting heading offset (compass degrees, VEX convention)
     * @param offsetX_m         GPS mount offset X in INTERNAL robot frame (metres, +fwd)
     * @param offsetY_m         GPS mount offset Y in INTERNAL robot frame (metres, +left)
     * @param stddev            measurement noise stddev (metres)
     */
    explicit GpsSensorModel(int port,
                            float headingOffsetDeg = 0.0f,
                            float offsetX_m = 0.0f,
                            float offsetY_m = 0.0f,
                            float stddev = CONFIG::GPS_STDDEV_BASE.convert(meter))
        : m_gps(port)
        , m_headingOffsetDeg(headingOffsetDeg)
        , m_offset(offsetX_m, offsetY_m)
        , m_stddev(std::max(stddev, 0.005f)) {}

    void update() override {
        const std::optional<GpsLocalization::FieldReading> reading =
            GpsLocalization::readFieldReading(m_gps, m_headingOffsetDeg, true);
        if (!reading) {
            m_robotCenter = std::nullopt;
            m_lastError = -1.0f;
            return;
        }
        m_lastError = reading->errorM;

        // If GPS error exceeds threshold, skip this update (don't trust it)
        if (m_lastError > CONFIG::GPS_ERROR_THRESHOLD.convert(meter)) {
            m_robotCenter = std::nullopt;
            return;
        }

        m_robotCenter = GpsLocalization::robotCenterFromSensorPose(
            reading->sensorFieldPos,
            reading->robotHeadingRad,
            m_offset);
    }

    bool hasObservation() const override { return m_robotCenter.has_value(); }
    bool isAbsolutePositionSensor() const override { return true; }

    std::optional<float> p(const Eigen::Vector3f& particle) override {
        if (!m_robotCenter) return std::nullopt;

        // Adaptively adjust stddev based on GPS error
        float currentStddev = m_stddev;
        if (m_lastError >= 0.0f) {
            if (m_lastError > CONFIG::GPS_ERROR_GOOD.convert(meter)) {
                // GPS error is higher than ideal; inflate stddev to reduce GPS influence
                float excessError = m_lastError - CONFIG::GPS_ERROR_GOOD.convert(meter);
                float inflationFactor = 1.0f + CONFIG::GPS_ERROR_SCALE_MULTIPLIER * excessError;
                currentStddev = m_stddev * inflationFactor;
                // Clamp to configured limits
                currentStddev = std::max(CONFIG::GPS_STDDEV_MIN.convert(meter),
                                       std::min(CONFIG::GPS_STDDEV_MAX.convert(meter), currentStddev));
            }
        }
        currentStddev = std::max(currentStddev, 0.005f);

        float dx = particle.x() - m_robotCenter->x();
        float dy = particle.y() - m_robotCenter->y();
        float distSq = dx * dx + dy * dy;
        return std::exp(-distSq / (2.0f * currentStddev * currentStddev));
    }

    /** Get last GPS error reading in metres (-1 if never valid). */
    float getLastError() const { return m_lastError; }

private:
    pros::Gps m_gps;
    float m_headingOffsetDeg;
    Eigen::Vector2f m_offset;
    float m_stddev;
    std::optional<Eigen::Vector2f> m_robotCenter;
    float m_lastError = -1.0f;  // last GPS error reading (-1 = invalid)
};
