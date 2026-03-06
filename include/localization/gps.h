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
#include <optional>

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
                            float stddev = 0.05f)
        : m_gps(port)
        , m_headingOffsetDeg(headingOffsetDeg)
        , m_offset(offsetX_m, offsetY_m)
        , m_stddev(stddev) {}

    void update() override {
        auto pos = m_gps.get_position();
        float sx = static_cast<float>(pos.x);
        float sy = static_cast<float>(pos.y);

        // Reject non-finite readings
        if (!LocMath::isFinite(sx) || !LocMath::isFinite(sy)) {
            m_robotCenter = std::nullopt;
            m_lastError = -1.0f;  // signal: invalid
            return;
        }

        // Reject absurd magnitudes (outside plausible field area)
        if (std::fabs(sx) > LocMath::GPS_ABSURD_LIMIT_M ||
            std::fabs(sy) > LocMath::GPS_ABSURD_LIMIT_M) {
            m_robotCenter = std::nullopt;
            m_lastError = -1.0f;
            return;
        }

        // Gate GPS trust using RMS error from GPS sensor
        // Query GPS error in metres; if error is too high, skip update
        float gpsErrorM = static_cast<float>(m_gps.get_error());
        if (!LocMath::isFinite(gpsErrorM) || gpsErrorM < 0.0f) {
            m_robotCenter = std::nullopt;
            m_lastError = -1.0f;
            return;
        }
        m_lastError = gpsErrorM;

        // If GPS error exceeds threshold, skip this update (don't trust it)
        if (gpsErrorM > CONFIG::GPS_ERROR_THRESHOLD.getValue()) {
            m_robotCenter = std::nullopt;
            return;
        }

        // Transform raw GPS output into configured field frame.
        Eigen::Vector2f gpsWorld = CONFIG::transformGpsToFieldFrame(Eigen::Vector2f(sx, sy));

        // Read GPS heading, apply mounting offset, convert to internal frame
        float rawHeadingDeg = static_cast<float>(m_gps.get_heading());
        if (!LocMath::isFinite(rawHeadingDeg)) {
            m_robotCenter = std::nullopt;
            return;
        }
        // Convert from VEX compass (0° = north, CW+) to internal (0° = east/fwd, CCW+)
        float robotHeading = CONFIG::gpsSensorHeadingDegToInternalRad(
            rawHeadingDeg - m_headingOffsetDeg);

        // Rotate mount offset into world frame
        float cosT = std::cos(robotHeading);
        float sinT = std::sin(robotHeading);
        Eigen::Vector2f offsetWorld(
            m_offset.x() * cosT - m_offset.y() * sinT,
            m_offset.x() * sinT + m_offset.y() * cosT);

        // Robot centre = sensor position − rotated mount offset
        m_robotCenter = gpsWorld - offsetWorld;
    }

    bool hasObservation() const override { return m_robotCenter.has_value(); }

    std::optional<float> p(const Eigen::Vector3f& particle) override {
        if (!m_robotCenter) return std::nullopt;

        // Adaptively adjust stddev based on GPS error
        float currentStddev = m_stddev;
        if (m_lastError >= 0.0f) {
            if (m_lastError > CONFIG::GPS_ERROR_GOOD.getValue()) {
                // GPS error is higher than ideal; inflate stddev to reduce GPS influence
                float excessError = m_lastError - CONFIG::GPS_ERROR_GOOD.getValue();
                float inflationFactor = 1.0f + CONFIG::GPS_ERROR_SCALE_MULTIPLIER * excessError;
                currentStddev = m_stddev * inflationFactor;
                // Clamp to configured limits
                currentStddev = std::max(CONFIG::GPS_STDDEV_MIN.getValue(),
                                       std::min(CONFIG::GPS_STDDEV_MAX.getValue(), currentStddev));
            }
        }

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
