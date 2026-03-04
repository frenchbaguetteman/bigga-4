/**
 * @file gps.h
 * VEX GPS sensor likelihood model.
 *
 * Evaluates a 2-D Gaussian on (x,y) between each particle's robot-center
 * and a robot-center estimate inferred from the raw GPS sensor position plus
 * configured GPS mount offset.
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
     * @param headingOffsetDeg  GPS mounting heading offset (compass degrees)
     * @param offsetX_m         GPS mount offset X in robot math frame (metres, +fwd)
     * @param offsetY_m         GPS mount offset Y in robot math frame (metres, +left)
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

        // Reject non-finite or absurd magnitudes
        if (!LocMath::isFinite(sx) || !LocMath::isFinite(sy) ||
            std::fabs(sx) > LocMath::GPS_ABSURD_LIMIT_M ||
            std::fabs(sy) > LocMath::GPS_ABSURD_LIMIT_M) {
            m_robotCenter = std::nullopt;
            return;
        }

        // Transform raw GPS output into configured field frame.
        Eigen::Vector2f gpsWorld = CONFIG::transformGpsToFieldFrame(Eigen::Vector2f(sx, sy));

        // Read GPS heading, apply mounting offset → robot heading
        float rawHeadingDeg = static_cast<float>(m_gps.get_heading());
        if (!LocMath::isFinite(rawHeadingDeg)) {
            m_robotCenter = std::nullopt;
            return;
        }
        float robotHeading = CONFIG::compassDegToMathRad(
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

    std::optional<float> p(const Eigen::Vector3f& particle) override {
        if (!m_robotCenter) return std::nullopt;

        float dx = particle.x() - m_robotCenter->x();
        float dy = particle.y() - m_robotCenter->y();
        float distSq = dx * dx + dy * dy;
        return std::exp(-distSq / (2.0f * m_stddev * m_stddev));
    }

private:
    pros::Gps m_gps;
    float m_headingOffsetDeg;
    Eigen::Vector2f m_offset;
    float m_stddev;
    std::optional<Eigen::Vector2f> m_robotCenter;
};
