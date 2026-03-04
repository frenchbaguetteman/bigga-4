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
#include "Eigen/Core"
#include "pros/gps.hpp"
#include <cmath>
#include <optional>

class GpsSensorModel : public SensorModel {
public:
    /**
     * @param port              PROS smart-port
     * @param gpsOffsetMeters   GPS offset in robot math frame (+forward, +left), metres
     * @param stddev            measurement noise stddev (metres)
     */
    explicit GpsSensorModel(int port,
                            Eigen::Vector2f gpsOffsetMeters = Eigen::Vector2f(0.0f, 0.0f),
                            float stddev = 0.05f)
        : m_gps(port)
        , m_offset(gpsOffsetMeters)
        , m_stddev(stddev) {}

    void update() override {
        auto pos = m_gps.get_position();
        float sx = static_cast<float>(pos.x);
        float sy = static_cast<float>(pos.y);

        if (!std::isfinite(sx) || !std::isfinite(sy)) {
            m_reading = std::nullopt;
            return;
        }

        // Store raw GPS sensor measurement in world frame.
        m_reading = Eigen::Vector2f(sx, sy);
    }

    std::optional<float> p(const Eigen::Vector3f& particle) override {
        if (!m_reading) return std::nullopt;

        // Transform GPS offset (robot frame) into world frame using particle heading.
        const float theta = particle.z();
        const float cosT = std::cos(theta);
        const float sinT = std::sin(theta);
        Eigen::Vector2f offsetWorld(
            m_offset.x() * cosT - m_offset.y() * sinT,
            m_offset.x() * sinT + m_offset.y() * cosT);

        // Convert measured sensor position to inferred robot-centre position.
        Eigen::Vector2f robotCenterFromGps = *m_reading - offsetWorld;

        float dx = particle.x() - robotCenterFromGps.x();
        float dy = particle.y() - robotCenterFromGps.y();
        float distSq = dx * dx + dy * dy;
        return std::exp(-distSq / (2.0f * m_stddev * m_stddev));
    }

private:
    pros::Gps m_gps;
    Eigen::Vector2f m_offset;
    float     m_stddev;
    std::optional<Eigen::Vector2f> m_reading;
};
