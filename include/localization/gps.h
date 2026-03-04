/**
 * @file gps.h
 * VEX GPS sensor likelihood model.
 *
 * Evaluates a 2-D Gaussian on the (x,y) distance between the GPS reading
 * and each particle.  Supports a heading offset for rear-facing GPS mounting.
 */
#pragma once

#include "localization/sensor.h"
#include "Eigen/Core"
#include "pros/gps.hpp"
#include <cmath>
#include <optional>

class GpsSensorModel : public SensorModel {
public:
    /**
     * @param port              PROS smart-port
     * @param headingOffsetDeg  heading offset of GPS relative to robot forward (degrees)
     * @param stddev            measurement noise stddev (metres)
     */
    explicit GpsSensorModel(int port, double headingOffsetDeg = 0.0, float stddev = 0.05f)
        : m_gps(port), m_headingOffsetDeg(headingOffsetDeg), m_stddev(stddev) {}

    void update() override {
        auto pos = m_gps.get_position();
        // GPS returns metres; treat low-confidence as invalid
        if (pos.x == 0.0 && pos.y == 0.0) {
            m_reading = std::nullopt;
        } else {
            m_reading = Eigen::Vector2f(
                static_cast<float>(pos.x),
                static_cast<float>(pos.y));
        }
    }

    std::optional<float> p(const Eigen::Vector3f& particle) override {
        if (!m_reading) return std::nullopt;
        float dx = particle.x() - m_reading->x();
        float dy = particle.y() - m_reading->y();
        float distSq = dx * dx + dy * dy;
        return std::exp(-distSq / (2.0f * m_stddev * m_stddev));
    }

    double getHeadingOffsetDeg() const { return m_headingOffsetDeg; }

private:
    pros::Gps m_gps;
    double    m_headingOffsetDeg;
    float     m_stddev;
    std::optional<Eigen::Vector2f> m_reading;
};
