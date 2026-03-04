/**
 * @file gps.h
 * VEX GPS sensor likelihood model.
 *
 * Evaluates a 2-D Gaussian on the (x,y) distance between the GPS reading
 * and each particle.  Supports a heading offset for rear-facing GPS mounting.
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
     * @param headingOffsetDeg  heading offset of GPS relative to robot forward (degrees)
     * @param offsetX           GPS offset X in robot math frame (+forward, metres)
     * @param offsetY           GPS offset Y in robot math frame (+left, metres)
     * @param stddev            measurement noise stddev (metres)
     */
    explicit GpsSensorModel(int port,
                            double headingOffsetDeg = 0.0,
                            float offsetX = 0.0f,
                            float offsetY = 0.0f,
                            float stddev = 0.05f)
        : m_gps(port)
        , m_headingOffsetDeg(headingOffsetDeg)
        , m_offsetX(offsetX)
        , m_offsetY(offsetY)
        , m_stddev(stddev) {}

    void update() override {
        auto pos = m_gps.get_position();
        float sx = static_cast<float>(pos.x);
        float sy = static_cast<float>(pos.y);

        if (!std::isfinite(sx) || !std::isfinite(sy)) {
            m_reading = std::nullopt;
            return;
        }

        // Convert GPS-reported heading into robot math heading and
        // transform sensor position -> robot-centre position.
        float robotHeading = CONFIG::compassDegToMathRad(
            static_cast<float>(m_gps.get_heading() - m_headingOffsetDeg));

        float cosT = std::cos(robotHeading);
        float sinT = std::sin(robotHeading);

        float cx = sx - (m_offsetX * cosT - m_offsetY * sinT);
        float cy = sy - (m_offsetX * sinT + m_offsetY * cosT);

        m_reading = Eigen::Vector2f(cx, cy);
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
    float     m_offsetX;
    float     m_offsetY;
    float     m_stddev;
    std::optional<Eigen::Vector2f> m_reading;
};
