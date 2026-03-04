/**
 * @file distance.h
 * Distance-sensor likelihood model.
 *
 * Given a particle pose and a known sensor mounting offset, computes the
 * expected distance to the nearest field wall by ray-casting, then
 * evaluates a Gaussian likelihood centred on the measured distance.
 *
 * Supports a per-sensor weight multiplier for MCL fusion tuning.
 */
#pragma once

#include "localization/sensor.h"
#include "Eigen/Dense"
#include "config.h"
#include "pros/distance.hpp"
#include <cmath>
#include <algorithm>

class DistanceSensorModel : public SensorModel {
public:
    /**
     * @param port       PROS smart-port number
     * @param offset     sensor mounting offset (x, y, angleOffset) relative
     *                   to robot centre when robot heading = 0
     * @param weight     fusion weight multiplier (0..1, default 1.0)
     * @param stddev     measurement noise standard deviation (metres)
     */
    DistanceSensorModel(int port, Eigen::Vector3f offset,
                        float weight = 1.0f, float stddev = 0.03f)
        : m_sensor(port), m_offset(offset), m_weight(weight), m_stddev(stddev) {}

    void update() override {
        int raw = m_sensor.get();
        m_reading = (raw > 0 && raw < 9999) ?
            std::optional<float>(raw / 1000.0f) : std::nullopt;  // mm → m
    }

    std::optional<float> p(const Eigen::Vector3f& particle) override {
        if (!m_reading) return std::nullopt;

        float expected = expectedDistance(particle);
        if (expected <= 0.0f) return std::nullopt;

        float diff = *m_reading - expected;
        // Gaussian likelihood scaled by sensor weight
        float exponent = -(diff * diff) / (2.0f * m_stddev * m_stddev);
        float likelihood = std::exp(exponent);
        // Blend towards uniform (1.0) based on weight: lower weight → less influence
        return m_weight * likelihood + (1.0f - m_weight);
    }

private:
    pros::Distance m_sensor;
    Eigen::Vector3f m_offset;
    float m_weight;
    float m_stddev;
    std::optional<float> m_reading;

    /**
     * Ray-cast from the sensor's world position along its world heading
     * to the nearest axis-aligned field wall.
     */
    float expectedDistance(const Eigen::Vector3f& particle) const {
        float robotTheta = particle.z();
        float cosT = std::cos(robotTheta);
        float sinT = std::sin(robotTheta);

        // Sensor world position
        float sx = particle.x() + m_offset.x() * cosT - m_offset.y() * sinT;
        float sy = particle.y() + m_offset.x() * sinT + m_offset.y() * cosT;

        // Sensor world heading
        float sTheta = robotTheta + m_offset.z();
        float dx = std::cos(sTheta);
        float dy = std::sin(sTheta);

        const float H = CONFIG::FIELD_HALF_SIZE;  // wall positions ±H
        float minDist = 1e6f;

        // Intersect with four walls: x = ±H, y = ±H
        auto tryWall = [&](float wallVal, float origin, float dir) -> float {
            if (std::fabs(dir) < 1e-9f) return 1e6f;
            float t = (wallVal - origin) / dir;
            return (t > 0.001f) ? t : 1e6f;
        };

        // x = +H  (right wall)
        { float t = tryWall(H, sx, dx);
          float iy = sy + t * dy;
          if (t < minDist && std::fabs(iy) <= H) minDist = t; }
        // x = -H  (left wall)
        { float t = tryWall(-H, sx, dx);
          float iy = sy + t * dy;
          if (t < minDist && std::fabs(iy) <= H) minDist = t; }
        // y = +H  (top wall)
        { float t = tryWall(H, sy, dy);
          float ix = sx + t * dx;
          if (t < minDist && std::fabs(ix) <= H) minDist = t; }
        // y = -H  (bottom wall)
        { float t = tryWall(-H, sy, dy);
          float ix = sx + t * dx;
          if (t < minDist && std::fabs(ix) <= H) minDist = t; }

        return (minDist < 1e5f) ? minDist : -1.0f;
    }
};
