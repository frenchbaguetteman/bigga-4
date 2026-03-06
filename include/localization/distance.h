/**
 * @file distance.h
 * Distance-sensor likelihood model for wall-based localization.
 *
 * The model ray-casts each mounted sensor against the square field walls,
 * compares the expected wall distance to the current sensor reading, then
 * evaluates a confidence-aware robust likelihood. Bad readings are ignored,
 * while improbable particles receive a soft but non-zero penalty instead of
 * collapsing the whole filter.
 */
#pragma once

#include "Eigen/Dense"
#include "config.h"
#include "localization/sensor.h"
#include "pros/distance.hpp"
#include "utils/localization_math.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>

class DistanceSensorModel : public SensorModel {
public:
    DistanceSensorModel(
        int port,
        Eigen::Vector3f offset,
        float weight = 1.0f,
        float stddev = CONFIG::MCL_DISTANCE_STDDEV.convert(meter),
        const char* name = "dist")
        : m_sensor(port)
        , m_offset(offset)
        , m_weight(std::clamp(weight, 0.0f, 1.0f))
        , m_baseStddev(std::max(stddev, 0.005f))
        , m_name(name) {}

    void update() override {
        m_reading = std::nullopt;

        const int rawMm = m_sensor.get_distance();
        if (rawMm <= 0 || rawMm >= 9999) return;

        const float rangeM = static_cast<float>(rawMm) / 1000.0f;
        if (!LocMath::isFinite(rangeM)) return;
        if (rangeM < CONFIG::MCL_DISTANCE_MIN_RANGE.convert(meter) ||
            rangeM > CONFIG::MCL_DISTANCE_MAX_RANGE.convert(meter)) {
            return;
        }

        m_reading = Reading{
            rangeM,
            m_sensor.get_confidence(),
        };
    }

    bool hasObservation() const override {
        return !CONFIG::MCL_DISABLE_DISTANCE_SENSORS_WHILE_DEBUGGING && m_reading.has_value();
    }

    std::optional<float> p(const Eigen::Vector3f& particle) override {
        if (!hasObservation()) return std::nullopt;
        if (!LocMath::isFinitePose(particle)) return std::nullopt;

        const std::optional<float> expected = expectedDistance(particle);
        if (!expected) {
            return static_cast<float>(LocMath::LIKELIHOOD_EPS);
        }

        const float sigma = effectiveStddev(*m_reading);
        const float residual = m_reading->rangeM - *expected;
        const float likelihood = std::pow(
            robustLikelihood(residual, sigma),
            m_weight);

        return std::clamp(likelihood,
                          static_cast<float>(LocMath::LIKELIHOOD_EPS),
                          1.0f);
    }

    const char* debugName() const override { return m_name; }

    void debugPrint(const Eigen::Vector3f& referencePose, size_t index) const override {
        if (!m_reading) {
            std::printf("[PFDBG] sensor[%zu] %s reading=INVALID\n", index, m_name);
            return;
        }

        const std::optional<float> expected = expectedDistance(referencePose);
        const float sigma = effectiveStddev(*m_reading);
        if (!expected) {
            std::printf(
                "[PFDBG] sensor[%zu] %s raw=%.3f conf=%d sigma=%.3f exp=INVALID\n",
                index,
                m_name,
                m_reading->rangeM,
                m_reading->confidence,
                sigma);
            return;
        }

        const float residual = m_reading->rangeM - *expected;
        const float likelihood = std::pow(
            robustLikelihood(residual, sigma),
            m_weight);
        std::printf(
            "[PFDBG] sensor[%zu] %s raw=%.3f exp=%.3f resid=%+.3f conf=%d sigma=%.3f like=%.5f w=%.2f\n",
            index,
            m_name,
            m_reading->rangeM,
            *expected,
            residual,
            m_reading->confidence,
            sigma,
            likelihood,
            m_weight);
    }

private:
    struct Reading {
        float rangeM = 0.0f;
        int confidence = -1;
    };

    pros::Distance m_sensor;
    Eigen::Vector3f m_offset;
    float m_weight;
    float m_baseStddev;
    std::optional<Reading> m_reading;
    const char* m_name;

    float effectiveStddev(const Reading& reading) const {
        if (reading.rangeM <= CONFIG::MCL_DISTANCE_CONFIDENCE_EXEMPT.convert(meter)) {
            return m_baseStddev;
        }

        if (reading.confidence < 0) {
            return m_baseStddev * CONFIG::MCL_DISTANCE_LOW_CONFIDENCE_SIGMA_SCALE;
        }

        const float confidence = std::clamp(
            static_cast<float>(reading.confidence) / 63.0f,
            0.0f,
            1.0f);
        const float scale = 1.0f +
            (1.0f - confidence) *
                (CONFIG::MCL_DISTANCE_LOW_CONFIDENCE_SIGMA_SCALE - 1.0f);
        return m_baseStddev * scale;
    }

    static float robustLikelihood(float residual, float sigma) {
        const float normalized = residual / std::max(sigma, 0.005f);
        const float gaussian = std::exp(-0.5f * normalized * normalized);
        const float floor = CONFIG::MCL_DISTANCE_LIKELIHOOD_FLOOR;
        return floor + (1.0f - floor) * gaussian;
    }

    std::optional<float> expectedDistance(const Eigen::Vector3f& particle) const {
        const float robotTheta = particle.z();
        const float cosT = std::cos(robotTheta);
        const float sinT = std::sin(robotTheta);

        const float sx = particle.x() + m_offset.x() * cosT - m_offset.y() * sinT;
        const float sy = particle.y() + m_offset.x() * sinT + m_offset.y() * cosT;

        const float H = CONFIG::FIELD_HALF_SIZE.convert(meter);
        if (sx < -H || sx > H || sy < -H || sy > H) {
            return std::nullopt;
        }

        const float sensorTheta = robotTheta + m_offset.z();
        const float dx = std::cos(sensorTheta);
        const float dy = std::sin(sensorTheta);

        float minDist = std::numeric_limits<float>::infinity();

        auto considerWall = [&](float t, float otherAxis) {
            if (t <= 0.001f || !LocMath::isFinite(t)) return;
            if (std::fabs(otherAxis) > H + 1e-4f) return;
            minDist = std::min(minDist, t);
        };

        if (std::fabs(dx) > 1e-6f) {
            float t = (H - sx) / dx;
            considerWall(t, sy + t * dy);
            t = (-H - sx) / dx;
            considerWall(t, sy + t * dy);
        }

        if (std::fabs(dy) > 1e-6f) {
            float t = (H - sy) / dy;
            considerWall(t, sx + t * dx);
            t = (-H - sy) / dy;
            considerWall(t, sx + t * dx);
        }

        if (!LocMath::isFinite(minDist) ||
            minDist > CONFIG::MCL_DISTANCE_MAX_RANGE.convert(meter)) {
            return std::nullopt;
        }

        return minDist;
    }
};
