/**
 * @file sensor.h
 * Abstract sensor model for particle-filter weighting.
 *
 * Each sensor subclass computes p(observation | particle_pose),
 * the likelihood of the current reading given a hypothesised pose.
 */
#pragma once

#include "Eigen/Core"
#include <optional>

class SensorModel {
public:
    virtual ~SensorModel() = default;

    /**
     * Compute the likelihood of the current sensor reading given a
     * hypothesised robot pose.
     *
     * @param particle  candidate pose (x, y, θ)
     * @return          likelihood value, or std::nullopt if the sensor
     *                  reading is unavailable / invalid this cycle.
     */
    virtual std::optional<float> p(const Eigen::Vector3f& particle) = 0;

    /** Refresh the sensor reading (called once per filter update). */
    virtual void update() {}
};
