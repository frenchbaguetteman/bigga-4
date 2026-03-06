/**
 * @file sensor.h
 * Abstract sensor model for particle-filter weighting.
 *
 * Each sensor subclass computes p(observation | particle_pose),
 * the likelihood of the current reading given a hypothesised pose.
 */
#pragma once

#include "Eigen/Core"
#include <cstddef>
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

    /** Whether this sensor has a usable observation after update(). */
    virtual bool hasObservation() const { return false; }

    /** Whether this observation constrains absolute field position directly. */
    virtual bool isAbsolutePositionSensor() const { return false; }

    /** Human-readable sensor name for debug logs. */
    virtual const char* debugName() const { return "sensor"; }

    /** Emit a one-line sensor debug summary at a reference pose. */
    virtual void debugPrint(const Eigen::Vector3f& referencePose, size_t index) const {}
};
