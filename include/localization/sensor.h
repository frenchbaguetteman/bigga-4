/**
 * @file sensor.h
 * Abstract sensor model for particle-filter weighting.
 *
 * Each sensor subclass computes p(observation | particle_pose),
 * the likelihood of the current reading given a hypothesised pose.
 */
#pragma once

#include "Eigen/Core"
#include "pros/rtos.hpp"
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

    // ── Staleness tracking ──────────────────────────────────────────────
    /** Timestamp (pros::millis()) of the last valid hardware reading. */
    uint32_t lastValidReadingMs() const { return m_lastValidReadingMs; }

    /** True when no valid reading has arrived within thresholdMs. */
    bool isStale(uint32_t thresholdMs = 500) const {
        if (m_lastValidReadingMs == 0) return true;
        return (pros::millis() - m_lastValidReadingMs) > thresholdMs;
    }

    // ── Dynamic obstacle detection (override in distance sensors) ───────
    /** Called once per PF cycle with the current best-estimate pose. */
    virtual void checkDynamicObstacle(const Eigen::Vector3f& /*referencePose*/) {}
    /** True when the sensor is being occluded by a dynamic obstacle. */
    virtual bool isDynamicObstacleDetected() const { return false; }

protected:
    /** Subclasses call this on every successful hardware read. */
    void markValid() { m_lastValidReadingMs = pros::millis(); }

private:
    uint32_t m_lastValidReadingMs = 0;
};
