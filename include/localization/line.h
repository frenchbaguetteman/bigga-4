/**
 * @file line.h
 * Line sensor likelihood model (placeholder).
 *
 * Uses ADI line tracker to detect field lines and score particles
 * based on proximity to known line positions.
 */
#pragma once

#include "localization/sensor.h"
#include "Eigen/Core"
#include <optional>

class LineSensorModel : public SensorModel {
public:
    LineSensorModel() = default;

    void update() override {
        // TODO: read ADI line tracker and compare to known field-line map
    }

    std::optional<float> p(const Eigen::Vector3f& /*particle*/) override {
        // Placeholder — always returns 1.0 (uniform; does not contribute)
        return 1.0f;
    }
};
