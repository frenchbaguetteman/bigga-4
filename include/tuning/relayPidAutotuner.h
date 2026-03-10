/**
 * @file relayPidAutotuner.h
 * Relay-method drivetrain PID autotuner for on-robot VEX tuning.
 */
#pragma once

#include "Eigen/Core"
#include "feedback/pid.h"

#include <cstddef>
#include <cstdint>
#include <functional>

class Drivetrain;

namespace tuning {

class RelayPidAutotuner {
public:
    enum class Mode {
        DriveDistance,
        TurnAngle,
    };

    struct Config {
        Mode mode = Mode::DriveDistance;
        float target = 0.0f;                    // metres or radians, relative to start
        float relayAmplitude = 0.4f;           // normalized drive command [0, 1]
        float noiseBand = 0.01f;               // metres or radians
        uint32_t sampleTimeMs = 10;
        uint32_t timeoutMs = 15000;
        std::size_t requiredPeakPairs = 4;     // full oscillations to collect
        std::size_t analysisPeakPairs = 3;     // most recent pairs used for Ku/Pu
        float minOscillationAmplitude = 0.005f;
    };

    struct GainSet {
        PID::Gains zieglerNichols{};
        PID::Gains noOvershoot{};
    };

    struct Result {
        bool success = false;
        bool cancelled = false;
        uint32_t elapsedMs = 0;
        std::size_t switchCount = 0;
        float oscillationAmplitude = 0.0f;
        float relayAmplitude = 0.0f;
        float ku = 0.0f;
        float puSec = 0.0f;
        GainSet gains{};
        char message[96]{};
    };

    RelayPidAutotuner(Drivetrain& drivetrain,
                      std::function<Eigen::Vector3f()> poseSource,
                      std::function<bool()> cancelRequested = {});

    Result run(const Config& config);

    static void clearStatus();
    static void copyStatus(char* buffer, std::size_t size);

private:
    Drivetrain& m_drivetrain;
    std::function<Eigen::Vector3f()> m_poseSource;
    std::function<bool()> m_cancelRequested;

    bool cancelled() const;
    Eigen::Vector3f samplePose() const;
    void applyRelayOutput(Mode mode, float output) const;
    float measureRelativeProcessValue(Mode mode,
                                      const Eigen::Vector3f& startPose,
                                      const Eigen::Vector3f& pose) const;

    static const char* modeName(Mode mode);
    static const char* modeUnit(Mode mode);
    static void setStatus(const char* fmt, ...);
};

} // namespace tuning
