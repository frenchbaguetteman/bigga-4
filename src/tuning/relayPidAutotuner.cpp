#include "tuning/relayPidAutotuner.h"

#include "subsystems/drivetrain.h"
#include "utils/localization_math.h"
#include "utils/utils.h"
#include "pros/rtos.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>
#include <vector>

namespace tuning {

namespace {

constexpr float kPi = 3.14159265358979323846f;

pros::Mutex& statusMutex() {
    static pros::Mutex* mutex = nullptr;
    if (!mutex) {
        mutex = new pros::Mutex();
    }
    return *mutex;
}

char* statusBuffer() {
    static char buffer[96] = "";
    return buffer;
}

void copyString(char* dst, std::size_t size, const char* src) {
    if (!dst || size == 0) return;
    std::snprintf(dst, size, "%s", src ? src : "");
}

::PID::Gains gainsFromKuPu(float kPScale,
                          float tiScale,
                          float tdScale,
                          float ku,
                          float puSec,
                          float controllerDtSec) {
    ::PID::Gains gains;
    gains.kP = kPScale * ku;
    const float continuousKi = (tiScale > 0.0f && puSec > 1e-5f)
        ? gains.kP / (tiScale * puSec)
        : 0.0f;
    const float continuousKd = gains.kP * tdScale * puSec;
    gains.kI = continuousKi * controllerDtSec;
    gains.kD = controllerDtSec > 1e-5f
        ? continuousKd / controllerDtSec
        : 0.0f;
    gains.integralCap = 0.0f;
    return gains;
}

float averageRecentAmplitude(const std::vector<float>& positivePeaks,
                             const std::vector<float>& negativePeaks,
                             std::size_t pairCount) {
    const std::size_t availablePairs = std::min(positivePeaks.size(), negativePeaks.size());
    if (availablePairs == 0) return 0.0f;

    const std::size_t usedPairs = std::min(pairCount, availablePairs);
    const std::size_t firstPair = availablePairs - usedPairs;

    float amplitudeSum = 0.0f;
    for (std::size_t i = firstPair; i < availablePairs; ++i) {
        amplitudeSum += 0.5f * std::fabs(positivePeaks[i] - negativePeaks[i]);
    }

    return amplitudeSum / static_cast<float>(usedPairs);
}

float averageRecentPeriodSec(const std::vector<uint32_t>& switchTimes,
                             std::size_t periodCount) {
    if (switchTimes.size() < 3) return 0.0f;

    const std::size_t availablePeriods = switchTimes.size() - 2;
    const std::size_t usedPeriods = std::min(periodCount, availablePeriods);
    const std::size_t firstIndex = switchTimes.size() - usedPeriods;

    float periodSum = 0.0f;
    std::size_t counted = 0;
    for (std::size_t i = firstIndex; i < switchTimes.size(); ++i) {
        if (i < 2) continue;
        periodSum += static_cast<float>(switchTimes[i] - switchTimes[i - 2]) / 1000.0f;
        ++counted;
    }

    if (counted == 0) return 0.0f;
    return periodSum / static_cast<float>(counted);
}

} // namespace

RelayPidAutotuner::RelayPidAutotuner(Drivetrain& drivetrain,
                                     std::function<Eigen::Vector3f()> poseSource,
                                     std::function<bool()> cancelRequested)
    : m_drivetrain(drivetrain)
    , m_poseSource(std::move(poseSource))
    , m_cancelRequested(std::move(cancelRequested)) {}

RelayPidAutotuner::Result RelayPidAutotuner::run(const Config& config) {
    Result result;
    result.relayAmplitude = std::clamp(std::fabs(config.relayAmplitude), 0.0f, 1.0f);

    auto fail = [&](const char* reason, bool cancelled = false) {
        result.success = false;
        result.cancelled = cancelled;
        copyString(result.message, sizeof(result.message), reason);
        setStatus("%s tune: %s", modeName(config.mode), reason);
        m_drivetrain.stop();
        m_drivetrain.resetDriverAssistState();
        return result;
    };

    if (result.relayAmplitude < 0.05f) {
        return fail("relay amplitude too small");
    }
    if (!(config.noiseBand > 0.0f)) {
        return fail("noise band must be positive");
    }
    if (config.requiredPeakPairs < 2) {
        return fail("need at least 2 peak pairs");
    }
    if (config.controllerPeriodMs == 0) {
        return fail("controller period must be positive");
    }

    const Eigen::Vector3f startPose = samplePose();
    const uint32_t startMs = pros::millis();
    const uint32_t sampleTimeMs = std::max<uint32_t>(1, config.sampleTimeMs);
    const float controllerDtSec = static_cast<float>(config.controllerPeriodMs) / 1000.0f;

    std::vector<float> positivePeaks;
    std::vector<float> negativePeaks;
    std::vector<uint32_t> switchTimes;
    positivePeaks.reserve(config.requiredPeakPairs + 2);
    negativePeaks.reserve(config.requiredPeakPairs + 2);
    switchTimes.reserve(config.requiredPeakPairs * 2 + 4);

    bool highOutput = true;
    float outputSign = 1.0f;
    bool polarityLocked = std::fabs(config.target) < 1e-5f;
    float measurement = measureRelativeProcessValue(config.mode, startPose, startPose);
    float halfCycleMax = measurement;
    float halfCycleMin = measurement;

    setStatus("%s tune: targeting %.2f %s",
              modeName(config.mode),
              config.target,
              modeUnit(config.mode));
    std::printf("[PID_TUNE] %s begin target=%.4f %s relay=%.3f noise=%.4f\n",
                modeName(config.mode),
                config.target,
                modeUnit(config.mode),
                result.relayAmplitude,
                config.noiseBand);

    applyRelayOutput(config.mode, outputSign * result.relayAmplitude);

    while (pros::millis() - startMs < config.timeoutMs) {
        if (cancelled()) {
            return fail("cancelled", true);
        }

        const Eigen::Vector3f pose = samplePose();
        measurement = measureRelativeProcessValue(config.mode, startPose, pose);

        if (!std::isfinite(measurement)) {
            pros::delay(sampleTimeMs);
            continue;
        }

        halfCycleMax = std::max(halfCycleMax, measurement);
        halfCycleMin = std::min(halfCycleMin, measurement);

        if (!polarityLocked && std::fabs(measurement) >= 0.5f * config.noiseBand) {
            polarityLocked = true;
            if (measurement * config.target < 0.0f) {
                outputSign = -1.0f;
                halfCycleMax = measurement;
                halfCycleMin = measurement;
                applyRelayOutput(config.mode, outputSign * result.relayAmplitude);
                std::printf("[PID_TUNE] %s relay polarity inverted\n", modeName(config.mode));
                setStatus("%s tune: polarity synced", modeName(config.mode));
                pros::delay(sampleTimeMs);
                continue;
            }
        }

        bool requestedHigh = highOutput;
        if (measurement > config.target + config.noiseBand) {
            requestedHigh = false;
        } else if (measurement < config.target - config.noiseBand) {
            requestedHigh = true;
        }

        if (requestedHigh != highOutput) {
            const uint32_t now = pros::millis();
            switchTimes.push_back(now);

            if (highOutput) {
                positivePeaks.push_back(halfCycleMax);
            } else {
                negativePeaks.push_back(halfCycleMin);
            }

            const float amplitude = averageRecentAmplitude(
                positivePeaks,
                negativePeaks,
                std::max<std::size_t>(1, std::min(config.analysisPeakPairs, config.requiredPeakPairs)));
            const float puSec = averageRecentPeriodSec(
                switchTimes,
                std::max<std::size_t>(2, config.analysisPeakPairs * 2));

            std::printf("[PID_TUNE] %s flip=%zu pv=%.4f peak+=(%zu) peak-=(%zu) amp=%.4f pu=%.3fs\n",
                        modeName(config.mode),
                        switchTimes.size(),
                        measurement,
                        positivePeaks.size(),
                        negativePeaks.size(),
                        amplitude,
                        puSec);

            setStatus("%s tune: %zu/%zu cycles",
                      modeName(config.mode),
                      std::min(positivePeaks.size(), negativePeaks.size()),
                      config.requiredPeakPairs);

            highOutput = requestedHigh;
            halfCycleMax = measurement;
            halfCycleMin = measurement;
            applyRelayOutput(
                config.mode,
                outputSign * (highOutput ? result.relayAmplitude : -result.relayAmplitude));

            if (std::min(positivePeaks.size(), negativePeaks.size()) >= config.requiredPeakPairs) {
                break;
            }
        }

        pros::delay(sampleTimeMs);
    }

    m_drivetrain.stop();
    m_drivetrain.resetDriverAssistState();

    result.elapsedMs = pros::millis() - startMs;
    result.switchCount = switchTimes.size();

    const std::size_t pairCount = std::min(positivePeaks.size(), negativePeaks.size());
    if (pairCount < 2) {
        return fail("not enough oscillation");
    }

    result.oscillationAmplitude = averageRecentAmplitude(
        positivePeaks,
        negativePeaks,
        std::min(config.analysisPeakPairs, pairCount));
    result.puSec = averageRecentPeriodSec(
        switchTimes,
        std::max<std::size_t>(2, std::min(config.analysisPeakPairs * 2, switchTimes.size())));

    if (!(result.oscillationAmplitude > config.minOscillationAmplitude)) {
        return fail("oscillation amplitude too small");
    }
    if (!(result.puSec > 1e-3f)) {
        return fail("oscillation period invalid");
    }

    result.ku = (4.0f * result.relayAmplitude) / (kPi * result.oscillationAmplitude);
    if (!std::isfinite(result.ku) || result.ku <= 0.0f) {
        return fail("Ku calculation invalid");
    }

    result.gains.zieglerNichols = gainsFromKuPu(
        0.6f, 0.5f, 0.125f, result.ku, result.puSec, controllerDtSec);
    result.gains.noOvershoot = gainsFromKuPu(
        0.2f, 0.5f, 0.33f, result.ku, result.puSec, controllerDtSec);
    result.success = true;
    copyString(result.message, sizeof(result.message), "ok");

    std::printf(
        "[PID_TUNE] %s done elapsed=%lums Ku=%.5f Pu=%.4fs amp=%.5f %s dt=%.3fs\n"
        "[PID_TUNE] %s ZN(discrete)     -> P=%.5f I=%.5f D=%.5f\n"
        "[PID_TUNE] %s NoOver(discrete) -> P=%.5f I=%.5f D=%.5f\n",
        modeName(config.mode),
        static_cast<unsigned long>(result.elapsedMs),
        result.ku,
        result.puSec,
        result.oscillationAmplitude,
        modeUnit(config.mode),
        controllerDtSec,
        modeName(config.mode),
        result.gains.zieglerNichols.kP,
        result.gains.zieglerNichols.kI,
        result.gains.zieglerNichols.kD,
        modeName(config.mode),
        result.gains.noOvershoot.kP,
        result.gains.noOvershoot.kI,
        result.gains.noOvershoot.kD);

    setStatus("%s tune OK: see terminal for gains", modeName(config.mode));
    return result;
}

void RelayPidAutotuner::clearStatus() {
    std::scoped_lock guard(statusMutex());
    statusBuffer()[0] = '\0';
}

void RelayPidAutotuner::copyStatus(char* buffer, std::size_t size) {
    if (!buffer || size == 0) return;

    std::scoped_lock guard(statusMutex());
    std::snprintf(buffer, size, "%s", statusBuffer());
}

bool RelayPidAutotuner::cancelled() const {
    return m_cancelRequested && m_cancelRequested();
}

Eigen::Vector3f RelayPidAutotuner::samplePose() const {
    const Eigen::Vector3f rawPose =
        m_poseSource ? m_poseSource() : Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    return LocMath::finitePoseOr(rawPose, m_drivetrain.getOdomPose());
}

void RelayPidAutotuner::applyRelayOutput(Mode mode, float output) const {
    const float command = std::clamp(output, -1.0f, 1.0f) * 127.0f;
    if (mode == Mode::TurnAngle) {
        m_drivetrain.arcade(0.0f, -command);
    } else {
        m_drivetrain.arcade(command, 0.0f);
    }
}

float RelayPidAutotuner::measureRelativeProcessValue(Mode mode,
                                                     const Eigen::Vector3f& startPose,
                                                     const Eigen::Vector3f& pose) const {
    if (mode == Mode::TurnAngle) {
        return utils::angleDifference(pose.z(), startPose.z());
    }

    const Eigen::Vector2f delta = pose.head<2>() - startPose.head<2>();
    const Eigen::Vector2f axis(std::cos(startPose.z()), std::sin(startPose.z()));
    return delta.dot(axis);
}

const char* RelayPidAutotuner::modeName(Mode mode) {
    switch (mode) {
        case Mode::DriveDistance: return "Drive PID";
        case Mode::TurnAngle: return "Turn PID";
    }
    return "PID";
}

const char* RelayPidAutotuner::modeUnit(Mode mode) {
    switch (mode) {
        case Mode::DriveDistance: return "m";
        case Mode::TurnAngle: return "rad";
    }
    return "";
}

void RelayPidAutotuner::setStatus(const char* fmt, ...) {
    char buffer[96];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    std::scoped_lock guard(statusMutex());
    std::snprintf(statusBuffer(), 96, "%s", buffer);
}

} // namespace tuning
