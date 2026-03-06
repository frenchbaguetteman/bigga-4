/**
 * @file particleFilter.h
 * Monte Carlo localization with shared IMU heading and wall-distance updates.
 *
 * The filter tracks robot position in the field plane. Heading is provided by
 * an external source (the IMU) and applied uniformly to every particle, while
 * motion comes from drivetrain odometry and measurement updates come from the
 * configured sensor models.
 */
#pragma once

#include "Eigen/Dense"
#include "config.h"
#include "localization/sensor.h"
#include "pros/rtos.hpp"
#include "utils/localization_math.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <optional>
#include <random>
#include <vector>

class ParticleFilter {
public:
    using PredictionFn = std::function<Eigen::Vector2f()>;
    using AngleFn = std::function<QAngle()>;

    ParticleFilter(
        PredictionFn predictionFunction,
        AngleFn angleFunction,
        std::vector<SensorModel*> sensors,
        Eigen::Vector3f initialPose = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        size_t numParticles = CONFIG::NUM_PARTICLES)
        : m_predictionFunction(std::move(predictionFunction))
        , m_angleFunction(std::move(angleFunction))
        , m_sensors(std::move(sensors))
        , L(numParticles)
        , m_particles(L)
        , m_weights(L, L > 0 ? 1.0 / static_cast<double>(L) : 1.0)
        , m_prediction(initialPose) {
        initializeParticles(initialPose);
        m_lastDebugTime = pros::millis();
    }

    Eigen::Vector3f update() {
        if (L == 0) return validatePose();

        const QAngle headingAngle = m_angleFunction();
        const float heading = headingAngle.getValue();
        if (!LocMath::isFinite(heading)) return validatePose();

        for (auto* sensor : m_sensors) {
            if (sensor) sensor->update();
        }

        size_t activeSensorCount = 0;
        size_t activeAbsoluteSensorCount = 0;
        for (auto* sensor : m_sensors) {
            if (!sensor || !sensor->hasObservation()) continue;
            ++activeSensorCount;
            if (sensor->isAbsolutePositionSensor()) {
                ++activeAbsoluteSensorCount;
            }
        }

        Eigen::Vector2f delta = m_predictionFunction();
        if (!LocMath::isFiniteVec2(delta)) {
            std::printf("[PF] non-finite prediction delta, forcing zero\n");
            delta = Eigen::Vector2f(0.0f, 0.0f);
        }

        applyPrediction(delta, activeSensorCount > 0);
        const Eigen::Vector3f priorPrediction = computeWeightedMean(heading);
        const std::vector<double> priorWeights = m_weights;

        bool usedMeasurements = false;
        bool didResample = false;
        double ess = static_cast<double>(L);

        if (activeSensorCount > 0) {
            std::vector<double> logWeights(L, 0.0);
            double maxLogWeight = -std::numeric_limits<double>::infinity();

            for (size_t i = 0; i < L; ++i) {
                const Eigen::Vector3f particlePose(
                    m_particles[i].x(),
                    m_particles[i].y(),
                    heading);

                double logWeight = std::log(std::max(
                    m_weights[i],
                    static_cast<double>(LocMath::LIKELIHOOD_EPS)));
                size_t particleMeasurementCount = 0;

                for (auto* sensor : m_sensors) {
                    if (!sensor || !sensor->hasObservation()) continue;

                    const std::optional<float> likelihood = sensor->p(particlePose);
                    if (!likelihood || !std::isfinite(likelihood.value())) continue;

                    logWeight += std::log(std::max(
                        static_cast<double>(likelihood.value()),
                        static_cast<double>(LocMath::LIKELIHOOD_EPS)));
                    ++particleMeasurementCount;
                }

                if (particleMeasurementCount > 0) {
                    usedMeasurements = true;
                }

                logWeights[i] = logWeight;
                maxLogWeight = std::max(maxLogWeight, logWeight);
            }

            if (usedMeasurements && std::isfinite(maxLogWeight)) {
                double totalWeight = 0.0;
                for (size_t i = 0; i < L; ++i) {
                    double w = std::exp(logWeights[i] - maxLogWeight);
                    if (!std::isfinite(w)) {
                        w = static_cast<double>(LocMath::LIKELIHOOD_EPS);
                    }
                    m_weights[i] = std::max(w, static_cast<double>(LocMath::LIKELIHOOD_EPS));
                    totalWeight += m_weights[i];
                }

                if (totalWeight > 0.0 && std::isfinite(totalWeight)) {
                    for (double& weight : m_weights) weight /= totalWeight;
                    ess = effectiveSampleSize();
                } else {
                    std::printf("[PF] weight normalization failed, keeping prior belief\n");
                    m_weights = priorWeights;
                    usedMeasurements = false;
                }
            } else {
                m_weights = priorWeights;
                usedMeasurements = false;
            }
        }

        m_prediction = usedMeasurements ? computeWeightedMean(heading) : priorPrediction;

        if (usedMeasurements &&
            ess < static_cast<double>(L) * 0.55) {
            didResample = true;
            systematicResample();
        }

        const bool debugDue = CONFIG::PF_DEBUG_ENABLE &&
            (pros::millis() - m_lastDebugTime >= CONFIG::PF_DEBUG_LOG_EVERY_ms);
        if (debugDue) {
            const float correctionDx = m_prediction.x() - priorPrediction.x();
            const float correctionDy = m_prediction.y() - priorPrediction.y();
            const float correctionNorm = std::sqrt(
                correctionDx * correctionDx + correctionDy * correctionDy);

            std::printf(
                "[PFDBG] prior=(%.3f,%.3f,%.3f) post=(%.3f,%.3f,%.3f) corr=(%+.3f,%+.3f)|%.3f delta=(%.3f,%.3f)|%.3f sensors=%zu/%zu abs=%zu used=%c ess=%.1f/%zu resample=%c\n",
                priorPrediction.x(),
                priorPrediction.y(),
                priorPrediction.z(),
                m_prediction.x(),
                m_prediction.y(),
                m_prediction.z(),
                correctionDx,
                correctionDy,
                correctionNorm,
                delta.x(),
                delta.y(),
                delta.norm(),
                activeSensorCount,
                m_sensors.size(),
                activeAbsoluteSensorCount,
                usedMeasurements ? 'Y' : 'N',
                ess,
                L,
                didResample ? 'Y' : 'N');

            if (!usedMeasurements ||
                correctionNorm >= CONFIG::PF_DEBUG_CORRECTION_WARN.getValue()) {
                for (size_t i = 0; i < m_sensors.size(); ++i) {
                    if (m_sensors[i]) m_sensors[i]->debugPrint(m_prediction, i);
                }
            }

            m_lastDebugTime = pros::millis();
        }

        m_lastActiveSensorCount = activeSensorCount;
        m_lastActiveAbsoluteSensorCount = activeAbsoluteSensorCount;
        m_lastUsedMeasurements = usedMeasurements;
        m_lastDidResample = didResample;
        m_lastEss = ess;

        return validatePose();
    }

    Eigen::Vector3f getPrediction() const { return m_prediction; }
    size_t getLastActiveSensorCount() const { return m_lastActiveSensorCount; }
    size_t getLastActiveAbsoluteSensorCount() const { return m_lastActiveAbsoluteSensorCount; }
    bool lastUpdateUsedMeasurements() const { return m_lastUsedMeasurements; }
    bool lastUpdateDidResample() const { return m_lastDidResample; }
    double getLastEss() const { return m_lastEss; }

    std::vector<Eigen::Vector3f> getParticles() const {
        std::vector<Eigen::Vector3f> out(L);
        for (size_t i = 0; i < L; ++i) {
            out[i] = Eigen::Vector3f(m_particles[i].x(), m_particles[i].y(), m_prediction.z());
        }
        return out;
    }

private:
    PredictionFn m_predictionFunction;
    AngleFn m_angleFunction;
    std::vector<SensorModel*> m_sensors;

    size_t L;
    std::vector<Eigen::Vector2f> m_particles;
    std::vector<double> m_weights;
    Eigen::Vector3f m_prediction;
    uint32_t m_lastDebugTime = 0;
    size_t m_lastActiveSensorCount = 0;
    size_t m_lastActiveAbsoluteSensorCount = 0;
    bool m_lastUsedMeasurements = false;
    bool m_lastDidResample = false;
    double m_lastEss = 0.0;

    std::mt19937 m_rng{42};

    void initializeParticles(const Eigen::Vector3f& initialPose) {
        const float localStddev = std::max(CONFIG::DRIVE_NOISE.getValue() * 2.0f, 0.08f);
        std::normal_distribution<float> localNoise(0.0f, localStddev);
        const size_t randomSeeds = std::min(
            L,
            static_cast<size_t>(std::round(static_cast<double>(L) * 0.08)));

        for (size_t i = 0; i < L; ++i) {
            if (i < randomSeeds) {
                m_particles[i] = randomFieldPoint();
            } else {
                m_particles[i] = Eigen::Vector2f(
                    initialPose.x() + localNoise(m_rng),
                    initialPose.y() + localNoise(m_rng));
                constrainToField(m_particles[i]);
            }
        }
    }

    void applyPrediction(const Eigen::Vector2f& delta, bool sensorsAvailable) {
        const float deltaNorm = delta.norm();
        float motionNoise = 0.0f;

        if (deltaNorm >= CONFIG::PF_STATIONARY_DEADBAND.getValue()) {
            motionNoise = CONFIG::DRIVE_NOISE.getValue();
            if constexpr (CONFIG::VERTICAL_TRACKING_PORT == 0) {
                motionNoise *= static_cast<float>(
                    CONFIG::MCL_DRIVE_ENCODER_FALLBACK_NOISE_SCALE);
            }
        } else if (sensorsAvailable) {
            motionNoise = CONFIG::PF_SENSOR_ONLY_EXPLORATION_NOISE.getValue();
        }

        std::normal_distribution<float> translationNoise(0.0f, motionNoise);
        for (auto& particle : m_particles) {
            particle.x() += delta.x() + translationNoise(m_rng);
            particle.y() += delta.y() + translationNoise(m_rng);
            constrainToField(particle);
        }
    }

    double effectiveSampleSize() const {
        double sumSquares = 0.0;
        for (double weight : m_weights) {
            sumSquares += weight * weight;
        }
        return sumSquares > 0.0 ? 1.0 / sumSquares : 0.0;
    }

    void systematicResample() {
        std::vector<Eigen::Vector2f> oldParticles = m_particles;
        std::vector<Eigen::Vector2f> resampledParticles(L);

        const double step = 1.0 / static_cast<double>(L);
        std::uniform_real_distribution<double> startDist(0.0, step);
        const double start = startDist(m_rng);

        size_t index = 0;
        double cumulative = m_weights[0];

        for (size_t i = 0; i < L; ++i) {
            const double threshold = start + static_cast<double>(i) * step;
            while (threshold > cumulative && index + 1 < L) {
                ++index;
                cumulative += m_weights[index];
            }
            resampledParticles[i] = oldParticles[index];
        }

        m_particles = std::move(resampledParticles);

        const float jitterStd =
            CONFIG::DRIVE_NOISE.getValue() * CONFIG::PF_RESAMPLE_JITTER_SCALE;
        std::normal_distribution<float> jitter(0.0f, jitterStd);
        for (auto& particle : m_particles) {
            particle.x() += jitter(m_rng);
            particle.y() += jitter(m_rng);
            constrainToField(particle);
        }

        const size_t injectCount = std::min(
            L,
            static_cast<size_t>(std::round(
                static_cast<double>(L) * CONFIG::PF_RANDOM_INJECTION_FRACTION)));
        for (size_t k = 0; k < injectCount; ++k) {
            const size_t indexToReplace = (k == 0) ? 0 : (m_rng() % L);
            m_particles[indexToReplace] = randomFieldPoint();
        }

        const double uniformWeight = 1.0 / static_cast<double>(L);
        std::fill(m_weights.begin(), m_weights.end(), uniformWeight);
    }

    Eigen::Vector3f validatePose() {
        if (LocMath::isFinitePose(m_prediction)) return m_prediction;

        std::printf("[PF] non-finite pose (%f,%f,%f), resetting\n",
                    m_prediction.x(),
                    m_prediction.y(),
                    m_prediction.z());
        m_prediction = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        return m_prediction;
    }

    Eigen::Vector3f computeWeightedMean(float heading) const {
        Eigen::Vector2d sum(0.0, 0.0);
        double totalWeight = 0.0;

        for (size_t i = 0; i < L; ++i) {
            sum.x() += static_cast<double>(m_particles[i].x()) * m_weights[i];
            sum.y() += static_cast<double>(m_particles[i].y()) * m_weights[i];
            totalWeight += m_weights[i];
        }

        if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
            return computeMean(heading);
        }

        sum /= totalWeight;
        return Eigen::Vector3f(
            static_cast<float>(sum.x()),
            static_cast<float>(sum.y()),
            heading);
    }

    Eigen::Vector3f computeMean(float heading) const {
        Eigen::Vector2f sum(0.0f, 0.0f);
        for (const auto& particle : m_particles) sum += particle;
        sum /= static_cast<float>(L);
        return Eigen::Vector3f(sum.x(), sum.y(), heading);
    }

    static void constrainToField(Eigen::Vector2f& particle) {
        const float H = CONFIG::FIELD_HALF_SIZE.getValue();
        particle.x() = std::clamp(particle.x(), -H, H);
        particle.y() = std::clamp(particle.y(), -H, H);
    }

    Eigen::Vector2f randomFieldPoint() {
        std::uniform_real_distribution<float> axis(
            -CONFIG::FIELD_HALF_SIZE.getValue(),
            CONFIG::FIELD_HALF_SIZE.getValue());
        return Eigen::Vector2f(axis(m_rng), axis(m_rng));
    }
};
