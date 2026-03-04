/**
 * @file particleFilter.h
 * Monte Carlo localisation — particle filter with systematic resampling.
 *
 * Maintains an ensemble of L particles (x, y) sharing a common heading
 * from an external source (IMU).  Sensor likelihoods are multiplied
 * together, and low-variance (systematic) resampling keeps the ensemble
 * healthy.
 *
 * Rate-limits full Bayesian updates: if the robot has not moved far
 * enough or enough time has not elapsed, only the cheap prediction mean
 * is returned.
 *
 * Field boundary enforcement: particles outside ±FIELD_HALF_SIZE are
 * re-randomized.
 */
#pragma once

#include "Eigen/Dense"
#include "localization/sensor.h"
#include "config.h"
#include "pros/rtos.hpp"

#include <vector>
#include <functional>
#include <random>
#include <cmath>
#include <cstdio>
#include <optional>
#include <algorithm>

class ParticleFilter {
public:
    using PredictionFn = std::function<Eigen::Vector2f()>;
    using AngleFn      = std::function<QAngle()>;

    // ── Construction ────────────────────────────────────────────────────

    /**
     * @param predictionFunction  returns 2-D displacement since last call
     * @param angleFunction       returns current heading (shared across particles)
     * @param sensors             sensor likelihood models
     * @param initialPose         starting (x, y, θ)
     * @param numParticles        ensemble size (default 250)
     */
    ParticleFilter(
        PredictionFn predictionFunction,
        AngleFn      angleFunction,
        std::vector<SensorModel*> sensors,
        Eigen::Vector3f initialPose = Eigen::Vector3f(0, 0, 0),
        size_t numParticles = CONFIG::NUM_PARTICLES)
        : m_predictionFunction(std::move(predictionFunction))
        , m_angleFunction(std::move(angleFunction))
        , m_sensors(std::move(sensors))
        , L(numParticles)
        , m_prediction(initialPose)
    {
        m_particles.resize(L);
        m_weights.resize(L, 1.0);
        // Initialise particles around initial pose
        std::normal_distribution<float> noiseDist(0.0f, CONFIG::DRIVE_NOISE);
        for (size_t i = 0; i < L; ++i) {
            m_particles[i] = Eigen::Vector2f(
                initialPose.x() + noiseDist(m_rng),
                initialPose.y() + noiseDist(m_rng));
        }
        m_lastUpdateTime = pros::millis();
    }

    // ── Main update (call each scheduler tick) ──────────────────────────

    Eigen::Vector3f update() {
        // Guard: if heading source is invalid, return last prediction
        QAngle heading = m_angleFunction();
        if (!std::isfinite(heading.getValue())) return m_prediction;

        // --- 1. Prediction: sample odometry once and propagate all particles ---
        Eigen::Vector2f delta = m_predictionFunction();
        float motionNoise = CONFIG::DRIVE_NOISE;
        if constexpr (CONFIG::VERTICAL_TRACKING_PORT == 0) {
            motionNoise *= static_cast<float>(CONFIG::MCL_DRIVE_ENCODER_FALLBACK_NOISE_SCALE);
        }
        std::normal_distribution<float> translationNoise(0.0f, motionNoise);

        for (size_t i = 0; i < L; ++i) {
            m_particles[i] += delta + Eigen::Vector2f(
                translationNoise(m_rng), translationNoise(m_rng));
        }

        // Accumulate distance for rate limiting (ignore tiny stationary creep)
        float deltaNorm = delta.norm();
        if (deltaNorm >= CONFIG::PF_STATIONARY_DEADBAND_M) {
            m_distanceSinceUpdate += deltaNorm;
        }

        // --- 2. Rate-limit check ---
        uint32_t now = pros::millis();
        bool tooSoon = m_distanceSinceUpdate < CONFIG::MAX_DISTANCE_SINCE_UPDATE;
        bool tooRecent = (now - m_lastUpdateTime) < static_cast<uint32_t>(CONFIG::MAX_UPDATE_INTERVAL_MS);

        if (tooSoon && tooRecent) {
            // Cheap estimate: just return mean of current particles
            m_prediction = computeMean(heading.getValue());
            return m_prediction;
        }

        // --- 3. Sensor update ---
        for (auto* s : m_sensors) s->update();

        // --- 4. Weight computation (product of per-sensor likelihoods) ---
        double totalWeight = 0.0;
        for (size_t i = 0; i < L; ++i) {
            Eigen::Vector3f particlePose(
                m_particles[i].x(), m_particles[i].y(), heading.getValue());

            double w = 1.0;
            for (auto* sensor : m_sensors) {
                auto likelihood = sensor->p(particlePose);
                if (likelihood && std::isfinite(likelihood.value())) {
                    w *= likelihood.value();
                }
            }

            // Field-boundary enforcement
            if (outOfField(m_particles[i])) {
                m_particles[i] = randomFieldPoint();
                w = 1.0 / static_cast<double>(L);  // minimal weight
            }

            m_weights[i] = w;
            totalWeight += w;
        }

        if (totalWeight <= 0.0) {
            std::printf("Warning: Total weight equal to 0\n");
            m_prediction = computeMean(heading.getValue());
            return m_prediction;
        }

        // Normalize weights and compute ESS
        for (double& w : m_weights) w /= totalWeight;
        double sumSq = 0.0;
        for (double w : m_weights) sumSq += w * w;
        double ess = (sumSq > 0.0) ? (1.0 / sumSq) : 0.0;

        // Weighted posterior before any optional resampling
        m_prediction = computeWeightedMean(heading.getValue());

        // --- 5. ESS-gated systematic resampling  O(L) ---
        const double essThreshold = 0.5 * static_cast<double>(L);
        if (ess < essThreshold) {
            std::vector<Eigen::Vector2f> oldParticles = m_particles;

            const double step = 1.0 / static_cast<double>(L);
            std::uniform_real_distribution<double> dist(0.0, step);
            const double start = dist(m_rng);

            size_t j = 0;
            double cumulativeWeight = m_weights[0];

            for (size_t i = 0; i < L; ++i) {
                const double threshold = start + static_cast<double>(i) * step;
                while (threshold > cumulativeWeight && j + 1 < L) {
                    ++j;
                    cumulativeWeight += m_weights[j];
                }
                m_particles[i] = oldParticles[j];
            }

            std::fill(m_weights.begin(), m_weights.end(), 1.0 / static_cast<double>(L));
        }

        // --- 6. Posterior estimate is already weighted-pre-resample ---
        m_lastUpdateTime = now;
        m_distanceSinceUpdate = 0.0f;

        return m_prediction;
    }

    // ── Accessors ───────────────────────────────────────────────────────

    Eigen::Vector3f getPrediction() const { return m_prediction; }

    /** Return all particles (with shared heading injected). */
    std::vector<Eigen::Vector3f> getParticles() const {
        float theta = m_prediction.z();
        std::vector<Eigen::Vector3f> out(L);
        for (size_t i = 0; i < L; ++i)
            out[i] = Eigen::Vector3f(m_particles[i].x(), m_particles[i].y(), theta);
        return out;
    }

private:
    PredictionFn m_predictionFunction;
    AngleFn      m_angleFunction;
    std::vector<SensorModel*> m_sensors;

    size_t L;  // particle count

    std::vector<Eigen::Vector2f> m_particles;
    std::vector<double>          m_weights;
    Eigen::Vector3f              m_prediction;

    uint32_t m_lastUpdateTime    = 0;
    float    m_distanceSinceUpdate = 0.0f;

    std::mt19937 m_rng{42};

    // ── Helpers ─────────────────────────────────────────────────────────

    Eigen::Vector3f computeMean(float heading) const {
        Eigen::Vector2f sum(0.0f, 0.0f);
        for (const auto& p : m_particles) sum += p;
        sum /= static_cast<float>(L);
        return Eigen::Vector3f(sum.x(), sum.y(), heading);
    }

    Eigen::Vector3f computeWeightedMean(float heading) const {
        Eigen::Vector2d sum(0.0, 0.0);
        double total = 0.0;
        for (size_t i = 0; i < L; ++i) {
            const double w = m_weights[i];
            sum.x() += static_cast<double>(m_particles[i].x()) * w;
            sum.y() += static_cast<double>(m_particles[i].y()) * w;
            total += w;
        }

        if (total <= 0.0) return computeMean(heading);

        sum /= total;
        return Eigen::Vector3f(static_cast<float>(sum.x()),
                               static_cast<float>(sum.y()),
                               heading);
    }

    static bool outOfField(const Eigen::Vector2f& p) {
        const float H = CONFIG::FIELD_HALF_SIZE;
        return std::fabs(p.x()) > H || std::fabs(p.y()) > H;
    }

    Eigen::Vector2f randomFieldPoint() {
        std::uniform_real_distribution<float> d(
            -CONFIG::FIELD_HALF_SIZE, CONFIG::FIELD_HALF_SIZE);
        return Eigen::Vector2f(d(m_rng), d(m_rng));
    }
};
