/**
 * @file localizationFusion.h
 * Odom-first sensor-fusion controller.
 *
 * Drivetrain odometry provides the backbone pose.  GPS (when stationary) and
 * the MCL particle filter (when moving) contribute bounded XY corrections
 * that are rate-limited and deadband-filtered to avoid jitter.
 *
 * Extracted from the monolithic computeCombinedPose() in main.cpp so that the
 * fusion logic is testable and self-contained.
 */
#pragma once

#include "Eigen/Dense"
#include "config.h"
#include "localization/gps.h"
#include "localization/particleFilter.h"
#include "subsystems/drivetrain.h"
#include "utils/localization_math.h"

#include <functional>
#include <optional>

class LocalizationFusion {
public:
    /** Lightweight GPS pose snapshot. */
    struct GpsPoseSample {
        Eigen::Vector3f pose{0.0f, 0.0f, 0.0f};
        float errorM = -1.0f;
    };

    /** Callable that returns the latest GPS sample (or nullopt). */
    using GpsSampleFn = std::function<std::optional<GpsPoseSample>()>;

    /**
     * @param drivetrain   Reference to the drivetrain (static lifetime).
     * @param pf           Pointer to the particle filter (non-owning, may be null
     *                     during early boot but must be valid before update() is called).
     * @param gpsSampleFn  Callable returning the latest GPS sample.
     */
    LocalizationFusion(Drivetrain& drivetrain,
                       ParticleFilter* pf,
                       GpsSampleFn gpsSampleFn)
        : m_drivetrain(drivetrain)
        , m_pf(pf)
        , m_gpsSampleFn(std::move(gpsSampleFn)) {}

    /**
     * Run one fusion cycle.
     *
     * Reads drivetrain odometry and, depending on motion state, blends in
     * either a GPS correction (when still) or an MCL correction (when moving).
     * The correction is rate-limited and deadband-filtered.
     *
     * @return  The fused pose (x, y in metres; θ in radians).
     */
    Eigen::Vector3f update() {
        Eigen::Vector3f odom = m_drivetrain.getOdomPose();
        if (!LocMath::isFinitePose(odom)) {
            m_correction = Eigen::Vector2f(0.0f, 0.0f);
            m_combinedPose = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
            return m_combinedPose;
        }

        const float motionNorm =
            m_drivetrain.getLastStepDisplacementDebug().norm();
        const bool still =
            motionNorm <= CONFIG::LOC_FUSION_STILLNESS_DEADBAND.convert(meter);

        bool haveTarget = false;
        Eigen::Vector2f targetCorrection = m_correction;
        float maxCorrection = 0.0f;
        float maxStep = 0.0f;
        float correctionDeadband = 0.0f;

        const std::optional<GpsPoseSample> gpsSample = m_gpsSampleFn();
        if (still && gpsSample &&
            gpsSample->errorM <=
                CONFIG::LOC_GPS_RUNTIME_ERROR_MAX.convert(meter)) {
            targetCorrection = Eigen::Vector2f(
                gpsSample->pose.x() - odom.x(),
                gpsSample->pose.y() - odom.y());
            maxCorrection = CONFIG::LOC_GPS_CORRECTION_MAX.convert(meter);
            maxStep = CONFIG::LOC_GPS_CORRECTION_STEP.convert(meter);
            correctionDeadband =
                CONFIG::LOC_GPS_CORRECTION_DEADBAND.convert(meter);
            haveTarget = true;
        } else if (!still && m_pf) {
            const Eigen::Vector3f pf = m_pf->getPrediction();
            const Eigen::Vector2f pfCorrection(
                pf.x() - odom.x(),
                pf.y() - odom.y());
            const float correctionJump =
                (pfCorrection - m_correction).norm();

            const double minEss =
                static_cast<double>(CONFIG::NUM_PARTICLES) *
                static_cast<double>(CONFIG::LOC_MCL_MIN_ESS_RATIO);

            if (LocMath::isFinitePose(pf) &&
                m_pf->lastUpdateUsedMeasurements() &&
                m_pf->getLastEss() >= minEss &&
                (m_pf->getLastActiveAbsoluteSensorCount() > 0 ||
                 m_pf->getLastActiveSensorCount() >=
                    static_cast<size_t>(
                        CONFIG::LOC_MCL_MIN_ACTIVE_SENSORS)) &&
                correctionJump <=
                    CONFIG::LOC_MCL_CORRECTION_JUMP_REJECT.convert(
                        meter)) {
                targetCorrection = pfCorrection;
                maxCorrection =
                    CONFIG::LOC_MCL_CORRECTION_MAX.convert(meter);
                maxStep =
                    CONFIG::LOC_MCL_CORRECTION_STEP.convert(meter);
                correctionDeadband =
                    CONFIG::LOC_MCL_CORRECTION_DEADBAND.convert(meter);
                haveTarget = true;
            }
        }

        if (haveTarget) {
            targetCorrection = LocMath::clampVectorMagnitude(
                targetCorrection, maxCorrection);
            targetCorrection = LocMath::applyTargetDeadband(
                m_correction, targetCorrection, correctionDeadband);
            m_correction = LocMath::moveTowardsVec2(
                m_correction, targetCorrection, maxStep);
        } else if (!LocMath::isFiniteVec2(m_correction)) {
            m_correction = Eigen::Vector2f(0.0f, 0.0f);
        }

        Eigen::Vector2f fusedXY(
            odom.x() + m_correction.x(),
            odom.y() + m_correction.y());
        if (!LocMath::isFiniteVec2(fusedXY)) {
            m_correction = Eigen::Vector2f(0.0f, 0.0f);
            fusedXY = Eigen::Vector2f(odom.x(), odom.y());
        }

        m_combinedPose =
            Eigen::Vector3f(fusedXY.x(), fusedXY.y(), odom.z());
        return m_combinedPose;
    }

    /** Reset correction accumulator and combined pose (e.g. after re-init). */
    void reset(const Eigen::Vector3f& pose) {
        m_correction = Eigen::Vector2f(0.0f, 0.0f);
        m_combinedPose = pose;
    }

    Eigen::Vector3f getCombinedPose() const { return m_combinedPose; }
    Eigen::Vector2f getCorrection() const { return m_correction; }

private:
    Drivetrain& m_drivetrain;
    ParticleFilter* m_pf;
    GpsSampleFn m_gpsSampleFn;

    Eigen::Vector2f m_correction{0.0f, 0.0f};
    Eigen::Vector3f m_combinedPose{0.0f, 0.0f, 0.0f};
};
