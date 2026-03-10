/**
 * @file controllerPoseGuard.h
 * Guard fused localization before exposing it to motion controllers.
 *
 * Autonomous controllers should not consume the UI/debug fused pose stream
 * directly. This guard keeps odometry as the base pose and lets fused XY
 * corrections in only if they stay finite, bounded, and non-snappy.
 */
#pragma once

#include "Eigen/Dense"
#include "config.h"
#include "utils/localization_math.h"

class ControllerPoseGuard {
public:
    struct UpdateResult {
        Eigen::Vector3f pose{0.0f, 0.0f, 0.0f};
        Eigen::Vector2f correction{0.0f, 0.0f};
        Eigen::Vector2f targetCorrection{0.0f, 0.0f};
        bool usedFusedTarget = false;
        bool rejectedSnap = false;
        bool acceptedStableJump = false;
        bool decayingToOdom = false;
        bool usedOdomFallback = false;
        int stableJumpCycles = 0;
    };

    ControllerPoseGuard(
        float maxCorrection = CONFIG::LOC_CONTROLLER_CORRECTION_MAX.convert(meter),
        float maxStep = CONFIG::LOC_CONTROLLER_CORRECTION_STEP.convert(meter),
        float deadband = CONFIG::LOC_CONTROLLER_CORRECTION_DEADBAND.convert(meter),
        float jumpReject = CONFIG::LOC_CONTROLLER_CORRECTION_JUMP_REJECT.convert(meter),
        float reacquireDeadband = CONFIG::LOC_CONTROLLER_REACQUIRE_DEADBAND.convert(meter),
        int reacquireStableCycles = CONFIG::LOC_CONTROLLER_REACQUIRE_STABLE_CYCLES)
        : m_maxCorrection(maxCorrection)
        , m_maxStep(maxStep)
        , m_deadband(deadband)
        , m_jumpReject(jumpReject)
        , m_reacquireDeadband(reacquireDeadband)
        , m_reacquireStableCycles(reacquireStableCycles) {}

    UpdateResult update(const Eigen::Vector3f& fusedPose,
                        const Eigen::Vector3f& odomPose) {
        UpdateResult result;

        if (!LocMath::isFinitePose(odomPose)) {
            m_correction = Eigen::Vector2f(0.0f, 0.0f);
            clearPendingTarget();
            if (!LocMath::isFinitePose(m_pose)) {
                m_pose = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
            }
            result.pose = m_pose;
            result.usedOdomFallback = true;
            return result;
        }

        if (!LocMath::isFiniteVec2(m_correction)) {
            m_correction = Eigen::Vector2f(0.0f, 0.0f);
        }

        Eigen::Vector2f targetCorrection(0.0f, 0.0f);
        if (LocMath::isFinitePose(fusedPose)) {
            const Eigen::Vector2f fusedCorrection(
                fusedPose.x() - odomPose.x(),
                fusedPose.y() - odomPose.y());
            targetCorrection = LocMath::clampVectorMagnitude(
                fusedCorrection, m_maxCorrection);

            const float jump = (targetCorrection - m_correction).norm();
            if (jump > m_jumpReject) {
                const bool stableCandidate =
                    LocMath::isFiniteVec2(m_pendingTargetCorrection) &&
                    (targetCorrection - m_pendingTargetCorrection).norm() <=
                        m_reacquireDeadband;
                m_pendingTargetCorrection = targetCorrection;
                m_pendingStableCycles = stableCandidate
                    ? (m_pendingStableCycles + 1)
                    : 1;
                result.stableJumpCycles = m_pendingStableCycles;

                if (m_pendingStableCycles >= m_reacquireStableCycles) {
                    targetCorrection = LocMath::applyTargetDeadband(
                        m_correction,
                        m_pendingTargetCorrection,
                        m_deadband);
                    result.usedFusedTarget = true;
                    result.acceptedStableJump = true;
                } else {
                    targetCorrection = m_correction;
                    result.rejectedSnap = true;
                }
            } else {
                clearPendingTarget();
                targetCorrection = LocMath::applyTargetDeadband(
                    m_correction, targetCorrection, m_deadband);
                result.usedFusedTarget = true;
            }
        } else {
            clearPendingTarget();
            result.decayingToOdom = true;
        }

        m_correction = LocMath::moveTowardsVec2(
            m_correction, targetCorrection, m_maxStep);
        if (!LocMath::isFiniteVec2(m_correction)) {
            m_correction = Eigen::Vector2f(0.0f, 0.0f);
            result.usedOdomFallback = true;
        }

        Eigen::Vector2f guardedXY(
            odomPose.x() + m_correction.x(),
            odomPose.y() + m_correction.y());
        if (!LocMath::isFiniteVec2(guardedXY)) {
            guardedXY = Eigen::Vector2f(odomPose.x(), odomPose.y());
            m_correction = Eigen::Vector2f(0.0f, 0.0f);
            result.usedOdomFallback = true;
        }

        m_pose = Eigen::Vector3f(guardedXY.x(), guardedXY.y(), odomPose.z());
        result.pose = m_pose;
        result.correction = m_correction;
        result.targetCorrection = targetCorrection;
        return result;
    }

    void reset(const Eigen::Vector3f& pose) {
        m_correction = Eigen::Vector2f(0.0f, 0.0f);
        clearPendingTarget();
        m_pose = LocMath::isFinitePose(pose)
            ? pose
            : Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }

    Eigen::Vector3f pose() const { return m_pose; }
    Eigen::Vector2f correction() const { return m_correction; }

private:
    void clearPendingTarget() {
        m_pendingTargetCorrection = Eigen::Vector2f(0.0f, 0.0f);
        m_pendingStableCycles = 0;
    }

    float m_maxCorrection = 0.0f;
    float m_maxStep = 0.0f;
    float m_deadband = 0.0f;
    float m_jumpReject = 0.0f;
    float m_reacquireDeadband = 0.0f;
    int m_reacquireStableCycles = 0;
    Eigen::Vector2f m_correction{0.0f, 0.0f};
    Eigen::Vector2f m_pendingTargetCorrection{0.0f, 0.0f};
    int m_pendingStableCycles = 0;
    Eigen::Vector3f m_pose{0.0f, 0.0f, 0.0f};
};
