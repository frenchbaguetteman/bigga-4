/**
 * @file ltvUnicycleController.h
 * Finite-horizon LTV-LQR unicycle path-tracking controller.
 *
 * Builds a time-varying linearisation of the unicycle tracking-error dynamics
 * along the desired trajectory, discretises each step, then solves the
 * backward Riccati recursion to pre-compute a gain schedule K[k].
 *
 * State error:  e = [x_err, y_err, θ_err]ᵀ  (robot frame)
 * Control:      u = [Δv, Δω]ᵀ  added on top of profile feedforward
 *
 * Runtime command:
 *     u_cmd = u_desired - K(t) e
 */
#pragma once

#include "motionProfiling/pathCommand.h"
#include "motionProfiling/motionProfile.h"
#include "subsystems/drivetrain.h"
#include "telemetry/telemetry.h"
#include "config.h"
#include "utils/localization_math.h"
#include "utils/utils.h"
#include "Eigen/Dense"
#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

class LtvUnicycleCommand : public PathCommand {
public:
    using StateVector = Eigen::Vector3f;
    using ControlVector = Eigen::Vector2f;
    using StateMatrix = Eigen::Matrix3f;
    using ControlMatrix = Eigen::Matrix2f;
    using InputMatrix = Eigen::Matrix<float, 3, 2>;
    using GainMatrix = Eigen::Matrix<float, 2, 3>;

private:
    struct DiscreteModel {
        StateMatrix A = StateMatrix::Identity();
        InputMatrix B = InputMatrix::Zero();
    };

    static constexpr float kAngularEpsilon = 1e-4f;
    static constexpr float kCostEpsilon = 1e-4f;

    static ControlMatrix defaultControlCost() {
        return ControlMatrix::Identity();
    }

    static StateMatrix diagonalStateCost(const StateVector& weights) {
        StateMatrix diagonal = StateMatrix::Zero();
        diagonal(0, 0) = std::max(std::fabs(weights.x()), kCostEpsilon);
        diagonal(1, 1) = std::max(std::fabs(weights.y()), kCostEpsilon);
        diagonal(2, 2) = std::max(std::fabs(weights.z()), kCostEpsilon);
        return diagonal;
    }

    static StateMatrix sanitizeStateCost(const StateMatrix& cost) {
        StateMatrix sanitized = 0.5f * (cost + cost.transpose());
        sanitized = sanitized + kCostEpsilon * StateMatrix::Identity();
        return sanitized;
    }

    static ControlMatrix sanitizeControlCost(const ControlMatrix& cost) {
        ControlMatrix sanitized = 0.5f * (cost + cost.transpose());
        sanitized = sanitized + kCostEpsilon * ControlMatrix::Identity();
        return sanitized;
    }

    static ControlMatrix inverse2x2(const ControlMatrix& matrix) {
        ControlMatrix inverse = ControlMatrix::Identity();
        float determinant = matrix(0, 0) * matrix(1, 1) - matrix(0, 1) * matrix(1, 0);
        if (std::fabs(determinant) < kCostEpsilon) {
            determinant = (determinant < 0.0f) ? -kCostEpsilon : kCostEpsilon;
        }

        const float invDet = 1.0f / determinant;
        inverse(0, 0) =  matrix(1, 1) * invDet;
        inverse(0, 1) = -matrix(0, 1) * invDet;
        inverse(1, 0) = -matrix(1, 0) * invDet;
        inverse(1, 1) =  matrix(0, 0) * invDet;
        return inverse;
    }

    static StateVector trackingError(const StateVector& currentPose,
                                     const StateVector& desiredPose) {
        StateVector error = StateVector::Zero();
        error.head<2>() =
            Eigen::Rotation2Df(-currentPose.z()) *
            (desiredPose - currentPose).head<2>();
        error.z() = utils::angleDifference(desiredPose.z(), currentPose.z());
        return error;
    }

    static DiscreteModel discreteTrackingModel(float desiredVelocity,
                                               float desiredAngularVelocity,
                                               float dt) {
        DiscreteModel model;
        if (dt <= 0.0f) {
            return model;
        }

        if (std::fabs(desiredAngularVelocity) < kAngularEpsilon) {
            model.A(1, 2) = desiredVelocity * dt;

            model.B(0, 0) = -dt;
            model.B(1, 1) = -0.5f * desiredVelocity * dt * dt;
            model.B(2, 1) = -dt;
            return model;
        }

        const float w = desiredAngularVelocity;
        const float wt = w * dt;
        const float cosWt = std::cos(wt);
        const float sinWt = std::sin(wt);
        const float invW = 1.0f / w;
        const float invW2 = invW * invW;

        model.A(0, 0) = cosWt;
        model.A(0, 1) = sinWt;
        model.A(0, 2) = desiredVelocity * (1.0f - cosWt) * invW;
        model.A(1, 0) = -sinWt;
        model.A(1, 1) = cosWt;
        model.A(1, 2) = desiredVelocity * sinWt * invW;
        model.A(2, 0) = 0.0f;
        model.A(2, 1) = 0.0f;
        model.A(2, 2) = 1.0f;

        model.B(0, 0) = -sinWt * invW;
        model.B(0, 1) = -desiredVelocity * (dt * invW - sinWt * invW2);
        model.B(1, 0) = (1.0f - cosWt) * invW;
        model.B(1, 1) = -desiredVelocity * ((1.0f - cosWt) * invW2);
        model.B(2, 0) = 0.0f;
        model.B(2, 1) = -dt;

        return model;
    }

    void buildGainSchedule() {
        m_gainSchedule.clear();
        m_scheduleDt = 0.0f;

        const auto& samples = m_profile.samples();
        if (samples.size() < 2 || m_profile.totalTime() <= 0.0f) {
            m_gainSchedule.push_back(GainMatrix::Zero());
            return;
        }

        m_scheduleDt = m_profile.totalTime() / static_cast<float>(samples.size() - 1);
        std::vector<StateMatrix> riccati(samples.size(), StateMatrix::Zero());
        riccati.back() = m_terminalQ;
        m_gainSchedule.resize(samples.size(), GainMatrix::Zero());

        for (int k = static_cast<int>(samples.size()) - 2; k >= 0; --k) {
            const DiscreteModel model =
                discreteTrackingModel(samples[k].linearVelocity,
                                      samples[k].angularVelocity,
                                      m_scheduleDt);

            const StateMatrix& pNext = riccati[static_cast<size_t>(k + 1)];
            ControlMatrix solveMatrix = m_r + model.B.transpose() * pNext * model.B;
            solveMatrix = sanitizeControlCost(solveMatrix);

            const GainMatrix gain =
                inverse2x2(solveMatrix) * (model.B.transpose() * pNext * model.A);
            m_gainSchedule[static_cast<size_t>(k)] = gain;

            StateMatrix pCurrent =
                m_q + model.A.transpose() * pNext * (model.A - model.B * gain);
            riccati[static_cast<size_t>(k)] = sanitizeStateCost(pCurrent);
        }

        m_gainSchedule.back() =
            (m_gainSchedule.size() > 1) ? m_gainSchedule[m_gainSchedule.size() - 2]
                                        : GainMatrix::Zero();
    }

    GainMatrix gainAtTime(float t) const {
        if (m_gainSchedule.empty()) {
            return GainMatrix::Zero();
        }
        if (m_gainSchedule.size() == 1 || m_profile.totalTime() <= 0.0f) {
            return m_gainSchedule.front();
        }

        const float clampedTime = std::clamp(t, 0.0f, m_profile.totalTime());
        const float position =
            clampedTime / m_profile.totalTime() * static_cast<float>(m_gainSchedule.size() - 1);
        const size_t index = static_cast<size_t>(position);
        const float alpha = position - static_cast<float>(index);

        if (index + 1 >= m_gainSchedule.size()) {
            return m_gainSchedule.back();
        }

        return (1.0f - alpha) * m_gainSchedule[index]
             + alpha * m_gainSchedule[index + 1];
    }

public:

    LtvUnicycleCommand(Drivetrain* drivetrain,
                       MotionProfile profile,
                       std::function<Eigen::Vector3f()> poseSource,
                       Eigen::Vector3f qWeights = CONFIG::defaultDtCostQ())
        : LtvUnicycleCommand(
            drivetrain,
            std::move(profile),
            std::move(poseSource),
            diagonalStateCost(qWeights),
            defaultControlCost(),
            diagonalStateCost(qWeights)) {}

    LtvUnicycleCommand(Drivetrain* drivetrain,
                       MotionProfile profile,
                       std::function<Eigen::Vector3f()> poseSource,
                       const StateMatrix& qWeights,
                       const ControlMatrix& rWeights = defaultControlCost())
        : LtvUnicycleCommand(
            drivetrain,
            std::move(profile),
            std::move(poseSource),
            qWeights,
            rWeights,
            qWeights) {}

    LtvUnicycleCommand(Drivetrain* drivetrain,
                       MotionProfile profile,
                       std::function<Eigen::Vector3f()> poseSource,
                       const StateMatrix& qWeights,
                       const ControlMatrix& rWeights,
                       const StateMatrix& terminalCost)
        : PathCommand(drivetrain, std::move(profile))
        , m_poseSource(std::move(poseSource))
        , m_q(sanitizeStateCost(qWeights))
        , m_r(sanitizeControlCost(rWeights))
        , m_terminalQ(sanitizeStateCost(terminalCost)) {
        buildGainSchedule();
    }

    void initialize() override {
        PathCommand::initialize();
    }

    void execute() override {
        const float t = elapsedSeconds();
        const ProfileState desired = m_profile.sample(t);

        const StateVector currentPose = samplePose();
        const StateVector desiredPose = desired.pose;
        const StateVector error = trackingError(currentPose, desiredPose);

        const GainMatrix gain = gainAtTime(t);
        const ControlVector desiredControl(desired.linearVelocity, desired.angularVelocity);
        const ControlVector correction = -(gain * error);
        ControlVector commandedControl = desiredControl + correction;
        commandedControl.x() = utils::clamp(
            commandedControl.x(),
            -CONFIG::MAX_SPEED.convert(mps),
            CONFIG::MAX_SPEED.convert(mps));
        commandedControl.y() = utils::clamp(
            commandedControl.y(),
            -CONFIG::MAX_ANGULAR_VEL.convert(radps),
            CONFIG::MAX_ANGULAR_VEL.convert(radps));

        m_drivetrain->setDriveSpeeds({commandedControl.x(), commandedControl.y()});

        Telemetry::send(t, currentPose, desiredPose);
    }

private:
    StateVector samplePose() const {
        const StateVector rawPose =
            m_poseSource ? m_poseSource() : StateVector(0.0f, 0.0f, 0.0f);
        const StateVector odomPose =
            m_drivetrain ? m_drivetrain->getOdomPose() : StateVector(0.0f, 0.0f, 0.0f);
        return LocMath::finitePoseOr(rawPose, odomPose);
    }

    std::function<Eigen::Vector3f()> m_poseSource;
    StateMatrix m_q = StateMatrix::Identity();
    ControlMatrix m_r = defaultControlCost();
    StateMatrix m_terminalQ = StateMatrix::Identity();
    std::vector<GainMatrix> m_gainSchedule;
    float m_scheduleDt = 0.0f;
};
