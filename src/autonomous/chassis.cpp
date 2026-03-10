#include "autonomous/chassis.h"

#include "command/commandGroup.h"
#include "commands/driveMove.h"
#include "commands/rotate.h"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <mutex>

namespace {

constexpr float kAutonPi = 3.14159265358979323846f;
constexpr float kInToM = 0.0254f;

} // namespace

Chassis::Chassis(Drivetrain* drivetrain,
                 std::function<Eigen::Vector3f()> poseSource,
                 std::function<bool()> cancelRequested)
    : m_drivetrain(drivetrain)
    , m_poseSource(std::move(poseSource))
    , m_cancelRequested(std::move(cancelRequested))
    , m_motionMutex(std::make_unique<pros::RecursiveMutex>()) {
    const Eigen::Vector3f pose = currentPose();
    m_headingZeroRad = pose.z();
    m_headingHoldRad = pose.z();
    m_localOriginPose = pose;
}

void Chassis::pid_targets_reset() {
    const Eigen::Vector3f pose = currentPose();
    m_headingZeroRad = pose.z();
    m_headingHoldRad = pose.z();
}

void Chassis::set_global_frame() {
    m_coordinateFrame = CoordinateFrame::Global;
}

void Chassis::set_local_frame() {
    m_coordinateFrame = CoordinateFrame::Local;
}

Chassis::CoordinateFrame Chassis::current_frame() const {
    return m_coordinateFrame;
}

void Chassis::set_local_origin_here() {
    m_localOriginPose = currentPose();
}

void Chassis::set_local_origin(double xInches, double yInches, double headingDeg) {
    m_localOriginPose = Eigen::Vector3f(
        inchesToMeters(xInches),
        inchesToMeters(yInches),
        ezDegreesToInternalRadians(headingDeg));
}

Eigen::Vector3f Chassis::local_origin() const {
    return m_localOriginPose;
}

Eigen::Vector3f Chassis::pose_to_global(const Eigen::Vector3f& pose,
                                        CoordinateFrame frame) const {
    if (frame == CoordinateFrame::Global) {
        return pose;
    }

    const float cosT = std::cos(m_localOriginPose.z());
    const float sinT = std::sin(m_localOriginPose.z());
    return Eigen::Vector3f(
        m_localOriginPose.x() + pose.x() * cosT - pose.y() * sinT,
        m_localOriginPose.y() + pose.x() * sinT + pose.y() * cosT,
        wrapRadians(m_localOriginPose.z() + pose.z()));
}

Eigen::Vector2f Chassis::point_to_global(const ez::pose& point,
                                         CoordinateFrame frame) const {
    return pose_to_global(
               Eigen::Vector3f(inchesToMeters(point.x), inchesToMeters(point.y), 0.0f),
               frame)
        .head<2>();
}

void Chassis::drive_imu_reset() {
    if (!m_drivetrain) return;
    m_drivetrain->resetHeading(0.0f);
    m_headingZeroRad = 0.0f;
    m_headingHoldRad = 0.0f;
    m_localOriginPose = currentPose();
}

void Chassis::drive_sensor_reset() {
    if (!m_drivetrain) return;
    m_drivetrain->resetEncoders();
}

void Chassis::odom_xyt_set(double xInches, double yInches, double headingDeg) {
    if (!m_drivetrain) return;

    const Eigen::Vector3f pose(
        inchesToMeters(xInches),
        inchesToMeters(yInches),
        ezDegreesToInternalRadians(headingDeg));
    m_drivetrain->syncLocalizationReference(pose);
    m_headingZeroRad = pose.z();
    m_headingHoldRad = pose.z();
    m_localOriginPose = pose;
}

void Chassis::pid_drive_set(double targetInches, int speed, bool /*slewOn*/) {
    if (!m_drivetrain || cancelled()) return;

    const Eigen::Vector3f start = currentPose();
    const float targetM = inchesToMeters(targetInches);
    const Eigen::Vector2f target(
        start.x() + targetM * std::cos(m_headingHoldRad),
        start.y() + targetM * std::sin(m_headingHoldRad));
    const Eigen::Vector3f targetPose(target.x(), target.y(), m_headingHoldRad);

    startMotion(
        std::make_unique<DriveMoveCommand>(
            m_drivetrain,
            target,
            m_poseSource,
            DriveMoveCommand::MotionMode::HoldHeading,
            m_headingHoldRad,
            targetM >= 0.0f ? 1 : -1,
            0.02f,
            speed),
        MotionKind::Drive,
        start,
        targetPose,
        {target});
}

void Chassis::pid_odom_set(double targetInches, int speed, bool slewOn) {
    pid_drive_set(targetInches, speed, slewOn);
}

void Chassis::pid_odom_set(ez::pose target,
                           ez::drive_directions dir,
                           int speed,
                           bool /*slewOn*/) {
    if (!m_drivetrain || cancelled()) return;

    const Eigen::Vector3f start = currentPose();
    const Eigen::Vector2f targetM = resolvePoint(target);
    const float targetHeading = headingFromPoint(start.head<2>(), targetM, dir, m_headingHoldRad);
    const Eigen::Vector3f targetPose(targetM.x(), targetM.y(), targetHeading);

    startMotion(
        std::make_unique<DriveMoveCommand>(
            m_drivetrain,
            targetM,
            m_poseSource,
            DriveMoveCommand::MotionMode::PointToPoint,
            0.0f,
            dir == ez::rev ? -1 : 1,
            0.02f,
            speed),
        MotionKind::OdomPath,
        start,
        targetPose,
        {targetM});
    m_headingHoldRad = targetHeading;
}

void Chassis::pid_odom_set(std::initializer_list<ez::movement> path, bool /*slewOn*/) {
    if (!m_drivetrain || cancelled() || path.size() == 0) return;

    const Eigen::Vector3f start = currentPose();
    std::vector<ez::movement> movements(path.begin(), path.end());
    std::vector<Eigen::Vector2f> pointsM;
    std::vector<Command*> segments;
    pointsM.reserve(movements.size());
    segments.reserve(movements.size());

    Eigen::Vector2f prev = start.head<2>();
    float finalHeading = m_headingHoldRad;

    for (const auto& movement : movements) {
        const Eigen::Vector2f targetM = resolvePoint(movement.target);
        pointsM.push_back(targetM);
        finalHeading = headingFromPoint(prev, targetM, movement.dir, finalHeading);
        segments.push_back(new DriveMoveCommand(
            m_drivetrain,
            targetM,
            m_poseSource,
            DriveMoveCommand::MotionMode::PointToPoint,
            0.0f,
            movement.dir == ez::rev ? -1 : 1,
            0.02f,
            movement.speed));
        prev = targetM;
    }

    const Eigen::Vector3f targetPose(pointsM.back().x(), pointsM.back().y(), finalHeading);

    startMotion(
        std::make_unique<SequentialCommandGroup>(std::move(segments)),
        MotionKind::OdomPath,
        start,
        targetPose,
        pointsM);

    m_headingHoldRad = finalHeading;
}

void Chassis::pid_turn_set(double targetDeg,
                           int speed,
                           ez::e_angle_behavior /*behavior*/,
                           bool /*slewOn*/) {
    if (!m_drivetrain || cancelled()) return;

    const Eigen::Vector3f start = currentPose();
    const float targetRad = wrapRadians(m_headingZeroRad + ezDegreesToInternalRadians(targetDeg));
    const Eigen::Vector3f targetPose(start.x(), start.y(), targetRad);

    startMotion(
        std::make_unique<RotateCommand>(m_drivetrain, targetRad, m_poseSource, 0.03f, speed),
        MotionKind::Turn,
        start,
        targetPose);
    m_headingHoldRad = targetRad;
}

void Chassis::pid_turn_relative_set(double targetDeg,
                                    int speed,
                                    ez::e_angle_behavior /*behavior*/,
                                    bool /*slewOn*/) {
    if (!m_drivetrain || cancelled()) return;

    const Eigen::Vector3f start = currentPose();
    const float targetRad = wrapRadians(start.z() + ezDegreesToInternalRadians(targetDeg));
    const Eigen::Vector3f targetPose(start.x(), start.y(), targetRad);

    startMotion(
        std::make_unique<RotateCommand>(m_drivetrain, targetRad, m_poseSource, 0.03f, speed),
        MotionKind::Turn,
        start,
        targetPose);
    m_headingHoldRad = targetRad;
}

void Chassis::pid_turn_set(ez::pose target,
                           ez::drive_directions dir,
                           int speed,
                           ez::e_angle_behavior behavior,
                           bool slewOn) {
    const Eigen::Vector3f pose = currentPose();
    const Eigen::Vector2f targetM = resolvePoint(target);
    const float targetRad = headingFromPoint(pose.head<2>(), targetM, dir, pose.z());

    const double ezDegrees =
        static_cast<double>(wrapRadians(targetRad - m_headingZeroRad)) * 180.0 / kAutonPi;
    pid_turn_set(ezDegrees, speed, behavior, slewOn);
}

void Chassis::pid_wait() {
    while (!cancelled()) {
        if (!motion_active()) {
            break;
        }
        stepMotion();
        pros::delay(10);
    }

    if (cancelled()) {
        cancel_motion();
    }
}

void Chassis::pid_wait_until(double target) {
    const float targetMeters = inchesToMeters(std::fabs(target));
    const float targetRadians = std::fabs(ezDegreesToInternalRadians(target));

    while (!cancelled()) {
        const MotionSnapshot motion = snapshotMotion();
        if (!motion.active) {
            break;
        }

        stepMotion();

        const Eigen::Vector3f pose = currentPose();
        if (motion.kind == MotionKind::Turn) {
            const float traveled = std::fabs(wrapRadians(pose.z() - motion.startPose.z()));
            if (traveled >= targetRadians) {
                break;
            }
        } else {
            const float traveled = motionProgressMeters(motion, pose.head<2>());
            if (traveled >= targetMeters) {
                break;
            }
        }
        pros::delay(10);
    }

    if (cancelled()) {
        cancel_motion();
    }
}

void Chassis::pid_wait_until(ez::pose target) {
    const Eigen::Vector2f targetM = resolvePoint(target);
    const float tolerance = inchesToMeters(2.0);

    while (!cancelled()) {
        const MotionSnapshot motion = snapshotMotion();
        if (!motion.active) {
            break;
        }
        stepMotion();
        const Eigen::Vector2f current = currentPose().head<2>();
        if ((current - targetM).norm() <= tolerance) {
            break;
        }
        pros::delay(10);
    }

    if (cancelled()) {
        cancel_motion();
    }
}

void Chassis::pid_wait_until_point(ez::pose target) {
    pid_wait_until(target);
}

void Chassis::cancel_motion() {
    std::unique_ptr<Command> motionToCancel;
    std::scoped_lock guard(*m_motionMutex);
    motionToCancel = std::move(m_motion);
    m_motionKind = MotionKind::None;
    m_pathPointsM.clear();
    if (motionToCancel) {
        motionToCancel->end(true);
    }
    if (m_drivetrain) {
        m_drivetrain->stop();
    }
}

bool Chassis::motion_active() const {
    std::scoped_lock guard(*m_motionMutex);
    return static_cast<bool>(m_motion);
}

bool Chassis::cancelled() const {
    return m_cancelRequested && m_cancelRequested();
}

Eigen::Vector3f Chassis::currentPose() const {
    if (m_poseSource) {
        const Eigen::Vector3f pose = m_poseSource();
        if (std::isfinite(pose.x()) && std::isfinite(pose.y()) && std::isfinite(pose.z())) {
            return pose;
        }
    }
    if (m_drivetrain) {
        return m_drivetrain->getOdomPose();
    }
    return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
}

Eigen::Vector2f Chassis::resolvePoint(const ez::pose& point) const {
    return point_to_global(point, m_coordinateFrame);
}

Chassis::MotionSnapshot Chassis::snapshotMotion() const {
    std::scoped_lock guard(*m_motionMutex);

    MotionSnapshot snapshot;
    snapshot.active = static_cast<bool>(m_motion);
    snapshot.kind = m_motionKind;
    snapshot.startPose = m_motionStartPose;
    snapshot.pathPointsM = m_pathPointsM;
    return snapshot;
}

float Chassis::motionProgressMeters(const MotionSnapshot& motion, const Eigen::Vector2f& current) {
    if (motion.pathPointsM.empty()) {
        return (current - motion.startPose.head<2>()).norm();
    }

    Eigen::Vector2f segmentStart = motion.startPose.head<2>();
    float cumulative = 0.0f;
    float bestProgress = 0.0f;
    float bestDistance = std::numeric_limits<float>::infinity();

    for (const Eigen::Vector2f& segmentEnd : motion.pathPointsM) {
        const Eigen::Vector2f segment = segmentEnd - segmentStart;
        const float segmentLength = segment.norm();

        if (segmentLength <= 1e-5f) {
            const float distanceToPoint = (current - segmentEnd).norm();
            if (distanceToPoint < bestDistance) {
                bestDistance = distanceToPoint;
                bestProgress = cumulative;
            }
            segmentStart = segmentEnd;
            continue;
        }

        const Eigen::Vector2f axis = segment / segmentLength;
        const float projected = std::clamp(
            (current - segmentStart).dot(axis),
            0.0f,
            segmentLength);
        const Eigen::Vector2f closestPoint = segmentStart + axis * projected;
        const float lateralError = (current - closestPoint).norm();
        const float candidateProgress = cumulative + projected;

        if (lateralError < bestDistance - 1e-4f ||
            (std::fabs(lateralError - bestDistance) <= 1e-4f &&
             candidateProgress > bestProgress)) {
            bestDistance = lateralError;
            bestProgress = candidateProgress;
        }

        cumulative += segmentLength;
        segmentStart = segmentEnd;
    }

    return bestProgress;
}

void Chassis::startMotion(std::unique_ptr<Command> motion,
                          MotionKind kind,
                          const Eigen::Vector3f& startPose,
                          const Eigen::Vector3f& targetPose,
                          std::vector<Eigen::Vector2f> pathPoints) {
    cancel_motion();
    if (!motion) return;

    std::scoped_lock guard(*m_motionMutex);
    m_motion = std::move(motion);
    m_motionKind = kind;
    m_motionStartPose = startPose;
    m_motionTargetPose = targetPose;
    m_pathPointsM = std::move(pathPoints);
    m_motion->initialize();
}

void Chassis::stepMotion() {
    std::unique_ptr<Command> finishedMotion;

    {
        std::scoped_lock guard(*m_motionMutex);
        if (!m_motion) {
            return;
        }

        m_motion->execute();
        if (m_motion->isFinished()) {
            finishedMotion = std::move(m_motion);
            m_motionKind = MotionKind::None;
            m_pathPointsM.clear();
        }
    }

    if (finishedMotion) {
        finishedMotion->end(false);
    }
}

float Chassis::inchesToMeters(double inches) {
    return static_cast<float>(inches * kInToM);
}

float Chassis::ezDegreesToInternalRadians(double degrees) {
    return static_cast<float>(degrees * kAutonPi / 180.0);
}

float Chassis::wrapRadians(float radians) {
    return std::atan2(std::sin(radians), std::cos(radians));
}

float Chassis::headingFromPoint(const Eigen::Vector2f& from,
                                const Eigen::Vector2f& to,
                                ez::drive_directions dir,
                                float fallbackHeading) {
    const Eigen::Vector2f delta = to - from;
    if (delta.norm() <= 1e-5f) {
        return fallbackHeading;
    }

    float heading = std::atan2(delta.y(), delta.x());
    if (dir == ez::rev) {
        heading = wrapRadians(heading + kAutonPi);
    }
    return heading;
}
