/**
 * @file chassis.h
 * EZ-Template-style autonomous chassis facade.
 */
#pragma once

#include "Eigen/Core"
#include "command/command.h"
#include "pros/rtos.hpp"
#include "subsystems/drivetrain.h"

#include <functional>
#include <initializer_list>
#include <memory>
#include <vector>

namespace ez {

enum drive_directions {
    fwd,
    rev,
};

enum e_angle_behavior {
    shortest,
    longest,
    left,
    right,
    raw,
};

struct pose {
    double x = 0.0;
    double y = 0.0;
};

struct movement {
    pose target{};
    drive_directions dir = fwd;
    int speed = 110;
};

} // namespace ez

class Chassis {
public:
    enum class CoordinateFrame {
        Global,
        Local,
    };

    Chassis(Drivetrain* drivetrain,
            std::function<Eigen::Vector3f()> poseSource,
            std::function<bool()> cancelRequested = {});

    void pid_targets_reset();
    // Global uses field coordinates. Local uses a frame anchored at the stored
    // local origin pose, with +X forward and +Y left relative to that origin.
    void set_global_frame();
    void set_local_frame();
    CoordinateFrame current_frame() const;
    void set_local_origin_here();
    void set_local_origin(double xInches, double yInches, double headingDeg);
    Eigen::Vector3f local_origin() const;
    Eigen::Vector3f pose_to_global(const Eigen::Vector3f& pose,
                                   CoordinateFrame frame = CoordinateFrame::Global) const;
    Eigen::Vector2f point_to_global(const ez::pose& point,
                                    CoordinateFrame frame = CoordinateFrame::Global) const;
    void drive_imu_reset();
    void drive_sensor_reset();
    void odom_xyt_set(double xInches, double yInches, double headingDeg);

    void pid_drive_set(double targetInches, int speed, bool slewOn = false);
    void pid_odom_set(double targetInches, int speed, bool slewOn = false);
    void pid_odom_set(ez::pose target,
                      ez::drive_directions dir = ez::fwd,
                      int speed = 110,
                      bool slewOn = false);
    void pid_odom_set(std::initializer_list<ez::movement> path, bool slewOn = true);

    void pid_turn_set(double targetDeg,
                      int speed,
                      ez::e_angle_behavior behavior = ez::shortest,
                      bool slewOn = false);
    void pid_turn_relative_set(double targetDeg,
                               int speed,
                               ez::e_angle_behavior behavior = ez::shortest,
                               bool slewOn = false);
    void pid_turn_set(ez::pose target,
                      ez::drive_directions dir,
                      int speed,
                      ez::e_angle_behavior behavior = ez::shortest,
                      bool slewOn = false);

    void pid_wait();
    void pid_wait_until(double target);
    void pid_wait_until(ez::pose target);
    void pid_wait_until_point(ez::pose target);

    void cancel_motion();
    bool motion_active() const;

private:
    enum class MotionKind {
        None,
        Drive,
        Turn,
        OdomPath,
    };

    struct MotionSnapshot {
        bool active = false;
        MotionKind kind = MotionKind::None;
        Eigen::Vector3f startPose{0.0f, 0.0f, 0.0f};
        std::vector<Eigen::Vector2f> pathPointsM;
    };

    Drivetrain* m_drivetrain = nullptr;
    std::function<Eigen::Vector3f()> m_poseSource;
    std::function<bool()> m_cancelRequested;
    mutable std::unique_ptr<pros::RecursiveMutex> m_motionMutex;
    std::unique_ptr<Command> m_motion;
    MotionKind m_motionKind = MotionKind::None;
    Eigen::Vector3f m_motionStartPose{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f m_motionTargetPose{0.0f, 0.0f, 0.0f};
    std::vector<Eigen::Vector2f> m_pathPointsM;
    float m_headingZeroRad = 0.0f;
    float m_headingHoldRad = 0.0f;
    CoordinateFrame m_coordinateFrame = CoordinateFrame::Global;
    Eigen::Vector3f m_localOriginPose{0.0f, 0.0f, 0.0f};

    bool cancelled() const;
    Eigen::Vector3f currentPose() const;
    Eigen::Vector2f resolvePoint(const ez::pose& point) const;
    MotionSnapshot snapshotMotion() const;
    static float motionProgressMeters(const MotionSnapshot& motion, const Eigen::Vector2f& current);
    void stepMotion();

    void startMotion(std::unique_ptr<Command> motion,
                     MotionKind kind,
                     const Eigen::Vector3f& startPose,
                     const Eigen::Vector3f& targetPose,
                     std::vector<Eigen::Vector2f> pathPoints = {});

    static float inchesToMeters(double inches);
    static float ezDegreesToInternalRadians(double degrees);
    static float wrapRadians(float radians);
    static float headingFromPoint(const Eigen::Vector2f& from,
                                  const Eigen::Vector2f& to,
                                  ez::drive_directions dir,
                                  float fallbackHeading);
};
