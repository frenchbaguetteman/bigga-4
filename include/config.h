/**
 * @file config.h
 * Central configuration surface for 2654E Echo.
 *
 * Contains all tunables: geometry, noise models, PID gains, RAMSETE / LTV
 * parameters, feedforward constants, sensor offsets, and localization config.
 *
 * Localization internals use metres and a math heading convention:
 *   θ = 0 along +X, positive CCW.
 * Sensor mounting dimensions are configured in inches (robot right/forward)
 * and converted to metres/math-frame constants below.
 */
#pragma once

#include "Eigen/Dense"
#include "units/units.hpp"
#include "feedback/pid.h"
#include <utility>
#include <cmath>
#include <cstdint>
#include <vector>

namespace CONFIG {

// ── Shared conversion helpers ──────────────────────────────────────────────

constexpr float IN_TO_M   = 0.0254f;
constexpr float INCH_TO_M = IN_TO_M;  // backward-compatible alias

inline float wrapAngleRadians(float angleRad) {
    return std::atan2(std::sin(angleRad), std::cos(angleRad));
}

// Compass heading: 0° = +Y (north), positive clockwise → math radians
inline float compassDegToMathRad(float headingDeg) {
    float compassRad = headingDeg * static_cast<float>(M_PI) / 180.0f;
    return wrapAngleRadians(static_cast<float>(M_PI / 2.0) - compassRad);
}

// Math radians → compass heading in [0, 360)
inline float mathRadToCompassDeg(float headingRad) {
    float wrapped = wrapAngleRadians(headingRad);
    float compassRad = static_cast<float>(M_PI / 2.0) - wrapped;
    float deg = compassRad * 180.0f / static_cast<float>(M_PI);
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

// ── Startup Pose Mode ───────────────────────────────────────────────────────

enum class StartupPoseMode {
    ConfiguredStartPoseOnly,
    GPSXYPlusIMUHeading,
    FullGPSInit,
};

// ── Physical geometry ───────────────────────────────────────────────────────

constexpr float DRIVE_RADIUS    = 0.04127f;    // drive wheel radius  (m)
constexpr float ODOM_RADIUS     = 0.034925f;   // tracking wheel radius (m)
constexpr float TRACK_WIDTH     = 0.254f;      // centre-to-centre (m)
constexpr float WHEEL_BASE      = 0.254f;      // front-to-back axle  (m)

// ── Noise model ─────────────────────────────────────────────────────────────

constexpr float DRIVE_NOISE     = 0.02f;       // stddev on odometry displacement (m)
constexpr float ANGLE_NOISE     = 0.01f;       // stddev on heading (rad)

// ── Drivetrain Motor Ports (negative = reversed) ────────────────────────────

inline const std::vector<std::int8_t> LEFT_DRIVE_PORTS  = {-11, -13, -14};
inline const std::vector<std::int8_t> RIGHT_DRIVE_PORTS = { 9,  17,  20};

// ── Intake Motor Ports ──────────────────────────────────────────────────────

inline const std::vector<std::int8_t> INTAKE_PORTS = {3, -5};

// ── Lift (currently disabled — uncomment when wired) ────────────────────────
// constexpr int8_t LIFT_PORT_1  = 8;
// constexpr int8_t LIFT_PORT_2  = -9;
// constexpr int    LIFT_SENSOR  = 15;

// ── Sensor Ports ────────────────────────────────────────────────────────────

constexpr int IMU_PORT = 2;

// Tracking wheels — set port to 0 to disable that wheel.
// Vertical = 0 → falls back to averaged drive motor encoders.
// Horizontal = 0 → lateral displacement assumed zero (no phantom strafe).
constexpr int  VERTICAL_TRACKING_PORT        = 0;
constexpr int  HORIZONTAL_TRACKING_PORT      = 16;
constexpr bool VERTICAL_TRACKING_REVERSED    = false;
constexpr bool HORIZONTAL_TRACKING_REVERSED  = false;

// ── MCL Distance Sensor Ports ───────────────────────────────────────────────

constexpr int MCL_LEFT_DISTANCE_PORT   = 1;    // facing -90° (left)
constexpr int MCL_RIGHT_DISTANCE_PORT  = 8;    // facing +90° (right)
constexpr int MCL_BACK_DISTANCE_PORT   = 6;    // facing 180° (back)
constexpr int MCL_FRONT_DISTANCE_PORT  = 10;   // facing 0°   (front)
constexpr int MCL_GPS_PORT             = 15;

// GPS heading offset: GPS module faces backwards relative to robot forward
constexpr double MCL_GPS_HEADING_OFFSET_DEG = 180.0;

// ── MCL Sensor Mounting Offsets (inches, robot frame) ──────────────────────
// Tuning inputs:
//   +offsetX = robot-right, -offsetX = robot-left
//   +offsetY = robot-forward, -offsetY = robot-backward

constexpr double MCL_LEFT_OFFSET_X   = -4.625;
constexpr double MCL_LEFT_OFFSET_Y   =  0.25;
constexpr double MCL_RIGHT_OFFSET_X  =  5.175;
constexpr double MCL_RIGHT_OFFSET_Y  =  0.25;
constexpr double MCL_BACK_OFFSET_X   =  6.1875;   // right overhang
constexpr double MCL_BACK_OFFSET_Y   = -3.625;
constexpr double MCL_FRONT_OFFSET_X  =  6.1875;   // right overhang
constexpr double MCL_FRONT_OFFSET_Y  = -1.875;
constexpr double MCL_GPS_OFFSET_X    =  6.0;
constexpr double MCL_GPS_OFFSET_Y    = -3.75;     // 0.125" behind back distance sensor

// Converted GPS offset in metres, math robot frame:
//   +X = forward, +Y = left
constexpr float MCL_GPS_OFFSET_X_M = static_cast<float>(MCL_GPS_OFFSET_Y) * IN_TO_M;
constexpr float MCL_GPS_OFFSET_Y_M = static_cast<float>(-MCL_GPS_OFFSET_X) * IN_TO_M;
inline const Eigen::Vector2f GPS_OFFSET_M{MCL_GPS_OFFSET_X_M, MCL_GPS_OFFSET_Y_M};

// Sensor facing angles (radians, math robot frame: +CCW)
constexpr float MCL_LEFT_FACING  = static_cast<float>( M_PI / 2.0);  // +90°
constexpr float MCL_RIGHT_FACING = static_cast<float>(-M_PI / 2.0);  // -90°
constexpr float MCL_BACK_FACING  = static_cast<float>( M_PI);         // 180°
constexpr float MCL_FRONT_FACING = 0.0f;                               //   0°

// Distance-sensor offsets in metres, math robot frame (+X fwd, +Y left)
inline const Eigen::Vector3f DIST_LEFT_OFFSET {
    static_cast<float>(MCL_LEFT_OFFSET_Y) * IN_TO_M,
    static_cast<float>(-MCL_LEFT_OFFSET_X) * IN_TO_M,
    MCL_LEFT_FACING};
inline const Eigen::Vector3f DIST_RIGHT_OFFSET{
    static_cast<float>(MCL_RIGHT_OFFSET_Y) * IN_TO_M,
    static_cast<float>(-MCL_RIGHT_OFFSET_X) * IN_TO_M,
    MCL_RIGHT_FACING};
inline const Eigen::Vector3f DIST_FRONT_OFFSET{
    static_cast<float>(MCL_FRONT_OFFSET_Y) * IN_TO_M,
    static_cast<float>(-MCL_FRONT_OFFSET_X) * IN_TO_M,
    MCL_FRONT_FACING};
inline const Eigen::Vector3f DIST_BACK_OFFSET {
    static_cast<float>(MCL_BACK_OFFSET_Y) * IN_TO_M,
    static_cast<float>(-MCL_BACK_OFFSET_X) * IN_TO_M,
    MCL_BACK_FACING};

// ── MCL Distance Sensor Fusion Controls ─────────────────────────────────────

constexpr bool MCL_ENABLE_DISTANCE_SENSORS         = true;
constexpr bool MCL_ENABLE_LEFT_DISTANCE_SENSOR     = true;
constexpr bool MCL_ENABLE_RIGHT_DISTANCE_SENSOR    = true;
constexpr bool MCL_ENABLE_BACK_DISTANCE_SENSOR     = true;
constexpr bool MCL_ENABLE_FRONT_DISTANCE_SENSOR    = true;

constexpr double MCL_LEFT_DISTANCE_WEIGHT   = 0.60;
constexpr double MCL_RIGHT_DISTANCE_WEIGHT  = 0.60;
constexpr double MCL_BACK_DISTANCE_WEIGHT   = 0.80;
constexpr double MCL_FRONT_DISTANCE_WEIGHT  = 0.80;

// If vertical tracking wheel is disabled (port 0), inflate MCL motion noise
constexpr double MCL_DRIVE_ENCODER_FALLBACK_NOISE_SCALE = 1.75;

// ── Starting Pose (field frame, inches / degrees) ───────────────────────────
// Set these to where the robot is physically placed at boot.
// theta: 0° = +Y (north), CW positive.

constexpr double START_POSE_X_IN      = 0.0;
constexpr double START_POSE_Y_IN      = 0.0;
constexpr double START_POSE_THETA_DEG = 0.0;

constexpr bool START_POSE_KNOWN = false;

constexpr StartupPoseMode STARTUP_POSE_MODE =
    StartupPoseMode::FullGPSInit;

// GPS readiness gate used at boot
constexpr uint32_t STARTUP_GPS_MAX_WAIT_MS      = 8000;
constexpr double   STARTUP_GPS_READY_ERROR_M    = 0.010;
constexpr int      STARTUP_GPS_STABLE_SAMPLES   = 8;

// ── Odom Debug Diagnostics ──────────────────────────────────────────────────

constexpr bool     ODOM_DEBUG_ENABLE               = true;
constexpr uint32_t ODOM_DEBUG_LOG_EVERY_MS          = 200;
constexpr double   ODOM_TURN_TRANSLATION_WARN_IN   = 0.35;

// ── Pneumatic (ADI) Ports ───────────────────────────────────────────────────

constexpr char SELECT1_PORT = 'A';
constexpr char SELECT2_PORT = 'B';
constexpr char TONGUE_PORT  = 'C';
constexpr char WING_PORT    = 'D';

// ── Localization ────────────────────────────────────────────────────────────

constexpr int   NUM_PARTICLES             = 250;
constexpr float FIELD_HALF_SIZE           = 1.78308f;  // metres (inner wall)
constexpr float MAX_DISTANCE_SINCE_UPDATE = 0.05f;     // metres
constexpr int   MAX_UPDATE_INTERVAL_MS    = 400;       // milliseconds (less idle jitter)
constexpr float PF_STATIONARY_DEADBAND_M  = 0.003f;    // ignore encoder creep below 3 mm/update

// ── Speed / acceleration limits ─────────────────────────────────────────────

constexpr float MAX_SPEED        = 1.8f;   // m/s empirical top speed
constexpr float MAX_ACCELERATION = 3.0f;   // m/s²
constexpr float MAX_ANGULAR_VEL  = 10.0f;  // rad/s

// ── PID gains ───────────────────────────────────────────────────────────────
//  PID(kP, kI, kD, integralCap)  — see feedback/pid.h

inline PID::Gains TURN_PID      {2.0f, 0.0f, 0.15f, 0.0f};
inline PID::Gains DISTANCE_PID  {5.0f, 0.0f, 0.3f,  0.0f};
inline PID::Gains INTAKE_PID    {1.0f, 0.0f, 0.0f,  0.0f};

// ── RAMSETE path-following parameters ───────────────────────────────────────

constexpr float RAMSETE_ZETA = 0.4f;       // damping  (0 < ζ < 1)
constexpr float RAMSETE_BETA = 45.0f;      // aggressiveness  (β > 0)

// ── LTV unicycle cost matrix  Q = diag(q1, q2, q3) ─────────────────────────

inline Eigen::Vector3f DEFAULT_DT_COST_Q{1.0f, 1.0f, 10.0f};

// ── Drivetrain feedforward model ────────────────────────────────────────────

constexpr float FF_kS = 1100.0f;   // static friction offset  (mV)
constexpr float FF_kV = 5200.0f;   // velocity gain           (mV·s/m)
constexpr float FF_kA = 400.0f;    // acceleration gain        (mV·s²/m)

inline std::pair<float, float> DRIVETRAIN_FEEDFORWARD(
        float v, float omega, float a = 0.0f, float alpha = 0.0f) {
    float v_left  = v - omega * TRACK_WIDTH / 2.0f;
    float v_right = v + omega * TRACK_WIDTH / 2.0f;
    float a_left  = a - alpha * TRACK_WIDTH / 2.0f;
    float a_right = a + alpha * TRACK_WIDTH / 2.0f;

    auto ff = [](float vel, float acc) -> float {
        float sign = (vel > 0.0f) ? 1.0f : ((vel < 0.0f) ? -1.0f : 0.0f);
        return FF_kS * sign + FF_kV * vel + FF_kA * acc;
    };

    return {ff(v_left, a_left), ff(v_right, a_right)};
}

} // namespace CONFIG
