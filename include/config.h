/**
 * @file config.h
 * Central configuration surface for 69580A.
 *
 * Contains all tunables: geometry, noise models, PID gains, RAMSETE / LTV
 * parameters, feedforward constants, sensor offsets, and localization config.
 *
 * ═ CANONICAL INTERNAL LOCALIZATION CONVENTION ═
 *
 * All internal pose, odometry, and sensor fusion use:
 *   • Position units: metres
 *   • Heading units: radians
 *   • Field frame: +X east/forward (θ=0), +Y north/left, positive CCW
 *
 * All human-tuned configuration inputs are specified in inch/deg/ms-style
 * constants (`*_in`, `*_deg`, `*_ms`, etc.). Internally, these are converted
 * once into typed SI quantities using `QLength`, `QAngle`, `QTime`, etc.
 *
 * VEX GPS convention (API boundary only):
 *   • Position units: metres (Cartesian, origin at field center)
 *   • Heading: 0° = north, 90° = east, positive clockwise (degrees)
 *
 * Conversions between the two are applied ONLY at API boundaries:
 *   – Reading GPS sensor → convert to internal frame
 *   – Writing to GPS init → convert from internal frame
 *
 * All internal offsets, paths, and sensor transforms use the internal convention.
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
constexpr float CM_TO_IN  = 1.0f / 2.54f;

constexpr QLength length_from_in(float inches) {
    return inches * inch;
}

constexpr QTime time_from_ms(float milliseconds) {
    return milliseconds * millisecond;
}

constexpr QAngle angle_from_deg(float degrees) {
    return degrees * degree;
}

constexpr QVelocity velocity_from_inps(float inchesPerSecond) {
    return inchesPerSecond * inch / second;
}

constexpr QAcceleration acceleration_from_inps2(float inchesPerSecondSq) {
    return inchesPerSecondSq * inch / (second * second);
}

inline float wrapAngleRadians(float angleRad) {
    return std::atan2(std::sin(angleRad), std::cos(angleRad));
}

/**
 * Convert VEX GPS compass heading (0° = north, CW positive) to internal heading
 * (0° = east/forward, CCW positive).
 *
 * @param compassDeg  VEX compass heading in degrees [0, 360)
 * @return internal heading in radians
 */
inline float gpsHeadingDegToInternalRad(float compassDeg) {
    // GPS: 0° = north (+Y), 90° = east (+X), etc., CW positive
    // Internal: 0° = east (+X), 90° = north (+Y), CCW positive
    // Transform: θ_internal = π/2 - θ_gps_rad
    float gpsRad = compassDeg * static_cast<float>(M_PI) / 180.0f;
    return wrapAngleRadians(static_cast<float>(M_PI / 2.0) - gpsRad);
}

/**
 * Convert internal heading (0° = east/forward, CCW positive) to VEX GPS
 * compass heading (0° = north, CW positive).
 *
 * @param internalRad  internal heading in radians
 * @return GPS compass heading in degrees [0, 360)
 */
inline float internalRadToGpsHeadingDeg(float internalRad) {
    // Inverse: θ_gps_rad = π/2 - θ_internal
    float wrapped = wrapAngleRadians(internalRad);
    float gpsRad = static_cast<float>(M_PI / 2.0) - wrapped;
    float deg = gpsRad * 180.0f / static_cast<float>(M_PI);
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

// Legacy aliases for backward compatibility
inline float compassDegToMathRad(float headingDeg) {
    return gpsHeadingDegToInternalRad(headingDeg);
}

inline float mathRadToCompassDeg(float headingRad) {
    return internalRadToGpsHeadingDeg(headingRad);
}

// ── Startup Pose Mode ───────────────────────────────────────────────────────

enum class StartupPoseMode {
    ConfiguredStartPoseOnly,
    GPSXYPlusIMUHeading,
    FullGPSInit,
};

// ── Physical geometry ───────────────────────────────────────────────────────

constexpr float DRIVE_RADIUS_in = 1.625f;
constexpr float ODOM_RADIUS_in  = 1.375f;
constexpr float TRACK_WIDTH_in  = 11.338583f;
constexpr float WHEEL_BASE_in   = 10.15748f;

inline constexpr QLength DRIVE_RADIUS = length_from_in(DRIVE_RADIUS_in);
inline constexpr QLength ODOM_RADIUS  = length_from_in(ODOM_RADIUS_in);
inline constexpr QLength TRACK_WIDTH  = length_from_in(TRACK_WIDTH_in);
inline constexpr QLength WHEEL_BASE   = length_from_in(WHEEL_BASE_in);

// ── Noise model ─────────────────────────────────────────────────────────────

constexpr float DRIVE_NOISE_in  = 0.15748f;
constexpr float ANGLE_NOISE_deg = 0.572958f;

inline constexpr QLength DRIVE_NOISE = length_from_in(DRIVE_NOISE_in);
inline constexpr QAngle ANGLE_NOISE = angle_from_deg(ANGLE_NOISE_deg);

// ── Drivetrain Motor Ports (negative = reversed) ────────────────────────────

inline const std::vector<std::int8_t> LEFT_DRIVE_PORTS  = {-11, -15, -14};
inline const std::vector<std::int8_t> RIGHT_DRIVE_PORTS = { 18,  17,  20};

// ── Intake Motor Ports ──────────────────────────────────────────────────────

inline const std::vector<std::int8_t> INTAKE_PORTS = {-6, 8};

// ── Sensor Ports ────────────────────────────────────────────────────────────

constexpr int IMU_PORT = 13;

// Tracking wheels — set port to 0 to disable that wheel.
// Vertical = 0 → falls back to averaged drive motor encoders.
// Horizontal = 0 → lateral displacement assumed zero (no phantom strafe).
constexpr int  VERTICAL_TRACKING_PORT        = 0;
constexpr int  HORIZONTAL_TRACKING_PORT      = 16;       // keep disabled until LATERAL_WHEEL_OFFSET_M is measured
constexpr bool VERTICAL_TRACKING_REVERSED    = false;
constexpr bool HORIZONTAL_TRACKING_REVERSED  = false;
constexpr float LATERAL_WHEEL_OFFSET_in      = -1.771654f;
inline constexpr QLength LATERAL_WHEEL_OFFSET = length_from_in(LATERAL_WHEEL_OFFSET_in);

// ── MCL Distance Sensor Ports ───────────────────────────────────────────────

constexpr int MCL_LEFT_DISTANCE_PORT   = 2;    // facing -90° (left)
constexpr int MCL_RIGHT_DISTANCE_PORT  = 5;    // facing +90° (right)
constexpr int MCL_BACK_DISTANCE_PORT   = 4;    // facing 180° (back)
constexpr int MCL_FRONT_DISTANCE_PORT  = 1;   // facing 0°   (front)
constexpr int MCL_GPS_PORT             = 3;

// GPS heading offset: GPS module faces robot-right (90° clockwise from forward)
constexpr float MCL_GPS_HEADING_OFFSET_deg = 90.0f;

// GPS field-frame rotation (degrees).
// Use this to compensate for GPS strip mounting orientation relative to field axes.
// Positive = CCW, negative = CW. Example: strip appears 90° CCW -> set to -90.
// This robot's GPS field frame is rotated +90° relative to the canonical
// internal frame, so apply -90° to convert raw GPS data back into the shared
// field axes used by odom/MCL/UI.
constexpr float GPS_FIELD_ROTATION_DEG = -90.0f;

inline float gpsFieldRotationRad() {
    return GPS_FIELD_ROTATION_DEG * static_cast<float>(M_PI) / 180.0f;
}

/**
 * Convert a raw GPS-sensor compass heading into the canonical internal field
 * frame, including configured field-strip rotation compensation.
 *
 * If the GPS field strips are physically rotated relative to the canonical
 * internal field axes, position is corrected with `transformGpsToFieldFrame()`
 * and heading must receive the same frame rotation.
 */
inline float gpsSensorHeadingDegToInternalRad(float compassDeg) {
    return wrapAngleRadians(gpsHeadingDegToInternalRad(compassDeg) + gpsFieldRotationRad());
}

inline Eigen::Vector2f transformGpsToFieldFrame(const Eigen::Vector2f& rawPosM) {
    const float a = gpsFieldRotationRad();
    const float c = std::cos(a);
    const float s = std::sin(a);
    return Eigen::Vector2f(
        rawPosM.x() * c - rawPosM.y() * s,
        rawPosM.x() * s + rawPosM.y() * c);
}

// ── MCL Sensor Mounting Offsets (robot frame) ───────────────────────────────
// 
// Raw tuning inputs in ROBOT frame (mechanics-facing):
//   +offsetX_IN = robot-right, -offsetX_IN = robot-left
//   +offsetY_IN = robot-forward, -offsetY_IN = robot-backward
//
// Converted to INTERNAL frame (+X forward/east, +Y left/north, CCW positive):
//   offset_X_M (forward)  = +offsetY_IN (robot forward) * IN_TO_M
//   offset_Y_M (left)     = -offsetX_IN (robot left is -X_robot = +Y_internal) * IN_TO_M
//
// This matches the structure used in drivetrain.cpp updateOdometry():
//   dx = dFwd * cos(θ) - dLat * sin(θ)  
//   dy = dFwd * sin(θ) + dLat * cos(θ)
//
// All INTERNAL offsets should be METRES, in the (+X fwd, +Y left) convention.
//
// Measurement notes from current robot:
// - Lateral references were measured from the left side of the robot.
// - Longitudinal references were measured from the back/front of the robot.
// - Implied chassis envelope from those measurements: 28.5 cm wide, 29.8 cm long.

constexpr float ROBOT_WIDTH_in  = 28.5f * CM_TO_IN;
constexpr float ROBOT_LENGTH_in = 29.8f * CM_TO_IN;

constexpr float MCL_LEFT_OFFSET_X_in   = 2.5f * CM_TO_IN - ROBOT_WIDTH_in / 2.0f;
constexpr float MCL_LEFT_OFFSET_Y_in   = (ROBOT_LENGTH_in - 10.8f * CM_TO_IN) - ROBOT_LENGTH_in / 2.0f;
constexpr float MCL_RIGHT_OFFSET_X_in  = 26.0f * CM_TO_IN - ROBOT_WIDTH_in / 2.0f;
constexpr float MCL_RIGHT_OFFSET_Y_in  = 23.0f * CM_TO_IN - ROBOT_LENGTH_in / 2.0f;
constexpr float MCL_BACK_OFFSET_X_in   = ROBOT_WIDTH_in / 2.0f + 0.8f * CM_TO_IN;
constexpr float MCL_BACK_OFFSET_Y_in   = 4.25f * CM_TO_IN - ROBOT_LENGTH_in / 2.0f;
constexpr float MCL_FRONT_OFFSET_X_in  = 3.5f * CM_TO_IN - ROBOT_WIDTH_in / 2.0f;
constexpr float MCL_FRONT_OFFSET_Y_in  = (ROBOT_LENGTH_in - 1.8f * CM_TO_IN) - ROBOT_LENGTH_in / 2.0f;
constexpr float MCL_GPS_OFFSET_X_in    = MCL_RIGHT_OFFSET_X_in;
constexpr float MCL_GPS_OFFSET_Y_in    = 28.0f * CM_TO_IN - ROBOT_LENGTH_in / 2.0f;

inline constexpr QLength ROBOT_WIDTH = length_from_in(ROBOT_WIDTH_in);
inline constexpr QLength ROBOT_LENGTH = length_from_in(ROBOT_LENGTH_in);
inline constexpr QLength MCL_LEFT_OFFSET_X = length_from_in(MCL_LEFT_OFFSET_X_in);
inline constexpr QLength MCL_LEFT_OFFSET_Y = length_from_in(MCL_LEFT_OFFSET_Y_in);
inline constexpr QLength MCL_RIGHT_OFFSET_X = length_from_in(MCL_RIGHT_OFFSET_X_in);
inline constexpr QLength MCL_RIGHT_OFFSET_Y = length_from_in(MCL_RIGHT_OFFSET_Y_in);
inline constexpr QLength MCL_BACK_OFFSET_X = length_from_in(MCL_BACK_OFFSET_X_in);
inline constexpr QLength MCL_BACK_OFFSET_Y = length_from_in(MCL_BACK_OFFSET_Y_in);
inline constexpr QLength MCL_FRONT_OFFSET_X = length_from_in(MCL_FRONT_OFFSET_X_in);
inline constexpr QLength MCL_FRONT_OFFSET_Y = length_from_in(MCL_FRONT_OFFSET_Y_in);
inline constexpr QLength MCL_GPS_OFFSET_X = length_from_in(MCL_GPS_OFFSET_X_in);
inline constexpr QLength MCL_GPS_OFFSET_Y = length_from_in(MCL_GPS_OFFSET_Y_in);

inline Eigen::Vector2f robot_offset_to_internal(QLength offsetXRight, QLength offsetYForward) {
    return Eigen::Vector2f(offsetYForward.getValue(), -offsetXRight.getValue());
}

// GPS offset in metres, INTERNAL robot frame (+X forward, +Y left)
inline const Eigen::Vector2f GPS_OFFSET = robot_offset_to_internal(MCL_GPS_OFFSET_X, MCL_GPS_OFFSET_Y);

constexpr float MCL_LEFT_FACING_deg  =  90.0f;
constexpr float MCL_RIGHT_FACING_deg = -90.0f;
constexpr float MCL_BACK_FACING_deg  = 180.0f;
constexpr float MCL_FRONT_FACING_deg =   0.0f;

inline constexpr QAngle MCL_LEFT_FACING  = angle_from_deg(MCL_LEFT_FACING_deg);
inline constexpr QAngle MCL_RIGHT_FACING = angle_from_deg(MCL_RIGHT_FACING_deg);
inline constexpr QAngle MCL_BACK_FACING  = angle_from_deg(MCL_BACK_FACING_deg);
inline constexpr QAngle MCL_FRONT_FACING = angle_from_deg(MCL_FRONT_FACING_deg);

inline Eigen::Vector3f sensor_offset_to_internal(QLength offsetXRight,
                                                 QLength offsetYForward,
                                                 QAngle facing) {
    return Eigen::Vector3f(offsetYForward.getValue(),
                           -offsetXRight.getValue(),
                           facing.getValue());
}

inline const Eigen::Vector3f DIST_LEFT_OFFSET {
    sensor_offset_to_internal(MCL_LEFT_OFFSET_X, MCL_LEFT_OFFSET_Y, MCL_LEFT_FACING)};
inline const Eigen::Vector3f DIST_RIGHT_OFFSET{
    sensor_offset_to_internal(MCL_RIGHT_OFFSET_X, MCL_RIGHT_OFFSET_Y, MCL_RIGHT_FACING)};
inline const Eigen::Vector3f DIST_FRONT_OFFSET{
    sensor_offset_to_internal(MCL_FRONT_OFFSET_X, MCL_FRONT_OFFSET_Y, MCL_FRONT_FACING)};
inline const Eigen::Vector3f DIST_BACK_OFFSET {
    sensor_offset_to_internal(MCL_BACK_OFFSET_X, MCL_BACK_OFFSET_Y, MCL_BACK_FACING)};

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

constexpr float MCL_DISTANCE_STDDEV_in                  = 1.75f;
constexpr float MCL_DISTANCE_MIN_RANGE_in               = 1.50f;
constexpr float MCL_DISTANCE_MAX_RANGE_in               = 72.0f;
constexpr float MCL_DISTANCE_CONFIDENCE_EXEMPT_in       = 8.0f;
constexpr float MCL_DISTANCE_LOW_CONFIDENCE_SIGMA_SCALE = 2.5f;
constexpr float MCL_DISTANCE_LIKELIHOOD_FLOOR           = 0.18f;

inline constexpr QLength MCL_DISTANCE_STDDEV =
    length_from_in(MCL_DISTANCE_STDDEV_in);
inline constexpr QLength MCL_DISTANCE_MIN_RANGE =
    length_from_in(MCL_DISTANCE_MIN_RANGE_in);
inline constexpr QLength MCL_DISTANCE_MAX_RANGE =
    length_from_in(MCL_DISTANCE_MAX_RANGE_in);
inline constexpr QLength MCL_DISTANCE_CONFIDENCE_EXEMPT =
    length_from_in(MCL_DISTANCE_CONFIDENCE_EXEMPT_in);

// If vertical tracking wheel is disabled (port 0), inflate MCL motion noise
constexpr double MCL_DRIVE_ENCODER_FALLBACK_NOISE_SCALE = 1.75;

// ── GPS Error Adaptive Weighting ────────────────────────────────────────────
// When GPS error (from gps_get_error()) is good, trust GPS fully.
// When GPS error is poor, reduce GPS influence.
//
// GPS stddev = base + (error - errorGood) * errorScale, clamped to [min, max]
// If GPS error > GPS_ERROR_THRESHOLD_M, skip update entirely (Phase 5).
// If GPS error in [errorGood, errorThreshold], inflate stddev adaptively.

constexpr float GPS_STDDEV_BASE_in          = 1.968504f;
constexpr float GPS_ERROR_GOOD_in           = 1.968504f;
constexpr float GPS_ERROR_SCALE_MULTIPLIER  = 2.0f;    // stddev *= (1 + error_scale * excess_error)
constexpr float GPS_STDDEV_MIN_in           = 1.968504f;
constexpr float GPS_STDDEV_MAX_in           = 11.811024f;
constexpr float GPS_ERROR_THRESHOLD_in      = 19.68504f;

inline constexpr QLength GPS_STDDEV_BASE = length_from_in(GPS_STDDEV_BASE_in);
inline constexpr QLength GPS_ERROR_GOOD = length_from_in(GPS_ERROR_GOOD_in);
inline constexpr QLength GPS_STDDEV_MIN = length_from_in(GPS_STDDEV_MIN_in);
inline constexpr QLength GPS_STDDEV_MAX = length_from_in(GPS_STDDEV_MAX_in);
inline constexpr QLength GPS_ERROR_THRESHOLD = length_from_in(GPS_ERROR_THRESHOLD_in);

// Distance sensors are enabled with the measured offsets above.
constexpr bool MCL_DISABLE_DISTANCE_SENSORS_WHILE_DEBUGGING = false;

// ── Starting Pose (field frame, inches / degrees) ───────────────────────────
// Set these to where the robot is physically placed at boot.
// theta: 0° = +Y (north), CW positive.

constexpr float START_POSE_X_in       = 0.0f;
constexpr float START_POSE_Y_in       = 0.0f;
constexpr float START_POSE_THETA_deg  = 0.0f;

inline constexpr QLength START_POSE_X = length_from_in(START_POSE_X_in);
inline constexpr QLength START_POSE_Y = length_from_in(START_POSE_Y_in);
inline constexpr QAngle START_POSE_THETA = angle_from_deg(START_POSE_THETA_deg);

constexpr bool START_POSE_KNOWN = false;

constexpr StartupPoseMode STARTUP_POSE_MODE =
    StartupPoseMode::GPSXYPlusIMUHeading;

// GPS readiness gate used at boot
constexpr uint32_t STARTUP_GPS_MAX_WAIT_ms      = 8000;
constexpr float    STARTUP_GPS_READY_ERROR_in   = 0.787402f;
constexpr int      STARTUP_GPS_STABLE_SAMPLES   = 6;

inline constexpr QTime STARTUP_GPS_MAX_WAIT = time_from_ms(static_cast<float>(STARTUP_GPS_MAX_WAIT_ms));
inline constexpr QLength STARTUP_GPS_READY_ERROR = length_from_in(STARTUP_GPS_READY_ERROR_in);

// ── Odom Debug Diagnostics ──────────────────────────────────────────────────

constexpr bool     ODOM_DEBUG_ENABLE               = true;
constexpr uint32_t ODOM_DEBUG_LOG_EVERY_ms          = 200;
constexpr double   ODOM_TURN_TRANSLATION_WARN_IN   = 0.35;

constexpr bool     PF_DEBUG_ENABLE                  = true;
constexpr uint32_t PF_DEBUG_LOG_EVERY_ms            = 150;
constexpr float    PF_DEBUG_CORRECTION_WARN_in      = 4.0f;

// ── Driver-control shaping / active brake ─────────────────────────────────

constexpr float DRIVER_JOYSTICK_DEADBAND           = 5.0f;
constexpr float DRIVER_FORWARD_CURVE_T             = 5.0f;
constexpr float DRIVER_TURN_CURVE_T                = 5.0f;
constexpr bool  DRIVER_ACTIVE_BRAKE_ENABLED        = true;
constexpr float DRIVER_ACTIVE_BRAKE_POWER          = 6.0f;   // softer than the reference 10.0
constexpr float DRIVER_ACTIVE_BRAKE_KP             = 0.08f;
constexpr float DRIVER_ACTIVE_BRAKE_STICK_DEADBAND = 5.0f;
constexpr float DRIVER_ACTIVE_BRAKE_POS_DEADBAND_deg = 3.0f;
constexpr float DRIVER_ACTIVE_BRAKE_OUTPUT_DEADBAND  = 1.0f;

// ── Pneumatic (ADI) Ports ───────────────────────────────────────────────────

constexpr char SELECT1_PORT = 'A';
constexpr char SELECT2_PORT = 'B';
constexpr char TONGUE_PORT  = 'C';
constexpr char WING_PORT    = 'D';

// ── Localization ────────────────────────────────────────────────────────────

constexpr int   NUM_PARTICLES             = 500;
constexpr float FIELD_HALF_SIZE_in        = 70.2f;
constexpr float MAX_DISTANCE_SINCE_UPDATE_in = 0.787402f;
constexpr int   MAX_UPDATE_INTERVAL_MS    = 50;       // milliseconds
constexpr float PF_STATIONARY_DEADBAND_in = 0.118110f;

inline constexpr QLength FIELD_HALF_SIZE = length_from_in(FIELD_HALF_SIZE_in);
inline constexpr QLength MAX_DISTANCE_SINCE_UPDATE = length_from_in(MAX_DISTANCE_SINCE_UPDATE_in);
inline constexpr QLength PF_STATIONARY_DEADBAND = length_from_in(PF_STATIONARY_DEADBAND_in);
inline constexpr QLength PF_DEBUG_CORRECTION_WARN = length_from_in(PF_DEBUG_CORRECTION_WARN_in);

// Particle-filter robustness (anti-impoverishment / kidnapped-robot recovery)
constexpr float PF_RESAMPLE_JITTER_SCALE      = 0.50f;  // jitter stddev = DRIVE_NOISE * scale
constexpr float PF_RANDOM_INJECTION_FRACTION  = 0.02f;  // 2% random particles after resample
constexpr float PF_SENSOR_ONLY_EXPLORATION_NOISE_in = 0.0787402f;

inline constexpr QLength PF_SENSOR_ONLY_EXPLORATION_NOISE =
    length_from_in(PF_SENSOR_ONLY_EXPLORATION_NOISE_in);

// Odom-first localization fusion: drivetrain odom remains the controller base,
// while GPS/MCL contribute bounded XY corrections on top of it.
constexpr float LOC_FUSION_STILLNESS_DEADBAND_in      = 0.20f;
constexpr float LOC_GPS_RUNTIME_ERROR_MAX_in          = 8.0f;
constexpr float LOC_GPS_CORRECTION_MAX_in             = 36.0f;
constexpr float LOC_GPS_CORRECTION_STEP_in            = 0.25f;
constexpr int   LOC_MCL_MIN_ACTIVE_SENSORS            = 3;
constexpr float LOC_MCL_CORRECTION_MAX_in             = 18.0f;
constexpr float LOC_MCL_CORRECTION_STEP_in            = 0.10f;
constexpr float LOC_MCL_CORRECTION_JUMP_REJECT_in     = 8.0f;

inline constexpr QLength LOC_FUSION_STILLNESS_DEADBAND =
    length_from_in(LOC_FUSION_STILLNESS_DEADBAND_in);
inline constexpr QLength LOC_GPS_RUNTIME_ERROR_MAX =
    length_from_in(LOC_GPS_RUNTIME_ERROR_MAX_in);
inline constexpr QLength LOC_GPS_CORRECTION_MAX =
    length_from_in(LOC_GPS_CORRECTION_MAX_in);
inline constexpr QLength LOC_GPS_CORRECTION_STEP =
    length_from_in(LOC_GPS_CORRECTION_STEP_in);
inline constexpr QLength LOC_MCL_CORRECTION_MAX =
    length_from_in(LOC_MCL_CORRECTION_MAX_in);
inline constexpr QLength LOC_MCL_CORRECTION_STEP =
    length_from_in(LOC_MCL_CORRECTION_STEP_in);
inline constexpr QLength LOC_MCL_CORRECTION_JUMP_REJECT =
    length_from_in(LOC_MCL_CORRECTION_JUMP_REJECT_in);

// ── Speed / acceleration limits ─────────────────────────────────────────────

constexpr float MAX_SPEED_inps         = 70.866142f;
constexpr float MAX_ACCELERATION_inps2 = 118.11024f;
constexpr float MAX_ANGULAR_VEL_degps  = 572.9578f;

inline constexpr QVelocity MAX_SPEED = velocity_from_inps(MAX_SPEED_inps);
inline constexpr QAcceleration MAX_ACCELERATION = acceleration_from_inps2(MAX_ACCELERATION_inps2);
inline constexpr QAngularVelocity MAX_ANGULAR_VEL = angle_from_deg(MAX_ANGULAR_VEL_degps) / second;

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
    float v_left  = v - omega * TRACK_WIDTH.getValue() / 2.0f;
    float v_right = v + omega * TRACK_WIDTH.getValue() / 2.0f;
    float a_left  = a - alpha * TRACK_WIDTH.getValue() / 2.0f;
    float a_right = a + alpha * TRACK_WIDTH.getValue() / 2.0f;

    auto ff = [](float vel, float acc) -> float {
        float sign = (vel > 0.0f) ? 1.0f : ((vel < 0.0f) ? -1.0f : 0.0f);
        return FF_kS * sign + FF_kV * vel + FF_kA * acc;
    };

    return {ff(v_left, a_left), ff(v_right, a_right)};
}

} // namespace CONFIG
