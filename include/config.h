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
 * Human-facing configuration values use Okapi-style typed quantities and
 * literals such as `24_in`, `90_deg`, `250_ms`, and `2_mps`.
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
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace CONFIG {

// ── Shared conversion helpers ──────────────────────────────────────────────

constexpr float IN_TO_M   = 0.0254f;
constexpr float INCH_TO_M = IN_TO_M;  // backward-compatible alias
constexpr float CM_TO_IN  = 1.0f / 2.54f;
constexpr float PI_F      = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI_F / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI_F;

inline float wrapAngleRadians(float angleRad) {
    return std::atan2(std::sin(angleRad), std::cos(angleRad));
}

constexpr float wrapDegreesSigned(float degrees) {
    while (degrees <= -180.0f) degrees += 360.0f;
    while (degrees > 180.0f) degrees -= 360.0f;
    return degrees;
}

constexpr float wrapDegreesPositive(float degrees) {
    while (degrees < 0.0f) degrees += 360.0f;
    while (degrees >= 360.0f) degrees -= 360.0f;
    return degrees;
}

constexpr float absf(float value) {
    return value < 0.0f ? -value : value;
}

constexpr bool approxEq(float a, float b, float eps = 1e-4f) {
    return absf(a - b) <= eps;
}

/**
 * Convert VEX GPS compass heading (0° = north, CW positive) to internal heading
 * (0° = east/forward, CCW positive).
 *
 * @param compassDeg  VEX compass heading in degrees [0, 360)
 * @return internal heading in radians
 */
constexpr float gpsHeadingDegToInternalRad(float compassDeg) {
    return wrapDegreesSigned(90.0f - compassDeg) * DEG_TO_RAD;
}

/**
 * Convert internal heading (0° = east/forward, CCW positive) to VEX GPS
 * compass heading (0° = north, CW positive).
 *
 * @param internalRad  internal heading in radians
 * @return GPS compass heading in degrees [0, 360)
 */
constexpr float internalRadToGpsHeadingDeg(float internalRad) {
    const float internalDeg = wrapDegreesSigned(internalRad * RAD_TO_DEG);
    return wrapDegreesPositive(90.0f - internalDeg);
}

// Legacy aliases for backward compatibility
inline float compassDegToMathRad(float headingDeg) {
    return gpsHeadingDegToInternalRad(headingDeg);
}

inline float mathRadToCompassDeg(float headingRad) {
    return internalRadToGpsHeadingDeg(headingRad);
}

static_assert(approxEq(gpsHeadingDegToInternalRad(0.0f), 90.0f * DEG_TO_RAD));
static_assert(approxEq(gpsHeadingDegToInternalRad(90.0f), 0.0f));
static_assert(approxEq(gpsHeadingDegToInternalRad(180.0f), -90.0f * DEG_TO_RAD));
static_assert(approxEq(gpsHeadingDegToInternalRad(270.0f), 180.0f * DEG_TO_RAD));
static_assert(approxEq(internalRadToGpsHeadingDeg(0.0f), 90.0f));
static_assert(approxEq(internalRadToGpsHeadingDeg(90.0f * DEG_TO_RAD), 0.0f));
static_assert(approxEq(internalRadToGpsHeadingDeg(-90.0f * DEG_TO_RAD), 180.0f));

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

inline constexpr QLength DRIVE_RADIUS = DRIVE_RADIUS_in * inch;
inline constexpr QLength ODOM_RADIUS  = ODOM_RADIUS_in * inch;
inline constexpr QLength TRACK_WIDTH  = TRACK_WIDTH_in * inch;
inline constexpr QLength WHEEL_BASE   = WHEEL_BASE_in * inch;

// ── Noise model ─────────────────────────────────────────────────────────────

constexpr float DRIVE_NOISE_in  = 0.15748f;
constexpr float ANGLE_NOISE_deg = 0.572958f;

inline constexpr QLength DRIVE_NOISE = DRIVE_NOISE_in * inch;
inline constexpr QAngle ANGLE_NOISE = ANGLE_NOISE_deg * degree;

// ── Drivetrain Motor Ports (negative = reversed) ────────────────────────────

inline constexpr std::array<std::int8_t, 3> LEFT_DRIVE_PORTS  {{-11, -15, -14}};
inline constexpr std::array<std::int8_t, 3> RIGHT_DRIVE_PORTS {{ 18,  17,  20}};

// ── Intake Motor Ports ──────────────────────────────────────────────────────

inline constexpr std::array<std::int8_t, 2> INTAKE_PORTS {{-6, 8}};

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
inline constexpr QLength LATERAL_WHEEL_OFFSET = LATERAL_WHEEL_OFFSET_in * inch;

// ── MCL Distance Sensor Ports ───────────────────────────────────────────────

constexpr int MCL_LEFT_DISTANCE_PORT   = 2;    // facing +90° (left)
constexpr int MCL_RIGHT_DISTANCE_PORT  = 5;    // facing -90° (right)
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

inline constexpr QLength ROBOT_WIDTH = ROBOT_WIDTH_in * inch;
inline constexpr QLength ROBOT_LENGTH = ROBOT_LENGTH_in * inch;
inline constexpr QLength MCL_LEFT_OFFSET_X = MCL_LEFT_OFFSET_X_in * inch;
inline constexpr QLength MCL_LEFT_OFFSET_Y = MCL_LEFT_OFFSET_Y_in * inch;
inline constexpr QLength MCL_RIGHT_OFFSET_X = MCL_RIGHT_OFFSET_X_in * inch;
inline constexpr QLength MCL_RIGHT_OFFSET_Y = MCL_RIGHT_OFFSET_Y_in * inch;
inline constexpr QLength MCL_BACK_OFFSET_X = MCL_BACK_OFFSET_X_in * inch;
inline constexpr QLength MCL_BACK_OFFSET_Y = MCL_BACK_OFFSET_Y_in * inch;
inline constexpr QLength MCL_FRONT_OFFSET_X = MCL_FRONT_OFFSET_X_in * inch;
inline constexpr QLength MCL_FRONT_OFFSET_Y = MCL_FRONT_OFFSET_Y_in * inch;
inline constexpr QLength MCL_GPS_OFFSET_X = MCL_GPS_OFFSET_X_in * inch;
inline constexpr QLength MCL_GPS_OFFSET_Y = MCL_GPS_OFFSET_Y_in * inch;

inline Eigen::Vector2f robot_offset_to_internal(QLength offsetXRight, QLength offsetYForward) {
    return Eigen::Vector2f(offsetYForward.convert(meter), -offsetXRight.convert(meter));
}

// GPS offset in metres, INTERNAL robot frame (+X forward, +Y left)
inline Eigen::Vector2f gpsOffset() {
    return robot_offset_to_internal(MCL_GPS_OFFSET_X, MCL_GPS_OFFSET_Y);
}

constexpr float MCL_LEFT_FACING_deg  =  90.0f;
constexpr float MCL_RIGHT_FACING_deg = -90.0f;
constexpr float MCL_BACK_FACING_deg  = 180.0f;
constexpr float MCL_FRONT_FACING_deg =   0.0f;

inline constexpr QAngle MCL_LEFT_FACING  = MCL_LEFT_FACING_deg * degree;
inline constexpr QAngle MCL_RIGHT_FACING = MCL_RIGHT_FACING_deg * degree;
inline constexpr QAngle MCL_BACK_FACING  = MCL_BACK_FACING_deg * degree;
inline constexpr QAngle MCL_FRONT_FACING = MCL_FRONT_FACING_deg * degree;

inline Eigen::Vector3f sensor_offset_to_internal(QLength offsetXRight,
                                                 QLength offsetYForward,
                                                 QAngle facing) {
    return Eigen::Vector3f(offsetYForward.convert(meter),
                           -offsetXRight.convert(meter),
                           facing.convert(radian));
}

inline Eigen::Vector3f distLeftOffset() {
    return sensor_offset_to_internal(
        MCL_LEFT_OFFSET_X, MCL_LEFT_OFFSET_Y, MCL_LEFT_FACING);
}

inline Eigen::Vector3f distRightOffset() {
    return sensor_offset_to_internal(
        MCL_RIGHT_OFFSET_X, MCL_RIGHT_OFFSET_Y, MCL_RIGHT_FACING);
}

inline Eigen::Vector3f distFrontOffset() {
    return sensor_offset_to_internal(
        MCL_FRONT_OFFSET_X, MCL_FRONT_OFFSET_Y, MCL_FRONT_FACING);
}

inline Eigen::Vector3f distBackOffset() {
    return sensor_offset_to_internal(
        MCL_BACK_OFFSET_X, MCL_BACK_OFFSET_Y, MCL_BACK_FACING);
}

// ── MCL Distance Sensor Fusion Controls ─────────────────────────────────────

constexpr bool MCL_ENABLE_DISTANCE_SENSORS         = true;
constexpr bool MCL_ENABLE_LEFT_DISTANCE_SENSOR     = true;
constexpr bool MCL_ENABLE_RIGHT_DISTANCE_SENSOR    = true;
constexpr bool MCL_ENABLE_BACK_DISTANCE_SENSOR     = true;
constexpr bool MCL_ENABLE_FRONT_DISTANCE_SENSOR    = true;
constexpr bool MCL_ENABLE_GPS_SENSOR               = true;

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

inline constexpr QLength MCL_DISTANCE_STDDEV = MCL_DISTANCE_STDDEV_in * inch;
inline constexpr QLength MCL_DISTANCE_MIN_RANGE = MCL_DISTANCE_MIN_RANGE_in * inch;
inline constexpr QLength MCL_DISTANCE_MAX_RANGE = MCL_DISTANCE_MAX_RANGE_in * inch;
inline constexpr QLength MCL_DISTANCE_CONFIDENCE_EXEMPT = MCL_DISTANCE_CONFIDENCE_EXEMPT_in * inch;

// If vertical tracking wheel is disabled (port 0), inflate MCL motion noise
constexpr double MCL_DRIVE_ENCODER_FALLBACK_NOISE_SCALE = 1.75;

// ── GPS Error Adaptive Weighting ────────────────────────────────────────────
// When GPS error (from gps_get_error()) is good, trust GPS fully.
// When GPS error is poor, reduce GPS influence.
//
// GPS stddev = base + (error - errorGood) * errorScale, clamped to [min, max]
// If GPS error > GPS_ERROR_THRESHOLD, skip update entirely (Phase 5).
// If GPS error in [errorGood, errorThreshold], inflate stddev adaptively.

constexpr float GPS_STDDEV_BASE_in          = 1.968504f;
constexpr float GPS_ERROR_GOOD_in           = 1.968504f;
constexpr float GPS_ERROR_SCALE_MULTIPLIER  = 2.0f;    // stddev *= (1 + error_scale * excess_error)
constexpr float GPS_STDDEV_MIN_in           = 1.968504f;
constexpr float GPS_STDDEV_MAX_in           = 11.811024f;
constexpr float GPS_ERROR_THRESHOLD_in      = 19.68504f;

inline constexpr QLength GPS_STDDEV_BASE = GPS_STDDEV_BASE_in * inch;
inline constexpr QLength GPS_ERROR_GOOD = GPS_ERROR_GOOD_in * inch;
inline constexpr QLength GPS_STDDEV_MIN = GPS_STDDEV_MIN_in * inch;
inline constexpr QLength GPS_STDDEV_MAX = GPS_STDDEV_MAX_in * inch;
inline constexpr QLength GPS_ERROR_THRESHOLD = GPS_ERROR_THRESHOLD_in * inch;

// Distance sensors are enabled with the measured offsets above.
constexpr bool MCL_DISABLE_DISTANCE_SENSORS_WHILE_DEBUGGING = false;

// ── Sensor Staleness ────────────────────────────────────────────────────────
constexpr uint32_t SENSOR_STALENESS_THRESHOLD_ms = 500;

// ── Dynamic Obstacle Rejection ──────────────────────────────────────────────
// Detects other robots / unexpected obstacles in front of distance sensors.
// When the measured distance is consistently much shorter than expected from
// the static field model, the sensor is temporarily excluded from the
// particle-filter weight update.
constexpr bool     MCL_DYNAMIC_OBSTACLE_ENABLED            = true;
constexpr float    MCL_DYNAMIC_OBSTACLE_THRESHOLD_in       = 6.0f;   // min shortfall to flag
constexpr size_t   MCL_DYNAMIC_OBSTACLE_WINDOW_SIZE        = 8;      // ring-buffer depth
constexpr size_t   MCL_DYNAMIC_OBSTACLE_CONSISTENCY_COUNT  = 5;      // # short reads to trigger

// ── Starting Pose (field frame, inches / degrees) ───────────────────────────
// Set these to where the robot is physically placed at boot.
// X/Y use the canonical field axes (+X east/forward, +Y north/left).
// theta stays human-friendly in VEX compass convention: 0° = north, CW positive.
//
// `DEFAULT_IMU_INIT_ANGLE_deg` is separate from `START_POSE_THETA_deg`:
// it seeds the raw IMU into the field frame before localization has locked a
// pose. This is mainly used by GPSXYPlusIMUHeading startup and for the brief
// boot window before the final localization pose is applied.

constexpr float DEFAULT_IMU_INIT_ANGLE_deg = 180.0f;
inline constexpr QAngle DEFAULT_IMU_INIT_ANGLE =
    QAngle(gpsHeadingDegToInternalRad(DEFAULT_IMU_INIT_ANGLE_deg));

constexpr float START_POSE_X_in       = 0.0f;
constexpr float START_POSE_Y_in       = 0.0f;
constexpr float START_POSE_THETA_deg  = 180.0f;

inline constexpr QLength START_POSE_X = START_POSE_X_in * inch;
inline constexpr QLength START_POSE_Y = START_POSE_Y_in * inch;
inline constexpr QAngle START_POSE_THETA = QAngle(gpsHeadingDegToInternalRad(START_POSE_THETA_deg));

constexpr bool START_POSE_KNOWN = false;

constexpr StartupPoseMode STARTUP_POSE_MODE =
    StartupPoseMode::GPSXYPlusIMUHeading;

// GPS readiness gate used at boot
constexpr uint32_t STARTUP_GPS_MAX_WAIT_ms      = 10000;
constexpr float    STARTUP_GPS_READY_ERROR_in   = 0.787402f;
constexpr float    STARTUP_GPS_READY_HEADING_deg = 5.0f;
constexpr int      STARTUP_GPS_STABLE_SAMPLES   = 6;
constexpr uint32_t STARTUP_SCREEN_MIN_VISIBLE_ms = 1200;
constexpr uint32_t STARTUP_READY_HOLD_ms         = 500;

inline constexpr QTime STARTUP_GPS_MAX_WAIT = static_cast<float>(STARTUP_GPS_MAX_WAIT_ms) * millisecond;
inline constexpr QLength STARTUP_GPS_READY_ERROR = STARTUP_GPS_READY_ERROR_in * inch;
inline constexpr QAngle STARTUP_GPS_READY_HEADING = STARTUP_GPS_READY_HEADING_deg * degree;

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

constexpr char TOP_PORT = 'A';
constexpr char SELECT_PORT = 'B';
constexpr char TONGUE_PORT  = 'C';
constexpr char WING_PORT    = 'D';

// ── Intake ───────────────────────────────────────────────────────────────────

constexpr int INTAKE_STALL_CURRENT_mA = 2500;

// ── Localization ────────────────────────────────────────────────────────────

constexpr int   NUM_PARTICLES             = 500;
constexpr float FIELD_HALF_SIZE_in        = 70.2f;
constexpr float MAX_DISTANCE_SINCE_UPDATE_in = 0.787402f;
constexpr int   MAX_UPDATE_INTERVAL_MS    = 50;       // milliseconds
constexpr float PF_STATIONARY_DEADBAND_in = 0.118110f;

inline constexpr QLength FIELD_HALF_SIZE = FIELD_HALF_SIZE_in * inch;
inline constexpr QLength MAX_DISTANCE_SINCE_UPDATE = MAX_DISTANCE_SINCE_UPDATE_in * inch;
inline constexpr QLength PF_STATIONARY_DEADBAND = PF_STATIONARY_DEADBAND_in * inch;
inline constexpr QLength PF_DEBUG_CORRECTION_WARN = PF_DEBUG_CORRECTION_WARN_in * inch;

// Particle-filter robustness (anti-impoverishment / kidnapped-robot recovery)
// Keep the particle cloud tighter so MCL stays focused on local verification,
// while GPS continues to provide broader absolute positioning context.
constexpr float PF_RESAMPLE_JITTER_SCALE      = 0.30f;  // jitter stddev = DRIVE_NOISE * scale
constexpr float PF_RANDOM_INJECTION_BASE_FRACTION = 0.002f;  // residual exploration after resample
constexpr float PF_RANDOM_INJECTION_MAX_FRACTION  = 0.04f;   // cap adaptive random-particle recovery
constexpr float PF_RECOVERY_ALPHA_SLOW           = 0.001f;  // textbook w_slow update rate
constexpr float PF_RECOVERY_ALPHA_FAST           = 0.10f;   // textbook w_fast update rate
constexpr float PF_SENSOR_ONLY_EXPLORATION_NOISE_in = 0.0472441f;
constexpr float PF_RESAMPLE_ESS_RATIO            = 0.40f;
constexpr float PF_MEASUREMENT_CORRECTION_DEADBAND_in = 0.10f;
constexpr float PF_MEASUREMENT_CORRECTION_MAX_STEP_in = 0.18f;
constexpr float PF_MEASUREMENT_CORRECTION_MAX_STEP_ABS_in = 0.35f;
constexpr float PF_MEASUREMENT_CORRECTION_BLEND_MIN = 0.08f;
constexpr float PF_MEASUREMENT_CORRECTION_BLEND_MAX = 0.20f;
constexpr float PF_MEASUREMENT_CORRECTION_ABS_BONUS = 0.08f;

inline constexpr QLength PF_SENSOR_ONLY_EXPLORATION_NOISE =
    PF_SENSOR_ONLY_EXPLORATION_NOISE_in * inch;
inline constexpr QLength PF_MEASUREMENT_CORRECTION_DEADBAND =
    PF_MEASUREMENT_CORRECTION_DEADBAND_in * inch;
inline constexpr QLength PF_MEASUREMENT_CORRECTION_MAX_STEP =
    PF_MEASUREMENT_CORRECTION_MAX_STEP_in * inch;
inline constexpr QLength PF_MEASUREMENT_CORRECTION_MAX_STEP_ABS =
    PF_MEASUREMENT_CORRECTION_MAX_STEP_ABS_in * inch;

// Odom-first localization fusion: drivetrain odom remains the controller base,
// while GPS/MCL contribute bounded XY corrections on top of it.
constexpr float LOC_FUSION_STILLNESS_DEADBAND_in      = 0.20f;
constexpr float LOC_GPS_RUNTIME_ERROR_MAX_in          = 8.0f;
constexpr float LOC_GPS_CORRECTION_MAX_in             = 36.0f;
constexpr float LOC_GPS_CORRECTION_STEP_in            = 0.25f;
constexpr float LOC_GPS_CORRECTION_DEADBAND_in        = 0.12f;
constexpr int   LOC_MCL_MIN_ACTIVE_SENSORS            = 3;
constexpr float LOC_MCL_CORRECTION_MAX_in             = 18.0f;
constexpr float LOC_MCL_CORRECTION_STEP_in            = 0.04f;
constexpr float LOC_MCL_CORRECTION_DEADBAND_in        = 0.10f;
constexpr float LOC_MCL_CORRECTION_JUMP_REJECT_in     = 3.5f;
constexpr float LOC_MCL_MIN_ESS_RATIO                 = 0.22f;
constexpr float LOC_CONTROLLER_CORRECTION_MAX_in      = 12.0f;
constexpr float LOC_CONTROLLER_CORRECTION_STEP_in     = 0.06f;
constexpr float LOC_CONTROLLER_CORRECTION_DEADBAND_in = 0.08f;
constexpr float LOC_CONTROLLER_CORRECTION_JUMP_REJECT_in = 1.5f;
constexpr float LOC_CONTROLLER_REACQUIRE_DEADBAND_in  = 0.20f;
constexpr int   LOC_CONTROLLER_REACQUIRE_STABLE_CYCLES = 6;

// Distance-model obstacles for the Push Back field composition.
// Coordinates are in canonical field-frame metres (origin at field centre).
//
// These are intentionally top-down approximations of the vertical surfaces that
// a horizontal distance sensor can plausibly see, based on the official V5RC
// Push Back Appendix A field drawings:
// - "Field Reference Specifications - V5RC"
// - "Object Placement (Top View) - V5RC"
// - "Object Placement Reference - V5RC"
//
// The long goals are not modeled as solid 12 x 48 in blocks because their
// middle spans are partially suspended. The center structure is also not a
// single square block; it is approximated as a hub plus four diagonal arms,
// along with the four small center goals around it.
struct FieldObstacle {
    float centerX;
    float centerY;
    float halfSizeX;
    float halfSizeY;
    float headingRad;
    float minZ;
    float maxZ;
};

constexpr float obstacleMeters(float inches) {
    return inches * IN_TO_M;
}

constexpr float obstacleRadians(float degrees) {
    return degrees * DEG_TO_RAD;
}

constexpr FieldObstacle makeAxisObstacle(float minXIn,
                                         float maxXIn,
                                         float minYIn,
                                         float maxYIn,
                                         float minZIn,
                                         float maxZIn) {
    return FieldObstacle{
        obstacleMeters((minXIn + maxXIn) * 0.5f),
        obstacleMeters((minYIn + maxYIn) * 0.5f),
        obstacleMeters((maxXIn - minXIn) * 0.5f),
        obstacleMeters((maxYIn - minYIn) * 0.5f),
        0.0f,
        obstacleMeters(minZIn),
        obstacleMeters(maxZIn),
    };
}

constexpr FieldObstacle makeRotatedObstacle(float centerXIn,
                                            float centerYIn,
                                            float sizeXIn,
                                            float sizeYIn,
                                            float headingDeg,
                                            float minZIn,
                                            float maxZIn) {
    return FieldObstacle{
        obstacleMeters(centerXIn),
        obstacleMeters(centerYIn),
        obstacleMeters(sizeXIn * 0.5f),
        obstacleMeters(sizeYIn * 0.5f),
        obstacleRadians(headingDeg),
        obstacleMeters(minZIn),
        obstacleMeters(maxZIn),
    };
}

constexpr float MCL_DISTANCE_SENSOR_HEIGHT_in = 4.5f;
inline constexpr QLength MCL_DISTANCE_SENSOR_HEIGHT =
    MCL_DISTANCE_SENSOR_HEIGHT_in * inch;

constexpr bool MCL_ENABLE_FIELD_OBSTACLES = true;
inline constexpr std::array<FieldObstacle, 10> MCL_FIELD_OBSTACLES{{
    // North long goal end supports centered at (x = +/-24, y = +48).
    makeAxisObstacle(-25.9f, -22.1f, 46.4f, 49.6f, 0.0f, 10.0f),
    makeAxisObstacle(22.1f, 25.9f, 46.4f, 49.6f, 0.0f, 10.0f),

    // South long goal end supports centered at (x = +/-24, y = -48).
    makeAxisObstacle(-25.9f, -22.1f, -49.6f, -46.4f, 0.0f, 10.0f),
    makeAxisObstacle(22.1f, 25.9f, -49.6f, -46.4f, 0.0f, 10.0f),

    // Center X-goal: two diagonal bars crossing at the field origin.
    makeRotatedObstacle(0.0f, 0.0f, 16.0f, 2.2f, 45.0f, 0.0f, 7.0f),
    makeRotatedObstacle(0.0f, 0.0f, 16.0f, 2.2f, -45.0f, 0.0f, 7.0f),

    // One small loader square in each west-side quadrant at y = +/-48 in.
    makeAxisObstacle(-70.2f, -67.6f, 46.7f, 49.3f, 0.0f, 10.0f),
    makeAxisObstacle(-70.2f, -67.6f, -49.3f, -46.7f, 0.0f, 10.0f),

    // One small loader square in each east-side quadrant at y = +/-48 in.
    makeAxisObstacle(67.6f, 70.2f, 46.7f, 49.3f, 0.0f, 10.0f),
    makeAxisObstacle(67.6f, 70.2f, -49.3f, -46.7f, 0.0f, 10.0f),
}};

inline constexpr QLength LOC_FUSION_STILLNESS_DEADBAND =
    LOC_FUSION_STILLNESS_DEADBAND_in * inch;
inline constexpr QLength LOC_GPS_RUNTIME_ERROR_MAX =
    LOC_GPS_RUNTIME_ERROR_MAX_in * inch;
inline constexpr QLength LOC_GPS_CORRECTION_MAX =
    LOC_GPS_CORRECTION_MAX_in * inch;
inline constexpr QLength LOC_GPS_CORRECTION_STEP =
    LOC_GPS_CORRECTION_STEP_in * inch;
inline constexpr QLength LOC_GPS_CORRECTION_DEADBAND =
    LOC_GPS_CORRECTION_DEADBAND_in * inch;
inline constexpr QLength LOC_MCL_CORRECTION_MAX =
    LOC_MCL_CORRECTION_MAX_in * inch;
inline constexpr QLength LOC_MCL_CORRECTION_STEP =
    LOC_MCL_CORRECTION_STEP_in * inch;
inline constexpr QLength LOC_MCL_CORRECTION_DEADBAND =
    LOC_MCL_CORRECTION_DEADBAND_in * inch;
inline constexpr QLength LOC_MCL_CORRECTION_JUMP_REJECT =
    LOC_MCL_CORRECTION_JUMP_REJECT_in * inch;
inline constexpr QLength LOC_CONTROLLER_CORRECTION_MAX =
    LOC_CONTROLLER_CORRECTION_MAX_in * inch;
inline constexpr QLength LOC_CONTROLLER_CORRECTION_STEP =
    LOC_CONTROLLER_CORRECTION_STEP_in * inch;
inline constexpr QLength LOC_CONTROLLER_CORRECTION_DEADBAND =
    LOC_CONTROLLER_CORRECTION_DEADBAND_in * inch;
inline constexpr QLength LOC_CONTROLLER_CORRECTION_JUMP_REJECT =
    LOC_CONTROLLER_CORRECTION_JUMP_REJECT_in * inch;
inline constexpr QLength LOC_CONTROLLER_REACQUIRE_DEADBAND =
    LOC_CONTROLLER_REACQUIRE_DEADBAND_in * inch;

// ── Speed / acceleration limits ─────────────────────────────────────────────

constexpr float MAX_SPEED_inps         = 70.866142f;
constexpr float MAX_ACCELERATION_inps2 = 118.11024f;
constexpr float MAX_ANGULAR_VEL_degps  = 572.9578f;

inline constexpr QSpeed MAX_SPEED = MAX_SPEED_inps * ips;
inline constexpr QAcceleration MAX_ACCELERATION = MAX_ACCELERATION_inps2 * ips2;
inline constexpr QAngularSpeed MAX_ANGULAR_VEL = MAX_ANGULAR_VEL_degps * dps;

// ── Path-controller telemetry ───────────────────────────────────────────────

constexpr bool     PATH_TELEMETRY_ENABLE       = false;
constexpr uint32_t PATH_TELEMETRY_LOG_EVERY_ms = 50;

// ── PID gains ───────────────────────────────────────────────────────────────
//  PID(kP, kI, kD, integralCap)  — see feedback/pid.h

constexpr float PID_AUTOTUNE_DRIVE_TARGET_in = 24.0f;
constexpr float PID_AUTOTUNE_DRIVE_RELAY = 0.55f;
constexpr float PID_AUTOTUNE_DRIVE_NOISE_in = 0.75f;
constexpr float PID_AUTOTUNE_TURN_TARGET_deg = 90.0f;
constexpr float PID_AUTOTUNE_TURN_RELAY = 0.40f;
constexpr float PID_AUTOTUNE_TURN_NOISE_deg = 2.0f;
constexpr uint32_t PID_AUTOTUNE_TIMEOUT_ms = 15000;
constexpr std::size_t PID_AUTOTUNE_REQUIRED_PEAK_PAIRS = 4;
constexpr std::size_t PID_AUTOTUNE_ANALYSIS_PEAK_PAIRS = 3;

inline constexpr PID::Gains TURN_PID      {2.0f, 0.0f, 0.15f, 0.0f};
inline constexpr PID::Gains DISTANCE_PID  {5.0f, 0.0f, 0.3f,  0.0f};

// ── RAMSETE path-following parameters ───────────────────────────────────────

constexpr float RAMSETE_ZETA = 0.4f;       // damping  (0 < ζ < 1)
constexpr float RAMSETE_BETA = 45.0f;      // aggressiveness  (β > 0)

// ── LTV unicycle cost matrix  Q = diag(q1, q2, q3) ─────────────────────────

inline Eigen::Vector3f defaultDtCostQ() {
    return Eigen::Vector3f(1.0f, 1.0f, 10.0f);
}

// ── Drivetrain feedforward model ────────────────────────────────────────────

constexpr float FF_kS = 1100.0f;   // static friction offset  (mV)
constexpr float FF_kV = 5200.0f;   // velocity gain           (mV·s/m)
constexpr float FF_kA = 400.0f;    // acceleration gain        (mV·s²/m)

inline std::pair<float, float> DRIVETRAIN_FEEDFORWARD(
        float v, float omega, float a = 0.0f, float alpha = 0.0f) {
    const float trackWidthM = TRACK_WIDTH.convert(meter);
    float v_left  = v - omega * trackWidthM / 2.0f;
    float v_right = v + omega * trackWidthM / 2.0f;
    float a_left  = a - alpha * trackWidthM / 2.0f;
    float a_right = a + alpha * trackWidthM / 2.0f;

    auto ff = [](float vel, float acc) -> float {
        float sign = (vel > 0.0f) ? 1.0f : ((vel < 0.0f) ? -1.0f : 0.0f);
        return FF_kS * sign + FF_kV * vel + FF_kA * acc;
    };

    return {ff(v_left, a_left), ff(v_right, a_right)};
}

} // namespace CONFIG
