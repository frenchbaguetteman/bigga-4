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
inline const Eigen::Vector2f GPS_OFFSET = robot_offset_to_internal(MCL_GPS_OFFSET_X, MCL_GPS_OFFSET_Y);

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

// ── Starting Pose (field frame, inches / degrees) ───────────────────────────
// Set these to where the robot is physically placed at boot.
// X/Y use the canonical field axes (+X east/forward, +Y north/left).
// theta stays human-friendly in VEX compass convention: 0° = north, CW positive.

constexpr float START_POSE_X_in       = 0.0f;
constexpr float START_POSE_Y_in       = 0.0f;
constexpr float START_POSE_THETA_deg  = 0.0f;

inline constexpr QLength START_POSE_X = START_POSE_X_in * inch;
inline constexpr QLength START_POSE_Y = START_POSE_Y_in * inch;
inline constexpr QAngle START_POSE_THETA = QAngle(gpsHeadingDegToInternalRad(START_POSE_THETA_deg));

constexpr bool START_POSE_KNOWN = false;

constexpr StartupPoseMode STARTUP_POSE_MODE =
    StartupPoseMode::FullGPSInit;

// GPS readiness gate used at boot
constexpr uint32_t STARTUP_GPS_MAX_WAIT_ms      = 8000;
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

inline constexpr QLength FIELD_HALF_SIZE = FIELD_HALF_SIZE_in * inch;
inline constexpr QLength MAX_DISTANCE_SINCE_UPDATE = MAX_DISTANCE_SINCE_UPDATE_in * inch;
inline constexpr QLength PF_STATIONARY_DEADBAND = PF_STATIONARY_DEADBAND_in * inch;
inline constexpr QLength PF_DEBUG_CORRECTION_WARN = PF_DEBUG_CORRECTION_WARN_in * inch;

// Particle-filter robustness (anti-impoverishment / kidnapped-robot recovery)
constexpr float PF_RESAMPLE_JITTER_SCALE      = 0.50f;  // jitter stddev = DRIVE_NOISE * scale
constexpr float PF_RANDOM_INJECTION_BASE_FRACTION = 0.02f;   // residual exploration after resample
constexpr float PF_RANDOM_INJECTION_MAX_FRACTION  = 0.20f;   // cap adaptive random-particle recovery
constexpr float PF_RECOVERY_ALPHA_SLOW           = 0.001f;  // textbook w_slow update rate
constexpr float PF_RECOVERY_ALPHA_FAST           = 0.10f;   // textbook w_fast update rate
constexpr float PF_SENSOR_ONLY_EXPLORATION_NOISE_in = 0.0787402f;

inline constexpr QLength PF_SENSOR_ONLY_EXPLORATION_NOISE =
    PF_SENSOR_ONLY_EXPLORATION_NOISE_in * inch;

// Odom-first localization fusion: drivetrain odom remains the controller base,
// while GPS/MCL contribute bounded XY corrections on top of it.
constexpr float LOC_FUSION_STILLNESS_DEADBAND_in      = 0.20f;
constexpr float LOC_GPS_RUNTIME_ERROR_MAX_in          = 8.0f;
constexpr float LOC_GPS_CORRECTION_MAX_in             = 36.0f;
constexpr float LOC_GPS_CORRECTION_STEP_in            = 0.25f;
constexpr int   LOC_MCL_MIN_ACTIVE_SENSORS            = 3;
constexpr float LOC_MCL_CORRECTION_MAX_in             = 18.0f;
constexpr float LOC_MCL_CORRECTION_STEP_in            = 0.06f;
constexpr float LOC_MCL_CORRECTION_JUMP_REJECT_in     = 3.5f;
constexpr float LOC_MCL_MIN_ESS_RATIO                 = 0.22f;

// Distance-model obstacles for the Push Back field composition.
// Coordinates are in canonical field-frame metres (origin at field centre).
struct FieldObstacle {
    float minX;
    float maxX;
    float minY;
    float maxY;
};

constexpr bool MCL_ENABLE_FIELD_OBSTACLES = true;
inline constexpr std::array<FieldObstacle, 3> MCL_FIELD_OBSTACLES{{
    {-0.3048f,  0.3048f, -0.3048f,  0.3048f},  // central field block (24 in square)
    {-1.3716f, -1.0668f, -0.3048f,  0.3048f},  // west lane obstruction
    { 1.0668f,  1.3716f, -0.3048f,  0.3048f},  // east lane obstruction
}};

inline constexpr QLength LOC_FUSION_STILLNESS_DEADBAND =
    LOC_FUSION_STILLNESS_DEADBAND_in * inch;
inline constexpr QLength LOC_GPS_RUNTIME_ERROR_MAX =
    LOC_GPS_RUNTIME_ERROR_MAX_in * inch;
inline constexpr QLength LOC_GPS_CORRECTION_MAX =
    LOC_GPS_CORRECTION_MAX_in * inch;
inline constexpr QLength LOC_GPS_CORRECTION_STEP =
    LOC_GPS_CORRECTION_STEP_in * inch;
inline constexpr QLength LOC_MCL_CORRECTION_MAX =
    LOC_MCL_CORRECTION_MAX_in * inch;
inline constexpr QLength LOC_MCL_CORRECTION_STEP =
    LOC_MCL_CORRECTION_STEP_in * inch;
inline constexpr QLength LOC_MCL_CORRECTION_JUMP_REJECT =
    LOC_MCL_CORRECTION_JUMP_REJECT_in * inch;

// ── Speed / acceleration limits ─────────────────────────────────────────────

constexpr float MAX_SPEED_inps         = 70.866142f;
constexpr float MAX_ACCELERATION_inps2 = 118.11024f;
constexpr float MAX_ANGULAR_VEL_degps  = 572.9578f;

inline constexpr QSpeed MAX_SPEED = MAX_SPEED_inps * ips;
inline constexpr QAcceleration MAX_ACCELERATION = MAX_ACCELERATION_inps2 * ips2;
inline constexpr QAngularSpeed MAX_ANGULAR_VEL = MAX_ANGULAR_VEL_degps * dps;

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
