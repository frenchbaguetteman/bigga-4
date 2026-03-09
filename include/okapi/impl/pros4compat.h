/**
 * @file pros4compat.h
 * Compatibility shim: stubs for PROS 3 C API functions removed in PROS 4.
 *
 * OkapiLib 4.8.0 was built against PROS 3 kernel which exposed motor PID
 * tuning and a couple of status-query helpers that no longer exist in
 * PROS 4.2.x.  Rather than gut the OkapiLib source, we provide thin stubs
 * that let the existing code compile and link.  The motor PID tuning was
 * a rarely-used advanced feature; it is a no-op on PROS 4.
 */
#pragma once

#include "pros/motors.h"
#include "pros/rtos.h"
#include <cstdint>

#ifdef __cplusplus
namespace pros::c {
#endif

/* ---------- status helpers (removed in PROS 4) ---------- */

static inline std::int32_t motor_is_stopped(std::int8_t port) {
    // Best-effort approximation: treat as "stopped" if velocity ~0
    double vel = motor_get_actual_velocity(port);
    return (vel > -0.5 && vel < 0.5) ? 1 : 0;
}

static inline std::int32_t motor_get_zero_position_flag(std::int8_t port) {
    (void)port;
    return 0; // no direct equivalent in PROS 4
}

/* ---------- motor PID structs (removed in PROS 4) ---------- */

#ifndef MOTOR_PID_COMPAT_DEFINED
#define MOTOR_PID_COMPAT_DEFINED

typedef struct motor_pid_s {
    std::uint8_t  kf;
    std::uint8_t  kp;
    std::uint8_t  ki;
    std::uint8_t  kd;
    std::uint8_t  filter;
    std::uint16_t limit;
    std::uint8_t  threshold;
    std::uint8_t  loopspeed;
} motor_pid_s_t;

typedef struct motor_pid_full_s {
    std::uint8_t  kf;
    std::uint8_t  kp;
    std::uint8_t  ki;
    std::uint8_t  kd;
    std::uint8_t  filter;
    std::uint16_t limit;
    std::uint8_t  threshold;
    std::uint8_t  loopspeed;
} motor_pid_full_s_t;

#endif

/* ---------- motor PID tuning (removed in PROS 4 — no-ops) ---------- */

static inline motor_pid_s_t motor_convert_pid(
    double ikF, double ikP, double ikI, double ikD) {
    (void)ikF; (void)ikP; (void)ikI; (void)ikD;
    return motor_pid_s_t{};
}

static inline motor_pid_full_s_t motor_convert_pid_full(
    double ikF, double ikP, double ikI, double ikD,
    double ifilter, double ilimit, double ithreshold, double iloopSpeed) {
    (void)ikF; (void)ikP; (void)ikI; (void)ikD;
    (void)ifilter; (void)ilimit; (void)ithreshold; (void)iloopSpeed;
    return motor_pid_full_s_t{};
}

static inline std::int32_t motor_set_pos_pid(
    std::int8_t port, motor_pid_s_t pid) {
    (void)port; (void)pid;
    return 1; // pretend success
}

static inline std::int32_t motor_set_vel_pid(
    std::int8_t port, motor_pid_s_t pid) {
    (void)port; (void)pid;
    return 1;
}

static inline std::int32_t motor_set_pos_pid_full(
    std::int8_t port, motor_pid_full_s_t pid) {
    (void)port; (void)pid;
    return 1;
}

static inline std::int32_t motor_set_vel_pid_full(
    std::int8_t port, motor_pid_full_s_t pid) {
    (void)port; (void)pid;
    return 1;
}

/* ---------- end of compat shim ---------- */

#ifdef __cplusplus
} // namespace pros::c
#endif
