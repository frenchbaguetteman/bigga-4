# Configuration Surface

Primary file: [`include/config.h`](../include/config.h)

## Core Rules

| Topic | Rule |
|---|---|
| Internal position units | meters |
| Internal heading units | radians |
| Internal field frame | `+X` forward/east, `+Y` left/north, positive CCW |
| Human-facing config helpers | typed units such as `24_in`, `90_deg`, `2_mps` |

## Hardware Ports

| Constant | Value |
|---|---|
| `LEFT_DRIVE_PORTS` | `{-11, -15, -14}` |
| `RIGHT_DRIVE_PORTS` | `{18, 17, 20}` |
| `INTAKE_PORTS` | `{-6, 8}` |
| `IMU_PORT` | `13` |
| `VERTICAL_TRACKING_PORT` | `0` |
| `HORIZONTAL_TRACKING_PORT` | `16` |
| `MCL_LEFT_DISTANCE_PORT` | `2` |
| `MCL_RIGHT_DISTANCE_PORT` | `5` |
| `MCL_BACK_DISTANCE_PORT` | `4` |
| `MCL_FRONT_DISTANCE_PORT` | `1` |
| `MCL_GPS_PORT` | `3` |

## Driver Tuning

| Constant | Value |
|---|---|
| `DRIVER_FORWARD_CURVE_T` | `5.0f` |
| `DRIVER_TURN_CURVE_T` | `5.0f` |
| `DRIVER_ACTIVE_BRAKE_ENABLED` | `true` |
| `DRIVER_ACTIVE_BRAKE_POWER` | `6.0f` |
| `DRIVER_ACTIVE_BRAKE_KP` | `0.08f` |
| `DRIVER_ACTIVE_BRAKE_STICK_DEADBAND` | `5.0f` |
| `DRIVER_ACTIVE_BRAKE_POS_DEADBAND_deg` | `3.0f` |
| `DRIVER_ACTIVE_BRAKE_OUTPUT_DEADBAND` | `1.0f` |

## Motion Control Gains

| Constant | Value |
|---|---|
| `TURN_PID` | `{2.0f, 0.0f, 0.15f, 0.0f}` |
| `DISTANCE_PID` | `{5.0f, 0.0f, 0.3f, 0.0f}` |
| `RAMSETE_ZETA` | `0.4f` |
| `RAMSETE_BETA` | `45.0f` |

## High-Impact Edit Zones

| Area | Typical constants |
|---|---|
| Driver feel | `DRIVER_*` |
| Point moves and turns | `DISTANCE_PID`, `TURN_PID` |
| Path following | `RAMSETE_ZETA`, `RAMSETE_BETA` |
| Sensor plumbing | `*_PORT`, GPS offsets, tracking wheel config |
| Geometry | wheel radii, track width, wheel base |

## Practical Rule of Thumb

- If teleop feels wrong, edit `DRIVER_*`.
- If simple move and turn examples feel wrong, edit `DISTANCE_PID` and `TURN_PID`.
- If only profiled paths feel wrong, inspect `RAMSETE_*` and profile constraints.
- If all motion feels wrong, validate geometry, ports, and localization before touching gains.
