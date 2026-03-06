# Localization Regression Tests

This document describes regression tests for verifying the localization system
improvements across all 7 phases.

## Test Setup

All tests assume the robot is placed at a **known field position** with access
to debug logs via USB or wireless console.

### Prerequisites

1. Robot fully charged and in good working order
2. IMU calibrated and not moving during startup
3. GPS module powered and stable
4. All distance sensors mounted and functioning
5. Console access to print debug logs (ODOM_DEBUG_ENABLE = true)

---

## Test 1: Boot Stationary (10 seconds)

**Objective**: Verify fused pose does not wander excessively when robot is
stationary.

### Setup

1. Place robot at a known field position (e.g., corner tile intersection).
2. Power on and complete initialization.
3. Do NOT move the robot.

### Execution

1. Let the robot sit for 10 seconds.
2. Observe console logs (format):
   ```
   [LOC] odom_delta=0.0000 imu=0.123 gps(Y)=(1.234,2.345,0.089,err=0.032)
         odom=(1.234,2.345,0.090) pf=(1.234,2.345,0.090)
   ```

### Acceptance Criteria

- **Odom delta** ≈ 0 (encoder creep should be minimal, < 5 mm)
- **GPS position** should vary by < 50 mm (typical VEX GPS quiet mode jitter)
- **PF fused pose** should remain within **±100 mm, ±5°** of start pose
  - Small drifts are normal (MCL will converge over time)
  - Wandering >150 mm indicates sensor fusion issues

### Pass/Fail

- **PASS**: Fused pose stable, wandering < 100 mm, no steady drift
- **FAIL**: Continuous creep > 150 mm, or heading drift > 10°

---

## Test 2: Rotate in Place 90°

**Objective**: Verify heading changes correctly; (x, y) should remain constant.

### Setup

1. Place robot at known position.
2. Initialize normally.

### Execution

1. After boot, stop all motors (let odometry settle).
2. Using arcade drive with only rotation input, turn the robot 90° CW.
   - Hold turn input for ~3 seconds to rotate approximately 90°.
3. Stop and hold for 2 seconds.
4. Observe console logs.

### Acceptance Criteria

- **IMU heading** should change by **~π/2 radians (≈ 1.57 rad)** ±0.2 rad
- **Odom x, y** should change by **< 50 mm** (should be nearly zero)
- **PF heading** should track IMU heading with small lag
- **PF x, y** should track odom (x, y) staying close to start

### Pass/Fail

- **PASS**: Heading ≈ 90°, (x,y) drifts < 50 mm
- **FAIL**: Heading off by > 20°, or (x,y) drift > 100 mm

---

## Test 3: Drive Straight Forward

**Objective**: Verify forward motion changes x correctly in internal frame
(+X = forward), y stays ~constant.

### Setup

1. Place robot at corner or known reference.
2. Point robot toward a field axis (e.g., facing along +X axis at start).
3. Initialize.

### Execution

1. After boot, run arcade drive with forward input (~50% power) for 3–4 seconds.
   - Robot should travel ~1–1.5 meters forward.
2. Stop and hold.
3. Observe console logs during and after motion.

### Acceptance Criteria

- **Odom x** should increase smoothly (0 → ~1.5 m)
- **Odom y** should stay ~constant (drift < 100 mm)
- **Heading** should stay ~constant (drift < ±10°)
- **PF pose** should track odom (x, y) closely

### Pass/Fail

- **PASS**: x increases smoothly, y drifts < 100 mm
- **FAIL**: x does not change, or y drifts > 200 mm

---

## Test 4: Known Start Pose Modes

**Objective**: Verify all three startup pose modes produce consistent results.

### Setup

Place robot at a known field position (e.g., **x=0.5m, y=0.5m, θ=0rad**).

### Execution

Run this test sequence three times, once for each mode:

1. Set `STARTUP_POSE_MODE = ConfiguredStartPoseOnly`
   - Set `START_POSE_X_IN`, `START_POSE_Y_IN`, `START_POSE_THETA_DEG` to
     match your known position.
   - Convert `x/y` from metres to inches, and convert `θ` from the internal
     frame to VEX compass degrees (`0° = north`, clockwise positive).
   - Boot and note final PF pose from console.

2. Set `STARTUP_POSE_MODE = GPSXYPlusIMUHeading`
   - Boot and note final PF pose.
   - GPS (x, y) should be used; IMU heading used for θ.

3. Set `STARTUP_POSE_MODE = FullGPSInit`
   - Boot and note final PF pose.
   - GPS (x, y, θ) should all be used.

### Acceptance Criteria

After allowing **~5 seconds** for MCL convergence, all three modes should place
the robot at approximately the same field position:

- **All three poses agree within ±200 mm (x, y), ±10° (θ)** after stabilization.

### Pass/Fail

- **PASS**: All modes converge to same location ± 200 mm, ± 10°
- **FAIL**: Modes differ by > 300 mm or > 15° — indicates inconsistent frame
  conversions or startup logic

---

## Test 5: GPS Error Handling

**Objective**: Verify GPS error gates trust appropriately.

### Setup

1. Boot normally with GPS available.
2. During operation (while moving or stationary), observe GPS error in logs:
   ```
   [LOC] ... gps(Y)=(...,...,...,...,err=0.032) ...
   ```

### Execution

1. Let robot run for 10 seconds in normal operation (moving or idle).
2. Observe GPS error column.
3. (Optional) Move robot closer to/away from GPS strip perimeter to vary error.

### Acceptance Criteria

- **When GPS error < 0.05 m (good)**: GPS influence should be high, fused
  pose should match GPS closely.
- **When GPS error > 0.2 m (degraded)**: GPS influence should be reduced
  (stddev inflated).
- **When GPS error > 0.5 m (poor)**: GPS updates skipped; logs show `gps(N)`
  instead of `gps(Y)`.

### Pass/Fail

- **PASS**: GPS error properly gates updates; pose tracks GPS when good,
  degrades gracefully when poor
- **FAIL**: GPS used even when error is high, OR GPS ignored when error is low

---

## Test 6: GPS + Distance Sensor Gating

**Objective**: Verify GPS remains active and distance sensors can be toggled
cleanly while you validate the wall-offset geometry.

### Setup

1. Boot normally.
2. Set `MCL_DISABLE_DISTANCE_SENSORS_WHILE_DEBUGGING = true` in config.h.
3. Rebuild and run.

### Execution

1. Move robot and observe logs.
2. Confirm the startup log still reports the GPS sensor as active.
3. Rebuild once more with `MCL_DISABLE_DISTANCE_SENSORS_WHILE_DEBUGGING = false`
   and repeat.

### Acceptance Criteria

- **With distance sensors disabled**: PF should still have GPS available as the
  absolute-position sensor, and localization should remain usable.
- **With distance sensors enabled**: `PFDBG` output should include distance
  sensor lines. Invalid wall solutions should show `reading=INVALID` or
  `exp=INVALID` rather than corrupting the pose estimate.

### Pass/Fail

- **PASS**: GPS stays active in both runs, and distance sensors can be enabled
  or disabled without destabilizing localization.
- **FAIL**: GPS disappears unexpectedly, or distance sensor toggling causes
  unstable pose jumps / invalid debug output

---

## Debug Log Format Reference

```
[LOC] odom_delta=<float> imu=<rad> gps(<Y|N>)=(<x>,<y>,<h>,err=<e>) \
      odom=(<x>,<y>,<rad>) pf=(<x>,<y>,<rad>)
```

- `odom_delta`: Odometry displacement magnitude (metres) from latest tick
- `imu`: IMU heading in radians (internal convention)
- `gps(Y|N)`: GPS valid (Y) or invalid (N)
  - `<x>, <y>`: GPS position in metres (field frame)
  - `<h>`: GPS heading in radians (converted to internal)
  - `err`: GPS error in metres
- `odom`: Odometry pose (x, y in metres, heading in radians)
- `pf`: Particle filter fused pose (same units/convention)

---

## Troubleshooting

### Issue: PF Pose Not Tracking GPS

1. **Verify GPS convention conversion**: Check that
   `CONFIG::gpsHeadingDegToInternalRad()` is being used.
2. **Check GPS error**: If error > 0.5 m, updates are skipped (Phase 5).
3. **Verify offset transform**: Check MCL sensor offset conversions in config.h.

### Issue: Heading Jumps or Drifts

1. **IMU calibration**: Recalibrate IMU with robot stationary and level.
2. **IMU mounting**: Verify IMU is securely mounted and not rattling.
3. **Heading offset**: If using `GPSXYPlusIMUHeading` mode, IMU heading MUST be
   field-aligned at startup.

### Issue: Distance to Goal Increasing After Motion

1. Distance sensors likely have unit/axis errors (Phase 4).
2. Set `MCL_DISABLE_DISTANCE_SENSORS_WHILE_DEBUGGING = true` to exclude them.
3. Verify distance sensor offsets match robot geometry.

### Issue: Fused Pose Diverges from GPS & Odometry

1. Check for multiple prediction function calls per tick (Phase 3 violation).
2. Verify `CommandScheduler::run()` runs BEFORE `ParticleFilter::update()`.
3. Check that `consumePendingDisplacement()` is called exactly once per update.

---

## Sign-Off

After all tests pass, document:

- Date tested: _________________
- Tester name: _________________
- Robot ID: _________________
- GPS strip location tested: _________________
- Any anomalies observed: _________________
- Recommended next steps: _________________
