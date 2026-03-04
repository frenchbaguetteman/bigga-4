# Localization System — Design Rationale

## Overview

This document explains the reasoning behind each of the 7-phase refactoring decisions.

---

## Problem Statement

The original localization system had several intertwined issues:

1. **Ambiguous internal frames**: GPS convention (0°=N, CW+) mixed with math convention (0°=E, CCW+) without clear boundaries
2. **Heading misalignment at startup**: GPS heading combined with raw IMU without field-frame alignment
3. **Double-counting motion**: Both drivetrain odometry and particle filter could independently "discover" motion from raw sensors
4. **Unit confusion**: Sensor offsets in inches, partially converted to metres, with unclear robot-frame vs world-frame semantics
5. **Weak sensor validation**: Non-finite, failed, or implausibly noisy readings accepted uncritically
6. **GPS wander**: No error-adaptive weighting; GPS jitter influenced pose equally at all error levels
7. **No test infrastructure**: No structured regression tests or debug logs for diagnostics

---

## Solution Architecture

### 1. Canonical Internal Frame (Phase 1)

**Decision**: Adopt single internal convention: **metres, radians, +X east/forward (θ=0), +Y north/left, CCW positive**

**Reasoning**:
- Eliminates convention conflicts within codebase
- Matches standard robotics math convention (easier to verify equations)
- GPS conversion happens ONLY at API boundary (cleaner separation)
- All sensor models, paths, and control laws use same frame

**Tradeoff**: Slight performance overhead from heading conversions, but gain is correctness and maintainability.

---

### 2. Startup Heading Alignment (Phase 2)

**Decision**: Convert GPS heading to internal convention BEFORE combining with GPS x/y

**Reasoning**:

Original code did (pseudocode):
```
imu_heading = get_imu()  // compass degrees
gps_heading = get_gps_heading()  // compass degrees  
gps_xy = get_gps_position()  // metres, arbitrary origin

start_pose.x = gps_xy.x
start_pose.y = gps_xy.y
start_pose.theta = imu_heading  // or gps_heading
```

Problem: If IMU heading is in compass degrees and we store it in internal radians without conversion, heading is wrong.

Fixed code:
```
imu_heading_rad = gpsHeadingDegToInternalRad(get_imu_compass_deg())
gps_heading_rad = gpsHeadingDegToInternalRad(get_gps_compass_deg())
// ... then use internal radians consistently
```

**Benefit**: No more heading convention mismatches at startup.

---

### 3. Odom-MCL Delta Interface (Phase 3)

**Decision**: Single authoritative motion delta source via `consumePendingDisplacement()`, called once per tick by MCL

**Reasoning**:

Original code had two paths reading motion:
```
// Path 1: ParticleFilter calls predictionFn()
getDisplacement() {
    delta = m_pending;
    m_pending = 0;
    return delta;
}

// But if drivetrain or other code also called getDisplacement(),
// the PF would miss that motion!
```

New code:
```
// Only ParticleFilter calls this:
consumePendingDisplacement() {
    delta = m_pending;
    m_pending = 0;
    return delta;
}

// Drivetrain::periodic() only writes to m_pending
// Nothing else reads it
```

**Benefit**: Motion counted exactly once, no double-counting or missed motion.

**Tradeoff**: Slight API change (new method name makes intent clear).

---

### 4. Units & Offset Axes (Phase 4)

**Decision**: Lock down offset convention and verify all offsets are metres in internal frame

**Reasoning**:

Original code had mix:
- Some offsets in inches (MCL_LEFT_OFFSET_X_IN)
- Conversions to metres sometimes correct, sometimes wrong
- Comments ambiguous about robot-frame vs world-frame

Example bug:
```
// Original (buggy):
// Comment says: "+offsetX = robot-right, +offsetY = robot-forward"
// But conversion was:
// m.x = y_in * IN_TO_M  // What? This doesn't match comment!
```

New code makes it explicit:
```
// Input: inches in robot frame (+X_robot=right, +Y_robot=forward)
MCL_LEFT_OFFSET_X_IN = -4.625;  // 4.625" to the left
MCL_LEFT_OFFSET_Y_IN = 0.25;    // 0.25" forward

// Transform to internal frame: +X_internal=forward, +Y_internal=left
offset_x_m = MCL_LEFT_OFFSET_Y_IN * IN_TO_M;  // forward
offset_y_m = -MCL_LEFT_OFFSET_X_IN * IN_TO_M; // left (neg of right)

// All offsets thereafter in metres, internal frame
```

**Benefit**: Removes ambiguity, enables verification against CAD.

---

### 5. Sensor Validation (Phase 5)

**Decision**: Reject invalid readings at sensor model level, skip them in particle filter with logging

**Reasoning**:

Garbage in → garbage out. Examples of bad readings:
- GPS returns NaN due to radio interference
- Distance sensor returns 9999 mm (no object) but treated as 9.999 m
- IMU returns invalid heading while calibrating

Original code: accept all readings, hope downstream filtering helps.

New code:
```
// GPS:
if (!isFinite(x) || !isFinite(y) || fabs(x) > ABSURD_LIMIT) {
    robotCenter = nullopt;
    return;  // Skip this update
}

// ParticleFilter:
if (!likelihood || !isFinite(likelihood.value())) {
    continue;  // Don't multiply weight by this sensor
}
// Log: [PF] Skipped sensors (invalid readings): [0,2]
```

**Benefit**: Garbage readings can't corrupt state. Logging helps diagnostics.

---

### 6. GPS Error Weighting (Phase 6)

**Decision**: Inflate GPS measurement stddev based on `gps_get_error()` signal

**Reasoning**:

GPS error (RMS, from `gps_get_error()`) indicates signal quality:
- Error < 0.05 m: GPS is confident, trust it fully
- Error 0.05–0.2 m: GPS is uncertain, reduce influence by inflating stddev
- Error > 0.5 m: GPS is garbage, skip update entirely (Phase 5)

This is called **adaptive covariance** in sensor fusion literature. Mathematical basis:
```
stddev_used = base_stddev * sqrt(1 + scale * (error - error_good)²)
```

This means:
- Good GPS (low error) → tight Gaussian → high likelihood → strong influence
- Bad GPS (high error) → wide Gaussian → low likelihood → weak influence
- Terrible GPS (very high error) → skip entirely (Phase 5)

**Benefit**: Graceful degradation. No abrupt jumps when GPS jitters.

---

### 7. Regression Tests & Logging (Phase 7)

**Decision**: Structured test suite + comprehensive debug logging

**Reasoning**:

Without tests, regressions go unnoticed. Without logging, bugs are hard to diagnose.

**Tests cover**:
1. Stationary wander (should be < 100 mm)
2. Rotation in place (heading should change, x/y constant)
3. Forward motion (x should increase, y constant)
4. All startup modes (should converge to same pose)
5. GPS error handling (should gate appropriately)
6. Distance sensors (should be skippable)

**Logging format**:
```
[LOC] odom_delta=0.0234 imu=0.567 gps(Y)=(1.234,2.345,0.089,err=0.032)
      odom=(1.234,2.345,0.090) pf=(1.234,2.345,0.090)
```

Each field traceable to a specific phase:
- `odom_delta` → Phase 3 (MCL interface)
- `imu` → Phase 2 (heading conversion)
- `gps(Y|N)` → Phase 5 (validation)
- `err` → Phase 6 (error weighting)

**Benefit**: Humans can diagnose issues by reading logs. Automated tests catch regressions.

---

## Cross-Phase Dependencies

```
Phase 1 ──→ Frame Convention
    ↓
Phase 2 ──→ Startup Heading (uses Phase 1 conversions)
    ↓
Phase 3 ──→ Odom-MCL Interface (Phase 2 ensures headed is right)
    ↓
Phase 4 ──→ Units & Axes (all offsets must be in Phase 1 frame)
    ↓
Phase 5 ──→ Sensor Validation (validates Phase 4 offsets)
    ↓
Phase 6 ──→ Error Weighting (adapts to Phase 5 validation results)
    ↓
Phase 7 ──→ Regression Tests (verify all 6 phases work together)
```

**Key insight**: Each phase depends on correct implementation of prior phases.

---

## Backward Compatibility

To minimize breaking changes:

1. **Legacy aliases** provided for old function names:
   - `compassDegToMathRad()` → calls `gpsHeadingDegToInternalRad()`
   - `mathRadToCompassDeg()` → calls `internalRadToGpsHeadingDeg()`
   - `getDisplacement()` → calls `consumePendingDisplacement()`

2. **Distance sensors** remain in sensor list but can be disabled via flag

3. **API functions** added without removing old ones

**Implication**: Old autonomous commands should continue working while new code uses new conventions.

---

## Performance Considerations

- **Heading conversions**: ~5 trig operations per tick per sensor. Negligible (~100 μs)
- **Skipped sensor logging**: ~20 μs per sensor per update. Only at reduced rate
- **GPS error weighting**: ~2 trig operations per MCL update. Negligible
- **Overall**: No measurable impact on 10 ms scheduler tick

---

## Testing Strategy

1. **Unit tests**: Each phase is independently testable
   - Phase 1: Conversion functions (by inspection + manual calculation)
   - Phase 2: Heading alignment (verify start pose is finite)
   - Phase 3: MCL delta (verify called once per tick)
   - Phase 4: Offset transforms (compare to CAD)
   - Phase 5: Validation (feed bad data, verify rejected)
   - Phase 6: Error weighting (vary GPS error, verify stddev adapts)
   - Phase 7: Regression suite (6 structured robot tests)

2. **Integration tests**: All phases together
   - Boot → init → idle 10 sec → rotate 90° → drive straight → check all poses consistent

3. **Field tests**: Real competition environment
   - Verify no regressions on known autonomous paths
   - Monitor convergence times during teleoperated match

---

## Known Limitations / Future Work

1. **Distance sensors**: Currently disabled pending unit/axis verification. Re-enable after Phase 4 is fully validated.

2. **GPS coordinate frame**: Assumes GPS strip is aligned with field axes. If strip is rotated, adjust `GPS_FIELD_ROTATION_DEG`.

3. **Heading only at startup**: IMU heading is used for initial θ. After that, heading stays constant unless explicitly reset. Consider IMU drift over 2+ minute match.

4. **No loop closure**: MCL only fuses forward motion + sensors. Long closed-loop paths may drift.

5. **Particle filter hyperparameters**: NUM_PARTICLES, noise models, resampling threshold hardcoded. Consider tuning for your robot.

---

## References

- **Robotics math convention**: θ=0 along +X, CCW positive (standard in most textbooks)
- **Adaptive covariance**: Kalman filtering literature (Welch & Bishop, "Introduction to Kalman Filtering")
- **Particle filtering**: Thrun, Burgard, Fox, "Probabilistic Robotics"
- **VEX GPS API**: PROS documentation (gps.h / gps.hpp)

