# Implementation Checklist & Quick Start

## Pre-Deployment Verification

- [ ] Code compiles without errors: `pros build`
- [ ] All 7 phases are implemented (see summary below)
- [ ] Debug flags are appropriate for your use case:
  - [ ] `ODOM_DEBUG_ENABLE = true` (to see logs)
  - [ ] `MCL_DISABLE_DISTANCE_SENSORS_WHILE_DEBUGGING = true` (until verified)
  - [ ] `STARTUP_POSE_MODE = FullGPSInit` (or your preferred mode)

---

## Phase Completion Checklist

### Phase 1: Canonical Internal Frame ✓

**Files modified**:
- [x] `include/config.h` — frame documentation, conversion functions

**Key functions added**:
- [x] `gpsHeadingDegToInternalRad(compassDeg)` — VEX GPS → internal rad
- [x] `internalRadToGpsHeadingDeg(internalRad)` — internal rad → VEX GPS
- [x] Legacy aliases: `compassDegToMathRad()`, `mathRadToCompassDeg()`

**Verification**: Open `include/config.h` line ~20–60, verify documentation and functions exist.

---

### Phase 2: Startup Heading Initialization ✓

**Files modified**:
- [x] `src/main.cpp` — updated `acquireInitialPose()`
- [x] `src/subsystems/drivetrain.cpp` — updated heading conversions
- [x] `include/localization/gps.h` — updated GPS model heading conversion

**Key changes**:
- [x] `acquireInitialPose()` uses `gpsHeadingDegToInternalRad()`
- [x] Drivetrain heading conversion uses new function
- [x] GPS sensor model converts heading in `update()`

**Verification**: 
```bash
grep -n "gpsHeadingDegToInternalRad" src/main.cpp src/subsystems/drivetrain.cpp include/localization/gps.h
# Should see 3+ matches
```

---

### Phase 3: Redesign Odom-to-MCL Interface ✓

**Files modified**:
- [x] `include/subsystems/drivetrain.h` — new `consumePendingDisplacement()`
- [x] `src/subsystems/drivetrain.cpp` — implementation
- [x] `src/main.cpp` — prediction lambda uses new function
- [x] Comments explain scheduler → PF ordering

**Key changes**:
- [x] `consumePendingDisplacement()` added to drivetrain
- [x] Prediction lambda updated: `drivetrain->consumePendingDisplacement()`
- [x] `update_loop()` has ordering comments

**Verification**:
```bash
grep -n "consumePendingDisplacement" src/main.cpp src/subsystems/drivetrain.cpp include/subsystems/drivetrain.h
# Should see 3+ matches
```

---

### Phase 4: Fix Units and Offset Axes ✓

**Files modified**:
- [x] `include/config.h` — clarified offset convention, all in metres
- [x] `include/localization/gps.h` — updated documentation

**Key changes**:
- [x] Offset transform documented (robot frame → internal frame)
- [x] GPS offset in metres (internal frame)
- [x] All distance sensor offsets in metres (internal frame)
- [x] Comments explain conversion: `offset_x = y_in * IN_TO_M`, `offset_y = -x_in * IN_TO_M`

**Verification**:
```bash
grep -n "DIST_.*_OFFSET_M\|GPS_OFFSET_M" include/config.h
# Should see all offsets in metres (constexpr float ... * IN_TO_M)
```

---

### Phase 5: Harden Sensor Validation ✓

**Files modified**:
- [x] `include/localization/gps.h` — enhanced `update()` with validation
- [x] `include/localization/distance.h` — enhanced `update()` with validation
- [x] `include/localization/particleFilter.h` — skip invalid readings with logging

**Key changes**:
- [x] GPS: reject non-finite, absurd magnitude, high error
- [x] GPS: store `m_lastError` for Phase 6
- [x] Distance: reject out-of-range, non-finite, invalid confidence
- [x] ParticleFilter: skip invalid sensor readings, log which ones skipped

**Verification**:
```bash
grep -n "isFinite\|absurd\|gpsErr\|m_lastError\|Skipped sensors" include/localization/gps.h include/localization/distance.h include/localization/particleFilter.h
# Should see validation logic in each
```

---

### Phase 6: Reduce Stationary Wander ✓

**Files modified**:
- [x] `include/config.h` — GPS error adaptive weighting constants
- [x] `include/localization/gps.h` — adaptive stddev in `p()`
- [x] `include/localization/distance.h` — optional disable flag

**Key changes**:
- [x] GPS constants: `GPS_STDDEV_BASE_M`, `GPS_ERROR_GOOD_M`, `GPS_ERROR_SCALE_MULTIPLIER`, etc.
- [x] GPS `p()` method inflates stddev when error > `GPS_ERROR_GOOD_M`
- [x] Distance sensors skip if `MCL_DISABLE_DISTANCE_SENSORS_WHILE_DEBUGGING`

**Verification**:
```bash
grep -n "GPS_ERROR\|GPS_STDDEV\|MCL_DISABLE_DISTANCE" include/config.h include/localization/gps.h
# Should see adaptive weighting logic
```

---

### Phase 7: Regression Tests & Debug Logs ✓

**Files created**:
- [x] `docs/LOCALIZATION_REGRESSION_TESTS.md` — 6 structured tests
- [x] `docs/LOCALIZATION_REFACTORING_SUMMARY.md` — high-level summary
- [x] `docs/LOCALIZATION_DESIGN_RATIONALE.md` — detailed reasoning

**Files modified**:
- [x] `src/main.cpp` — enhanced debug loop, startup diagnostics
- [x] `include/subsystems/drivetrain.h` — added `getPendingDisplacementDebug()`

**Key changes**:
- [x] Debug log format: `[LOC] odom_delta=... imu=... gps(...)=... odom=... pf=...`
- [x] Startup logs include frame convention, pose mode, sensor count
- [x] ParticleFilter logs skipped sensors: `[PF] Skipped sensors (invalid readings): [0,2]`

**Verification**:
```bash
grep -n "\[LOC\]\|\[LOC_CONFIG\]\|\[PF\]" src/main.cpp include/localization/particleFilter.h
# Should see multiple debug printf statements
```

---

## Configuration Tuning

Before your first test run, review these settings in `include/config.h`:

1. **Startup Pose Mode** (line ~200–205):
   ```cpp
   constexpr StartupPoseMode STARTUP_POSE_MODE =
       StartupPoseMode::FullGPSInit;  // or GPSXYPlusIMUHeading, ConfiguredStartPoseOnly
   ```
   Choose based on your GPS reliability.

2. **GPS Error Thresholds** (line ~230–240):
   ```cpp
   constexpr float GPS_ERROR_THRESHOLD_M = 0.5f;   // reject if error > this
   constexpr float GPS_ERROR_GOOD_M = 0.05f;       // full trust if error < this
   ```
   Tune based on observed GPS performance.

3. **Distance Sensors** (line ~248):
   ```cpp
   constexpr bool MCL_DISABLE_DISTANCE_SENSORS_WHILE_DEBUGGING = true;  // set false when verified
   ```
   Keep `true` until you've verified offset transforms match robot.

4. **Debug Output** (line ~206):
   ```cpp
   constexpr bool ODOM_DEBUG_ENABLE = true;  // for initial testing
   constexpr uint32_t ODOM_DEBUG_LOG_EVERY_MS = 200;  // every 200 ms
   ```
   Set to `false` in production to reduce console spam.

---

## First Test Run

1. **Build and upload**:
   ```bash
   cd /Users/main/Documents/bigga\ 4/bigga-4
   pros build
   pros upload  # or use V5 brain GUI
   ```

2. **Connect console** (USB or wireless, depending on your setup)

3. **Power on robot** and watch for startup logs:
   ```
   [LOC_CONFIG] ═════════════════════════════════════════════════════════
   [LOC_CONFIG] Canonical Internal Frame Convention:
   [LOC_CONFIG]   Position: metres
   [LOC_CONFIG]   Heading: radians (0 = +X east/forward, CCW+)
   ...
   [INIT] startup pose: (x.xxx, y.xxx, z.xxx)
   ```

4. **Run Test 1 from regression suite** (Boot Stationary):
   - Place robot at known position
   - Power on
   - Let sit for 10 seconds
   - Watch console for logs
   - Verify pose wander < 100 mm, < 10°

5. **If test passes**: Proceed to other tests
   If test fails: Check troubleshooting guide in `LOCALIZATION_REGRESSION_TESTS.md`

---

## Common Issues & Fixes

### Issue: Code doesn't compile

**Fix**:
```bash
# Clean and rebuild
pros clean
pros build

# If still failing, check for syntax errors
grep -n "error:" /tmp/pros_build.log
```

### Issue: Debug logs not appearing

**Fix**:
1. Verify `ODOM_DEBUG_ENABLE = true` in config.h
2. Check console connection (USB might be disconnected)
3. Verify PROS is running: `pros serial ports`
4. Try opening PROS terminal: `pros cli`

### Issue: Pose wanders > 100 mm when stationary

**Fix**:
1. Check GPS error: is `gps(Y)` showing high `err=` value?
2. If `gps(N)`: GPS invalid, uses fallback. Try moving robot closer to GPS strip.
3. If error is high: decrease `GPS_ERROR_GOOD_M` threshold or increase `GPS_ERROR_SCALE_MULTIPLIER`
4. If problem persists: Check IMU calibration (should be done with robot stationary and level)

### Issue: Heading jumps during rotation

**Fix**:
1. Verify IMU is not moving/vibrating during startup calibration
2. Check heading offset: `MCL_GPS_HEADING_OFFSET_DEG = 180.0` (should match GPS module orientation)
3. Run Test 2 (Rotate in Place) to isolate IMU vs GPS heading issues

### Issue: Distance sensors still affecting pose when disabled

**Fix**:
1. Verify `MCL_DISABLE_DISTANCE_SENSORS_WHILE_DEBUGGING = true` in config.h
2. Rebuild: `pros build && pros upload`
3. Check logs: should NOT see distance sensor skipped messages

---

## Support & Documentation

- **Quick reference**: See `LOCALIZATION_REFACTORING_SUMMARY.md`
- **Design rationale**: See `LOCALIZATION_DESIGN_RATIONALE.md`
- **Regression tests**: See `LOCALIZATION_REGRESSION_TESTS.md`
- **Debug log format**: See `LOCALIZATION_DESIGN_RATIONALE.md` → Testing Strategy

---

## Sign-Off / Completion

Once all tests pass:

- [ ] Run through all 6 regression tests (record results)
- [ ] Verify no regressions in known autonomous paths
- [ ] Verify teleoperated driving is smooth (no jerky MCL updates)
- [ ] Document any tuning changes made to config.h
- [ ] Commit changes to version control with detailed commit message

### Commit Message Template

```
Phase 1-7 Localization System Refactoring

- Phase 1: Establish canonical internal frame (m, rad, +X fwd, +Y left, CCW+)
- Phase 2: Fix startup heading conversion (VEX GPS → internal)
- Phase 3: Redesign odom-MCL interface (consumePendingDisplacement)
- Phase 4: Fix sensor offset units & axes (all metres, internal frame)
- Phase 5: Harden sensor validation (reject non-finite, failed, high-error readings)
- Phase 6: Adaptive GPS weighting (inflate stddev based on gps_get_error)
- Phase 7: Add regression tests & debug logging

Fixes issues with:
- Heading misalignment at startup
- Double-counting motion delta
- GPS jitter causing wander
- Unit/axis ambiguity in offset transforms
- Invalid sensor readings corrupting filter
- Lack of diagnostics

All regression tests pass. See docs/LOCALIZATION_* for details.
```

