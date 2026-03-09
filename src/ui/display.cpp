#include "ui/display.h"

#include "config.h"
#include "ui/theme.h"
#include "utils/localization_math.h"
#include "pros/screen.hpp"

#include <array>
#include <cmath>
#include <cstdio>

namespace DisplayUI {

namespace {

static constexpr UITheme::Rect FIELD_PANEL = UITheme::makeRect(12, UITheme::kContentY + 36, 228, 166);
static constexpr UITheme::Rect TELEMETRY_PANEL = UITheme::makeRect(248, UITheme::kContentY + 36, 220, 166);

static constexpr int MAP_PX = 140;
static constexpr float FIELD_M = 2.0f * CONFIG::FIELD_HALF_SIZE_in * CONFIG::IN_TO_M;
static constexpr float SCALE = MAP_PX / FIELD_M;
static constexpr int MAP_X0 = FIELD_PANEL.x0 + (UITheme::width(FIELD_PANEL) - MAP_PX) / 2;
static constexpr int MAP_Y0 = FIELD_PANEL.y0 + 18;
static constexpr int CX = MAP_X0 + MAP_PX / 2;
static constexpr int CY = MAP_Y0 + MAP_PX / 2;

static constexpr uint32_t COL_FIELD = 0x00141D28;
static constexpr uint32_t COL_GRID = 0x00253444;
static constexpr uint32_t COL_FIELD_BORDER = UITheme::kBorderStrong;
static constexpr uint32_t COL_OBSTACLE_FILL = 0x00343D49;
static constexpr uint32_t COL_OBSTACLE_BORDER = 0x008798A8;
static constexpr uint32_t COL_TRAIL_HOT = UITheme::kAmber;
static constexpr uint32_t COL_TRAIL_COOL = UITheme::kBlue;
static constexpr uint32_t COL_SIGHTLINE = 0x004A657D;
static constexpr uint32_t COL_SENSOR_RAY = UITheme::kCyan;
static constexpr uint32_t COL_SENSOR_HIT = UITheme::kWhite;
static constexpr uint32_t COL_IMU_INIT = 0x00D2DCE7;
static constexpr uint32_t COL_PARTICLE = 0x00557799;  // subtle blue-grey

static constexpr int TRAIL_MAX = 300;
struct TrailPoint { int px; int py; };
static std::array<TrailPoint, TRAIL_MAX> trail{};
static int trailCount = 0;
static int trailHead = 0;

struct SourceSpec {
    const char* label = "";
    Eigen::Vector3f pose = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    bool valid = false;
    uint32_t accent = UITheme::kBorderStrong;
    const char* statusLine = "";
};

struct ScreenPoint {
    float x = 0.0f;
    float y = 0.0f;
};

static ScreenPoint lerpScreenPoint(const ScreenPoint& a,
                                   const ScreenPoint& b,
                                   float t) {
    return ScreenPoint{
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
    };
}

static int toScreenX(float mX) { return CX + static_cast<int>(mX * SCALE); }
static int toScreenY(float mY) { return CY - static_cast<int>(mY * SCALE); }
static ScreenPoint toScreenPoint(const Eigen::Vector2f& point) {
    return ScreenPoint{
        static_cast<float>(CX) + point.x() * SCALE,
        static_cast<float>(CY) - point.y() * SCALE,
    };
}
static int clampMapX(int x) { return std::max(MAP_X0, std::min(MAP_X0 + MAP_PX - 1, x)); }
static int clampMapY(int y) { return std::max(MAP_Y0, std::min(MAP_Y0 + MAP_PX - 1, y)); }

static float headingToCompassDeg(float headingRad) { return LocMath::headingToCompassDeg(headingRad); }
static float headingDeltaCompassDeg(float deltaRad) { return LocMath::headingDeltaCompassDeg(deltaRad); }
static float mToIn(float m) { return LocMath::mToIn(m); }

static uint32_t blendTrailColor(float t) {
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    const int r0 = (COL_TRAIL_HOT >> 16) & 0xFF;
    const int g0 = (COL_TRAIL_HOT >> 8) & 0xFF;
    const int b0 = COL_TRAIL_HOT & 0xFF;
    const int r1 = (COL_TRAIL_COOL >> 16) & 0xFF;
    const int g1 = (COL_TRAIL_COOL >> 8) & 0xFF;
    const int b1 = COL_TRAIL_COOL & 0xFF;
    const int r = r0 + static_cast<int>((r1 - r0) * t);
    const int g = g0 + static_cast<int>((g1 - g0) * t);
    const int b = b0 + static_cast<int>((b1 - b0) * t);
    return (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8) |
           static_cast<uint32_t>(b);
}

static SourceSpec sourceSpec(const BrainScreen::RuntimeViewModel& vm, LocalizationView view) {
    auto finitePose = [](const Eigen::Vector3f& pose) {
        return std::isfinite(pose.x()) &&
               std::isfinite(pose.y()) &&
               std::isfinite(pose.z());
    };

    switch (view) {
        case LocalizationView::PureOdom:
            return {"PURE ODOM", vm.pureOdomPose, finitePose(vm.pureOdomPose),
                    UITheme::kAmber, "Base drivetrain odometry"};
        case LocalizationView::PureGps:
            return {"PURE GPS", vm.gpsPose, vm.gpsPoseValid && finitePose(vm.gpsPose), UITheme::kBlue,
                    vm.gpsPoseValid ? "Absolute GPS pose" : "No valid GPS reading"};
        case LocalizationView::PureMcl:
            return {"PURE MCL", vm.pureMclPose, finitePose(vm.pureMclPose),
                    UITheme::kBlue, "Particle-filter estimate"};
        case LocalizationView::Combined:
        default:
            return {"COMBINED", vm.combinedPose, finitePose(vm.combinedPose),
                    UITheme::kGreen, "Controller pose used by paths"};
    }
}

static bool isFinitePose(const Eigen::Vector3f& pose) {
    return std::isfinite(pose.x()) &&
           std::isfinite(pose.y()) &&
           std::isfinite(pose.z());
}

static Eigen::Vector2f rotateLocalToField(const Eigen::Vector2f& local, float heading) {
    const float cosT = std::cos(heading);
    const float sinT = std::sin(heading);
    return Eigen::Vector2f(
        local.x() * cosT - local.y() * sinT,
        local.x() * sinT + local.y() * cosT);
}

static void drawFieldObstacle(const CONFIG::FieldObstacle& obstacle) {
    const Eigen::Vector2f center(obstacle.centerX, obstacle.centerY);
    const std::array<Eigen::Vector2f, 4> localCorners{{
        { obstacle.halfSizeX,  obstacle.halfSizeY},
        { obstacle.halfSizeX, -obstacle.halfSizeY},
        {-obstacle.halfSizeX, -obstacle.halfSizeY},
        {-obstacle.halfSizeX,  obstacle.halfSizeY},
    }};

    std::array<ScreenPoint, 4> corners{};
    for (size_t i = 0; i < localCorners.size(); ++i) {
        const Eigen::Vector2f world =
            center + rotateLocalToField(localCorners[i], obstacle.headingRad);
        corners[i] = toScreenPoint(world);
    }

    const int fillSteps = std::max(
        1,
        static_cast<int>(std::ceil(std::max(
            obstacle.halfSizeX,
            obstacle.halfSizeY) * SCALE * 2.0f)));
    for (int step = 0; step <= fillSteps; ++step) {
        const float t = static_cast<float>(step) / static_cast<float>(fillSteps);
        const ScreenPoint start = lerpScreenPoint(corners[0], corners[1], t);
        const ScreenPoint end = lerpScreenPoint(corners[3], corners[2], t);
        UITheme::drawLine(
            clampMapX(static_cast<int>(std::lround(start.x))),
            clampMapY(static_cast<int>(std::lround(start.y))),
            clampMapX(static_cast<int>(std::lround(end.x))),
            clampMapY(static_cast<int>(std::lround(end.y))),
            COL_OBSTACLE_FILL);
    }

    for (size_t i = 0; i < corners.size(); ++i) {
        const ScreenPoint& a = corners[i];
        const ScreenPoint& b = corners[(i + 1) % corners.size()];
        UITheme::drawLine(
            clampMapX(static_cast<int>(std::lround(a.x))),
            clampMapY(static_cast<int>(std::lround(a.y))),
            clampMapX(static_cast<int>(std::lround(b.x))),
            clampMapY(static_cast<int>(std::lround(b.y))),
            COL_OBSTACLE_BORDER);
    }
}

static void drawFieldObstacles() {
    for (const auto& obstacle : CONFIG::MCL_FIELD_OBSTACLES) {
        drawFieldObstacle(obstacle);
    }
}

static void drawImuInitReference() {
    const int baseX = MAP_X0 + 10;
    const int baseY = MAP_Y0 + 10;
    const float heading = CONFIG::DEFAULT_IMU_INIT_ANGLE.convert(radian);
    const int tipX = clampMapX(baseX + static_cast<int>(std::lround(std::cos(heading) * 10.0f)));
    const int tipY = clampMapY(baseY - static_cast<int>(std::lround(std::sin(heading) * 10.0f)));

    UITheme::drawLine(baseX, baseY, tipX, tipY, COL_IMU_INIT);
    pros::screen::set_pen(COL_IMU_INIT);
    pros::screen::fill_circle(baseX, baseY, 2);
}

static void drawParticleCloud(const std::vector<Eigen::Vector2f>& particles) {
    if (particles.empty()) return;
    pros::screen::set_pen(COL_PARTICLE);
    for (const auto& p : particles) {
        const int px = clampMapX(toScreenX(p.x()));
        const int py = clampMapY(toScreenY(p.y()));
        pros::screen::draw_pixel(px, py);
    }
}

static void drawFieldBase(const SourceSpec& spec) {
    UITheme::drawPanel(FIELD_PANEL, UITheme::kPanelAlt, UITheme::kBorderStrong, spec.accent);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, FIELD_PANEL.x0 + 12, FIELD_PANEL.y0 + 8,
                          UITheme::kTextMuted, UITheme::kPanelAlt, "LOCALIZATION MAP");
    UITheme::drawChip(UITheme::makeRect(FIELD_PANEL.x1 - 112, FIELD_PANEL.y0 + 8, 100, 18),
                      spec.label, UITheme::kPanelMuted, spec.accent, UITheme::kText);

    const UITheme::Rect mapRect = UITheme::Rect{MAP_X0, MAP_Y0, MAP_X0 + MAP_PX - 1, MAP_Y0 + MAP_PX - 1};
    UITheme::fillRect(mapRect, COL_FIELD);

    for (int i = 0; i <= 6; ++i) {
        const int px = MAP_X0 + i * MAP_PX / 6;
        const int py = MAP_Y0 + i * MAP_PX / 6;
        UITheme::drawLine(px, MAP_Y0, px, MAP_Y0 + MAP_PX - 1, COL_GRID);
        UITheme::drawLine(MAP_X0, py, MAP_X0 + MAP_PX - 1, py, COL_GRID);
    }

    const int tilePx = MAP_PX / 6;
    UITheme::fillRect(UITheme::Rect{MAP_X0, MAP_Y0, MAP_X0 + tilePx, MAP_Y0 + tilePx}, UITheme::kRedDeep);
    UITheme::fillRect(UITheme::Rect{MAP_X0, MAP_Y0 + MAP_PX - tilePx - 1,
                                    MAP_X0 + tilePx, MAP_Y0 + MAP_PX - 1}, UITheme::kRedDeep);
    UITheme::fillRect(UITheme::Rect{MAP_X0 + MAP_PX - tilePx - 1, MAP_Y0,
                                    MAP_X0 + MAP_PX - 1, MAP_Y0 + tilePx}, UITheme::kBlueDeep);
    UITheme::fillRect(UITheme::Rect{MAP_X0 + MAP_PX - tilePx - 1, MAP_Y0 + MAP_PX - tilePx - 1,
                                    MAP_X0 + MAP_PX - 1, MAP_Y0 + MAP_PX - 1}, UITheme::kBlueDeep);
    drawFieldObstacles();
    drawImuInitReference();
    UITheme::outlineRect(mapRect, COL_FIELD_BORDER);
}

static void drawTrail() {
    if (trailCount == 0) return;
    for (int i = 0; i < trailCount; ++i) {
        const int idx = (trailCount < TRAIL_MAX) ? i : (trailHead + i) % TRAIL_MAX;
        const float blend = static_cast<float>(i + 1) / static_cast<float>(trailCount);
        pros::screen::set_pen(blendTrailColor(blend));
        pros::screen::draw_pixel(trail[idx].px, trail[idx].py);
    }
}

static void drawPoseCard(const UITheme::Rect& r, const SourceSpec& spec) {
    UITheme::drawPanel(r, UITheme::kPanelMuted, spec.accent, 0, false);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 8,
                          UITheme::kTextMuted, UITheme::kPanelMuted, "%s", spec.label);

    if (!spec.valid) {
        UITheme::printTextfOn(pros::E_TEXT_MEDIUM, r.x0 + 8, r.y0 + 24,
                              UITheme::kText, UITheme::kPanelMuted, "No valid pose");
        UITheme::printTextfOn(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 40,
                              UITheme::kTextSoft, UITheme::kPanelMuted, "%s", spec.statusLine);
        return;
    }

    UITheme::printTextfOn(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 24,
                          UITheme::kText, UITheme::kPanelMuted, "X %+.1f in   Y %+.1f in",
                          mToIn(spec.pose.x()), mToIn(spec.pose.y()));
    UITheme::printTextfOn(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 38,
                          UITheme::kText, UITheme::kPanelMuted, "H %.1f deg",
                          headingToCompassDeg(spec.pose.z()));
    UITheme::printTextfOn(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 52,
                          UITheme::kTextSoft, UITheme::kPanelMuted, "%s", spec.statusLine);
}

static void drawInfoTile(const UITheme::Rect& r,
                         const char* label,
                         const char* value,
                         uint32_t accent) {
    UITheme::drawPanel(r, UITheme::kPanelMuted, accent, 0, false);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 6,
                          UITheme::kTextMuted, UITheme::kPanelMuted, "%s", label);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 20,
                          UITheme::kText, UITheme::kPanelMuted, "%s", value);
}

static void drawSensorTile(const UITheme::Rect& r,
                           const BrainScreen::RuntimeViewModel::DistanceSensorViewModel& sensor,
                           uint32_t accent) {
    UITheme::drawPanel(r, UITheme::kPanelMuted, accent, 0, false);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 6,
                          UITheme::kTextMuted, UITheme::kPanelMuted, "%s", sensor.label.c_str());
    if (!sensor.valid) {
        UITheme::printTextfOn(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 19,
                              UITheme::kText, UITheme::kPanelMuted, "--");
        UITheme::printTextfOn(pros::E_TEXT_SMALL, r.x0 + 52, r.y0 + 19,
                              UITheme::kTextSoft, UITheme::kPanelMuted, "inv");
        return;
    }

    UITheme::printTextfOn(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 19,
                          UITheme::kText, UITheme::kPanelMuted, "%.1f in", mToIn(sensor.rangeM));
    UITheme::printTextfOn(pros::E_TEXT_SMALL, r.x0 + 56, r.y0 + 19,
                          UITheme::kTextSoft, UITheme::kPanelMuted, "c%d", sensor.confidence);
}

static void drawRobotFootprint(const Eigen::Vector3f& pose, uint32_t accent) {
    const float halfLength = CONFIG::ROBOT_LENGTH.convert(meter) * 0.5f;
    const float halfWidth = CONFIG::ROBOT_WIDTH.convert(meter) * 0.5f;

    const std::array<Eigen::Vector2f, 4> localCorners{{
        { halfLength,  halfWidth},
        { halfLength, -halfWidth},
        {-halfLength, -halfWidth},
        {-halfLength,  halfWidth},
    }};

    std::array<ScreenPoint, 4> corners{};
    for (size_t i = 0; i < localCorners.size(); ++i) {
        const Eigen::Vector2f world =
            pose.head<2>() + rotateLocalToField(localCorners[i], pose.z());
        corners[i] = toScreenPoint(world);
    }

    for (size_t i = 0; i < corners.size(); ++i) {
        const ScreenPoint& a = corners[i];
        const ScreenPoint& b = corners[(i + 1) % corners.size()];
        UITheme::drawLine(
            clampMapX(static_cast<int>(std::lround(a.x))),
            clampMapY(static_cast<int>(std::lround(a.y))),
            clampMapX(static_cast<int>(std::lround(b.x))),
            clampMapY(static_cast<int>(std::lround(b.y))),
            accent);
    }

    const Eigen::Vector2f frontCenterWorld =
        pose.head<2>() + rotateLocalToField(Eigen::Vector2f(halfLength, 0.0f), pose.z());
    const ScreenPoint center = toScreenPoint(pose.head<2>());
    const ScreenPoint frontCenter = toScreenPoint(frontCenterWorld);
    UITheme::drawLine(
        clampMapX(static_cast<int>(std::lround(center.x))),
        clampMapY(static_cast<int>(std::lround(center.y))),
        clampMapX(static_cast<int>(std::lround(frontCenter.x))),
        clampMapY(static_cast<int>(std::lround(frontCenter.y))),
        UITheme::kWhite);
    pros::screen::set_pen(accent);
    pros::screen::fill_circle(
        clampMapX(static_cast<int>(std::lround(center.x))),
        clampMapY(static_cast<int>(std::lround(center.y))),
        2);
}

static void drawSensorSightlines(const Eigen::Vector3f& pose,
                                 const BrainScreen::RuntimeViewModel& vm) {
    struct SensorRenderSpec {
        const BrainScreen::RuntimeViewModel::DistanceSensorViewModel* sensor = nullptr;
        Eigen::Vector3f offset = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    };

    const std::array<SensorRenderSpec, 4> sensors{{
        {&vm.distanceSensors[0], CONFIG::DIST_LEFT_OFFSET},
        {&vm.distanceSensors[1], CONFIG::DIST_RIGHT_OFFSET},
        {&vm.distanceSensors[2], CONFIG::DIST_FRONT_OFFSET},
        {&vm.distanceSensors[3], CONFIG::DIST_BACK_OFFSET},
    }};

    constexpr float kSightlinePreviewM = 0.4064f;  // 16 in
    const float maxRangeM = CONFIG::MCL_DISTANCE_MAX_RANGE.convert(meter);

    for (const auto& sensorSpec : sensors) {
        if (!sensorSpec.sensor) continue;

        const Eigen::Vector2f sensorWorld =
            pose.head<2>() + rotateLocalToField(sensorSpec.offset.head<2>(), pose.z());
        const float sensorHeading = pose.z() + sensorSpec.offset.z();
        const Eigen::Vector2f sensorDir(std::cos(sensorHeading), std::sin(sensorHeading));

        const Eigen::Vector2f sightEnd = sensorWorld + sensorDir * kSightlinePreviewM;
        const ScreenPoint sensorPx = toScreenPoint(sensorWorld);
        const ScreenPoint sightPx = toScreenPoint(sightEnd);
        UITheme::drawLine(
            clampMapX(static_cast<int>(std::lround(sensorPx.x))),
            clampMapY(static_cast<int>(std::lround(sensorPx.y))),
            clampMapX(static_cast<int>(std::lround(sightPx.x))),
            clampMapY(static_cast<int>(std::lround(sightPx.y))),
            COL_SIGHTLINE);

        pros::screen::set_pen(COL_SIGHTLINE);
        pros::screen::fill_circle(
            clampMapX(static_cast<int>(std::lround(sensorPx.x))),
            clampMapY(static_cast<int>(std::lround(sensorPx.y))),
            1);

        if (!sensorSpec.sensor->valid) continue;

        const float rayDistance = std::min(sensorSpec.sensor->rangeM, maxRangeM);
        const Eigen::Vector2f hitWorld = sensorWorld + sensorDir * rayDistance;
        const ScreenPoint hitPx = toScreenPoint(hitWorld);
        UITheme::drawLine(
            clampMapX(static_cast<int>(std::lround(sensorPx.x))),
            clampMapY(static_cast<int>(std::lround(sensorPx.y))),
            clampMapX(static_cast<int>(std::lround(hitPx.x))),
            clampMapY(static_cast<int>(std::lround(hitPx.y))),
            COL_SENSOR_RAY);

        pros::screen::set_pen(COL_SENSOR_HIT);
        pros::screen::fill_circle(
            clampMapX(static_cast<int>(std::lround(hitPx.x))),
            clampMapY(static_cast<int>(std::lround(hitPx.y))),
            2);
    }
}

static void drawMapPose(const BrainScreen::RuntimeViewModel& vm,
                        const SourceSpec& spec,
                        LocalizationView view) {
    if (!spec.valid || !isFinitePose(spec.pose)) {
        UITheme::printCenteredfOn(pros::E_TEXT_MEDIUM,
                                  UITheme::Rect{MAP_X0, MAP_Y0, MAP_X0 + MAP_PX - 1, MAP_Y0 + MAP_PX - 1},
                                  MAP_Y0 + MAP_PX / 2 - 8,
                                  UITheme::kTextSoft,
                                  COL_FIELD,
                                  "No valid source pose");
        return;
    }

    int rx = toScreenX(spec.pose.x());
    int ry = toScreenY(spec.pose.y());
    rx = std::max(MAP_X0, std::min(MAP_X0 + MAP_PX - 1, rx));
    ry = std::max(MAP_Y0, std::min(MAP_Y0 + MAP_PX - 1, ry));

    bool add = true;
    if (trailCount > 0) {
        const int prevIdx = (trailHead == 0) ? TRAIL_MAX - 1 : trailHead - 1;
        const int dx = rx - trail[prevIdx].px;
        const int dy = ry - trail[prevIdx].py;
        if (dx * dx + dy * dy < 4) add = false;
    }
    if (add) {
        trail[trailHead] = {rx, ry};
        trailHead = (trailHead + 1) % TRAIL_MAX;
        if (trailCount < TRAIL_MAX) ++trailCount;
    }

    drawTrail();

    if (view == LocalizationView::PureMcl || view == LocalizationView::Combined) {
        drawParticleCloud(vm.pfParticleSample);
        drawSensorSightlines(spec.pose, vm);
    }

    drawRobotFootprint(spec.pose, spec.accent);
}

static void drawOdomDetails(const BrainScreen::RuntimeViewModel& vm) {
    const int x = TELEMETRY_PANEL.x0 + 12;
    const int y = TELEMETRY_PANEL.y0 + 84;
    constexpr int w = 94;
    constexpr int h = 34;
    constexpr int gap = 8;

    char dxBuf[20];
    char dyBuf[20];
    char dhBuf[20];
    char magBuf[20];

    const float dx = mToIn(vm.combinedPose.x() - vm.pureOdomPose.x());
    const float dy = mToIn(vm.combinedPose.y() - vm.pureOdomPose.y());
    const float dh = headingDeltaCompassDeg(vm.combinedPose.z() - vm.pureOdomPose.z());
    const float correctionMag = std::sqrt(dx * dx + dy * dy);

    std::snprintf(dxBuf, sizeof(dxBuf), "%+.1f in", dx);
    std::snprintf(dyBuf, sizeof(dyBuf), "%+.1f in", dy);
    std::snprintf(dhBuf, sizeof(dhBuf), "%+.1f deg", dh);
    std::snprintf(magBuf, sizeof(magBuf), "%.1f in", correctionMag);

    drawInfoTile(UITheme::makeRect(x, y, w, h), "COMB dX", dxBuf, UITheme::kAmber);
    drawInfoTile(UITheme::makeRect(x + w + gap, y, w, h), "COMB dY", dyBuf, UITheme::kBlue);
    drawInfoTile(UITheme::makeRect(x, y + h + gap, w, h), "COMB dH", dhBuf, UITheme::kBlue);
    drawInfoTile(UITheme::makeRect(x + w + gap, y + h + gap, w, h),
                 "CORR MAG", magBuf, UITheme::kGreen);
}

static void drawGpsDetails(const BrainScreen::RuntimeViewModel& vm) {
    const int x = TELEMETRY_PANEL.x0 + 12;
    const int y = TELEMETRY_PANEL.y0 + 84;
    constexpr int w = 94;
    constexpr int h = 34;
    constexpr int gap = 8;

    if (!vm.gpsPoseValid) {
        UITheme::drawPanel(UITheme::makeRect(x, y, 196, 76),
                           UITheme::kPanelMuted, UITheme::kBorder, UITheme::kBlue, false);
        UITheme::printTextfOn(pros::E_TEXT_MEDIUM, x + 12, y + 18,
                              UITheme::kText, UITheme::kPanelMuted, "No GPS lock");
        UITheme::printTextfOn(pros::E_TEXT_SMALL, x + 12, y + 40,
                              UITheme::kTextSoft, UITheme::kPanelMuted, "Check strip alignment and error");
        return;
    }

    char dxBuf[20];
    char dyBuf[20];
    char dhBuf[20];
    char errBuf[20];

    const float dx = mToIn(vm.combinedPose.x() - vm.gpsPose.x());
    const float dy = mToIn(vm.combinedPose.y() - vm.gpsPose.y());
    const float dh = headingDeltaCompassDeg(vm.combinedPose.z() - vm.gpsPose.z());

    std::snprintf(dxBuf, sizeof(dxBuf), "%+.1f in", dx);
    std::snprintf(dyBuf, sizeof(dyBuf), "%+.1f in", dy);
    std::snprintf(dhBuf, sizeof(dhBuf), "%+.1f deg", dh);
    if (vm.gpsErrorM >= 0.0f) {
        std::snprintf(errBuf, sizeof(errBuf), "%.1f in", mToIn(vm.gpsErrorM));
    } else {
        std::snprintf(errBuf, sizeof(errBuf), "--");
    }

    drawInfoTile(UITheme::makeRect(x, y, w, h), "COMB dX", dxBuf, UITheme::kAmber);
    drawInfoTile(UITheme::makeRect(x + w + gap, y, w, h), "COMB dY", dyBuf, UITheme::kBlue);
    drawInfoTile(UITheme::makeRect(x, y + h + gap, w, h), "COMB dH", dhBuf, UITheme::kBlue);
    drawInfoTile(UITheme::makeRect(x + w + gap, y + h + gap, w, h), "GPS ERR", errBuf, UITheme::kBlue);
}

static void drawCombinedDetails(const BrainScreen::RuntimeViewModel& vm) {
    const int x = TELEMETRY_PANEL.x0 + 12;
    const int y = TELEMETRY_PANEL.y0 + 84;
    constexpr int w = 94;
    constexpr int h = 34;
    constexpr int gap = 8;

    char odxBuf[20];
    char odyBuf[20];
    char gpsBuf[20];
    char mclBuf[20];

    const float odx = mToIn(vm.combinedPose.x() - vm.pureOdomPose.x());
    const float ody = mToIn(vm.combinedPose.y() - vm.pureOdomPose.y());
    const float gpsDrift = vm.gpsPoseValid
        ? mToIn((vm.combinedPose.head<2>() - vm.gpsPose.head<2>()).norm())
        : -1.0f;
    const float mclDrift = mToIn((vm.combinedPose.head<2>() - vm.pureMclPose.head<2>()).norm());

    std::snprintf(odxBuf, sizeof(odxBuf), "%+.1f in", odx);
    std::snprintf(odyBuf, sizeof(odyBuf), "%+.1f in", ody);
    if (gpsDrift >= 0.0f) {
        std::snprintf(gpsBuf, sizeof(gpsBuf), "%.1f in", gpsDrift);
    } else {
        std::snprintf(gpsBuf, sizeof(gpsBuf), "--");
    }
    std::snprintf(mclBuf, sizeof(mclBuf), "%.1f in", mclDrift);

    drawInfoTile(UITheme::makeRect(x, y, w, h), "ODOM dX", odxBuf, UITheme::kAmber);
    drawInfoTile(UITheme::makeRect(x + w + gap, y, w, h), "ODOM dY", odyBuf, UITheme::kBlue);
    drawInfoTile(UITheme::makeRect(x, y + h + gap, w, h), "GPS DRIFT", gpsBuf, UITheme::kBlue);
    drawInfoTile(UITheme::makeRect(x + w + gap, y + h + gap, w, h), "MCL DRIFT", mclBuf, UITheme::kBlue);
}

static void drawMclDetails(const BrainScreen::RuntimeViewModel& vm) {
    const int x = TELEMETRY_PANEL.x0 + 12;
    const int y = TELEMETRY_PANEL.y0 + 84;
    constexpr int w = 94;
    constexpr int h = 30;
    constexpr int gap = 8;

    char usedBuf[24];
    char essBuf[24];
    const int configuredSensors =
        static_cast<int>(CONFIG::MCL_ENABLE_GPS_SENSOR) +
        static_cast<int>(CONFIG::MCL_ENABLE_DISTANCE_SENSORS && CONFIG::MCL_ENABLE_LEFT_DISTANCE_SENSOR) +
        static_cast<int>(CONFIG::MCL_ENABLE_DISTANCE_SENSORS && CONFIG::MCL_ENABLE_RIGHT_DISTANCE_SENSOR) +
        static_cast<int>(CONFIG::MCL_ENABLE_DISTANCE_SENSORS && CONFIG::MCL_ENABLE_FRONT_DISTANCE_SENSOR) +
        static_cast<int>(CONFIG::MCL_ENABLE_DISTANCE_SENSORS && CONFIG::MCL_ENABLE_BACK_DISTANCE_SENSOR);

    std::snprintf(usedBuf, sizeof(usedBuf), "%c %zu/%d",
                  vm.pfUsedMeasurements ? 'Y' : 'N',
                  vm.pfActiveSensors,
                  configuredSensors);
    std::snprintf(essBuf, sizeof(essBuf), "%.0f/%d",
                  vm.pfEss,
                  CONFIG::NUM_PARTICLES);

    drawInfoTile(UITheme::makeRect(x, y, w, h), "USED/ACT", usedBuf, UITheme::kTeal);
    drawInfoTile(UITheme::makeRect(x + w + gap, y, w, h), "ESS", essBuf, UITheme::kAmber);

    drawSensorTile(UITheme::makeRect(x, y + h + gap, w, h), vm.distanceSensors[0], UITheme::kBlue);
    drawSensorTile(UITheme::makeRect(x + w + gap, y + h + gap, w, h), vm.distanceSensors[1], UITheme::kBlue);
    drawSensorTile(UITheme::makeRect(x, y + 2 * (h + gap), w, h), vm.distanceSensors[2], UITheme::kBlue);
    drawSensorTile(UITheme::makeRect(x + w + gap, y + 2 * (h + gap), w, h), vm.distanceSensors[3], UITheme::kBlue);
}

}  // namespace

void init() {
    trailCount = 0;
    trailHead = 0;
}

void clearTrail() {
    trailCount = 0;
    trailHead = 0;
}

void update(const BrainScreen::RuntimeViewModel& vm, LocalizationView view) {
    const SourceSpec spec = sourceSpec(vm, view);
    drawFieldBase(spec);
    drawMapPose(vm, spec, view);

    UITheme::drawPanel(TELEMETRY_PANEL, UITheme::kPanel, UITheme::kBorderStrong, spec.accent);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, TELEMETRY_PANEL.x0 + 12, TELEMETRY_PANEL.y0 + 8,
                          UITheme::kTextMuted, UITheme::kPanel, "SOURCE DETAILS");
    drawPoseCard(UITheme::makeRect(TELEMETRY_PANEL.x0 + 12, TELEMETRY_PANEL.y0 + 28, 196, 56), spec);

    switch (view) {
        case LocalizationView::PureOdom:
            drawOdomDetails(vm);
            break;
        case LocalizationView::PureGps:
            drawGpsDetails(vm);
            break;
        case LocalizationView::PureMcl:
            drawMclDetails(vm);
            break;
        case LocalizationView::Combined:
        default:
            drawCombinedDetails(vm);
            break;
    }
}

}  // namespace DisplayUI
