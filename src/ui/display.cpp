#include "ui/display.h"

#include "ui/theme.h"
#include "pros/screen.hpp"

#include <array>
#include <cmath>
#include <cstdio>

namespace DisplayUI {

namespace {

static constexpr UITheme::Rect FIELD_PANEL = UITheme::makeRect(12, UITheme::kContentY + 8, 228, 190);
static constexpr UITheme::Rect TELEMETRY_PANEL = UITheme::makeRect(248, UITheme::kContentY + 8, 220, 190);

static constexpr int MAP_PX = 172;
static constexpr float FIELD_M = 3.6576f;
static constexpr float SCALE = MAP_PX / FIELD_M;
static constexpr int MAP_X0 = FIELD_PANEL.x0 + (UITheme::width(FIELD_PANEL) - MAP_PX) / 2;
static constexpr int MAP_Y0 = FIELD_PANEL.y0 + 18;
static constexpr int CX = MAP_X0 + MAP_PX / 2;
static constexpr int CY = MAP_Y0 + MAP_PX / 2;

static constexpr uint32_t COL_FIELD = 0x00141D28;
static constexpr uint32_t COL_GRID = 0x00253444;
static constexpr uint32_t COL_FIELD_BORDER = UITheme::kBorderStrong;
static constexpr uint32_t COL_ROBOT = UITheme::kGreen;
static constexpr uint32_t COL_TRAIL_HOT = UITheme::kAmber;
static constexpr uint32_t COL_TRAIL_COOL = UITheme::kCyan;

static constexpr int TRAIL_MAX = 300;
struct TrailPoint { int px; int py; };
static std::array<TrailPoint, TRAIL_MAX> trail{};
static int trailCount = 0;
static int trailHead = 0;

static int toScreenX(float mX) { return CX + static_cast<int>(mX * SCALE); }
static int toScreenY(float mY) { return CY - static_cast<int>(mY * SCALE); }

static float radToDeg(float r) {
    return r * 180.0f / static_cast<float>(M_PI);
}

static float mToIn(float m) {
    return m * 39.3701f;
}

static const char* trimmedStatus(const std::string& status) {
    static char buf[40];
    if (status.empty()) {
        std::snprintf(buf, sizeof(buf), "Nominal");
    } else {
        std::snprintf(buf, sizeof(buf), "%.36s", status.c_str());
    }
    return buf;
}

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

static void drawFieldBase() {
    UITheme::drawPanel(FIELD_PANEL, UITheme::kPanelAlt, UITheme::kBorderStrong, UITheme::kTeal);
    UITheme::printTextf(pros::E_TEXT_SMALL, FIELD_PANEL.x0 + 12, FIELD_PANEL.y0 + 8,
                        UITheme::kTextMuted, "FIELD OVERVIEW");
    char trailBuf[20];
    std::snprintf(trailBuf, sizeof(trailBuf), "%d pts", trailCount);
    UITheme::drawChip(UITheme::makeRect(FIELD_PANEL.x1 - 76, FIELD_PANEL.y0 + 8, 64, 18),
                      trailBuf, UITheme::kPanelMuted, UITheme::kBorder, UITheme::kTextMuted);

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

static void drawPoseCard(const UITheme::Rect& r,
                         const char* title,
                         const Eigen::Vector3f& pose,
                         uint32_t accent) {
    UITheme::drawPanel(r, UITheme::kPanelMuted, UITheme::kBorder, accent, false);
    UITheme::printTextf(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 8, UITheme::kTextMuted, "%s", title);
    UITheme::printTextf(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 22, UITheme::kText,
                        "X %+.1f in", mToIn(pose.x()));
    UITheme::printTextf(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 34, UITheme::kText,
                        "Y %+.1f in", mToIn(pose.y()));
    UITheme::printTextf(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 46, UITheme::kText,
                        "H %+.1f deg", radToDeg(pose.z()));
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

void update(const BrainScreen::RuntimeViewModel& vm) {
    UITheme::drawContentBackdrop();
    drawFieldBase();

    Eigen::Vector3f mapPose = vm.combinedPose;
    if (!std::isfinite(mapPose.x()) || !std::isfinite(mapPose.y()) || !std::isfinite(mapPose.z())) {
        mapPose = vm.pose;
    }

    int rx = toScreenX(mapPose.x());
    int ry = toScreenY(mapPose.y());
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

    pros::screen::set_pen(COL_ROBOT);
    pros::screen::fill_circle(rx, ry, 5);
    pros::screen::set_pen(UITheme::kWhite);
    pros::screen::draw_circle(rx, ry, 7);

    const float h = mapPose.z();
    const int lx = rx + static_cast<int>(std::cos(h) * 14.0f);
    const int ly = ry - static_cast<int>(std::sin(h) * 14.0f);
    UITheme::drawLine(rx, ry, lx, ly, UITheme::kWhite);

    UITheme::drawPanel(TELEMETRY_PANEL, UITheme::kPanel, UITheme::kBorderStrong, UITheme::kBlue);
    UITheme::printTextf(pros::E_TEXT_SMALL, TELEMETRY_PANEL.x0 + 12, TELEMETRY_PANEL.y0 + 8,
                        UITheme::kTextMuted, "POSE SOURCES");
    UITheme::drawChip(UITheme::makeRect(TELEMETRY_PANEL.x1 - 90, TELEMETRY_PANEL.y0 + 8, 74, 18),
                      "FUSED", UITheme::kTealDeep, UITheme::kTeal, UITheme::kText);

    const int cardY0 = TELEMETRY_PANEL.y0 + 28;
    const int cardW = 94;
    const int cardH = 58;
    const int gap = 8;
    const int leftX = TELEMETRY_PANEL.x0 + 12;
    const int rightX = leftX + cardW + gap;

    drawPoseCard(UITheme::makeRect(leftX, cardY0, cardW, cardH), "ODOM", vm.pureOdomPose, UITheme::kAmber);
    drawPoseCard(UITheme::makeRect(rightX, cardY0, cardW, cardH), "MCL", vm.pureMclPose, UITheme::kCyan);
    drawPoseCard(UITheme::makeRect(leftX, cardY0 + cardH + gap, cardW, cardH), "GPS", vm.gpsPose, UITheme::kBlue);
    drawPoseCard(UITheme::makeRect(rightX, cardY0 + cardH + gap, cardW, cardH), "COMBINED", vm.combinedPose, UITheme::kGreen);

    UITheme::drawDividerH(TELEMETRY_PANEL.x0 + 12, TELEMETRY_PANEL.x1 - 12,
                          TELEMETRY_PANEL.y1 - 28, UITheme::kBorder);
    UITheme::printTextf(pros::E_TEXT_SMALL, TELEMETRY_PANEL.x0 + 12, TELEMETRY_PANEL.y1 - 22,
                        UITheme::kTextMuted, "STATUS");
    UITheme::printTextf(pros::E_TEXT_SMALL, TELEMETRY_PANEL.x0 + 12, TELEMETRY_PANEL.y1 - 10,
                        UITheme::kText, "%s", trimmedStatus(vm.status));
}

}  // namespace DisplayUI
