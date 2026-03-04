#include "ui/display.h"

#include "pros/screen.hpp"

#include <array>
#include <cmath>

namespace DisplayUI {

static constexpr int MAP_X0 = 6;
static constexpr int MAP_Y0 = 6;
static constexpr int MAP_PX = 228;
static constexpr float FIELD_M = 3.6576f;  // 144in
static constexpr float SCALE = MAP_PX / FIELD_M;

static constexpr int CX = MAP_X0 + MAP_PX / 2;
static constexpr int CY = MAP_Y0 + MAP_PX / 2;

static constexpr int TXT_X = 244;

static constexpr uint32_t COL_BG = 0x00000000;
static constexpr uint32_t COL_GRAY = 0x00303030;
static constexpr uint32_t COL_GRID = 0x00505050;
static constexpr uint32_t COL_RED = 0x00A02020;
static constexpr uint32_t COL_BLUE = 0x00203090;
static constexpr uint32_t COL_WHITE = 0x00FFFFFF;
static constexpr uint32_t COL_ROBOT = 0x0000FF00;

static constexpr int TRAIL_MAX = 220;
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

static void drawFieldBase() {
    pros::screen::set_pen(COL_BG);
    pros::screen::fill_rect(0, 0, 479, 239);

    pros::screen::set_pen(COL_GRAY);
    pros::screen::fill_rect(MAP_X0, MAP_Y0, MAP_X0 + MAP_PX, MAP_Y0 + MAP_PX);

    pros::screen::set_pen(COL_GRID);
    for (int i = 0; i <= 6; ++i) {
        int px = MAP_X0 + i * MAP_PX / 6;
        int py = MAP_Y0 + i * MAP_PX / 6;
        pros::screen::draw_line(px, MAP_Y0, px, MAP_Y0 + MAP_PX);
        pros::screen::draw_line(MAP_X0, py, MAP_X0 + MAP_PX, py);
    }

    int tilePx = MAP_PX / 6;
    pros::screen::set_pen(COL_RED);
    pros::screen::fill_rect(MAP_X0, MAP_Y0, MAP_X0 + tilePx, MAP_Y0 + tilePx);
    pros::screen::fill_rect(MAP_X0, MAP_Y0 + MAP_PX - tilePx,
                            MAP_X0 + tilePx, MAP_Y0 + MAP_PX);

    pros::screen::set_pen(COL_BLUE);
    pros::screen::fill_rect(MAP_X0 + MAP_PX - tilePx, MAP_Y0,
                            MAP_X0 + MAP_PX, MAP_Y0 + tilePx);
    pros::screen::fill_rect(MAP_X0 + MAP_PX - tilePx, MAP_Y0 + MAP_PX - tilePx,
                            MAP_X0 + MAP_PX, MAP_Y0 + MAP_PX);

    pros::screen::set_pen(COL_WHITE);
    pros::screen::draw_rect(MAP_X0, MAP_Y0, MAP_X0 + MAP_PX, MAP_Y0 + MAP_PX);
}

static void drawTrail() {
    if (trailCount == 0) return;
    for (int i = 0; i < trailCount; ++i) {
        int idx = (trailCount < TRAIL_MAX) ? i : (trailHead + i) % TRAIL_MAX;
        float b = static_cast<float>(i + 1) / static_cast<float>(trailCount);
        uint8_t g = static_cast<uint8_t>(b * 220.0f);
        uint8_t r = static_cast<uint8_t>((1.0f - b) * 90.0f);
        uint32_t col = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8);
        pros::screen::set_pen(col);
        pros::screen::draw_pixel(trail[idx].px, trail[idx].py);
    }
}

static void drawPoseCard(int x0, int y0, int x1, int y1,
                         const char* title, const Eigen::Vector3f& p) {
    pros::screen::set_pen(0x00181818);
    pros::screen::fill_rect(x0, y0, x1, y1);
    pros::screen::set_pen(0x00555555);
    pros::screen::draw_rect(x0, y0, x1, y1);

    pros::screen::set_pen(COL_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL, x0 + 4, y0 + 4, "%s", title);
    pros::screen::print(pros::E_TEXT_SMALL, x0 + 4, y0 + 18,
                        "x:%+5.1f in", mToIn(p.x()));
    pros::screen::print(pros::E_TEXT_SMALL, x0 + 4, y0 + 31,
                        "y:%+5.1f in", mToIn(p.y()));
    pros::screen::print(pros::E_TEXT_SMALL, x0 + 4, y0 + 44,
                        "h:%+5.1f", radToDeg(p.z()));
}

void init() {
    trailCount = 0;
    trailHead = 0;
    drawFieldBase();
}

void clearTrail() {
    trailCount = 0;
    trailHead = 0;
}

void update(const BrainScreen::RuntimeViewModel& vm) {
    drawFieldBase();

    int rx = toScreenX(vm.combinedPose.x());
    int ry = toScreenY(vm.combinedPose.y());
    if (rx < MAP_X0) rx = MAP_X0;
    if (rx > MAP_X0 + MAP_PX) rx = MAP_X0 + MAP_PX;
    if (ry < MAP_Y0) ry = MAP_Y0;
    if (ry > MAP_Y0 + MAP_PX) ry = MAP_Y0 + MAP_PX;

    bool add = true;
    if (trailCount > 0) {
        int prevIdx = (trailHead == 0) ? TRAIL_MAX - 1 : trailHead - 1;
        int dx = rx - trail[prevIdx].px;
        int dy = ry - trail[prevIdx].py;
        if (dx * dx + dy * dy < 4) add = false;
    }
    if (add) {
        trail[trailHead] = {rx, ry};
        trailHead = (trailHead + 1) % TRAIL_MAX;
        if (trailCount < TRAIL_MAX) ++trailCount;
    }

    drawTrail();

    pros::screen::set_pen(COL_ROBOT);
    pros::screen::fill_circle(rx, ry, 6);

    float h = vm.combinedPose.z();
    int lx = rx + static_cast<int>(std::sin(h) * 14.0f);
    int ly = ry - static_cast<int>(std::cos(h) * 14.0f);
    pros::screen::set_pen(COL_WHITE);
    pros::screen::draw_line(rx, ry, lx, ly);

    pros::screen::set_pen(COL_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL, TXT_X, 4, "ODOM TOOLS");

    drawPoseCard(TXT_X, 18, 356, 96, "pure odom", vm.pureOdomPose);
    drawPoseCard(360, 18, 474, 96, "pure mcl", vm.pureMclPose);
    drawPoseCard(TXT_X, 100, 356, 178, "gps", vm.gpsPose);
    drawPoseCard(360, 100, 474, 178, "combined", vm.combinedPose);

    pros::screen::set_pen(COL_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL, 370, 182, "bot = combined");
    pros::screen::print(pros::E_TEXT_SMALL, TXT_X, 196, "%s", vm.status.c_str());
    pros::screen::print(pros::E_TEXT_SMALL, TXT_X, 212, "Trail:%d", trailCount);
}

}  // namespace DisplayUI
