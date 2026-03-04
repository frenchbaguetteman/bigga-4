#include "ui/screenManager.h"

#include "ui/autonSelector.h"
#include "ui/display.h"
#include "ui/pidGraph.h"
#include "pros/screen.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ScreenManagerUI {

namespace {

// ── Page enum ───────────────────────────────────────────────────────────────
enum class Page {
    SELECT,   // auton/alliance selector
    ODOM,     // field map + 4 pose cards
    PID,      // PID error/output graph
    PATH,     // path / auton info
    GPS       // raw GPS data + deltas
};

static constexpr int PAGE_COUNT = 5;

// ── Layout constants ────────────────────────────────────────────────────────
// Tab bar lives in y = 0..25.  Content area is y = 28..239.
static constexpr int TAB_H   = 24;
static constexpr int TAB_Y0  = 0;
static constexpr int TAB_Y1  = TAB_H;
static constexpr int CONTENT_Y = TAB_H + 4;   // 28

struct Rect { int x0, y0, x1, y1; };

// 5 evenly-spaced tabs across 480 px
static constexpr int TAB_W = 480 / PAGE_COUNT;  // 96
static constexpr Rect TAB_SELECT {0 * TAB_W, TAB_Y0, 1 * TAB_W - 1, TAB_Y1};
static constexpr Rect TAB_ODOM   {1 * TAB_W, TAB_Y0, 2 * TAB_W - 1, TAB_Y1};
static constexpr Rect TAB_PID    {2 * TAB_W, TAB_Y0, 3 * TAB_W - 1, TAB_Y1};
static constexpr Rect TAB_PATH   {3 * TAB_W, TAB_Y0, 4 * TAB_W - 1, TAB_Y1};
static constexpr Rect TAB_GPS    {4 * TAB_W, TAB_Y0, 5 * TAB_W - 1, TAB_Y1};

// SELECT page controls (inside content area)
static constexpr int BTN_Y0 = CONTENT_Y + 24;
static constexpr int BTN_Y1 = BTN_Y0 + 28;
static constexpr Rect RED_BTN    {16,  BTN_Y0,  96, BTN_Y1};
static constexpr Rect BLUE_BTN   {104, BTN_Y0, 184, BTN_Y1};
static constexpr Rect SKILLS_BTN {192, BTN_Y0, 292, BTN_Y1};
static constexpr Rect PREV_BTN   {308, BTN_Y0, 354, BTN_Y1};
static constexpr Rect NEXT_BTN   {360, BTN_Y0, 406, BTN_Y1};

static Page activePage = Page::SELECT;
static Page prevRenderedPage = Page::SELECT;
static uint32_t lastTouchMs = 0;

// ── Helpers ─────────────────────────────────────────────────────────────────

static bool inside(const Rect& r, int x, int y) {
    return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1;
}

static float radToDeg(float r) {
    return r * 180.0f / static_cast<float>(M_PI);
}

static float wrapDeg(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static const char* safeStatus(const std::string& s) {
    static char buf[44];
    if (s.empty()) {
        std::snprintf(buf, sizeof(buf), "Status: --");
        return buf;
    }
    std::snprintf(buf, sizeof(buf), "Status: %.34s", s.c_str());
    return buf;
}

static void clearContent() {
    pros::screen::set_pen(0x00000000);
    pros::screen::fill_rect(0, CONTENT_Y, 479, 239);
}

static void drawTab(const Rect& r, const char* txt, bool active) {
    uint32_t bg = active ? 0x00224466 : 0x00181818;
    pros::screen::set_pen(bg);
    pros::screen::fill_rect(r.x0, r.y0, r.x1, r.y1);
    pros::screen::set_pen(active ? 0x00FFFFFF : 0x00555555);
    pros::screen::draw_rect(r.x0, r.y0, r.x1, r.y1);
    int textW = static_cast<int>(__builtin_strlen(txt)) * 7;  // approx
    int cx = r.x0 + (r.x1 - r.x0 - textW) / 2;
    pros::screen::set_pen(active ? 0x00FFFFFF : 0x00999999);
    pros::screen::print(pros::E_TEXT_MEDIUM, cx, r.y0 + 5, "%s", txt);
}

static void drawButton(const Rect& r, const char* txt, uint32_t bg,
                        bool highlight = false) {
    pros::screen::set_pen(bg);
    pros::screen::fill_rect(r.x0, r.y0, r.x1, r.y1);
    pros::screen::set_pen(highlight ? 0x00FFFFFF : 0x00666666);
    pros::screen::draw_rect(r.x0, r.y0, r.x1, r.y1);
    pros::screen::set_pen(0x00FFFFFF);
    int cx = r.x0 + (r.x1 - r.x0) / 2 -
             static_cast<int>(__builtin_strlen(txt)) * 4;
    pros::screen::print(pros::E_TEXT_MEDIUM, cx, r.y0 + 5, "%s", txt);
}

// ── Tab bar (always drawn) ──────────────────────────────────────────────────

static void drawTabBar() {
    // black background behind tabs
    pros::screen::set_pen(0x00000000);
    pros::screen::fill_rect(0, 0, 479, TAB_Y1);

    drawTab(TAB_SELECT, "SELECT", activePage == Page::SELECT);
    drawTab(TAB_ODOM,   "ODOM",   activePage == Page::ODOM);
    drawTab(TAB_PID,    "PID",    activePage == Page::PID);
    drawTab(TAB_PATH,   "PATH",   activePage == Page::PATH);
    drawTab(TAB_GPS,    "GPS",    activePage == Page::GPS);
}

// ── SELECT page ─────────────────────────────────────────────────────────────

static void drawSelectPage(const BrainScreen::RuntimeViewModel& vm) {
    clearContent();

    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, CONTENT_Y + 2, "Auton Selector");

    drawButton(RED_BTN,    "RED",    0x00700000, vm.alliance == "RED");
    drawButton(BLUE_BTN,   "BLUE",   0x00000080, vm.alliance == "BLUE");
    drawButton(SKILLS_BTN, "SKILLS", 0x00444444, vm.auton == "Skills");

    drawButton(PREV_BTN, "<", 0x002A2A2A);
    drawButton(NEXT_BTN, ">", 0x002A2A2A);

    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_LARGE, 16, BTN_Y1 + 10, "%s", vm.auton.c_str());

    // Quick pose summary
    pros::screen::set_pen(0x00888888);
    float xi = vm.combinedPose.x() * 39.3701f;
    float yi = vm.combinedPose.y() * 39.3701f;
    float hd = radToDeg(vm.combinedPose.z());
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, BTN_Y1 + 40,
                        "Pose: x=%.1f in  y=%.1f in  h=%.1f deg", xi, yi, hd);
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, BTN_Y1 + 58,
                        "Alliance: %s   Auton: %s", vm.alliance.c_str(), vm.auton.c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, BTN_Y1 + 76, "%s", safeStatus(vm.status));
}

// ── PID page ────────────────────────────────────────────────────────────────

static void drawPidPage(const BrainScreen::RuntimeViewModel& vm) {
    double errIn = static_cast<double>(vm.combinedPose.x() - vm.pureOdomPose.x()) * 39.3701;
    double headingErrDeg = static_cast<double>(
        wrapDeg(radToDeg(vm.combinedPose.z() - vm.pureOdomPose.z())));

    if (!std::isfinite(errIn)) errIn = 0.0;
    if (!std::isfinite(headingErrDeg)) headingErrDeg = 0.0;

    // PID monitor shows proxy tracking errors until direct PID telemetry is wired.
    // Red = position error (in), Cyan = heading error (deg).
    double err = errIn;
    double out = headingErrDeg;
    PIDGraphUI::addSample(err, out);

    clearContent();
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, CONTENT_Y + 2, "PID Monitor (tracking)");
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, CONTENT_Y + 18,
                        "ErrX: %+.2f in   ErrH: %+.2f deg", errIn, headingErrDeg);
    PIDGraphUI::draw(6, CONTENT_Y + 36, 474, 236);
}

// ── PATH page ───────────────────────────────────────────────────────────────

static void drawPathPage(const BrainScreen::RuntimeViewModel& vm) {
    clearContent();
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, CONTENT_Y + 2, "Path Tool");

    int y = CONTENT_Y + 26;
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, y,      "Auton: %s", vm.auton.c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, y + 18, "Alliance: %s", vm.alliance.c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, y + 36, "%s", safeStatus(vm.status));

    float xi = vm.combinedPose.x() * 39.3701f;
    float yi = vm.combinedPose.y() * 39.3701f;
    float hd = vm.combinedPose.z() * 180.0f / static_cast<float>(M_PI);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, y + 60,
                        "Pose: (%.1f, %.1f) in  h=%.1f deg", xi, yi, hd);
}

// ── GPS page ────────────────────────────────────────────────────────────────

static void drawGpsPage(const BrainScreen::RuntimeViewModel& vm) {
    clearContent();
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, CONTENT_Y + 2, "GPS Tool");

    int y = CONTENT_Y + 26;
    float gxi = vm.gpsPose.x() * 39.3701f;
    float gyi = vm.gpsPose.y() * 39.3701f;
    float ghd = vm.gpsPose.z() * 180.0f / static_cast<float>(M_PI);

    pros::screen::print(pros::E_TEXT_MEDIUM, 10, y,
                        "GPS x: %.2f m  (%.1f in)", vm.gpsPose.x(), gxi);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, y + 18,
                        "GPS y: %.2f m  (%.1f in)", vm.gpsPose.y(), gyi);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, y + 36,
                        "GPS h: %.1f deg", ghd);

    float dx = (vm.combinedPose.x() - vm.gpsPose.x()) * 39.3701f;
    float dy = (vm.combinedPose.y() - vm.gpsPose.y()) * 39.3701f;

    pros::screen::set_pen(0x00AAAAAA);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, y + 62,
                        "delta x: %+.1f in", dx);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, y + 80,
                        "delta y: %+.1f in", dy);

    float cxi = vm.combinedPose.x() * 39.3701f;
    float cyi = vm.combinedPose.y() * 39.3701f;
    float chd = vm.combinedPose.z() * 180.0f / static_cast<float>(M_PI);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, y + 106,
                        "Combined: (%.1f, %.1f) in  h=%.1f", cxi, cyi, chd);
}

// ── Touch handling ──────────────────────────────────────────────────────────

static void handleTouch() {
    auto t = pros::screen::touch_status();
    if (t.touch_status != pros::E_TOUCH_PRESSED &&
        t.touch_status != pros::E_TOUCH_HELD) {
        return;
    }

    uint32_t now = pros::millis();
    if (now - lastTouchMs < 140) return;
    lastTouchMs = now;

    int x = t.x;
    int y = t.y;

    // Tab bar touches
    if (y <= TAB_Y1 + 4) {
        if (inside(TAB_SELECT, x, y)) { activePage = Page::SELECT; return; }
        if (inside(TAB_ODOM,   x, y)) { activePage = Page::ODOM;   return; }
        if (inside(TAB_PID,    x, y)) { activePage = Page::PID;    return; }
        if (inside(TAB_PATH,   x, y)) { activePage = Page::PATH;   return; }
        if (inside(TAB_GPS,    x, y)) { activePage = Page::GPS;    return; }
    }

    // SELECT page content buttons
    if (activePage == Page::SELECT) {
        if (inside(RED_BTN, x, y))    { AutonSelector::selectAlliance(Alliance::RED);  return; }
        if (inside(BLUE_BTN, x, y))   { AutonSelector::selectAlliance(Alliance::BLUE); return; }
        if (inside(SKILLS_BTN, x, y)) { AutonSelector::selectAuton(Auton::SKILLS);     return; }
        if (inside(PREV_BTN, x, y))   { AutonSelector::prevAuton();  return; }
        if (inside(NEXT_BTN, x, y))   { AutonSelector::nextAuton();  return; }
    }
}

}  // namespace

void init() {
    pros::screen::set_eraser(0x00000000);
    pros::screen::erase();
    DisplayUI::init();
    PIDGraphUI::init();
    activePage = Page::SELECT;
    prevRenderedPage = Page::SELECT;
}

void render(const BrainScreen::RuntimeViewModel& vm) {
    handleTouch();

    // Always redraw the tab bar
    drawTabBar();

    // Clear content when switching pages
    if (activePage != prevRenderedPage) {
        clearContent();
        prevRenderedPage = activePage;
    }

    switch (activePage) {
        case Page::SELECT:
            drawSelectPage(vm);
            break;
        case Page::ODOM:
            DisplayUI::update(vm);
            break;
        case Page::PID:
            drawPidPage(vm);
            break;
        case Page::PATH:
            drawPathPage(vm);
            break;
        case Page::GPS:
            drawGpsPage(vm);
            break;
    }
}

}  // namespace ScreenManagerUI
