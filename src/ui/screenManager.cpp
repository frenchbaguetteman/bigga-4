#include "ui/screenManager.h"

#include "ui/autonSelector.h"
#include "ui/display.h"
#include "ui/pidGraph.h"
#include "pros/screen.hpp"
#include "pros/rtos.hpp"

#include <cmath>

namespace ScreenManagerUI {

namespace {

enum class ToolPage {
    ODOM,
    PID,
    PATH,
    GPS
};

struct Rect { int x0, y0, x1, y1; };

static constexpr Rect RED_BTN{8, 6, 66, 30};
static constexpr Rect BLUE_BTN{70, 6, 132, 30};
static constexpr Rect SKILLS_BTN{136, 6, 212, 30};
static constexpr Rect PREV_BTN{8, 34, 38, 58};
static constexpr Rect NEXT_BTN{216, 34, 246, 58};

static constexpr Rect TAB_ODOM{252, 6, 306, 30};
static constexpr Rect TAB_PID{310, 6, 356, 30};
static constexpr Rect TAB_PATH{360, 6, 418, 30};
static constexpr Rect TAB_GPS{422, 6, 474, 30};

static ToolPage activePage = ToolPage::ODOM;
static uint32_t lastTouchMs = 0;

static bool inside(const Rect& r, int x, int y) {
    return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1;
}

static void drawButton(const Rect& r, const char* txt, uint32_t bg, bool active = false) {
    pros::screen::set_pen(bg);
    pros::screen::fill_rect(r.x0, r.y0, r.x1, r.y1);
    pros::screen::set_pen(active ? 0x00FFFFFF : 0x00666666);
    pros::screen::draw_rect(r.x0, r.y0, r.x1, r.y1);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_SMALL, r.x0 + 6, r.y0 + 7, "%s", txt);
}

static void drawSelectorHeader(const BrainScreen::RuntimeViewModel& vm) {
    drawButton(RED_BTN, "RED", 0x00700000, vm.alliance == "RED");
    drawButton(BLUE_BTN, "BLUE", 0x00000080, vm.alliance == "BLUE");
    drawButton(SKILLS_BTN, "SKILLS", 0x00444444, vm.auton == "SKILLS");

    drawButton(PREV_BTN, "<", 0x002A2A2A);
    drawButton(NEXT_BTN, ">", 0x002A2A2A);

    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 44, 38, "%s", vm.auton.c_str());

    drawButton(TAB_ODOM, "ODOM", 0x00114444, activePage == ToolPage::ODOM);
    drawButton(TAB_PID, "PID", 0x00302244, activePage == ToolPage::PID);
    drawButton(TAB_PATH, "PATH", 0x00332222, activePage == ToolPage::PATH);
    drawButton(TAB_GPS, "GPS", 0x00223322, activePage == ToolPage::GPS);
}

static void drawPidPage(const BrainScreen::RuntimeViewModel& vm) {
    double err = static_cast<double>(vm.combinedPose.x() - vm.pureOdomPose.x()) * 100.0;
    double out = static_cast<double>(vm.combinedPose.z() - vm.pureOdomPose.z()) * 180.0 / M_PI;
    PIDGraphUI::addSample(err, out);

    pros::screen::set_pen(0x00000000);
    pros::screen::fill_rect(6, 62, 474, 236);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, 66, "PID Tool");
    PIDGraphUI::draw(10, 86, 470, 228);
}

static void drawPathPage(const BrainScreen::RuntimeViewModel& vm) {
    pros::screen::set_pen(0x00000000);
    pros::screen::fill_rect(6, 62, 474, 236);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, 66, "Path Tool");
    pros::screen::print(pros::E_TEXT_SMALL, 10, 92, "Auton: %s", vm.auton.c_str());
    pros::screen::print(pros::E_TEXT_SMALL, 10, 108, "Status: %s", vm.status.c_str());
    pros::screen::print(pros::E_TEXT_SMALL, 10, 124, "Pose: (%.2f, %.2f)",
                        vm.combinedPose.x(), vm.combinedPose.y());
}

static void drawGpsPage(const BrainScreen::RuntimeViewModel& vm) {
    pros::screen::set_pen(0x00000000);
    pros::screen::fill_rect(6, 62, 474, 236);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, 66, "GPS Tool");
    pros::screen::print(pros::E_TEXT_SMALL, 10, 92, "GPS x: %.2f m", vm.gpsPose.x());
    pros::screen::print(pros::E_TEXT_SMALL, 10, 108, "GPS y: %.2f m", vm.gpsPose.y());
    pros::screen::print(pros::E_TEXT_SMALL, 10, 124, "GPS h: %.1f deg",
                        vm.gpsPose.z() * 180.0f / static_cast<float>(M_PI));
    pros::screen::print(pros::E_TEXT_SMALL, 10, 148, "delta x: %.2f m",
                        vm.combinedPose.x() - vm.gpsPose.x());
    pros::screen::print(pros::E_TEXT_SMALL, 10, 164, "delta y: %.2f m",
                        vm.combinedPose.y() - vm.gpsPose.y());
}

static void handleTouch() {
    auto t = pros::screen::touch_status();
    if (t.touch_status != pros::E_TOUCH_PRESSED) return;

    uint32_t now = pros::millis();
    if (now - lastTouchMs < 180) return;
    lastTouchMs = now;

    int x = t.x;
    int y = t.y;

    if (inside(RED_BTN, x, y)) {
        AutonSelector::selectAlliance(Alliance::RED);
        return;
    }
    if (inside(BLUE_BTN, x, y)) {
        AutonSelector::selectAlliance(Alliance::BLUE);
        return;
    }
    if (inside(SKILLS_BTN, x, y)) {
        AutonSelector::selectAuton(Auton::SKILLS);
        return;
    }
    if (inside(PREV_BTN, x, y)) {
        AutonSelector::prevAuton();
        return;
    }
    if (inside(NEXT_BTN, x, y)) {
        AutonSelector::nextAuton();
        return;
    }

    if (inside(TAB_ODOM, x, y)) { activePage = ToolPage::ODOM; return; }
    if (inside(TAB_PID, x, y)) { activePage = ToolPage::PID; return; }
    if (inside(TAB_PATH, x, y)) { activePage = ToolPage::PATH; return; }
    if (inside(TAB_GPS, x, y)) { activePage = ToolPage::GPS; return; }
}

}  // namespace

void init() {
    pros::screen::set_eraser(0x00000000);
    pros::screen::erase();
    DisplayUI::init();
    PIDGraphUI::init();
}

void render(const BrainScreen::RuntimeViewModel& vm) {
    handleTouch();
    drawSelectorHeader(vm);

    switch (activePage) {
        case ToolPage::ODOM:
            DisplayUI::update(vm);
            break;
        case ToolPage::PID:
            drawPidPage(vm);
            break;
        case ToolPage::PATH:
            drawPathPage(vm);
            break;
        case ToolPage::GPS:
            drawGpsPage(vm);
            break;
    }
}

}  // namespace ScreenManagerUI
