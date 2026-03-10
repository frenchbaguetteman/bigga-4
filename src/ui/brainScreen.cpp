#include "ui/brainScreen.h"

#include "config.h"
#include "ui/autonSelector.h"

#include "pros/rtos.hpp"
#include "pros/screen.hpp"

#include <cmath>
#include <cstdint>

namespace BrainScreen {

namespace {

constexpr std::uint32_t kBackground = 0x00101010;
constexpr std::uint32_t kText = 0x00ffffff;
constexpr std::uint32_t kMuted = 0x00b8b8b8;

constexpr int kButtonTop = 176;
constexpr int kButtonBottom = 228;
constexpr int kLeftButtonX0 = 18;
constexpr int kLeftButtonX1 = 232;
constexpr int kRightButtonX0 = 248;
constexpr int kRightButtonX1 = 462;

std::uint32_t g_lastTouchMs = 0;

float clamp01(float x) {
    if (!std::isfinite(x)) return 0.0f;
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

float metersToInches(float meters) {
    return meters * 39.3701f;
}

float headingToCompassDeg(float headingRad) {
    return CONFIG::internalRadToGpsHeadingDeg(headingRad);
}

bool pointInside(int x, int y, int x0, int y0, int x1, int y1) {
    return x >= x0 && x <= x1 && y >= y0 && y <= y1;
}

void drawRuntimeTouchTargets() {
    const auto touch = pros::screen::touch_status();
    if (touch.touch_status != pros::E_TOUCH_PRESSED &&
        touch.touch_status != pros::E_TOUCH_HELD) {
        return;
    }

    const std::uint32_t now = pros::millis();
    if (now - g_lastTouchMs < 180) {
        return;
    }

    if (pointInside(touch.x, touch.y, kLeftButtonX0, kButtonTop, kLeftButtonX1, kButtonBottom)) {
        g_lastTouchMs = now;
        AutonSelector::prevAuton();
        return;
    }

    if (pointInside(touch.x, touch.y, kRightButtonX0, kButtonTop, kRightButtonX1, kButtonBottom)) {
        g_lastTouchMs = now;
        AutonSelector::nextAuton();
    }
}

} // namespace

void initialize() {
    pros::screen::set_eraser(kBackground);
    pros::screen::erase();
    g_lastTouchMs = 0;
}

void renderInit(const InitViewModel& vm) {
    const int percent = static_cast<int>(std::round(clamp01(vm.progress) * 100.0f));

    pros::screen::set_eraser(kBackground);
    pros::screen::erase();

    pros::screen::set_pen(kText);
    pros::screen::print(pros::E_TEXT_LARGE, 16, 12, "PLAIN UI BUILD");
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 36, "Marker: BRAINSCREEN_V2");
    pros::screen::set_pen(kMuted);
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 72, "Stage: %s", vm.stageTitle.c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 100, "%s", vm.detail.c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 140, "Progress: %d%%", percent);
}

void renderRuntime(const RuntimeViewModel& vm) {
    drawRuntimeTouchTargets();

    pros::screen::set_eraser(kBackground);
    pros::screen::erase();

    const char* autonNameText = vm.auton.empty() ? "None" : vm.auton.c_str();
    const char* statusText = vm.status.empty() ? "Ready" : vm.status.c_str();
    const float xIn = metersToInches(vm.combinedPose.x());
    const float yIn = metersToInches(vm.combinedPose.y());
    const float headingDeg = headingToCompassDeg(vm.combinedPose.z());

    pros::screen::set_pen(kText);
    pros::screen::print(pros::E_TEXT_LARGE, 16, 8, "PLAIN UI BUILD");
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 32, "Marker: BRAINSCREEN_V2");
    pros::screen::print(pros::E_TEXT_LARGE, 16, 56, "Auton: %s", autonNameText);
    pros::screen::set_pen(kMuted);
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 92, "Status: %s", statusText);
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 120, "Pose X: %+.1f in", xIn);
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 148, "Pose Y: %+.1f in", yIn);
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 176, "Heading: %.1f deg", headingDeg);
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 204, "Left tap=prev  Right tap=next");
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, 224, "Down+B runs selected auton");
}

} // namespace BrainScreen
