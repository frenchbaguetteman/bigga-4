/**
 * @file autonSelector.cpp
 * Brain-screen autonomous selector implementation.
 */
#include "ui/autonSelector.h"
#include "pros/llemu.hpp"
#include <cstdio>
#include <cmath>

// ── LLEMU button callbacks (must be free functions) ─────────────────────────

static void on_left()   { AutonSelector::prevAuton();   }
static void on_center() { AutonSelector::toggleAlliance(); }
static void on_right()  { AutonSelector::nextAuton();   }

// ── Static methods ──────────────────────────────────────────────────────────

void AutonSelector::init() {
    // LCD should already be initialized by main, but call again to be safe
    // (multiple calls are harmless)
    pros::lcd::initialize();
    pros::lcd::register_btn0_cb(on_left);
    pros::lcd::register_btn1_cb(on_center);
    pros::lcd::register_btn2_cb(on_right);
}

void AutonSelector::nextAuton() {
    s_index = (s_index + 1) % static_cast<int>(s_autonList.size());
}

void AutonSelector::prevAuton() {
    s_index = (s_index - 1 + static_cast<int>(s_autonList.size()))
              % static_cast<int>(s_autonList.size());
}

void AutonSelector::toggleAlliance() {
    s_alliance = (s_alliance == Alliance::RED) ? Alliance::BLUE : Alliance::RED;
}

Auton AutonSelector::getAuton() {
    return s_autonList[static_cast<size_t>(s_index)];
}

Alliance AutonSelector::getAlliance() {
    return s_alliance;
}

std::string AutonSelector::getAutonStr() {
    return autonName(getAuton());
}

std::string AutonSelector::getAllianceStr() {
    return (s_alliance == Alliance::RED) ? "RED" : "BLUE";
}

void AutonSelector::render(const Eigen::Vector3f& pose,
                           const std::string& status) {
    float xIn  = pose.x() * 39.3701f;   // metres → inches for display
    float yIn  = pose.y() * 39.3701f;
    float tDeg = pose.z() * 180.0f / static_cast<float>(M_PI);

    pros::lcd::print(0, "2654E Echo");
    pros::lcd::print(1, "<< %s >>", getAutonStr().c_str());
    pros::lcd::print(2, "Alliance: %s", getAllianceStr().c_str());
    pros::lcd::print(3, " ");
    pros::lcd::print(4, "Pose: (%.1f, %.1f) in", xIn, yIn);
    pros::lcd::print(5, "Heading: %.1f deg", tDeg);
    pros::lcd::print(6, "%s", status.c_str());
    pros::lcd::print(7, "[<Prev]  [Alliance]  [Next>]");
}

void AutonSelector::renderInit(float progress, const std::string& status) {
    if (!std::isfinite(progress)) progress = 0.0f;
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;

    constexpr int barWidth = 16;
    int filled = static_cast<int>(std::round(progress * barWidth));
    if (filled < 0) filled = 0;
    if (filled > barWidth) filled = barWidth;

    char bar[barWidth + 1];
    for (int i = 0; i < barWidth; ++i) bar[i] = (i < filled) ? '#' : '-';
    bar[barWidth] = '\0';

    int pct = static_cast<int>(std::round(progress * 100.0f));
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    pros::lcd::print(0, "2654E Echo");
    pros::lcd::print(1, "Initializing...");
    pros::lcd::print(2, "[%s] %3d%%", bar, pct);
    pros::lcd::print(3, "%s", status.c_str());
    pros::lcd::print(4, " ");
    pros::lcd::print(5, " ");
    pros::lcd::print(6, " ");
    pros::lcd::print(7, "Please wait");
}
