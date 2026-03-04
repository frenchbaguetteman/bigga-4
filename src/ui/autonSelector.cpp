/**
 * @file autonSelector.cpp
 * Brain-screen autonomous selector implementation.
 */
#include "ui/autonSelector.h"
#include "ui/brainScreen.h"
#include <cstdio>
#include <cmath>

// ── Static methods ──────────────────────────────────────────────────────────

void AutonSelector::init() {
    // No-op for LVGL UI. Kept for compatibility with existing call sites.
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

void AutonSelector::selectAlliance(Alliance alliance) {
    s_alliance = alliance;
}

void AutonSelector::selectAuton(Auton auton) {
    for (size_t i = 0; i < s_autonList.size(); ++i) {
        if (s_autonList[i] == auton) {
            s_index = static_cast<int>(i);
            return;
        }
    }
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
    BrainScreen::RuntimeViewModel vm;
    vm.pose = pose;
    vm.auton = getAutonStr();
    vm.alliance = getAllianceStr();
    vm.status = status;
    BrainScreen::renderRuntime(vm);
}

void AutonSelector::renderInit(float progress, const std::string& status) {
    BrainScreen::InitViewModel vm;
    vm.progress = progress;
    vm.stageTitle = "Startup";
    vm.detail = status;
    BrainScreen::renderInit(vm);
}
