/**
 * @file autonSelector.cpp
 * Brain-screen autonomous selector implementation.
 */
#include "ui/autonSelector.h"
#include "ui/brainScreen.h"
#include <cstdio>
#include <cmath>

// ── Static methods ──────────────────────────────────────────────────────────

namespace {

const std::vector<Auton>& autonList() {
    return availableAutons();
}

} // namespace

void AutonSelector::init() {
    const auto& list = autonList();
    if (list.empty()) {
        s_index = 0;
        return;
    }
    if (s_index < 0 || s_index >= static_cast<int>(list.size())) {
        s_index = 0;
    }
}

void AutonSelector::nextAuton() {
    const auto& list = autonList();
    if (list.empty()) return;
    s_index = (s_index + 1) % static_cast<int>(list.size());
}

void AutonSelector::prevAuton() {
    const auto& list = autonList();
    if (list.empty()) return;
    s_index = (s_index - 1 + static_cast<int>(list.size()))
              % static_cast<int>(list.size());
}

void AutonSelector::toggleAlliance() {
    s_alliance = (s_alliance == Alliance::RED) ? Alliance::BLUE : Alliance::RED;
}

void AutonSelector::selectAlliance(Alliance alliance) {
    s_alliance = alliance;
}

void AutonSelector::selectAuton(Auton auton) {
    const auto& list = autonList();
    for (size_t i = 0; i < list.size(); ++i) {
        if (list[i] == auton) {
            s_index = static_cast<int>(i);
            return;
        }
    }

    s_index = 0;
}

Auton AutonSelector::getAuton() {
    const auto& list = autonList();
    if (list.empty()) return Auton::NONE;
    if (s_index < 0 || s_index >= static_cast<int>(list.size())) {
        s_index = 0;
    }
    return list[static_cast<size_t>(s_index)];
}

Alliance AutonSelector::getAlliance() {
    return s_alliance;
}

std::string AutonSelector::getAutonStr() {
    return std::string(autonName(getAuton()));
}

std::string AutonSelector::getAllianceStr() {
    return std::string(allianceName(getAlliance()));
}

void AutonSelector::render(const Eigen::Vector3f& pose,
                           const std::string& status) {
    BrainScreen::RuntimeViewModel vm;
    vm.pose = pose;
    vm.selectedAuton = getAuton();
    vm.selectedAlliance = getAlliance();
    vm.auton = std::string(autonName(vm.selectedAuton));
    vm.alliance = std::string(allianceName(vm.selectedAlliance));
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
