/**
 * @file autonSelector.cpp
 * Brain-screen autonomous selector implementation.
 */
#include "ui/autonSelector.h"
#include "ui/brainScreen.h"

#include <cstdio>

// ── Static methods ──────────────────────────────────────────────────────────

namespace {

const AutonList& autonList() {
    return availableAutons();
}

int indexOfAuton(const AutonEntry* auton) {
    const auto& list = autonList();
    for (size_t i = 0; i < list.size(); ++i) {
        if (&list[i] == auton) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void logSelection(const AutonEntry* entry) {
    std::printf("[AUTON_UI] selected=%s\n", entry ? entry->name : "None");
}

} // namespace

void AutonSelector::init() {
    const auto& list = autonList();
    if (list.empty()) {
        s_auton = nullptr;
        return;
    }

    if (indexOfAuton(s_auton) < 0 || s_auton == nullptr) {
        s_auton = &list.front();
    }
}

void AutonSelector::nextAuton() {
    init();
    const auto& list = autonList();
    if (list.empty()) return;

    const int current = indexOfAuton(s_auton);
    if (current < 0 || current >= static_cast<int>(list.size()) - 1) {
        s_auton = &list.front();
        logSelection(s_auton);
        return;
    }

    s_auton = &list[static_cast<size_t>(current + 1)];
    logSelection(s_auton);
}

void AutonSelector::prevAuton() {
    init();
    const auto& list = autonList();
    if (list.empty()) return;

    const int current = indexOfAuton(s_auton);
    if (current <= 0) {
        s_auton = &list.back();
        logSelection(s_auton);
        return;
    }

    s_auton = &list[static_cast<size_t>(current - 1)];
    logSelection(s_auton);
}

void AutonSelector::selectAuton(Auton auton) {
    s_auton = findAuton(auton);
    init();
    logSelection(s_auton);
}

Auton AutonSelector::getAuton() {
    init();
    return s_auton ? s_auton->id : Auton::NONE;
}

const AutonEntry* AutonSelector::getAutonEntry() {
    init();
    return s_auton;
}

std::string AutonSelector::getAutonStr() {
    init();
    return std::string(s_auton ? s_auton->name : "None");
}

void AutonSelector::render(const Eigen::Vector3f& pose,
                           const std::string& status) {
    BrainScreen::RuntimeViewModel vm;
    vm.pose = pose;
    vm.selectedAuton = getAuton();
    vm.auton = getAutonStr();
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
