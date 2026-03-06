#pragma once

#include "ui/brainScreen.h"

namespace DisplayUI {

enum class LocalizationView {
    PureOdom,
    PureGps,
    PureMcl,
    Combined,
};

void init();
void clearTrail();
void update(const BrainScreen::RuntimeViewModel& vm, LocalizationView view);

}  // namespace DisplayUI
