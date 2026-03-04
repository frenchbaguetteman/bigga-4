#pragma once

#include "ui/brainScreen.h"

namespace DisplayUI {

void init();
void clearTrail();
void update(const BrainScreen::RuntimeViewModel& vm);

}  // namespace DisplayUI
