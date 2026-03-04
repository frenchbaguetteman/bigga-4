#include "ui/brainScreen.h"
#include "ui/screenManager.h"
#include "pros/screen.hpp"

#include <cmath>

namespace BrainScreen {

static float clamp01(float x) {
    if (!std::isfinite(x)) return 0.0f;
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

void initialize() {
    pros::screen::set_eraser(0x00000000);
    pros::screen::erase();
    ScreenManagerUI::init();
}

void renderInit(const InitViewModel& vm) {
    const float progress = clamp01(vm.progress);
    int pct = static_cast<int>(std::round(progress * 100.0f));
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    pros::screen::set_pen(0x00000000);
    pros::screen::fill_rect(0, 0, 479, 239);

    pros::screen::set_pen(0x00303030);
    pros::screen::fill_rect(24, 40, 455, 200);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::draw_rect(24, 40, 455, 200);

    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_LARGE, 150, 58, "2654E Echo");
    pros::screen::print(pros::E_TEXT_MEDIUM, 56, 92, "%s", vm.stageTitle.c_str());

    int barX0 = 56;
    int barY0 = 126;
    int barX1 = 424;
    int barY1 = 146;
    int filledX = barX0 + (barX1 - barX0) * pct / 100;

    pros::screen::set_pen(0x00181818);
    pros::screen::fill_rect(barX0, barY0, barX1, barY1);
    pros::screen::set_pen(0x0000CC99);
    pros::screen::fill_rect(barX0, barY0, filledX, barY1);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::draw_rect(barX0, barY0, barX1, barY1);

    pros::screen::print(pros::E_TEXT_SMALL, 56, 156, "%s", vm.detail.c_str());
    pros::screen::print(pros::E_TEXT_SMALL, 394, 156, "%d%%", pct);
}

void renderRuntime(const RuntimeViewModel& vm) {
    ScreenManagerUI::render(vm);
}

}  // namespace BrainScreen
