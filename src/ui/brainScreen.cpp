#include "ui/brainScreen.h"
#include "ui/screenManager.h"
#include "ui/theme.h"
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
}

void renderInit(const InitViewModel& vm) {
    const float progress = clamp01(vm.progress);
    int pct = static_cast<int>(std::round(progress * 100.0f));
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    const UITheme::Rect canvas = UITheme::makeRect(0, 0, UITheme::kScreenW, UITheme::kScreenH);
    const UITheme::Rect brand = UITheme::makeRect(22, 24, 166, 180);
    const UITheme::Rect status = UITheme::makeRect(202, 24, 256, 180);
    const UITheme::Rect progressBar = UITheme::makeRect(status.x0 + 18, status.y0 + 92, 220, 18);

    UITheme::fillRect(canvas, UITheme::kBackground);
    UITheme::fillRect(UITheme::makeRect(0, 0, UITheme::kScreenW, 48), UITheme::kBackgroundBand);
    UITheme::fillRect(UITheme::makeRect(0, 0, 10, UITheme::kScreenH), UITheme::kTealDeep);
    for (int y = 54; y < UITheme::kScreenH; y += 26) {
        UITheme::drawDividerH(0, UITheme::kScreenW - 1, y);
    }

    UITheme::drawPanel(brand, UITheme::kPanelAlt, UITheme::kBorderStrong, UITheme::kTeal);
    UITheme::drawPanel(status, UITheme::kPanel, UITheme::kBorderStrong, UITheme::kBlue);

    UITheme::printTextf(pros::E_TEXT_SMALL, brand.x0 + 14, brand.y0 + 14,
                        UITheme::kTextMuted, "COMPETITION BRAIN");
    UITheme::printTextf(pros::E_TEXT_LARGE, brand.x0 + 14, brand.y0 + 36,
                        UITheme::kText, "69580A");
    UITheme::printTextf(pros::E_TEXT_MEDIUM, brand.x0 + 14, brand.y0 + 72,
                        UITheme::kTextMuted, "System startup");
    UITheme::printTextf(pros::E_TEXT_MEDIUM, brand.x0 + 14, brand.y0 + 98,
                        UITheme::kTextSoft, "Localization");
    UITheme::printTextf(pros::E_TEXT_MEDIUM, brand.x0 + 14, brand.y0 + 116,
                        UITheme::kTextSoft, "Drive services");
    UITheme::printTextf(pros::E_TEXT_MEDIUM, brand.x0 + 14, brand.y0 + 134,
                        UITheme::kTextSoft, "Screen tools");

    UITheme::drawChip(UITheme::makeRect(status.x0 + 18, status.y0 + 16, 92, 22),
                      "BOOT", UITheme::kTealDeep, UITheme::kTeal, UITheme::kText);
    UITheme::printTextf(pros::E_TEXT_SMALL, status.x0 + 18, status.y0 + 48,
                        UITheme::kTextMuted, "CURRENT STEP");
    UITheme::printTextf(pros::E_TEXT_LARGE, status.x0 + 18, status.y0 + 64,
                        UITheme::kText, "%s", vm.stageTitle.c_str());

    UITheme::drawProgressBar(progressBar, progress, UITheme::kTeal);
    UITheme::printTextf(pros::E_TEXT_SMALL, progressBar.x0, progressBar.y0 - 12,
                        UITheme::kTextMuted, "BOOT PROGRESS");
    UITheme::printTextf(pros::E_TEXT_MEDIUM, progressBar.x0, progressBar.y1 + 12,
                        UITheme::kTextSoft, "%s", vm.detail.c_str());
    UITheme::printTextf(pros::E_TEXT_LARGE, status.x1 - 42, status.y0 + 136,
                        UITheme::kText, "%d%%", pct);
}

void renderRuntime(const RuntimeViewModel& vm) {
    ScreenManagerUI::render(vm);
}

}  // namespace BrainScreen
