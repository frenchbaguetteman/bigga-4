#include "ui/pidGraph.h"
#include "ui/theme.h"

#include "pros/screen.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace PIDGraphUI {

static constexpr int MAX_SAMPLES = 240;
static std::array<double, MAX_SAMPLES> errorBuf{};
static std::array<double, MAX_SAMPLES> outputBuf{};
static int head = 0;
static int count = 0;

static constexpr uint32_t BG = UITheme::kPanelAlt;
static constexpr uint32_t FRAME = UITheme::kBorderStrong;
static constexpr uint32_t GRID = 0x0022313F;
static constexpr uint32_t ZERO = 0x00384B59;
static constexpr uint32_t ERR_COL = UITheme::kRed;
static constexpr uint32_t OUT_COL = UITheme::kCyan;
static constexpr uint32_t TXT = UITheme::kText;
static constexpr uint32_t LABEL = UITheme::kTextMuted;

void init() {
    head = 0;
    count = 0;
    errorBuf.fill(0.0);
    outputBuf.fill(0.0);
}

void addSample(double error, double output) {
    if (!std::isfinite(error)) error = 0.0;
    if (!std::isfinite(output)) output = 0.0;
    errorBuf[head] = error;
    outputBuf[head] = output;
    head = (head + 1) % MAX_SAMPLES;
    if (count < MAX_SAMPLES) ++count;
}

void draw(int x0, int y0, int x1, int y1) {
    int w = x1 - x0 + 1;
    int h = y1 - y0 + 1;
    int midY = y0 + h / 2;

    UITheme::fillRect(UITheme::Rect{x0, y0, x1, y1}, BG);
    UITheme::outlineRect(UITheme::Rect{x0, y0, x1, y1}, FRAME);
    for (int gx = x0 + 18; gx < x1; gx += 28) {
        UITheme::drawDividerV(gx, y0 + 1, y1 - 1, GRID);
    }
    for (int gy = y0 + 18; gy < y1; gy += 22) {
        UITheme::drawDividerH(x0 + 1, x1 - 1, gy, GRID);
    }
    UITheme::drawDividerH(x0 + 1, x1 - 1, midY, ZERO);

    if (count == 0) {
        UITheme::printTextf(pros::E_TEXT_MEDIUM, x0 + 10, y0 + 10,
                            LABEL, "No PID data yet");
        return;
    }

    double maxVal = 1.0;
    for (int i = 0; i < count; ++i) {
        maxVal = std::max(maxVal, std::fabs(errorBuf[i]));
        maxVal = std::max(maxVal, std::fabs(outputBuf[i]));
    }
    maxVal = std::min(maxVal * 1.1, 1000.0);
    double sy = (h / 2.0 - 2.0) / maxVal;

    auto idxNewestBack = [&](int i) {
        return (head - 1 - i + MAX_SAMPLES) % MAX_SAMPLES;
    };
    auto toY = [&](double val) {
        int py = midY - static_cast<int>(val * sy);
        return std::max(y0 + 1, std::min(y1 - 1, py));
    };

    int visible = std::max(0, std::min(count, w - 2));
    if (visible < 2) {
        UITheme::printTextf(pros::E_TEXT_MEDIUM, x0 + 10, y0 + 10,
                            LABEL, "PID graph warming up");
        return;
    }

    for (int i = 0; i < visible - 1; ++i) {
        int sx1 = x1 - 1 - i;
        int sx2 = x1 - 2 - i;
        UITheme::drawLine(sx1, toY(errorBuf[idxNewestBack(i)]),
                          sx2, toY(errorBuf[idxNewestBack(i + 1)]), ERR_COL);
    }

    for (int i = 0; i < visible - 1; ++i) {
        int sx1 = x1 - 1 - i;
        int sx2 = x1 - 2 - i;
        UITheme::drawLine(sx1, toY(outputBuf[idxNewestBack(i)]),
                          sx2, toY(outputBuf[idxNewestBack(i + 1)]), OUT_COL);
    }

    UITheme::printTextf(pros::E_TEXT_SMALL, x0 + 6, y0 + 5, ERR_COL, "POS");
    UITheme::printTextf(pros::E_TEXT_SMALL, x0 + 34, y0 + 5, OUT_COL, "HEAD");
    UITheme::printTextf(pros::E_TEXT_SMALL, x1 - 56, y0 + 5, TXT, "x%.1f", maxVal);
    UITheme::printTextf(pros::E_TEXT_SMALL, x0 + 6, y0 + 18, LABEL, "+%.1f", maxVal);
    UITheme::printTextf(pros::E_TEXT_SMALL, x0 + 6, midY - 6, LABEL, "0");
    UITheme::printTextf(pros::E_TEXT_SMALL, x0 + 6, y1 - 14, LABEL, "-%.1f", maxVal);
}

}  // namespace PIDGraphUI
