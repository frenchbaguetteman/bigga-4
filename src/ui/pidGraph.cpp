#include "ui/pidGraph.h"

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

static constexpr uint32_t BG = 0x00101010;
static constexpr uint32_t FRAME = 0x00555555;
static constexpr uint32_t ZERO = 0x00333333;
static constexpr uint32_t ERR_COL = 0x00FF4444;
static constexpr uint32_t OUT_COL = 0x0044CCCC;
static constexpr uint32_t TXT = 0x00FFFFFF;
static constexpr uint32_t LABEL = 0x00888888;

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
    int w = x1 - x0;
    int h = y1 - y0;
    int midY = y0 + h / 2;

    pros::screen::set_pen(BG);
    pros::screen::fill_rect(x0, y0, x1, y1);

    pros::screen::set_pen(FRAME);
    pros::screen::draw_rect(x0, y0, x1, y1);
    pros::screen::set_pen(ZERO);
    pros::screen::draw_line(x0 + 1, midY, x1 - 1, midY);

    if (count == 0) {
        pros::screen::set_pen(LABEL);
        pros::screen::print(pros::E_TEXT_MEDIUM, x0 + 8, y0 + 8, "No PID data");
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
        pros::screen::set_pen(LABEL);
        pros::screen::print(pros::E_TEXT_MEDIUM, x0 + 8, y0 + 8, "PID graph warming up");
        return;
    }

    pros::screen::set_pen(ERR_COL);
    for (int i = 0; i < visible - 1; ++i) {
        int sx1 = x1 - 1 - i;
        int sx2 = x1 - 2 - i;
        pros::screen::draw_line(sx1, toY(errorBuf[idxNewestBack(i)]),
                                sx2, toY(errorBuf[idxNewestBack(i + 1)]));
    }

    pros::screen::set_pen(OUT_COL);
    for (int i = 0; i < visible - 1; ++i) {
        int sx1 = x1 - 1 - i;
        int sx2 = x1 - 2 - i;
        pros::screen::draw_line(sx1, toY(outputBuf[idxNewestBack(i)]),
                                sx2, toY(outputBuf[idxNewestBack(i + 1)]));
    }

    pros::screen::set_pen(ERR_COL);
    pros::screen::print(pros::E_TEXT_MEDIUM, x0 + 4, y0 + 2, "Err");
    pros::screen::set_pen(OUT_COL);
    pros::screen::print(pros::E_TEXT_MEDIUM, x0 + 48, y0 + 2, "Out");
    pros::screen::set_pen(TXT);
    pros::screen::print(pros::E_TEXT_MEDIUM, x0 + 94, y0 + 2, "Scale %.1f", maxVal);
}

}  // namespace PIDGraphUI
