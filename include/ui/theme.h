#pragma once

#include "pros/screen.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace UITheme {

inline constexpr int kScreenW = 480;
inline constexpr int kScreenH = 240;
inline constexpr int kTopBarH = 26;
inline constexpr int kContentY = kTopBarH + 4;

inline constexpr uint32_t kBackground = 0x000B1118;
inline constexpr uint32_t kBackgroundBand = 0x00101822;
inline constexpr uint32_t kBackdropLine = 0x0018212C;
inline constexpr uint32_t kShadow = 0x0004080C;
inline constexpr uint32_t kPanel = 0x00111822;
inline constexpr uint32_t kPanelAlt = 0x0017202C;
inline constexpr uint32_t kPanelMuted = 0x001A2430;
inline constexpr uint32_t kBorder = 0x00314A5D;
inline constexpr uint32_t kBorderStrong = 0x00486A86;
inline constexpr uint32_t kText = 0x00F1F6FA;
inline constexpr uint32_t kTextMuted = 0x0097A8B7;
inline constexpr uint32_t kTextSoft = 0x006C7D8D;
inline constexpr uint32_t kTeal = 0x001FC8B0;
inline constexpr uint32_t kTealDeep = 0x00127D73;
inline constexpr uint32_t kBlue = 0x003F8BFF;
inline constexpr uint32_t kAmber = 0x00E0A23A;
inline constexpr uint32_t kRed = 0x00D75D5D;
inline constexpr uint32_t kRedDeep = 0x007B2626;
inline constexpr uint32_t kBlueDeep = 0x001F4678;
inline constexpr uint32_t kGreen = 0x0057D28C;
inline constexpr uint32_t kGreenDeep = 0x001E6842;
inline constexpr uint32_t kCyan = 0x0049D3D9;
inline constexpr uint32_t kWhite = 0x00FFFFFF;

struct Rect {
    int x0;
    int y0;
    int x1;
    int y1;
};

inline constexpr Rect makeRect(int x, int y, int w, int h) {
    return Rect{x, y, x + w - 1, y + h - 1};
}

inline constexpr int width(const Rect& r) {
    return r.x1 - r.x0 + 1;
}

inline constexpr int height(const Rect& r) {
    return r.y1 - r.y0 + 1;
}

inline constexpr Rect offset(const Rect& r, int dx, int dy) {
    return Rect{r.x0 + dx, r.y0 + dy, r.x1 + dx, r.y1 + dy};
}

inline constexpr Rect inset(const Rect& r, int dx, int dy) {
    return Rect{r.x0 + dx, r.y0 + dy, r.x1 - dx, r.y1 - dy};
}

inline Rect clampRect(const Rect& r) {
    Rect out = r;
    out.x0 = std::max(0, std::min(kScreenW - 1, out.x0));
    out.y0 = std::max(0, std::min(kScreenH - 1, out.y0));
    out.x1 = std::max(0, std::min(kScreenW - 1, out.x1));
    out.y1 = std::max(0, std::min(kScreenH - 1, out.y1));
    if (out.x1 < out.x0) out.x1 = out.x0;
    if (out.y1 < out.y0) out.y1 = out.y0;
    return out;
}

inline int charWidth(pros::text_format_e_t fmt) {
    switch (fmt) {
        case pros::E_TEXT_SMALL:
            return 5;
        case pros::E_TEXT_LARGE:
        case pros::E_TEXT_LARGE_CENTER:
            return 11;
        case pros::E_TEXT_MEDIUM_CENTER:
        case pros::E_TEXT_MEDIUM:
        default:
            return 7;
    }
}

inline int textWidth(pros::text_format_e_t fmt, const char* txt) {
    return static_cast<int>(std::strlen(txt)) * charWidth(fmt);
}

inline int textHeight(pros::text_format_e_t fmt) {
    switch (fmt) {
        case pros::E_TEXT_SMALL:
            return 12;
        case pros::E_TEXT_LARGE:
        case pros::E_TEXT_LARGE_CENTER:
            return 24;
        case pros::E_TEXT_MEDIUM_CENTER:
        case pros::E_TEXT_MEDIUM:
        default:
            return 18;
    }
}

inline void fillRect(const Rect& r, uint32_t color) {
    Rect c = clampRect(r);
    pros::screen::set_pen(color);
    pros::screen::fill_rect(c.x0, c.y0, c.x1, c.y1);
}

inline void outlineRect(const Rect& r, uint32_t color) {
    Rect c = clampRect(r);
    pros::screen::set_pen(color);
    pros::screen::draw_rect(c.x0, c.y0, c.x1, c.y1);
}

inline void drawLine(int x0, int y0, int x1, int y1, uint32_t color) {
    pros::screen::set_pen(color);
    pros::screen::draw_line(x0, y0, x1, y1);
}

inline void drawShadow(const Rect& r) {
    fillRect(offset(r, 2, 2), kShadow);
}

inline void drawPanel(const Rect& r,
                      uint32_t fill = kPanel,
                      uint32_t border = kBorder,
                      uint32_t accent = 0,
                      bool shadow = true) {
    Rect c = clampRect(r);
    if (shadow) drawShadow(c);
    fillRect(c, fill);
    if (accent != 0 && height(c) >= 8) {
        fillRect(Rect{c.x0 + 1, c.y0 + 1, c.x1 - 1, std::min(c.y0 + 2, c.y1 - 1)}, accent);
    }
    outlineRect(c, border);
}

template <typename... Params>
inline void printTextf(pros::text_format_e_t fmt,
                       int x,
                       int y,
                       uint32_t color,
                       const char* text,
                       Params... args) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), text, args...);
    const Rect bg = clampRect(Rect{
        x - 1,
        y - 1,
        x + textWidth(fmt, buf) + 2,
        y + textHeight(fmt),
    });
    fillRect(bg, kBackground);
    pros::screen::set_eraser(kBackground);
    pros::screen::set_pen(color);
    pros::screen::print(fmt, x, y, "%s", buf);
}

template <typename... Params>
inline void printTextfOn(pros::text_format_e_t fmt,
                         int x,
                         int y,
                         uint32_t color,
                         uint32_t background,
                         const char* text,
                         Params... args) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), text, args...);
    const Rect bg = clampRect(Rect{
        x - 1,
        y - 1,
        x + textWidth(fmt, buf) + 2,
        y + textHeight(fmt),
    });
    fillRect(bg, background);
    pros::screen::set_eraser(background);
    pros::screen::set_pen(color);
    pros::screen::print(fmt, x, y, "%s", buf);
}

template <typename... Params>
inline void printCenteredf(pros::text_format_e_t fmt,
                           const Rect& r,
                           int y,
                           uint32_t color,
                           const char* text,
                           Params... args) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), text, args...);
    int x = r.x0 + std::max(0, (width(r) - textWidth(fmt, buf)) / 2);
    printTextf(fmt, x, y, color, "%s", buf);
}

template <typename... Params>
inline void printCenteredfOn(pros::text_format_e_t fmt,
                             const Rect& r,
                             int y,
                             uint32_t color,
                             uint32_t background,
                             const char* text,
                             Params... args) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), text, args...);
    int x = r.x0 + std::max(0, (width(r) - textWidth(fmt, buf)) / 2);
    printTextfOn(fmt, x, y, color, background, "%s", buf);
}

inline void drawDividerH(int x0, int x1, int y, uint32_t color = kBackdropLine) {
    drawLine(x0, y, x1, y, color);
}

inline void drawDividerV(int x, int y0, int y1, uint32_t color = kBackdropLine) {
    drawLine(x, y0, x, y1, color);
}

inline void drawChip(const Rect& r,
                     const char* label,
                     uint32_t fill,
                     uint32_t border,
                     uint32_t textColor) {
    fillRect(r, fill);
    outlineRect(r, border);
    printCenteredfOn(pros::E_TEXT_MEDIUM, r, r.y0 + 6, textColor, fill, "%s", label);
}

inline void drawProgressBar(const Rect& r,
                            float progress,
                            uint32_t fill,
                            uint32_t track = kPanelMuted,
                            uint32_t border = kBorderStrong) {
    Rect c = clampRect(r);
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;

    fillRect(c, track);
    int innerW = std::max(0, width(c) - 2);
    int fillW = static_cast<int>(innerW * progress);
    if (fillW > 0) {
        fillRect(Rect{c.x0 + 1, c.y0 + 1, c.x0 + fillW, c.y1 - 1}, fill);
    }
    outlineRect(c, border);
}

inline void drawContentBackdrop() {
    fillRect(makeRect(0, kContentY, kScreenW, kScreenH - kContentY), kBackground);
    fillRect(makeRect(0, kContentY, kScreenW, 16), kBackgroundBand);
    fillRect(makeRect(0, kContentY, 6, kScreenH - kContentY), kTealDeep);
    for (int y = kContentY + 18; y < kScreenH; y += 22) {
        drawDividerH(0, kScreenW - 1, y, kBackdropLine);
    }
}

}  // namespace UITheme
