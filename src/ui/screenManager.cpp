#include "ui/screenManager.h"

#include "config.h"
#include "ui/autonSelector.h"
#include "ui/display.h"
#include "ui/pidGraph.h"
#include "ui/theme.h"
#include "utils/localization_math.h"
#include "pros/screen.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ScreenManagerUI {

namespace {

enum class Page {
    SELECT,
    LOCALIZATION,
    PID,
    PATH,
    GPS
};

static constexpr UITheme::Rect BRAND_CARD {8, 4, 84, 21};
static constexpr UITheme::Rect TAB_SELECT {96, 4, 168, 21};
static constexpr UITheme::Rect TAB_ODOM   {172, 4, 244, 21};
static constexpr UITheme::Rect TAB_PID    {248, 4, 320, 21};
static constexpr UITheme::Rect TAB_PATH   {324, 4, 396, 21};
static constexpr UITheme::Rect TAB_GPS    {400, 4, 472, 21};

static constexpr UITheme::Rect LOCAL_SUB_ODOM     = UITheme::makeRect(12, UITheme::kContentY + 8, 108, 22);
static constexpr UITheme::Rect LOCAL_SUB_GPS      = UITheme::makeRect(128, UITheme::kContentY + 8, 108, 22);
static constexpr UITheme::Rect LOCAL_SUB_MCL      = UITheme::makeRect(244, UITheme::kContentY + 8, 108, 22);
static constexpr UITheme::Rect LOCAL_SUB_COMBINED = UITheme::makeRect(360, UITheme::kContentY + 8, 108, 22);

static constexpr UITheme::Rect RED_BTN    {24, 70, 105, 95};
static constexpr UITheme::Rect BLUE_BTN   {114, 70, 195, 95};
static constexpr UITheme::Rect PREV_BTN   {307, 58, 374, 87};
static constexpr UITheme::Rect NEXT_BTN   {387, 58, 454, 87};
static constexpr UITheme::Rect SKILLS_BTN {307, 98, 454, 121};

static Page activePage = Page::SELECT;
static Page prevRenderedPage = Page::SELECT;
static DisplayUI::LocalizationView activeLocalizationView = DisplayUI::LocalizationView::Combined;
static uint32_t lastTouchMs = 0;
static bool needsFullRedraw = true;

static bool inside(const UITheme::Rect& r, int x, int y) {
    return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1;
}

static float headingToCompassDeg(float headingRad) { return LocMath::headingToCompassDeg(headingRad); }
static float headingDeltaCompassDeg(float deltaRad) { return LocMath::headingDeltaCompassDeg(deltaRad); }
static float mToIn(float m) { return LocMath::mToIn(m); }

static const char* safeStatus(const std::string& s, int maxChars = 34) {
    static char buf[48];
    if (s.empty()) {
        std::snprintf(buf, sizeof(buf), "Ready");
        return buf;
    }
    std::snprintf(buf, sizeof(buf), "%.*s", maxChars, s.c_str());
    return buf;
}

static bool isRedAlliance(Alliance alliance) {
    return alliance == Alliance::RED;
}

static uint32_t allianceAccent(Alliance alliance) {
    return isRedAlliance(alliance) ? UITheme::kRed : UITheme::kBlue;
}

static uint32_t allianceFill(Alliance alliance) {
    return isRedAlliance(alliance) ? UITheme::kRedDeep : UITheme::kBlueDeep;
}

static bool isSkillsAuton(Auton auton) {
    return auton == Auton::SKILLS;
}

static const char* routineMode(Auton auton) {
    return isSkillsAuton(auton) ? "Skills Run" : "Match Routine";
}

static const char* routineBadge(Auton auton) {
    return isSkillsAuton(auton) ? "SKILLS" : "MATCH";
}

static const char* driftLabel(float driftIn) {
    if (driftIn < 3.0f) return "Tight";
    if (driftIn < 6.0f) return "Nominal";
    return "Watch";
}

static uint32_t driftAccent(float driftIn) {
    if (driftIn < 3.0f) return UITheme::kGreen;
    if (driftIn < 6.0f) return UITheme::kAmber;
    return UITheme::kRed;
}

static void drawTab(const UITheme::Rect& r, const char* txt, bool active) {
    const uint32_t fill = active ? UITheme::kPanel : UITheme::kPanelMuted;
    UITheme::drawPanel(
        r,
        fill,
        active ? UITheme::kBorderStrong : UITheme::kBorder,
        0,
        false
    );
    UITheme::printCenteredfOn(pros::E_TEXT_SMALL, r, r.y0 + 6,
                              active ? UITheme::kText : UITheme::kTextMuted,
                              fill, "%s", txt);
}

static void drawMetricTile(const UITheme::Rect& r,
                           const char* label,
                           const char* value,
                           uint32_t accent,
                           uint32_t fill = UITheme::kPanelMuted) {
    UITheme::drawPanel(r, fill, accent, 0, false);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, r.x0 + 8, r.y0 + 8,
                          UITheme::kTextMuted, fill, "%s", label);
    UITheme::printTextfOn(pros::E_TEXT_MEDIUM, r.x0 + 8, r.y0 + 24,
                          UITheme::kText, fill, "%s", value);
}

static void drawTopBar() {
    UITheme::fillRect(UITheme::makeRect(0, 0, UITheme::kScreenW, UITheme::kTopBarH), UITheme::kBackgroundBand);
    UITheme::drawDividerH(0, UITheme::kScreenW - 1, UITheme::kTopBarH - 1, UITheme::kBorderStrong);

    UITheme::drawPanel(BRAND_CARD, UITheme::kPanelAlt, UITheme::kBorderStrong, UITheme::kAmber, false);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, BRAND_CARD.x0 + 8, BRAND_CARD.y0 + 3,
                          UITheme::kTextMuted, UITheme::kPanelAlt, "TEAM");
    UITheme::printTextfOn(pros::E_TEXT_MEDIUM, BRAND_CARD.x0 + 8, BRAND_CARD.y0 + 11,
                          UITheme::kText, UITheme::kPanelAlt, "69580A");

    drawTab(TAB_SELECT, "SELECT", activePage == Page::SELECT);
    drawTab(TAB_ODOM,   "LOCAL",  activePage == Page::LOCALIZATION);
    drawTab(TAB_PID,    "PID",    activePage == Page::PID);
    drawTab(TAB_PATH,   "PATH",   activePage == Page::PATH);
    drawTab(TAB_GPS,    "GPS",    activePage == Page::GPS);
}

static void drawLocalizationSubtabs() {
    auto drawLocalTab = [](const UITheme::Rect& r,
                           const char* label,
                           DisplayUI::LocalizationView view) {
        const bool active = activeLocalizationView == view;
        const uint32_t fill = active ? UITheme::kPanel : UITheme::kPanelMuted;
        UITheme::drawPanel(
            r,
            fill,
            active ? UITheme::kBorderStrong : UITheme::kBorder,
            0,
            false);
        UITheme::printCenteredfOn(pros::E_TEXT_SMALL, r, r.y0 + 6,
                                  active ? UITheme::kText : UITheme::kTextMuted,
                                  fill, "%s", label);
    };

    drawLocalTab(LOCAL_SUB_ODOM, "PURE ODOM", DisplayUI::LocalizationView::PureOdom);
    drawLocalTab(LOCAL_SUB_GPS, "PURE GPS", DisplayUI::LocalizationView::PureGps);
    drawLocalTab(LOCAL_SUB_MCL, "PURE MCL", DisplayUI::LocalizationView::PureMcl);
    drawLocalTab(LOCAL_SUB_COMBINED, "COMBINED", DisplayUI::LocalizationView::Combined);
}

static void drawSelectPage(const BrainScreen::RuntimeViewModel& vm) {
    const UITheme::Rect hero = UITheme::makeRect(12, UITheme::kContentY + 8, 272, 100);
    const UITheme::Rect nav = UITheme::makeRect(296, UITheme::kContentY + 8, 172, 100);
    const UITheme::Rect live = UITheme::makeRect(12, UITheme::kContentY + 118, 456, 85);

    const uint32_t allianceCol = allianceAccent(vm.selectedAlliance);
    const uint32_t allianceBg = allianceFill(vm.selectedAlliance);

    UITheme::drawPanel(hero, UITheme::kPanelAlt, UITheme::kBorderStrong, allianceCol);
    UITheme::drawPanel(nav, UITheme::kPanel, UITheme::kBorderStrong, UITheme::kAmber);
    UITheme::drawPanel(live, UITheme::kPanel, UITheme::kBorderStrong, UITheme::kBlue);

    UITheme::printTextfOn(pros::E_TEXT_SMALL, hero.x0 + 12, hero.y0 + 10,
                          UITheme::kTextMuted, UITheme::kPanelAlt, "MATCH SETUP");
    UITheme::drawChip(UITheme::makeRect(hero.x1 - 82, hero.y0 + 8, 70, 20),
                      vm.alliance.c_str(), allianceBg, allianceCol, UITheme::kText);

    UITheme::drawChip(RED_BTN, "RED",
                      0x00401A1A,
                      isRedAlliance(vm.selectedAlliance) ? UITheme::kRed : UITheme::kBorder,
                      UITheme::kText);
    UITheme::drawChip(BLUE_BTN, "BLUE",
                      0x0015223D,
                      isRedAlliance(vm.selectedAlliance) ? UITheme::kBorder : UITheme::kBlue,
                      UITheme::kText);

    UITheme::printTextfOn(pros::E_TEXT_LARGE, hero.x0 + 12, hero.y0 + 50,
                          UITheme::kText, UITheme::kPanelAlt, "%s", vm.auton.c_str());
    UITheme::printTextfOn(pros::E_TEXT_SMALL, hero.x0 + 12, hero.y1 - 16,
                          UITheme::kTextSoft, UITheme::kPanelAlt, "%s", routineMode(vm.selectedAuton));

    UITheme::printTextfOn(pros::E_TEXT_SMALL, nav.x0 + 12, nav.y0 + 10,
                          UITheme::kTextMuted, UITheme::kPanel, "AUTON NAV");
    UITheme::drawChip(PREV_BTN, "PREV", UITheme::kPanelMuted, UITheme::kBorderStrong, UITheme::kText);
    UITheme::drawChip(NEXT_BTN, "NEXT", UITheme::kPanelMuted, UITheme::kBorderStrong, UITheme::kText);
    UITheme::drawChip(SKILLS_BTN, "QUICK: SKILLS",
                      isSkillsAuton(vm.selectedAuton) ? UITheme::kTealDeep : UITheme::kPanelMuted,
                      isSkillsAuton(vm.selectedAuton) ? UITheme::kTeal : UITheme::kBorderStrong,
                      UITheme::kText);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, nav.x0 + 12, nav.y1 - 16,
                          UITheme::kTextSoft, UITheme::kPanel, "Tap tabs for diagnostics");

    UITheme::printTextfOn(pros::E_TEXT_SMALL, live.x0 + 12, live.y0 + 10,
                          UITheme::kTextMuted, UITheme::kPanel, "LIVE POSE");

    char xBuf[24];
    char yBuf[24];
    char hBuf[24];
    std::snprintf(xBuf, sizeof(xBuf), "%+.1f in", mToIn(vm.combinedPose.x()));
    std::snprintf(yBuf, sizeof(yBuf), "%+.1f in", mToIn(vm.combinedPose.y()));
    std::snprintf(hBuf, sizeof(hBuf), "%.1f deg", headingToCompassDeg(vm.combinedPose.z()));

    drawMetricTile(UITheme::makeRect(live.x0 + 12, live.y0 + 24, 134, 38), "X", xBuf, UITheme::kAmber);
    drawMetricTile(UITheme::makeRect(live.x0 + 160, live.y0 + 24, 134, 38), "Y", yBuf, UITheme::kBlue);
    drawMetricTile(UITheme::makeRect(live.x0 + 308, live.y0 + 24, 136, 38), "HEADING", hBuf, UITheme::kTeal);

    UITheme::drawDividerH(live.x0 + 12, live.x1 - 12, live.y1 - 22, UITheme::kBorder);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, live.x0 + 12, live.y1 - 14,
                          UITheme::kText, UITheme::kPanel, "%s", safeStatus(vm.status, 42));
}

static void drawPidPage(const BrainScreen::RuntimeViewModel& vm) {
    double errIn = static_cast<double>(vm.combinedPose.x() - vm.pureOdomPose.x()) * 39.3701;
    double headingErrDeg = static_cast<double>(
        headingDeltaCompassDeg(vm.combinedPose.z() - vm.pureOdomPose.z()));

    if (!std::isfinite(errIn)) errIn = 0.0;
    if (!std::isfinite(headingErrDeg)) headingErrDeg = 0.0;

    PIDGraphUI::addSample(errIn, headingErrDeg);

    const UITheme::Rect errCard = UITheme::makeRect(12, UITheme::kContentY + 8, 144, 48);
    const UITheme::Rect headCard = UITheme::makeRect(168, UITheme::kContentY + 8, 144, 48);
    const UITheme::Rect modeCard = UITheme::makeRect(324, UITheme::kContentY + 8, 144, 48);
    const UITheme::Rect graphPanel = UITheme::makeRect(12, UITheme::kContentY + 66, 456, 162);
    const UITheme::Rect graphArea = UITheme::makeRect(graphPanel.x0 + 10, graphPanel.y0 + 30, 436, 122);

    char errBuf[24];
    char headBuf[24];
    std::snprintf(errBuf, sizeof(errBuf), "%+.2f in", errIn);
    std::snprintf(headBuf, sizeof(headBuf), "%+.2f deg", headingErrDeg);

    drawMetricTile(errCard, "POSITION DELTA", errBuf, UITheme::kRed);
    drawMetricTile(headCard, "HEADING DELTA", headBuf, UITheme::kBlue);
    drawMetricTile(modeCard, "MODE", "Tracking Proxy", UITheme::kAmber);

    UITheme::drawPanel(graphPanel, UITheme::kPanel, UITheme::kBorderStrong, 0);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, graphPanel.x0 + 12, graphPanel.y0 + 8,
                          UITheme::kTextMuted, UITheme::kPanel, "PID HISTORY");
    UITheme::printTextfOn(pros::E_TEXT_SMALL, graphPanel.x1 - 120, graphPanel.y0 + 8,
                          UITheme::kTextSoft, UITheme::kPanel, "Red=pos  Blue=head");
    PIDGraphUI::draw(graphArea.x0, graphArea.y0, graphArea.x1, graphArea.y1);
}

static void drawPathPage(const BrainScreen::RuntimeViewModel& vm) {
    const UITheme::Rect route = UITheme::makeRect(12, UITheme::kContentY + 8, 278, 96);
    const UITheme::Rect brief = UITheme::makeRect(300, UITheme::kContentY + 8, 168, 96);
    const UITheme::Rect prep = UITheme::makeRect(12, UITheme::kContentY + 114, 456, 89);

    UITheme::drawPanel(route, UITheme::kPanelAlt, UITheme::kBorderStrong, UITheme::kAmber);
    UITheme::drawPanel(brief, UITheme::kPanel, UITheme::kBorderStrong, 0);
    UITheme::drawPanel(prep, UITheme::kPanel, UITheme::kBorderStrong, UITheme::kBlue);

    UITheme::printTextfOn(pros::E_TEXT_SMALL, route.x0 + 12, route.y0 + 10,
                          UITheme::kTextMuted, UITheme::kPanelAlt, "CURRENT ROUTINE");
    UITheme::printTextfOn(pros::E_TEXT_LARGE, route.x0 + 12, route.y0 + 42,
                          UITheme::kText, UITheme::kPanelAlt, "%s", vm.auton.c_str());
    UITheme::drawChip(UITheme::makeRect(route.x1 - 96, route.y0 + 8, 84, 20),
                      routineBadge(vm.selectedAuton), UITheme::kPanelMuted, UITheme::kAmber, UITheme::kText);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, route.x0 + 12, route.y1 - 16,
                          UITheme::kTextSoft, UITheme::kPanelAlt, "Alliance %s", vm.alliance.c_str());

    UITheme::printTextfOn(pros::E_TEXT_SMALL, brief.x0 + 12, brief.y0 + 10,
                          UITheme::kTextMuted, UITheme::kPanel, "MATCH BRIEF");
    UITheme::printTextfOn(pros::E_TEXT_MEDIUM, brief.x0 + 12, brief.y0 + 32,
                          UITheme::kText, UITheme::kPanel, "Status");
    UITheme::printTextfOn(pros::E_TEXT_SMALL, brief.x0 + 12, brief.y0 + 50,
                          UITheme::kText, UITheme::kPanel, "%s", safeStatus(vm.status, 22));
    UITheme::printTextfOn(pros::E_TEXT_SMALL, brief.x0 + 12, brief.y1 - 16,
                          UITheme::kTextSoft, UITheme::kPanel, "Adjust on SELECT tab");

    char xBuf[24];
    char yBuf[24];
    char hBuf[24];
    std::snprintf(xBuf, sizeof(xBuf), "%+.1f in", mToIn(vm.combinedPose.x()));
    std::snprintf(yBuf, sizeof(yBuf), "%+.1f in", mToIn(vm.combinedPose.y()));
    std::snprintf(hBuf, sizeof(hBuf), "%.1f deg", headingToCompassDeg(vm.combinedPose.z()));

    UITheme::printTextfOn(pros::E_TEXT_SMALL, prep.x0 + 12, prep.y0 + 10,
                          UITheme::kTextMuted, UITheme::kPanel, "ROUTING POSTURE");
    drawMetricTile(UITheme::makeRect(prep.x0 + 12, prep.y0 + 26, 136, 42), "POSE X", xBuf, UITheme::kAmber);
    drawMetricTile(UITheme::makeRect(prep.x0 + 160, prep.y0 + 26, 136, 42), "POSE Y", yBuf, UITheme::kBlue);
    drawMetricTile(UITheme::makeRect(prep.x0 + 308, prep.y0 + 26, 136, 42), "HEADING", hBuf, UITheme::kGreen);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, prep.x0 + 12, prep.y1 - 14,
                          UITheme::kTextSoft, UITheme::kPanel,
                          "Use LOCAL, PID, and GPS tabs for live verification.");
}

static void drawGpsPage(const BrainScreen::RuntimeViewModel& vm) {
    const UITheme::Rect raw = UITheme::makeRect(12, UITheme::kContentY + 8, 216, 106);
    const UITheme::Rect delta = UITheme::makeRect(240, UITheme::kContentY + 8, 228, 106);
    const UITheme::Rect fused = UITheme::makeRect(12, UITheme::kContentY + 124, 456, 79);

    const bool gpsValid = vm.gpsPoseValid;
    const float dx = gpsValid ? (vm.combinedPose.x() - vm.gpsPose.x()) * 39.3701f : 0.0f;
    const float dy = gpsValid ? (vm.combinedPose.y() - vm.gpsPose.y()) * 39.3701f : 0.0f;
    const float dh = gpsValid ? headingDeltaCompassDeg(vm.combinedPose.z() - vm.gpsPose.z()) : 0.0f;
    const float drift = gpsValid ? std::sqrt(dx * dx + dy * dy) : 0.0f;
    char driftBuf[16];
    if (gpsValid) {
        std::snprintf(driftBuf, sizeof(driftBuf), "%.1f in", drift);
    } else {
        std::snprintf(driftBuf, sizeof(driftBuf), "--");
    }

    UITheme::drawPanel(raw, UITheme::kPanelAlt, UITheme::kBorderStrong, UITheme::kBlue);
    UITheme::drawPanel(delta, UITheme::kPanel, UITheme::kBorderStrong, driftAccent(drift));
    UITheme::drawPanel(fused, UITheme::kPanel, UITheme::kBorderStrong, 0);

    UITheme::printTextfOn(pros::E_TEXT_SMALL, raw.x0 + 12, raw.y0 + 10,
                          UITheme::kTextMuted, UITheme::kPanelAlt, "RAW GPS");
    if (gpsValid) {
        UITheme::printTextfOn(pros::E_TEXT_MEDIUM, raw.x0 + 12, raw.y0 + 34,
                              UITheme::kText, UITheme::kPanelAlt, "X  %+.1f in", mToIn(vm.gpsPose.x()));
        UITheme::printTextfOn(pros::E_TEXT_MEDIUM, raw.x0 + 12, raw.y0 + 54,
                              UITheme::kText, UITheme::kPanelAlt, "Y  %+.1f in", mToIn(vm.gpsPose.y()));
        UITheme::printTextfOn(pros::E_TEXT_MEDIUM, raw.x0 + 12, raw.y0 + 74,
                              UITheme::kText, UITheme::kPanelAlt, "H  %.1f deg", headingToCompassDeg(vm.gpsPose.z()));
    } else {
        UITheme::printTextfOn(pros::E_TEXT_MEDIUM, raw.x0 + 12, raw.y0 + 42,
                              UITheme::kText, UITheme::kPanelAlt, "No valid GPS lock");
        UITheme::printTextfOn(pros::E_TEXT_SMALL, raw.x0 + 12, raw.y0 + 68,
                              UITheme::kTextSoft, UITheme::kPanelAlt, "Use LOCAL > PURE GPS");
    }

    UITheme::printTextfOn(pros::E_TEXT_SMALL, delta.x0 + 12, delta.y0 + 10,
                          UITheme::kTextMuted, UITheme::kPanel, "FUSION DELTA");
    if (gpsValid) {
        UITheme::drawChip(UITheme::makeRect(delta.x1 - 86, delta.y0 + 8, 74, 20),
                          driftLabel(drift), UITheme::kPanelMuted, driftAccent(drift), UITheme::kText);
        UITheme::printTextfOn(pros::E_TEXT_MEDIUM, delta.x0 + 12, delta.y0 + 34,
                              UITheme::kText, UITheme::kPanel, "dX %+.1f in", dx);
        UITheme::printTextfOn(pros::E_TEXT_MEDIUM, delta.x0 + 12, delta.y0 + 54,
                              UITheme::kText, UITheme::kPanel, "dY %+.1f in", dy);
        UITheme::printTextfOn(pros::E_TEXT_MEDIUM, delta.x0 + 12, delta.y0 + 74,
                              UITheme::kText, UITheme::kPanel, "dH %+.1f deg", dh);
    } else {
        UITheme::printTextfOn(pros::E_TEXT_MEDIUM, delta.x0 + 12, delta.y0 + 42,
                              UITheme::kText, UITheme::kPanel, "Waiting for usable GPS");
        UITheme::printTextfOn(pros::E_TEXT_SMALL, delta.x0 + 12, delta.y0 + 68,
                              UITheme::kTextSoft, UITheme::kPanel, "No drift metrics available");
    }

    UITheme::printTextfOn(pros::E_TEXT_SMALL, fused.x0 + 12, fused.y0 + 10,
                          UITheme::kTextMuted, UITheme::kPanel, "COMBINED SOLUTION");
    UITheme::printTextfOn(pros::E_TEXT_MEDIUM, fused.x0 + 12, fused.y0 + 30,
                          UITheme::kText, UITheme::kPanel, "X %+.1f in   Y %+.1f in",
                          mToIn(vm.combinedPose.x()), mToIn(vm.combinedPose.y()));
    UITheme::printTextfOn(pros::E_TEXT_MEDIUM, fused.x0 + 12, fused.y0 + 50,
                          UITheme::kText, UITheme::kPanel, "H %.1f deg   Drift %s",
                          headingToCompassDeg(vm.combinedPose.z()),
                          driftBuf);
    UITheme::printTextfOn(pros::E_TEXT_SMALL, fused.x0 + 12, fused.y1 - 14,
                          UITheme::kTextSoft, UITheme::kPanel, "%s", safeStatus(vm.status, 52));
}

static void handleTouch() {
    const auto t = pros::screen::touch_status();
    if (t.touch_status != pros::E_TOUCH_PRESSED &&
        t.touch_status != pros::E_TOUCH_HELD) {
        return;
    }

    const uint32_t now = pros::millis();
    if (now - lastTouchMs < 140) return;
    lastTouchMs = now;

    const int x = t.x;
    const int y = t.y;

    if (y <= UITheme::kTopBarH) {
        if (inside(TAB_SELECT, x, y)) { activePage = Page::SELECT; return; }
        if (inside(TAB_ODOM,   x, y)) { activePage = Page::LOCALIZATION; return; }
        if (inside(TAB_PID,    x, y)) { activePage = Page::PID;    return; }
        if (inside(TAB_PATH,   x, y)) { activePage = Page::PATH;   return; }
        if (inside(TAB_GPS,    x, y)) { activePage = Page::GPS;    return; }
    }

    if (activePage == Page::LOCALIZATION) {
        auto setView = [&](DisplayUI::LocalizationView view) {
            if (activeLocalizationView != view) {
                activeLocalizationView = view;
                DisplayUI::clearTrail();
            }
        };

        if (inside(LOCAL_SUB_ODOM, x, y))     { setView(DisplayUI::LocalizationView::PureOdom); return; }
        if (inside(LOCAL_SUB_GPS, x, y))      { setView(DisplayUI::LocalizationView::PureGps); return; }
        if (inside(LOCAL_SUB_MCL, x, y))      { setView(DisplayUI::LocalizationView::PureMcl); return; }
        if (inside(LOCAL_SUB_COMBINED, x, y)) { setView(DisplayUI::LocalizationView::Combined); return; }
    }

    if (activePage == Page::SELECT) {
        if (inside(RED_BTN, x, y))    { AutonSelector::selectAlliance(Alliance::RED);  return; }
        if (inside(BLUE_BTN, x, y))   { AutonSelector::selectAlliance(Alliance::BLUE); return; }
        if (inside(SKILLS_BTN, x, y)) { AutonSelector::selectAuton(Auton::SKILLS);     return; }
        if (inside(PREV_BTN, x, y))   { AutonSelector::prevAuton();                     return; }
        if (inside(NEXT_BTN, x, y))   { AutonSelector::nextAuton();                     return; }
    }
}

}  // namespace

void init() {
    pros::screen::set_eraser(UITheme::kBackground);
    pros::screen::erase();
    DisplayUI::init();
    PIDGraphUI::init();
    activePage = Page::SELECT;
    prevRenderedPage = Page::SELECT;
    activeLocalizationView = DisplayUI::LocalizationView::Combined;
    needsFullRedraw = true;
}

void render(const BrainScreen::RuntimeViewModel& vm) {
    handleTouch();

    if (activePage != prevRenderedPage) {
        prevRenderedPage = activePage;
        needsFullRedraw = true;
    }

    if (needsFullRedraw) {
        pros::screen::set_eraser(UITheme::kBackground);
        pros::screen::erase();
        UITheme::drawContentBackdrop();
        needsFullRedraw = false;
    }

    switch (activePage) {
        case Page::SELECT:
            drawSelectPage(vm);
            break;
        case Page::LOCALIZATION:
            DisplayUI::update(vm, activeLocalizationView);
            drawLocalizationSubtabs();
            break;
        case Page::PID:
            drawPidPage(vm);
            break;
        case Page::PATH:
            drawPathPage(vm);
            break;
        case Page::GPS:
            drawGpsPage(vm);
            break;
    }

    drawTopBar();
}

}  // namespace ScreenManagerUI
