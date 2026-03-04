/**
 * @file autonSelector.h
 * Brain-screen autonomous selector using LLEMU (3-button LCD).
 *
 * Layout:
 *   Line 0:  "2654E Echo"
 *   Line 1:  "<< AUTON_NAME >>"
 *   Line 2:  "Alliance: RED / BLUE"
 *   Line 3:  (blank or status)
 *   Line 4:  "Pose: (x, y)"
 *   Line 5:  "Heading: θ°"
 *   Line 6:  (status)
 *   Line 7:  "[< Prev]  [Alliance]  [Next >]"
 *
 * Left  button → previous auton
 * Centre button → toggle alliance
 * Right button → next auton
 */
#pragma once

#include "autonomous/autons.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <functional>

class AutonSelector {
public:
    /// Initialise the selector and register LLEMU button callbacks.
    static void init();

    /// Cycle to the next / previous autonomous routine.
    static void nextAuton();
    static void prevAuton();

    /// Flip the alliance between RED and BLUE.
    static void toggleAlliance();
    static void selectAlliance(Alliance alliance);
    static void selectAuton(Auton auton);

    /// Read the current selection.
    static Auton    getAuton();
    static Alliance getAlliance();

    /// Human-readable strings for the current selection.
    static std::string getAutonStr();
    static std::string getAllianceStr();

    /// Refresh all 8 LCD lines.
    /// @param pose  current robot pose (x, y, θ in radians)
    /// @param status optional one-line status message for line 6
    static void render(const Eigen::Vector3f& pose,
                       const std::string& status = "");

    /// Render a startup/init screen with a loading bar.
    /// @param progress 0.0–1.0
    /// @param status   optional one-line status message
    static void renderInit(float progress,
                           const std::string& status = "");

private:
    static inline const std::vector<Auton> s_autonList = {
        Auton::NEGATIVE_1,
        Auton::NEGATIVE_2,
        Auton::POSITIVE_1,
        Auton::POSITIVE_2,
        Auton::SKILLS,
        Auton::NONE
    };

    static inline int      s_index    = 0;
    static inline Alliance s_alliance = Alliance::RED;
};
