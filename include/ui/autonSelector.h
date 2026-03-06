/**
 * @file autonSelector.h
 * Brain-screen autonomous selector for the touch UI.
 */
#pragma once

#include "autonomous/autons.h"
#include "Eigen/Dense"
#include <string>

class AutonSelector {
public:
    /// Initialise selector state before the touch UI begins polling it.
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

    /// Render the runtime screen model directly.
    /// @param pose  current robot pose (x, y, θ in radians)
    /// @param status optional one-line status message
    static void render(const Eigen::Vector3f& pose,
                       const std::string& status = "");

    /// Render a startup/init screen with a loading bar.
    /// @param progress 0.0–1.0
    /// @param status   optional one-line status message
    static void renderInit(float progress,
                           const std::string& status = "");

private:
    static inline int      s_index    = 0;
    static inline Alliance s_alliance = Alliance::RED;
};
