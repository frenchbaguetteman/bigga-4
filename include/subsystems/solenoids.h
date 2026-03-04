/**
 * @file solenoids.h
 * Pneumatics subsystem — four ADI digital outputs:
 *   SELECT1 (A), SELECT2 (B), TONGUE (C), WING (D).
 */
#pragma once

#include "command/subsystem.h"
#include "pros/adi.hpp"

class Solenoids : public Subsystem {
public:
    Solenoids();

    void periodic() override;

    // ── Individual control ──────────────────────────────────────────────
    void setSelect1(bool on);
    bool getSelect1() const { return m_stateSelect1; }
    void toggleSelect1();

    void setSelect2(bool on);
    bool getSelect2() const { return m_stateSelect2; }
    void toggleSelect2();

    void setTongue(bool on);
    bool getTongue() const { return m_stateTongue; }
    void toggleTongue();

    void setWing(bool on);
    bool getWing() const { return m_stateWing; }
    void toggleWing();

private:
    pros::adi::DigitalOut m_select1;
    pros::adi::DigitalOut m_select2;
    pros::adi::DigitalOut m_tongue;
    pros::adi::DigitalOut m_wing;

    bool m_stateSelect1 = false;
    bool m_stateSelect2 = false;
    bool m_stateTongue  = false;
    bool m_stateWing    = false;
};
