/**
 * @file solenoids.h
 * Plain pneumatics aggregate — four ADI digital outputs:
 *   SELECT1 (A), SELECT2 (B), TONGUE (C), WING (D).
 */
#pragma once

#include "config.h"
#include "pros/adi.hpp"

struct Solenoids {
    Solenoids()
        : select1(CONFIG::SELECT1_PORT)
        , select2(CONFIG::SELECT2_PORT)
        , tongue(CONFIG::TONGUE_PORT)
        , wing(CONFIG::WING_PORT) {}

    pros::adi::DigitalOut select1;
    pros::adi::DigitalOut select2;
    pros::adi::DigitalOut tongue;
    pros::adi::DigitalOut wing;

    bool select1State = false;
    bool select2State = false;
    bool tongueState  = false;
    bool wingState    = false;
};
