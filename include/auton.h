/**
 * @file auton.h
 * Generated header that selects the active autonomous routine.
 *
 * In production this file is overwritten by uploadAllAutons.py before
 * each firmware slot upload.
 */
#pragma once

#include "autonomous/autons.h"

// ── Active auton (change per-slot or via uploadAllAutons.py) ────────────────

#ifndef SELECTED_AUTON
#define SELECTED_AUTON Auton::NEGATIVE_1
#endif

#ifndef DEFAULT_ALLIANCE
#define DEFAULT_ALLIANCE Alliance::RED
#endif

inline constexpr Auton DEFAULT_AUTON_SELECTION = SELECTED_AUTON;
inline constexpr Alliance DEFAULT_ALLIANCE_SELECTION = DEFAULT_ALLIANCE;
