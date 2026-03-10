#!/usr/bin/env python3
"""
uploadAllAutons.py
Writes include/auton.h for each autonomous selection, then compiles and
uploads to the V5 brain via `pros mu --slot N --name "name"`.

Usage:
    python3 uploadAllAutons.py          # uploads all competition autons to slots 1-3
    python3 uploadAllAutons.py --dry    # print commands without executing
"""
import os
import subprocess
import sys

AUTON_HEADER = "include/auton.h"

# (slot, auton enum value, display name, alliance)
AUTONS = [
    (1, "Auton::NEGATIVE_1",  "Negative 1",  "Alliance::RED"),
    (2, "Auton::POSITIVE_1",  "Positive 1",  "Alliance::RED"),
    (3, "Auton::SKILLS",      "Skills",       "Alliance::RED"),
]

HEADER_TEMPLATE = """\
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
#define SELECTED_AUTON {auton}
#endif

#ifndef DEFAULT_ALLIANCE
#define DEFAULT_ALLIANCE {alliance}
#endif

inline constexpr Auton DEFAULT_AUTON_SELECTION = SELECTED_AUTON;
inline constexpr Alliance DEFAULT_ALLIANCE_SELECTION = DEFAULT_ALLIANCE;
"""


def write_header(auton: str, alliance: str) -> None:
    with open(AUTON_HEADER, "w") as f:
        f.write(HEADER_TEMPLATE.format(auton=auton, alliance=alliance))


def main() -> None:
    dry_run = "--dry" in sys.argv

    for slot, auton, name, alliance in AUTONS:
        print(f"\n{'='*60}")
        print(f"  Slot {slot}: {name}  ({auton}, {alliance})")
        print(f"{'='*60}")

        write_header(auton, alliance)

        cmd = ["pros", "mu", "--slot", str(slot), "--name", name, "--no-analytics"]
        print(f"  > {' '.join(cmd)}")

        if not dry_run:
            result = subprocess.run(cmd)
            if result.returncode != 0:
                print(f"  *** Upload failed for slot {slot} ***")
                sys.exit(result.returncode)

    # Restore default header
    write_header("Auton::NEGATIVE_1", "Alliance::RED")
    print("\nDone — all autons uploaded.")


if __name__ == "__main__":
    main()
