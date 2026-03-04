/**
 * @file solenoids.cpp
 * Solenoid / pneumatics subsystem implementation — 4 outputs.
 */
#include "subsystems/solenoids.h"
#include "config.h"

Solenoids::Solenoids()
    : m_select1(CONFIG::SELECT1_PORT)
    , m_select2(CONFIG::SELECT2_PORT)
    , m_tongue(CONFIG::TONGUE_PORT)
    , m_wing(CONFIG::WING_PORT)
{}

void Solenoids::periodic() {
    // Nothing needed — outputs are latched by the ADI
}

void Solenoids::setSelect1(bool on) { m_stateSelect1 = on; m_select1.set_value(on ? 1 : 0); }
void Solenoids::setSelect2(bool on) { m_stateSelect2 = on; m_select2.set_value(on ? 1 : 0); }
void Solenoids::setTongue(bool on)  { m_stateTongue  = on; m_tongue.set_value(on ? 1 : 0);  }
void Solenoids::setWing(bool on)    { m_stateWing    = on; m_wing.set_value(on ? 1 : 0);    }

void Solenoids::toggleSelect1() { setSelect1(!m_stateSelect1); }
void Solenoids::toggleSelect2() { setSelect2(!m_stateSelect2); }
void Solenoids::toggleTongue()  { setTongue(!m_stateTongue);   }
void Solenoids::toggleWing()    { setWing(!m_stateWing);       }
