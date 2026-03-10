/**
 * @file intakes.cpp
 * Intake subsystem implementation — OkapiLib MotorGroup wrapper.
 */
#include "subsystems/intakes.h"
#include "config.h"
#include <cmath>

Intakes::Intakes()
    : m_motors({okapi::Motor(CONFIG::INTAKE_PORTS[0]),
                okapi::Motor(CONFIG::INTAKE_PORTS[1])}) {
    m_motors.setGearing(okapi::AbstractMotor::gearset::green);
    m_motors.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
}

void Intakes::periodic() {}

void Intakes::spin(int voltage) {
    // OkapiLib moveVoltage takes millivolts [-12000, 12000]
    int mv = std::clamp(voltage, -127, 127) * 12000 / 127;
    m_motors.moveVoltage(mv);
}

void Intakes::stop() {
    m_motors.moveVoltage(0);
}

bool Intakes::isStalled() const {
    // OkapiLib MotorGroup getCurrentDraw() returns average mA as int32_t
    // Cast away const since OkapiLib's getCurrentDraw() is non-const
    auto& motors = const_cast<okapi::MotorGroup&>(m_motors);
    return motors.getCurrentDraw() > CONFIG::INTAKE_STALL_CURRENT_mA;
}

float Intakes::getCurrent() const {
    auto& motors = const_cast<okapi::MotorGroup&>(m_motors);
    return static_cast<float>(motors.getCurrentDraw());
}
