/**
 * @file intakes.cpp
 * Intake subsystem implementation — linked motor pair.
 */
#include "subsystems/intakes.h"
#include "config.h"
#include <cmath>

Intakes::Intakes()
    : m_motors(CONFIG::INTAKE_PORTS) {
    m_motors.set_gearing(pros::E_MOTOR_GEAR_GREEN);  // 200 RPM
    m_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void Intakes::periodic() {
    // Could add jam detection logic here
}

void Intakes::spin(int voltage) {
    m_motors.move(std::clamp(voltage, -127, 127));
}

void Intakes::spinVelocity(int rpm) {
    m_motors.move_velocity(rpm);
}

void Intakes::stop() {
    m_motors.brake();
}

bool Intakes::isStalled() const {
    // Simple current-spike detection (> 2500 mA avg = likely stalled)
    return m_motors.get_current_draw() > 2500;
}

float Intakes::getCurrent() const {
    return static_cast<float>(m_motors.get_current_draw());
}
