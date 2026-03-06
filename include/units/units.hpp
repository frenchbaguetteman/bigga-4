/**
 * @file units.hpp
 * Lightweight quantity system with an Okapi-style API.
 *
 * Quantities store SI base values internally and expose:
 * - user-defined literals like `24_in`, `90_deg`, `250_ms`
 * - `convert(unit)` for explicit extraction into a requested unit
 * - compile-time dimension checks for quantity arithmetic
 */
#pragma once

#include <cmath>

namespace units {

template <int LengthPower, int TimePower, int AnglePower>
class Quantity {
public:
    constexpr Quantity() : m_value(0.0f) {}
    constexpr explicit Quantity(float value) : m_value(value) {}

    constexpr float getValue() const { return m_value; }

    template <int OtherLengthPower, int OtherTimePower, int OtherAnglePower>
    constexpr float convert(Quantity<OtherLengthPower, OtherTimePower, OtherAnglePower> unit) const {
        static_assert(LengthPower == OtherLengthPower &&
                      TimePower == OtherTimePower &&
                      AnglePower == OtherAnglePower,
                      "Quantity::convert requires matching dimensions");
        return m_value / unit.m_value;
    }

    constexpr Quantity operator+(Quantity other) const {
        return Quantity(m_value + other.m_value);
    }

    constexpr Quantity operator-(Quantity other) const {
        return Quantity(m_value - other.m_value);
    }

    constexpr Quantity operator-() const {
        return Quantity(-m_value);
    }

    constexpr Quantity operator*(float scalar) const {
        return Quantity(m_value * scalar);
    }

    constexpr Quantity operator/(float scalar) const {
        return Quantity(m_value / scalar);
    }

    constexpr bool operator<(Quantity other) const { return m_value < other.m_value; }
    constexpr bool operator>(Quantity other) const { return m_value > other.m_value; }
    constexpr bool operator<=(Quantity other) const { return m_value <= other.m_value; }
    constexpr bool operator>=(Quantity other) const { return m_value >= other.m_value; }
    constexpr bool operator==(Quantity other) const { return m_value == other.m_value; }
    constexpr bool operator!=(Quantity other) const { return m_value != other.m_value; }

    constexpr Quantity& operator+=(Quantity other) {
        m_value += other.m_value;
        return *this;
    }

    constexpr Quantity& operator-=(Quantity other) {
        m_value -= other.m_value;
        return *this;
    }

    constexpr Quantity& operator*=(float scalar) {
        m_value *= scalar;
        return *this;
    }

    constexpr Quantity& operator/=(float scalar) {
        m_value /= scalar;
        return *this;
    }

private:
    float m_value;

    template <int L1, int T1, int A1, int L2, int T2, int A2>
    friend constexpr Quantity<L1 + L2, T1 + T2, A1 + A2>
    operator*(Quantity<L1, T1, A1> lhs, Quantity<L2, T2, A2> rhs);

    template <int L1, int T1, int A1, int L2, int T2, int A2>
    friend constexpr Quantity<L1 - L2, T1 - T2, A1 - A2>
    operator/(Quantity<L1, T1, A1> lhs, Quantity<L2, T2, A2> rhs);
};

template <int LengthPower, int TimePower, int AnglePower>
constexpr Quantity<LengthPower, TimePower, AnglePower>
operator*(float scalar, Quantity<LengthPower, TimePower, AnglePower> quantity) {
    return Quantity<LengthPower, TimePower, AnglePower>(scalar * quantity.getValue());
}

template <int LengthPower, int TimePower, int AnglePower>
constexpr Quantity<LengthPower, TimePower, AnglePower>
operator*(double scalar, Quantity<LengthPower, TimePower, AnglePower> quantity) {
    return Quantity<LengthPower, TimePower, AnglePower>(
        static_cast<float>(scalar) * quantity.getValue());
}

template <int L1, int T1, int A1, int L2, int T2, int A2>
constexpr Quantity<L1 + L2, T1 + T2, A1 + A2>
operator*(Quantity<L1, T1, A1> lhs, Quantity<L2, T2, A2> rhs) {
    return Quantity<L1 + L2, T1 + T2, A1 + A2>(lhs.m_value * rhs.m_value);
}

template <int L1, int T1, int A1, int L2, int T2, int A2>
constexpr Quantity<L1 - L2, T1 - T2, A1 - A2>
operator/(Quantity<L1, T1, A1> lhs, Quantity<L2, T2, A2> rhs) {
    return Quantity<L1 - L2, T1 - T2, A1 - A2>(lhs.m_value / rhs.m_value);
}

using QNumber = Quantity<0, 0, 0>;
using QLength = Quantity<1, 0, 0>;
using QTime = Quantity<0, 1, 0>;
using QAngle = Quantity<0, 0, 1>;
using QSpeed = Quantity<1, -1, 0>;
using QAcceleration = Quantity<1, -2, 0>;
using QAngularSpeed = Quantity<0, -1, 1>;
using QAngularAcceleration = Quantity<0, -2, 1>;
using QJerk = Quantity<1, -3, 0>;
using QCurvature = Quantity<-1, 0, 0>;

// Backward-compatible aliases for the previous repo vocabulary.
using Dimensionless = QNumber;
using QVelocity = QSpeed;
using QAngularVelocity = QAngularSpeed;

constexpr float kPi = 3.14159265358979323846f;

constexpr QLength meter{1.0f};
constexpr QLength metre = meter;
constexpr QLength millimeter{0.001f};
constexpr QLength millimetre = millimeter;
constexpr QLength centimeter{0.01f};
constexpr QLength centimetre = centimeter;
constexpr QLength inch{0.0254f};
constexpr QLength foot{0.3048f};
constexpr QLength tile{0.6096f};

constexpr QTime second{1.0f};
constexpr QTime millisecond{0.001f};
constexpr QTime minute{60.0f};

constexpr QAngle radian{1.0f};
constexpr QAngle degree{kPi / 180.0f};

constexpr QSpeed mps{1.0f};
constexpr QSpeed ips = inch / second;
constexpr QAcceleration mps2{1.0f};
constexpr QAcceleration ips2 = inch / (second * second);
constexpr QAngularSpeed radps{1.0f};
constexpr QAngularSpeed dps = degree / second;

template <int LengthPower, int TimePower, int AnglePower>
constexpr Quantity<2 * LengthPower, 2 * TimePower, 2 * AnglePower>
qsquare(Quantity<LengthPower, TimePower, AnglePower> quantity) {
    return Quantity<2 * LengthPower, 2 * TimePower, 2 * AnglePower>(
        quantity.getValue() * quantity.getValue());
}

template <int LengthPower, int TimePower, int AnglePower>
constexpr Quantity<LengthPower / 2, TimePower / 2, AnglePower / 2>
qsqrt(Quantity<LengthPower, TimePower, AnglePower> quantity) {
    return Quantity<LengthPower / 2, TimePower / 2, AnglePower / 2>(
        std::sqrt(quantity.getValue()));
}

template <int LengthPower, int TimePower, int AnglePower>
constexpr Quantity<LengthPower, TimePower, AnglePower>
qabs(Quantity<LengthPower, TimePower, AnglePower> quantity) {
    return Quantity<LengthPower, TimePower, AnglePower>(
        std::fabs(quantity.getValue()));
}

// Backward-compatible helper names.
template <int LengthPower, int TimePower, int AnglePower>
constexpr auto Qsq(Quantity<LengthPower, TimePower, AnglePower> quantity) {
    return qsquare(quantity);
}

template <int LengthPower, int TimePower, int AnglePower>
constexpr auto Qsqrt(Quantity<LengthPower, TimePower, AnglePower> quantity) {
    return qsqrt(quantity);
}

template <int LengthPower, int TimePower, int AnglePower>
constexpr auto Qabs(Quantity<LengthPower, TimePower, AnglePower> quantity) {
    return qabs(quantity);
}

namespace literals {

constexpr QLength operator""_m(long double value) {
    return QLength(static_cast<float>(value));
}

constexpr QLength operator""_m(unsigned long long value) {
    return QLength(static_cast<float>(value));
}

constexpr QLength operator""_cm(long double value) {
    return QLength(static_cast<float>(value) * centimeter.getValue());
}

constexpr QLength operator""_cm(unsigned long long value) {
    return QLength(static_cast<float>(value) * centimeter.getValue());
}

constexpr QLength operator""_mm(long double value) {
    return QLength(static_cast<float>(value) * millimeter.getValue());
}

constexpr QLength operator""_mm(unsigned long long value) {
    return QLength(static_cast<float>(value) * millimeter.getValue());
}

constexpr QLength operator""_in(long double value) {
    return QLength(static_cast<float>(value) * inch.getValue());
}

constexpr QLength operator""_in(unsigned long long value) {
    return QLength(static_cast<float>(value) * inch.getValue());
}

constexpr QLength operator""_ft(long double value) {
    return QLength(static_cast<float>(value) * foot.getValue());
}

constexpr QLength operator""_ft(unsigned long long value) {
    return QLength(static_cast<float>(value) * foot.getValue());
}

constexpr QLength operator""_tile(long double value) {
    return QLength(static_cast<float>(value) * tile.getValue());
}

constexpr QLength operator""_tile(unsigned long long value) {
    return QLength(static_cast<float>(value) * tile.getValue());
}

constexpr QTime operator""_s(long double value) {
    return QTime(static_cast<float>(value));
}

constexpr QTime operator""_s(unsigned long long value) {
    return QTime(static_cast<float>(value));
}

constexpr QTime operator""_ms(long double value) {
    return QTime(static_cast<float>(value) * millisecond.getValue());
}

constexpr QTime operator""_ms(unsigned long long value) {
    return QTime(static_cast<float>(value) * millisecond.getValue());
}

constexpr QAngle operator""_rad(long double value) {
    return QAngle(static_cast<float>(value));
}

constexpr QAngle operator""_rad(unsigned long long value) {
    return QAngle(static_cast<float>(value));
}

constexpr QAngle operator""_deg(long double value) {
    return QAngle(static_cast<float>(value) * degree.getValue());
}

constexpr QAngle operator""_deg(unsigned long long value) {
    return QAngle(static_cast<float>(value) * degree.getValue());
}

constexpr QSpeed operator""_mps(long double value) {
    return QSpeed(static_cast<float>(value));
}

constexpr QSpeed operator""_mps(unsigned long long value) {
    return QSpeed(static_cast<float>(value));
}

constexpr QSpeed operator""_ips(long double value) {
    return QSpeed(static_cast<float>(value) * ips.getValue());
}

constexpr QSpeed operator""_ips(unsigned long long value) {
    return QSpeed(static_cast<float>(value) * ips.getValue());
}

constexpr QAcceleration operator""_mps2(long double value) {
    return QAcceleration(static_cast<float>(value));
}

constexpr QAcceleration operator""_mps2(unsigned long long value) {
    return QAcceleration(static_cast<float>(value));
}

constexpr QAcceleration operator""_ips2(long double value) {
    return QAcceleration(static_cast<float>(value) * ips2.getValue());
}

constexpr QAcceleration operator""_ips2(unsigned long long value) {
    return QAcceleration(static_cast<float>(value) * ips2.getValue());
}

constexpr QAngularSpeed operator""_radps(long double value) {
    return QAngularSpeed(static_cast<float>(value));
}

constexpr QAngularSpeed operator""_radps(unsigned long long value) {
    return QAngularSpeed(static_cast<float>(value));
}

constexpr QAngularSpeed operator""_degps(long double value) {
    return QAngularSpeed(static_cast<float>(value) * dps.getValue());
}

constexpr QAngularSpeed operator""_degps(unsigned long long value) {
    return QAngularSpeed(static_cast<float>(value) * dps.getValue());
}

} // namespace literals

} // namespace units

using namespace units;
using namespace units::literals;
