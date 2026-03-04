/**
 * @file units.hpp
 * Compile-time dimensional-analysis unit system.
 *
 * Template parameters encode powers of Length, Time, and Angle so that
 * multiplying/dividing quantities yields the correct result type.
 *
 *   QVelocity v = QLength(1.0f) / QTime(1.0f);  // OK
 *   QLength   d = v * QTime(2.0f);               // OK
 */
#pragma once

#include <cmath>

namespace units {

// ── Unit template ───────────────────────────────────────────────────────────

template <int L, int T, int A>
struct Unit {
    float value;

    constexpr Unit() : value(0.0f) {}
    constexpr explicit Unit(float v) : value(v) {}
    constexpr float getValue() const { return value; }

    // Same-dimension arithmetic
    constexpr Unit operator+(Unit o) const { return Unit(value + o.value); }
    constexpr Unit operator-(Unit o) const { return Unit(value - o.value); }
    constexpr Unit operator-() const { return Unit(-value); }

    // Scalar multiply / divide
    constexpr Unit operator*(float s) const { return Unit(value * s); }
    constexpr Unit operator/(float s) const { return Unit(value / s); }

    // Comparisons
    constexpr bool operator<(Unit o)  const { return value < o.value; }
    constexpr bool operator>(Unit o)  const { return value > o.value; }
    constexpr bool operator<=(Unit o) const { return value <= o.value; }
    constexpr bool operator>=(Unit o) const { return value >= o.value; }
    constexpr bool operator==(Unit o) const { return value == o.value; }
    constexpr bool operator!=(Unit o) const { return value != o.value; }

    Unit& operator+=(Unit o) { value += o.value; return *this; }
    Unit& operator-=(Unit o) { value -= o.value; return *this; }
    Unit& operator*=(float s) { value *= s; return *this; }
    Unit& operator/=(float s) { value /= s; return *this; }
};

// Scalar * Unit
template <int L, int T, int A>
constexpr Unit<L, T, A> operator*(float s, Unit<L, T, A> u) {
    return Unit<L, T, A>(s * u.value);
}

// Cross-dimension multiply
template <int L1, int T1, int A1, int L2, int T2, int A2>
constexpr Unit<L1 + L2, T1 + T2, A1 + A2>
operator*(Unit<L1, T1, A1> a, Unit<L2, T2, A2> b) {
    return Unit<L1 + L2, T1 + T2, A1 + A2>(a.value * b.value);
}

// Cross-dimension divide
template <int L1, int T1, int A1, int L2, int T2, int A2>
constexpr Unit<L1 - L2, T1 - T2, A1 - A2>
operator/(Unit<L1, T1, A1> a, Unit<L2, T2, A2> b) {
    return Unit<L1 - L2, T1 - T2, A1 - A2>(a.value / b.value);
}

// ── Named aliases ───────────────────────────────────────────────────────────

using Dimensionless         = Unit<0, 0, 0>;
using QLength               = Unit<1, 0, 0>;
using QTime                 = Unit<0, 1, 0>;
using QAngle                = Unit<0, 0, 1>;
using QVelocity             = Unit<1, -1, 0>;   // m/s
using QAcceleration         = Unit<1, -2, 0>;   // m/s²
using QAngularVelocity      = Unit<0, -1, 1>;   // rad/s
using QAngularAcceleration  = Unit<0, -2, 1>;   // rad/s²
using QJerk                 = Unit<1, -3, 0>;   // m/s³
using QCurvature            = Unit<-1, 0, 0>;   // 1/m

// ── Fundamental constants ───────────────────────────────────────────────────

constexpr QLength metre{1.0f};
constexpr QLength meter = metre;
constexpr QLength inch{0.0254f};
constexpr QLength foot{0.3048f};
constexpr QLength tile{0.6096f};    // VEX field tile = 24 in

constexpr QTime second{1.0f};
constexpr QTime millisecond{0.001f};

constexpr QAngle radian{1.0f};
constexpr QAngle degree{static_cast<float>(M_PI / 180.0)};

constexpr QVelocity mps{1.0f};     // metres per second

// ── Helper functions ────────────────────────────────────────────────────────

template <int L, int T, int A>
constexpr Unit<2 * L, 2 * T, 2 * A> Qsq(Unit<L, T, A> u) {
    return Unit<2 * L, 2 * T, 2 * A>(u.value * u.value);
}

template <int L, int T, int A>
constexpr Unit<L / 2, T / 2, A / 2> Qsqrt(Unit<L, T, A> u) {
    return Unit<L / 2, T / 2, A / 2>(std::sqrt(u.value));
}

template <int L, int T, int A>
constexpr Unit<L, T, A> Qabs(Unit<L, T, A> u) {
    return Unit<L, T, A>(std::fabs(u.value));
}

} // namespace units

using namespace units;
