#pragma once

#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <array>
#include <optional>

namespace blazing {
enum class AngularDirection {
    LEFT,
    RIGHT
};

Angle angleError(Angle target,
                 Angle heading,
                 std::optional<AngularDirection> direction = std::nullopt);

// returns opposite angle, in the range [0,2pi)
Angle reverseAngle(Angle angle);

template<isQuantity T, size_t size>
std::array<T, size> desaturate(std::array<T, size> saturated, T max);

// calculates delta time given some last time (which can be nullopt)
// also updates last_time to equal current time
// NOTE: returns 0 if last_time is nullopt!
Time deltaTime(std::optional<Time>& last_time);

// same as units::sgn, but returns 1.0 if the number is equal to zero (never
// returns 0 for the sign)

template<isQuantity Q>
Number signed_sgn(Q num) {
    return units::sgn(num) == 0 ? Number(1.0) : units::sgn(num);
}

// returns time since program started
// uses pros::millis to get the information
inline Time now() {
    return from_msec(pros::millis());
}

inline Divided<Number, Angle> sinc(Angle theta) {
    if (units::abs(theta) < 1e-6 * rad) {
        return (1.0 - theta.internal() * theta.internal() / 6.0) / rad;
    } else {
        return units::sin(theta) / theta;
    }
};

} // namespace blazing
