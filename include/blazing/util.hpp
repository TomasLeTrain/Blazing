#pragma once

#include "units/Angle.hpp"
#include "units/units.hpp"
#include <array>
#include <optional>

namespace blazing {
enum class AngularDirection {
    LEFT,
    RIGHT
};

inline Angle
angleError(Angle target,
           Angle heading,
           std::optional<AngularDirection> direction = std::nullopt) {
    Angle error = units::constrainAngle180(target - heading);

    if (!direction)
        return error;
    else if (direction == AngularDirection::LEFT)
        // error should be positive
        return error < 0_stRot ? error + rot : error;
    else
        // error should be negative
        return error > 0_stRot ? error - rot : error;
}

// returns opposite angle, in the range [0,2pi)
inline Angle reverseAngle(Angle angle) {
    return units::constrainAngle2pi(angle + 180_stDeg);
}

template<isQuantity T, size_t size>
inline std::array<T, size> desaturate(std::array<T, size> saturated, T max) {

    auto abs_compare = [](T a, T b) {
        return units::abs(a) < units::abs(b);
    };

    T largest_magnitude = *std::ranges::max_element(saturated, abs_compare);

    if (largest_magnitude > max) {
        std::transform(saturated.cbegin(),
                       saturated.cend(),
                       saturated.begin(),
                       [max, largest_magnitude](T num) {
                           return num * max / largest_magnitude;
                       });
    };

    return saturated;
}

} // namespace blazing
