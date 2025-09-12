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

Angle angleError(Angle target,
                 Angle heading,
                 std::optional<AngularDirection> direction = std::nullopt);

// returns opposite angle, in the range [0,2pi)
Angle reverseAngle(Angle angle);

template<isQuantity T, size_t size>
std::array<T, size> desaturate(std::array<T, size> saturated, T max);

} // namespace blazing
