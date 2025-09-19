#include "blazing/utils.hpp"
#include "pros/rtos.hpp"

namespace blazing {

Angle angleError(Angle target,
                 Angle heading,
                 std::optional<AngularDirection> direction) {
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
Angle reverseAngle(Angle angle) {
    return units::constrainAngle2pi(angle + 180_stDeg);
}

template<isQuantity T, size_t size>
std::array<T, size> desaturate(std::array<T, size> saturated, T max) {

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

// explicit instantiation to avoid linking errors
// TODO: move desaturate to header file to avoid having to do this
template std::array<Voltage, 2>
desaturate<Voltage, 2>(std::array<Voltage, 2> saturated, Voltage max);

Time getDeltaTime(std::optional<Time>& last_time) {
    Time current_time = from_msec(pros::millis());

    // can't return becaue we need to set last_time!
    auto result = last_time
                    .transform([current_time](Time last_time) -> Time {
                        return current_time - last_time;
                    })
                    .value_or(0.0_sec);

    last_time = current_time;

    return result;
}

} // namespace blazing
