#pragma once

#include <concepts>
#include "units/units.hpp"

namespace blazing {

// Feedback Concept
template<typename Controller, typename Input, typename Output>
concept Feedback = requires(Controller controller,
                            Input measurement,
                            Input setpoint,
                            Time duration) {
    {
        controller.update(measurement, setpoint, duration)
    } -> std::same_as<Output>;
};
}; // namespace blazing
