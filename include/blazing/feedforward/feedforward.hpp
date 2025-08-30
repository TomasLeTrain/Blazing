#pragma once

#include "blazing/feedback/pid.hpp"
#include "blazing/motion.hpp"
#include "units/Angle.hpp"

namespace blazing {

// Feedforward Concept
template<typename Controller, typename Input, typename Output>
concept Feedforward =
  requires(Controller controller, Input input, Time duration) {
      { controller.update(input, duration) } -> std::same_as<Output>;
  };

}; // namespace blazing
