#pragma once

#include "blazing/feedback/pid.hpp"
#include "blazing/motion.hpp"
#include "units/Angle.hpp"

namespace blazing {

// Feedforward Concept
template<typename Controller, typename Input, typename Output>
concept Feedforward = requires(Controller controller, Input input, Time duration) {
    { controller.update(input, duration) } -> std::same_as<Output>;
};

template<typename LinearController>
    requires Feedforward<LinearController, Length, Voltage>
class LinearFeedforwardMotion : public virtual Motion {
  protected:
    LinearController linear_controller;

    LinearFeedforwardMotion(LinearController linear_controller)
        : linear_controller(linear_controller) {}
};

template<typename AngularController>
    requires Feedforward<AngularController, Angle, Voltage>
class AngularFeedforwardMotion : public virtual Motion {
  protected:
    AngularController angular_controller;

    AngularFeedforwardMotion(AngularController angular_controller)
        : angular_controller(angular_controller) {}
};

}; // namespace blazing
