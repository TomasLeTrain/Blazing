#pragma once

#include "blazing/feedback/feedback.hpp"
#include "blazing/feedback/pid.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <concepts>
#include <type_traits>

namespace blazing {

struct ControllerBase {};

template<typename Controller>
    requires Feedback<Controller, Length, Voltage>
struct LinearFeedbackController : public ControllerBase {
    Controller linear_feedback_controller;

    LinearFeedbackController(Controller linear_feedback_controller)
        : linear_feedback_controller(linear_feedback_controller) {}
};

template<typename Controller>
    requires Feedback<Controller, Angle, Voltage>
struct AngularFeedbackController : public ControllerBase {
    Controller angular_feedback_controller;

    AngularFeedbackController(Controller angular_feedback_controller)
        : angular_feedback_controller(angular_feedback_controller) {}
};

using PIDLinearController = LinearFeedbackController<PID<Length, Voltage>>;
using PIDAngularController = AngularFeedbackController<PID<Angle, Voltage>>;

// inherits all the properties from the controllers being used
template<std::derived_from<ControllerBase>... ControllerTypes>
struct Controllers : public ControllerTypes... {
    Controllers(ControllerTypes... controllers)
        : ControllerTypes(std::move(controllers))... {}
};

// Linear/Angular Feedback Concepts
template<typename Controller>
concept hasLinearFeedbackController =
  requires(Controller controller) { controller.linear_feedback_controller; };

template<typename Controller>
concept hasAngularFeedbackController =
  requires(Controller controller) { controller.angular_feedback_controller; };
} // namespace blazing
