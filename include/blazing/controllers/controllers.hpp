#pragma once

#include "blazing/controllers/feedback/feedback.hpp"
#include "blazing/controllers/feedback/pid.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <concepts>

namespace blazing {

struct ControllerBase {};

template<typename Controller>
    requires Feedback<Controller, Length, Voltage>
struct LinearFeedbackController : virtual ControllerBase {
  public:
    Controller linear_feedback;

    LinearFeedbackController(Controller linear_feedback_controller)
        : linear_feedback(linear_feedback_controller) {}
};

template<typename Controller>
    requires Feedback<Controller, Angle, Voltage>
struct AngularFeedbackController : virtual ControllerBase {
  public:
    Controller angular_feedback;

    AngularFeedbackController(Controller angular_feedback)
        : angular_feedback(angular_feedback) {}
};

using PIDLinearController = LinearFeedbackController<PID<Length, Voltage>>;
using PIDAngularController = AngularFeedbackController<PID<Angle, Voltage>>;

// inherits all the properties from the controllers being used
template<typename... ControllerTypes>
    requires(std::is_base_of_v<ControllerBase, ControllerTypes> && ...)
struct Controllers : virtual ControllerBase,
                     public ControllerTypes... {
  public:
    template<typename... U>
        requires(sizeof...(U) == sizeof...(ControllerTypes) &&
                 (std::is_constructible_v<ControllerTypes, U> && ...))
    Controllers(U&&... controllers)
        : ControllerTypes(std::forward<U>(controllers))... {}
};

// deduction guide allows specifying tolerance types from constructor
template<typename... ControllerTypes>
Controllers(ControllerTypes&&...)
  -> Controllers<std::remove_cvref_t<ControllerTypes>...>;

// Linear/Angular Feedback Concepts
template<typename Controller>
concept hasLinearFeedback =
  requires(Controller controller) { controller.linear_feedback; };

template<typename Controller>
concept hasAngularFeedback =
  requires(Controller controller) { controller.angular_feedback; };
} // namespace blazing
