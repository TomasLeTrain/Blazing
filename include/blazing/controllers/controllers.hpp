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
    Controller linear_feedback_controller;

    LinearFeedbackController(Controller linear_feedback_controller)
        : linear_feedback_controller(linear_feedback_controller) {}
};

template<typename Controller>
    requires Feedback<Controller, Angle, Voltage>
struct AngularFeedbackController : virtual ControllerBase {
  public:
    Controller angular_feedback_controller;

    AngularFeedbackController(Controller angular_feedback_controller)
        : angular_feedback_controller(angular_feedback_controller) {
		std::cout << "controller constructor called" << std::endl;
	}
};

using PIDLinearController = LinearFeedbackController<PID<Length, Voltage>>;
using PIDAngularController = AngularFeedbackController<PID<Angle, Voltage>>;

// inherits all the properties from the controllers being used
template<typename... ControllerTypes>
    requires(std::is_base_of_v<ControllerBase, ControllerTypes> && ...)
struct Controllers : virtual ControllerBase,
                     public ControllerTypes... {
  public:
    Controllers(ControllerTypes&&... controllers)
        : ControllerTypes(std::move(controllers))... {
		std::cout << "controllers constructor called" << std::endl;
	}
};

// Linear/Angular Feedback Concepts
template<typename Controller>
concept hasLinearFeedbackController =
  requires(Controller controller) { controller.linear_feedback_controller; };

template<typename Controller>
concept hasAngularFeedbackController =
  requires(Controller controller) { controller.angular_feedback_controller; };
} // namespace blazing
