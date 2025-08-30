#pragma once

#include "blazing/feedback/feedback.hpp"
#include "blazing/feedback/pid.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"

namespace blazing {

template<typename Controller>
    requires Feedback<Controller, Length, Voltage>
struct LinearFeedbackController {
    Controller linear_feedback_controller;

    LinearFeedbackController(Controller linear_feedback_controller)
        : linear_feedback_controller(linear_feedback_controller) {}
};

template<typename Controller>
    requires Feedback<Controller, Angle, Voltage>
struct AngularFeedbackController {
    Controller angular_feedback_controller;

    AngularFeedbackController(Controller angular_feedback_controller)
        : angular_feedback_controller(angular_feedback_controller) {}
};

// inherits all the properties from the controllers being used
template<typename... ControllerTypes>
struct Controllers : public ControllerTypes... {
    Controllers(ControllerTypes&&... controllers)
        : ControllerTypes(std::move(controllers))... {}
};

using PIDLinearController = LinearFeedbackController<PID<Length, Voltage>>;
using PIDAngularController = AngularFeedbackController<PID<Angle, Voltage>>;
}; // namespace blazing
