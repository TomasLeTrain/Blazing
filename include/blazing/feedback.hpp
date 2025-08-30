#pragma once

#include "blazing/motion.hpp"
#include "blazing/pid.hpp"
#include "units/Angle.hpp"

namespace blazing {

// Feedback Concept
template<typename T, typename Input, typename Output>
concept Feedback =
  requires(T controller, Input measurement, Input setpoint, Time duration) {
      {
          controller.update(measurement, setpoint, duration)
      } -> std::same_as<Output>;
  };

template<typename LinearController>
    requires Feedback<LinearController, Length, Voltage>
class LinearFeedbackMotion : public virtual Motion {
  protected:
    LinearController linear_controller;

    LinearFeedbackMotion(LinearController linear_controller)
        : linear_controller(linear_controller) {}

  public:
    [[nodiscard("motion won't be executed!")]]
    LinearFeedbackMotion& lateral_kP(KP_t<Length, Voltage> kP)
        requires hasKP<LinearController, Length, Voltage>
    {
        linear_controller.set_kP(kP);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    LinearFeedbackMotion& lateral_kI(KI_t<Length, Voltage> kI)
        requires hasKI<LinearController, Length, Voltage>
    {
        linear_controller.set_kI(kI);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    LinearFeedbackMotion& lateral_kD(KD_t<Length, Voltage> kD)
        requires hasKD<LinearController, Length, Voltage>
    {
        linear_controller.set_kD(kD);
        return *this;
    }
};

template<typename AngularController>
    requires Feedback<AngularController, Angle, Voltage>
class AngularFeedbackMotion : public virtual Motion {
  protected:
    AngularController angular_controller;

    AngularFeedbackMotion(AngularController angular_controller)
        : angular_controller(angular_controller) {}

  public:
    [[nodiscard("motion won't be executed!")]]
    AngularFeedbackMotion& angular_kP(KP_t<Angle, Voltage> kP)
        requires hasKP<AngularController, Angle, Voltage>
    {
        angular_controller.set_kP(kP);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    AngularFeedbackMotion& angular_kI(KI_t<Angle, Voltage> kI)
        requires hasKI<AngularController, Angle, Voltage>
    {
        angular_controller.set_kI(kI);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    AngularFeedbackMotion& angular_kD(KD_t<Angle, Voltage> kD)
        requires hasKD<AngularController, Angle, Voltage>
    {
        angular_controller.set_kD(kD);
        return *this;
    }
};

}; // namespace blazing
