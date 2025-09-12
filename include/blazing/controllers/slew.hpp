#pragma once

#include "blazing/controllers/controllers.hpp"
#include "units/units.hpp"

namespace blazing {

class SlewController {
    std::optional<Voltage> last_output = std::nullopt;

    std::optional<Divided<Voltage, Time>> decel_slew;
    std::optional<Divided<Voltage, Time>> accel_slew;

  public:
    SlewController(
      std::optional<Divided<Voltage, Time>> accel_slew = std::nullopt,
      std::optional<Divided<Voltage, Time>> decel_slew = std::nullopt)
        : accel_slew(accel_slew),
          decel_slew(decel_slew) {}

    SlewController(std::optional<Voltage> accel_slew = std::nullopt,
                   std::optional<Voltage> decel_slew = std::nullopt,
                   Time delta_time = 10_msec)
        : accel_slew(accel_slew.transform([delta_time](Voltage slew) {
              return slew / delta_time;
          })),
          decel_slew(decel_slew.transform([delta_time](Voltage slew) {
              return slew / delta_time;
          })) {}

    // output should be signed, indicating its direction of travel
    Voltage apply(Voltage output, Time delta_time) {
        if (!last_output) {
            last_output = output;
            return output;
        }

        auto output_vel = (output - *last_output) / delta_time;

        bool accelerating = units::sgn(output) == units::sgn(output_vel);

        if (decel_slew && !accelerating) {
            output_vel = units::sgn(output_vel) *
                         units::min(units::abs(output_vel), *decel_slew);
        }
        if (accel_slew && accelerating) {
            output_vel = units::sgn(output_vel) *
                         units::min(units::abs(output_vel), *accel_slew);
        }

        Voltage adjusted_output = *last_output + output_vel * delta_time;

        return output;
    }
};

class LinearSlewController : virtual ControllerBase {
  public:
    SlewController linear_slew_controller;

    LinearSlewController(SlewController linear_slew_controller)
        : linear_slew_controller(linear_slew_controller) {}

    LinearSlewController(
      std::optional<Divided<Voltage, Time>> accel_slew = std::nullopt,
      std::optional<Divided<Voltage, Time>> decel_slew = std::nullopt)
        : linear_slew_controller(accel_slew, decel_slew) {}

    LinearSlewController(std::optional<Voltage> accel_slew = std::nullopt,
                         std::optional<Voltage> decel_slew = std::nullopt,
                         Time delta_time = 10_msec)
        : linear_slew_controller(accel_slew, decel_slew, delta_time) {}
};

class AngularSlewController : virtual ControllerBase {
  public:
    SlewController angular_slew_controller;

    AngularSlewController(SlewController angular_slew_controller)
        : angular_slew_controller(angular_slew_controller) {}

    AngularSlewController(
      std::optional<Divided<Voltage, Time>> accel_slew = std::nullopt,
      std::optional<Divided<Voltage, Time>> decel_slew = std::nullopt)
        : angular_slew_controller(accel_slew, decel_slew) {}

    AngularSlewController(std::optional<Voltage> accel_slew = std::nullopt,
                          std::optional<Voltage> decel_slew = std::nullopt,
                          Time delta_time = 10_msec)
        : angular_slew_controller(accel_slew, decel_slew, delta_time) {}
};

template<typename Controller>
concept hasLinearSlewController =
  requires(Controller controller) { controller.linear_slew_controller; };
template<typename Controller>
concept hasAngularSlewController =
  requires(Controller controller) { controller.angular_slew_controller; };

} // namespace blazing
