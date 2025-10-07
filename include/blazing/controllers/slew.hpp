#pragma once

#include "blazing/controllers/controllers.hpp"
#include "units/units.hpp"

namespace blazing {

class SlewController {
  private:
    std::optional<Voltage> last_output = std::nullopt;

    Time targeted_delta_time = 10_msec;

    std::optional<Divided<Voltage, Time>> accel_slew;
    std::optional<Divided<Voltage, Time>> decel_slew;

  public:
    SlewController(
      std::optional<Divided<Voltage, Time>> accel_slew = std::nullopt,
      std::optional<Divided<Voltage, Time>> decel_slew = std::nullopt)
        : accel_slew(accel_slew),
          decel_slew(decel_slew) {}

    SlewController(std::optional<Voltage> accel_slew = std::nullopt,
                   std::optional<Voltage> decel_slew = std::nullopt,
                   Time delta_time = 10_msec)
        : targeted_delta_time(delta_time),
          accel_slew(accel_slew.transform([delta_time](Voltage slew) {
              return slew / delta_time;
          })),
          decel_slew(decel_slew.transform([delta_time](Voltage slew) {
              return slew / delta_time;
          })) {}

    void set_accel(std::optional<Divided<Voltage, Time>> accel_slew) {
        this->accel_slew = accel_slew;
    }

    void set_decel(std::optional<Divided<Voltage, Time>> decel_slew) {
        this->decel_slew = decel_slew;
    }

    void set_accel(std::optional<Voltage> accel_slew) {
        this->accel_slew = accel_slew.transform([this](Voltage slew) {
            return slew / targeted_delta_time;
        });
    }

    void set_decel(std::optional<Voltage> decel_slew) {
        this->decel_slew = decel_slew.transform([this](Voltage slew) {
            return slew / targeted_delta_time;
        });
    }

    // output should be signed, indicating its direction of travel
    Voltage apply(Voltage output, Time delta_time) {
        if (!last_output) {
            last_output = 0_volt;
        }

        auto output_vel = delta_time == 0_sec ?
                            0.0_volt / sec :
                            (output - *last_output) / delta_time;

        bool accelerating = units::sgn(output) == units::sgn(output_vel);
        // std::cout << "accel " << output << " " << output_vel << " "
        //           << *accel_slew << std::endl;

        if (decel_slew && !accelerating) {
            output_vel = units::sgn(output_vel) *
                         units::min(units::abs(output_vel), *decel_slew);
        }
        if (accel_slew && accelerating) {
            // std::cout << "tfff " << output_vel << std::endl;
            output_vel = units::sgn(output_vel) *
                         units::min(units::abs(output_vel), *accel_slew);
            // std::cout << "tfff2 " << output_vel << std::endl;
        }

        Voltage adjusted_output = *last_output + output_vel * delta_time;

        last_output = adjusted_output;
        return adjusted_output;
    }
};

class LinearSlewController : virtual ControllerBase {
  public:
    SlewController linear_slew;

    LinearSlewController(SlewController linear_slew)
        : linear_slew(linear_slew) {}

    LinearSlewController(
      std::optional<Divided<Voltage, Time>> accel_slew = std::nullopt,
      std::optional<Divided<Voltage, Time>> decel_slew = std::nullopt)
        : linear_slew(accel_slew, decel_slew) {}

    LinearSlewController(std::optional<Voltage> accel_slew = std::nullopt,
                         std::optional<Voltage> decel_slew = std::nullopt,
                         Time delta_time = 10_msec)
        : linear_slew(accel_slew, decel_slew, delta_time) {}
};

class AngularSlewController : virtual ControllerBase {
  public:
    SlewController angular_slew;

    AngularSlewController(SlewController angular_slew)
        : angular_slew(angular_slew) {}

    AngularSlewController(
      std::optional<Divided<Voltage, Time>> accel_slew = std::nullopt,
      std::optional<Divided<Voltage, Time>> decel_slew = std::nullopt)
        : angular_slew(accel_slew, decel_slew) {}

    AngularSlewController(std::optional<Voltage> accel_slew = std::nullopt,
                          std::optional<Voltage> decel_slew = std::nullopt,
                          Time delta_time = 10_msec)
        : angular_slew(accel_slew, decel_slew, delta_time) {}
};

template<typename Controller>
concept hasLinearSlew =
  requires(Controller controller) { controller.linear_slew; };
template<typename Controller>
concept hasAngularSlew =
  requires(Controller controller) { controller.angular_slew; };

} // namespace blazing
