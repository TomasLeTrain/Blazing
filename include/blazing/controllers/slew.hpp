#pragma once

#include "blazing/controllers/controllers.hpp"
#include "units/units.hpp"

namespace blazing {

class SlewController {
  public:
    using slew_t = std::optional<Divided<Voltage, Time>>;

  private:
    std::optional<Voltage> last_output = std::nullopt;

    Time targeted_delta_time = 10_msec;

    slew_t accel_slew;
    slew_t backwards_accel_slew;

    slew_t decel_slew;
    slew_t backwards_decel_slew;

  public:
    SlewController(slew_t accel_slew = std::nullopt,
                   slew_t backwards_accel_slew = std::nullopt,
                   slew_t decel_slew = std::nullopt,
                   slew_t backwards_decel_slew = std::nullopt)
        : accel_slew(accel_slew),
          backwards_accel_slew(backwards_accel_slew),
          decel_slew(decel_slew),
          backwards_decel_slew(backwards_decel_slew) {}

    SlewController(std::optional<Voltage> accel_slew = std::nullopt,
                   std::optional<Voltage> backwards_accel_slew = std::nullopt,
                   std::optional<Voltage> decel_slew = std::nullopt,
                   std::optional<Voltage> backwards_decel_slew = std::nullopt,
                   Time delta_time = 10_msec)
        : targeted_delta_time(delta_time),
          accel_slew(accel_slew.transform([delta_time](Voltage slew) {
              return slew / delta_time;
          })),
          backwards_accel_slew(
            backwards_accel_slew.transform([delta_time](Voltage slew) {
                return slew / delta_time;
            })),
          decel_slew(decel_slew.transform([delta_time](Voltage slew) {
              return slew / delta_time;
          })),
          backwards_decel_slew(
            backwards_decel_slew.transform([delta_time](Voltage slew) {
                return slew / delta_time;
            })) {}

    void set_accel(slew_t accel_slew) {
        this->accel_slew = accel_slew;
    }

    void set_backwards_accel(slew_t backwards_accel_slew) {
        this->backwards_accel_slew = backwards_accel_slew;
    }

    void set_decel(slew_t decel_slew) {
        this->decel_slew = decel_slew;
    }

    void set_backwards_decel(slew_t backwards_decel_slew) {
        this->backwards_decel_slew = backwards_decel_slew;
    }

    void set_accel(std::optional<Voltage> accel_slew) {
        set_accel(accel_slew.transform([this](Voltage slew) {
            return slew / targeted_delta_time;
        }));
    }

    void set_backwards_accel(std::optional<Voltage> backwards_accel_slew) {
        set_backwards_accel(
          backwards_accel_slew.transform([this](Voltage slew) {
              return slew / targeted_delta_time;
          }));
    }

    void set_decel(std::optional<Voltage> decel_slew) {
        set_decel(decel_slew.transform([this](Voltage slew) {
            return slew / targeted_delta_time;
        }));
    }

    void set_backwards_decel(std::optional<Voltage> backwards_decel_slew) {
        set_backwards_decel(
          backwards_decel_slew.transform([this](Voltage slew) {
              return slew / targeted_delta_time;
          }));
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
        bool forwards = units::sgn(output) >= 0.0;

        auto applySlew = [&](SlewController::slew_t slew) {
            output_vel = units::sgn(output_vel) *
                         units::min(units::abs(output_vel), *slew);
        };

        // decelerating
        if (!accelerating) {
            if (!forwards && backwards_decel_slew) {
                // backwards decel set, use
                applySlew(backwards_decel_slew);
            } else if (decel_slew) {
                // use decel regardless of direction
                applySlew(decel_slew);
            }
        } else if (accelerating) {
            if (!forwards && backwards_accel_slew) {
                // backwards decel set, use
                applySlew(backwards_accel_slew);
            } else if (accel_slew) {
                // use accel regardless of direction
                applySlew(accel_slew);
            }
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
      SlewController::slew_t accel_slew = std::nullopt,
      SlewController::slew_t backwards_accel_slew = std::nullopt,
      SlewController::slew_t decel_slew = std::nullopt,
      SlewController::slew_t backwards_decel_slew = std::nullopt)
        : linear_slew(accel_slew,
                      backwards_accel_slew,
                      decel_slew,
                      backwards_decel_slew) {}

    LinearSlewController(
      std::optional<Voltage> accel_slew = std::nullopt,
      std::optional<Voltage> backwards_accel_slew = std::nullopt,
      std::optional<Voltage> decel_slew = std::nullopt,
      std::optional<Voltage> backwards_decel_slew = std::nullopt,
      Time delta_time = 10_msec)
        : linear_slew(accel_slew,
                      backwards_accel_slew,
                      decel_slew,
                      backwards_decel_slew,
                      delta_time) {}
};

class AngularSlewController : virtual ControllerBase {
  public:
    SlewController angular_slew;

    AngularSlewController(SlewController angular_slew)
        : angular_slew(angular_slew) {}

    AngularSlewController(
      SlewController::slew_t accel_slew = std::nullopt,
      SlewController::slew_t backwards_accel_slew = std::nullopt,
      SlewController::slew_t decel_slew = std::nullopt,
      SlewController::slew_t backwards_decel_slew = std::nullopt)
        : angular_slew(accel_slew,
                       backwards_accel_slew,
                       decel_slew,
                       backwards_decel_slew) {}

    AngularSlewController(
      std::optional<Voltage> accel_slew = std::nullopt,
      std::optional<Voltage> backwards_accel_slew = std::nullopt,
      std::optional<Voltage> decel_slew = std::nullopt,
      std::optional<Voltage> backwards_decel_slew = std::nullopt,
      Time delta_time = 10_msec)
        : angular_slew(accel_slew,
                       backwards_accel_slew,
                       decel_slew,
                       backwards_decel_slew,
                       delta_time) {}
};

template<typename Controller>
concept hasLinearSlew =
  requires(Controller controller) { controller.linear_slew; };
template<typename Controller>
concept hasAngularSlew =
  requires(Controller controller) { controller.angular_slew; };

} // namespace blazing
