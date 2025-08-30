#pragma once

#include "pros/rtos.hpp"
#include "units/units.hpp"
#include <optional>

namespace blazing {

template<typename Input, typename Output>
using KP_t = Divided<Output, Input>;
template<typename Input, typename Output>
using KI_t = Divided<Divided<Output, Time>, Input>;
template<typename Input, typename Output>
using KD_t = Divided<Multiplied<Output, Time>, Input>;

template<typename Input, typename Output>
class PID {
  private:
    KP_t<Input, Output> kP;
    KI_t<Input, Output> kI;
    KD_t<Input, Output> kD;

    std::optional<Input> windupRange;

    std::optional<Output> positiveSlew;
    std::optional<Output> negativeSlew;

    Input previousError = Input(0);
    Multiplied<Input, Time> integral = Multiplied<Input, Time>(0);

    std::optional<Time> previousTime = std::nullopt;

  public:
    PID(KP_t<Input, Output> kP,
        KI_t<Input, Output> kI,
        KD_t<Input, Output> kD,
        std::optional<Input> windupRange = std::nullopt,
        std::optional<Output> positiveSlew = std::nullopt,
        std::optional<Output> negativeSlew = std::nullopt)
        : kP(kP),
          kI(kI),
          kD(kD),
          windupRange(windupRange),
          positiveSlew(positiveSlew),
          negativeSlew(negativeSlew) {}

    void reset() {
        integral = 0;
        previousError = 0;
        previousTime = std::nullopt;
    }

    Output update(Input target, Input error, Time dt) {
        // calculate the derivative (change in error / time passed)
        const Divided<Input, Time> derivative = (dt != 0_sec) ?
                                                  (error - previousError) / dt :
                                                  Divided<Input, Time>(0);
        previousError = error;

        // calculate the integral (change in error * time passed)
        integral += error * dt;
        // sign flip reset. If the sign of error changes, set the integral to 0
        if (units::sgn(error) != units::sgn(previousError)) integral = Multiplied<Input, Time>(0);
        // anti windup range. Unless error is small enough, set the integral to
        // 0
        if (windupRange
              .transform([error](Input windupRange) {
                  return units::abs(error) > windupRange;
              })
              .value_or(false))
            integral = Multiplied<Input, Time>(0);

        // output = error * kP + integral * kP + derivative * kD
        Output result = error * kP + integral * kI + derivative * kD;

        // apply slew
        if (positiveSlew.has_value() && positiveSlew.value() < result)
            result = positiveSlew.value();
        if (negativeSlew.has_value() && -negativeSlew.value() > result)
            result = -negativeSlew.value();

        return result;
    }

    KP_t<Input, Output> get_kP() {
        return kP;
    }

    KI_t<Input, Output> get_kI() {
        return kI;
    }

    KD_t<Input, Output> get_kD() {
        return kD;
    }

    std::optional<Input> get_windupRange() {
        return windupRange;
    }

    void set_kP(KP_t<Input, Output> kP) {
        this->kP = kP;
    }

    void set_kI(KI_t<Input, Output> kI) {
        this->kI = kI;
    }

    void set_kD(KD_t<Input, Output> kD) {
        this->kD = kD;
    }

    void set_windupRange(std::optional<Input> windupRange) {
        this->windupRange = windupRange;
    }
};

template<typename T, typename Input, typename Output>
concept hasKP =
  requires(T t, KP_t<Input, Output> kp_value) { t.set_kP(kp_value); };

template<typename T, typename Input, typename Output>
concept hasKI =
  requires(T t, KI_t<Input, Output> kp_value) { t.set_kI(kp_value); };

template<typename T, typename Input, typename Output>
concept hasKD =
  requires(T t, KD_t<Input, Output> kp_value) { t.set_kD(kp_value); };

} // namespace blazing
