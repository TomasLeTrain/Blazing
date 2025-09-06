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
    // units used to convert doubles (since specyfing the units every time can
    // become annoying)
    Time timeUnits = 1_sec;
    Input inputUnits = Input(1);
    Output outputUnits = Output(1);
    KP_t<Input, Output> UKP = KP_t<Input, Output>(1);
    KI_t<Input, Output> UKI = KI_t<Input, Output>(1);
    KD_t<Input, Output> UKD = KD_t<Input, Output>(1);

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

    PID(double kP,
        double kI,
        double kD,
        std::optional<double> windupRange = std::nullopt,
        std::optional<double> positiveSlew = std::nullopt,
        std::optional<double> negativeSlew = std::nullopt,
        Time timeUnits = 1_sec,
        Input inputUnits = Input(1),
        Output outputUnits = Output(1))
        : UKP(outputUnits / inputUnits),
          UKI(outputUnits / timeUnits / inputUnits),
          UKD(outputUnits * timeUnits / inputUnits),
          kP(kP * UKP),
          kI(kI * UKI),
          kD(kD * UKD),
          windupRange(
            windupRange.transform([inputUnits](auto windupRange) -> Input {
                return windupRange * inputUnits;
            })),
          positiveSlew(
            positiveSlew.transform([outputUnits](auto positiveSlew) -> Output {
                return positiveSlew * outputUnits;
            })),
          negativeSlew(
            negativeSlew.transform([outputUnits](auto negativeSlew) -> Output {
                return negativeSlew * outputUnits;
            })) {}

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
        if (units::sgn(error) != units::sgn(previousError))
            integral = Multiplied<Input, Time>(0);
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

		// TODO: not slew? max/min voltage instead
		// TODO: actually implement slew
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

    std::optional<Output> get_positiveSlew() {
        return positiveSlew;
    }

    std::optional<Output> get_negativeSlew() {
        return negativeSlew;
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

    void set_positiveSlew(std::optional<Output> positiveSlew) {
        this->positiveSlew = positiveSlew;
    }

    void set_negativeSlew(std::optional<Output> negativeSlew) {
        this->negativeSlew = negativeSlew;
    }

    // double versions
    void set_kP(double kP) {
        this->kP = kP * UKP;
    }

    void set_kI(double kI) {
        this->kI = kI * UKI;
    }

    void set_kD(double kD) {
        this->kD = kD * UKD;
    }

    void set_windupRange(std::optional<double> windupRange) {
        this->windupRange = windupRange.transform(
          [inputUnits = this->inputUnits](auto windupRange) -> Input {
              return windupRange * inputUnits;
          });
    }

    void set_positiveSlew(std::optional<double> positiveSlew) {
        this->positiveSlew = positiveSlew.transform(
          [outputUnits = this->outputUnits](auto positiveSlew) -> Output {
              return positiveSlew * outputUnits;
          });
    }

    void set_negativeSlew(std::optional<double> negativeSlew) {
        this->negativeSlew = negativeSlew.transform(
          [outputUnits = this->outputUnits](auto negativeSlew) -> Output {
              return negativeSlew * outputUnits;
          });
    }
};
} // namespace blazing
