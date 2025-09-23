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

    std::optional<Output> maxVoltage;

    std::optional<Input> previousError;
    Multiplied<Input, Time> integral = Multiplied<Input, Time>(0);

    std::optional<Time> previousTime = std::nullopt;

  public:
    // units used to convert doubles (since specyfing the units every time can
    // become annoying)
    Time m_timeUnits = 1_sec;
    Input m_inputUnits = Input(1);
    Output m_outputUnits = Output(1);

    KP_t<Input, Output> UKP;
    KI_t<Input, Output> UKI;
    KD_t<Input, Output> UKD;

    PID(KP_t<Input, Output> kP,
        KI_t<Input, Output> kI,
        KD_t<Input, Output> kD,
        std::optional<Input> windupRange = std::nullopt,
        std::optional<Output> maxVoltage = std::nullopt)
        : kP(kP),
          kI(kI),
          kD(kD),
          windupRange(windupRange),
          maxVoltage(maxVoltage) {
        // std::cout << "constructor 1 called " << std::endl;
    }

    PID(double mkP,
        double mkI,
        double mkD,
        std::optional<double> windupRange = std::nullopt,
        std::optional<double> maxVoltage = std::nullopt,
        Time timeUnits = 1_sec,
        Input inputUnits = Input(1),
        Output outputUnits = Output(1))
        : m_timeUnits(timeUnits),
          m_inputUnits(inputUnits),
          m_outputUnits(outputUnits),
          UKP(outputUnits / inputUnits),
          UKI((outputUnits / timeUnits) / inputUnits),
          UKD((outputUnits * timeUnits) / inputUnits),
          kP(mkP * (outputUnits / inputUnits)),
          kI(mkI * ((outputUnits / timeUnits) / inputUnits)),
          kD(mkD * ((outputUnits * timeUnits) / inputUnits)),
          windupRange(
            windupRange.transform([inputUnits](double windupRange) -> Input {
                return windupRange * inputUnits;
            })),
          maxVoltage(
            maxVoltage.transform([outputUnits](double maxVoltage) -> Output {
                return maxVoltage * outputUnits;
            })) {
        // std::cout << "constructor 2 called " << std::endl;
        // std::cout << " tf " << mkP * UKP << " " << mkI * UKI << " " << mkD *
        // UKD
        //           << std::endl;
        // std::cout << " tf " << kP << " " << kI << " " << kD << std::endl;
    }

    void reset() {
        integral = 0;
        previousTime = std::nullopt;
    }

    Output update(Input measurement, Input target, Time dt) {
        Input error = target - measurement;

        if (!previousError) previousError = error;

        const Divided<Input, Time> derivative =
          (dt != 0_sec) ? (error - *previousError) / dt :
                          Divided<Input, Time>(0);
        previousError = error;

        // calculate the integral (change in error * time passed)
        integral += error * dt;
        // sign flip reset. If the sign of error changes, set the integral to 0
        if (units::sgn(error) != units::sgn(*previousError))
            integral = Multiplied<Input, Time>(0);
        // anti windup range. Unless error is small enough, set the integral to
        // 0
        if (windupRange
              .transform([error](Input windupRange) {
                  return units::abs(error) > windupRange;
              })
              .value_or(false))
            integral = Multiplied<Input, Time>(0);

        Output result = error * kP + integral * kI + derivative * kD;

        // std::cout << "[PID] UKP/I/D: " << UKP << " " << UKI << " " << UKD
        //           << std::endl;
        // std::cout << "[PID] error/kp/integral/ki/der/kd: " << error << " " <<
        // kP
        //           << " " << integral << " " << kI << " " << derivative << " "
        //           << kD << std::endl;
        //
        // std::cout << "[PID] unclamped result: " << result;

        if (maxVoltage) {
			std::cout  << "unclamped " << result.internal() << std::endl;
            result = units::clamp(result, -(*maxVoltage), *maxVoltage);
			std::cout  << "clamped " << result.internal() << std::endl;
        }
        // std::cout << ", clamped result: " << result << std::endl;

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

    std::optional<Output> get_maxVoltage() {
        return maxVoltage;
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
        this->maxVoltage = positiveSlew;
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
          [inputUnits = this->m_inputUnits](auto windupRange) -> Input {
              return windupRange * inputUnits;
          });
    }

    void set_maxVoltage(std::optional<double> maxVoltage) {
        this->maxVoltage = maxVoltage.transform(
          [outputUnits = this->m_outputUnits](auto maxVoltage) -> Output {
              return maxVoltage * outputUnits;
          });
    }
};
} // namespace blazing
