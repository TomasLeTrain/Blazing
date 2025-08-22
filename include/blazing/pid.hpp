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

template<typename Input>
using windup_t = Multiplied<Input, Time>;

template<typename Input, typename Output>
class PID {
  private:
    KP_t<Input, Output> kP;
    KI_t<Input, Output> kI;
    KD_t<Input, Output> kD;

    std::optional<windup_t<Input>> windupRange;

    Input target;
    Output range;

    Input previousError = 0;
    Multiplied<Input, Time> integral = 0;

    std::optional<Time> previousTime = std::nullopt;

  public:
    PID(KP_t<Input, Output> kP,
        KI_t<Input, Output> kI,
        KD_t<Input, Output> kD,
        std::optional<windup_t<Input>> windupRange = std::nullopt);

    void reset();

    void setTarget(Input target);

    Output update(Input error);

    KP_t<Input, Output> get_kP();
    KI_t<Input, Output> get_kI();
    KD_t<Input, Output> get_kD();

    std::optional<windup_t<Input>> get_windupRange();

    void set_kP(KP_t<Input, Output> kP);
    void set_kI(KI_t<Input, Output> kI);
    void set_kD(KD_t<Input, Output> kD);

    void set_windupRange(std::optional<windup_t<Input>> windupRange);
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
