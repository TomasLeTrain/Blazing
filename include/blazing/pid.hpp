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

    windup_t<Input> windupRange;
    bool signFlipReset;

    Input target;

    Input previousError = 0;
    Multiplied<Input, Time> integral = 0;

    std::optional<Time> previousTime = std::nullopt;

  public:
    PID(KP_t<Input, Output> kP,
        KI_t<Input, Output> kI,
        KD_t<Input, Output> kD,
        windup_t<Input> windupRange,
        bool signFlipReset);

    void reset();

    void setTarget(Input target);

    Output update(Input error);

    KP_t<Input, Output> get_kP();
    KI_t<Input, Output> get_kI();
    KD_t<Input, Output> get_kD();

    windup_t<Input> get_windupRange();
    bool get_signFlipReset();

    void set_kP(KP_t<Input, Output> kP);
    void set_kI(KI_t<Input, Output> kI);
    void set_kD(KD_t<Input, Output> kD);

    void set_windupRange(windup_t<Input> windupRange);
    void set_signFlipReset(bool signFlipReset);
};
} // namespace blazing
