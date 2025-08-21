#include "blazing/pid.hpp"

namespace blazing {

template<typename Input, typename Output>
PID<Input, Output>::PID(KP_t<Input, Output> kP,
                        KI_t<Input, Output> kI,
                        KD_t<Input, Output> kD,
                        windup_t<Input> windupRange,
                        bool signFlipReset)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      signFlipReset(signFlipReset) {}

template<typename Input, typename Output>
void PID<Input, Output>::reset() {
    integral = 0;
    previousError = 0;
    previousTime = std::nullopt;
}

template<typename Input, typename Output>
void PID<Input, Output>::setTarget(Input target) {
    this->target = target;
}

template<typename Input, typename Output>
Output PID<Input, Output>::update(Input error) {
    // find time delta
    const Time now = from_msec(pros::millis());
    // if this is the first iteration, previousTime won't be set
    // if it is not set, then assume dt is 0
    Time dt = (previousTime == std::nullopt) ? 0_sec : now - *previousTime;
    previousTime = now;

    // calculate the derivative (change in error / time passed)
    const Divided<Input, Time> derivative =
      (dt != 0_sec) ? (error - previousError) / dt : Divided<Input, Time>(0);
    previousError = error;

    // calculate the integral (change in error * time passed)
    integral += error * dt;
    // sign flip reset. If the sign of error changes, set the integral to 0
    if (units::sgn(error) != units::sgn((previousError)) && signFlipReset)
        integral = 0;
    // anti windup range. Unless error is small enough, set the integral to
    // 0
    if (units::abs(error) > windupRange && windupRange != 0) integral = 0;

    // output. error * kP + integral * kP + derivative * kD
    return error * kP + integral * kI + derivative * kD;
}

template<typename Input, typename Output>
KP_t<Input, Output> PID<Input, Output>::get_kP() {
    return kP;
}

template<typename Input, typename Output>
KI_t<Input, Output> PID<Input, Output>::get_kI() {
    return kI;
}

template<typename Input, typename Output>
KD_t<Input, Output> PID<Input, Output>::get_kD() {
    return kD;
}

template<typename Input, typename Output>
windup_t<Input> PID<Input, Output>::get_windupRange() {
    return windupRange;
}

template<typename Input, typename Output>
bool PID<Input, Output>::get_signFlipReset() {
    return windupRange;
}

template<typename Input, typename Output>
void PID<Input, Output>::set_kP(KP_t<Input, Output> kP) {
    this->kP = kP;
}

template<typename Input, typename Output>
void PID<Input, Output>::set_kI(KI_t<Input, Output> kI) {
    this->kI = kI;
}

template<typename Input, typename Output>
void PID<Input, Output>::set_kD(KD_t<Input, Output> kD) {
    this->kD = kD;
}

template<typename Input, typename Output>
void PID<Input, Output>::set_windupRange(windup_t<Input> windupRange) {
    this->windupRange = windupRange;
}

template<typename Input, typename Output>
void PID<Input, Output>::set_signFlipReset(bool signFlipReset) {
    this->signFlipReset = signFlipReset;
}

} // namespace blazing
