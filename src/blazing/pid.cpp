#include "blazing/pid.hpp"
#include "units/units.hpp"
#include <optional>

namespace blazing {
template<typename Input, typename Output>
// template<typename Input, typename Output>
// void PID<Input, Output>::reset() {
//     integral = 0;
//     previousError = 0;
//     previousTime = std::nullopt;
// }
//
// template<typename Input, typename Output>
// Output PID<Input, Output>::update(Input target, Input error, Time dt) {
//     // calculate the derivative (change in error / time passed)
//     const Divided<Input, Time> derivative =
//       (dt != 0_sec) ? (error - previousError) / dt : Divided<Input, Time>(0);
//     previousError = error;
//
//     // calculate the integral (change in error * time passed)
//     integral += error * dt;
//     // sign flip reset. If the sign of error changes, set the integral to 0
//     if (units::sgn(error) != units::sgn(previousError)) integral = 0;
//     // anti windup range. Unless error is small enough, set the integral to
//     // 0
//     if (windupRange
//           .transform([error](windup_t<Input> windupRange) {
//               return units::abs(error) > windupRange;
//           })
//           .value_or(false))
//         integral = 0;
//
//     // output = error * kP + integral * kP + derivative * kD
//     Output result = error * kP + integral * kI + derivative * kD;
//
//     if (range.has_value()) units::clamp(result, -range, range);
//
//     return result;
// }

// template<typename Input, typename Output>
// KP_t<Input, Output> PID<Input, Output>::get_kP() {
//     return kP;
// }
//
// template<typename Input, typename Output>
// KI_t<Input, Output> PID<Input, Output>::get_kI() {
//     return kI;
// }
//
// template<typename Input, typename Output>
// KD_t<Input, Output> PID<Input, Output>::get_kD() {
//     return kD;
// }
//
// template<typename Input, typename Output>
// std::optional<windup_t<Input>> PID<Input, Output>::get_windupRange() {
//     return windupRange;
// }
//
// template<typename Input, typename Output>
// void PID<Input, Output>::set_kP(KP_t<Input, Output> kP) {
//     this->kP = kP;
// }
//
// template<typename Input, typename Output>
// void PID<Input, Output>::set_kI(KI_t<Input, Output> kI) {
//     this->kI = kI;
// }
//
// template<typename Input, typename Output>
// void PID<Input, Output>::set_kD(KD_t<Input, Output> kD) {
//     this->kD = kD;
// }
//
// template<typename Input, typename Output>
// void PID<Input, Output>::set_windupRange(
//   std::optional<windup_t<Input>> windupRange) {
//     this->windupRange = windupRange;
// }

} // namespace blazing
