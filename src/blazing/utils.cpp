#include "blazing/utils.hpp"
#include "pros/rtos.hpp"

namespace blazing {

Angle angleError(Angle target,
                 Angle heading,
                 std::optional<AngularDirection> direction) {
    Angle error = units::constrainAngle180(target - heading);

    if (!direction)
        return error;
    else if (direction == AngularDirection::LEFT)
        // error should be positive
        return error < 0_stRot ? error + rot : error;
    else
        // error should be negative
        return error > 0_stRot ? error - rot : error;
}

// returns opposite angle, in the range [0,2pi)
Angle reverseAngle(Angle angle) {
    return units::constrainAngle2pi(angle + 180_stDeg);
}

Time deltaTime(std::optional<Time>& last_time) {
    Time current_time = now();

    // can't return becaue we need to set last_time!
    auto result = last_time
                    .transform([current_time](Time last_time) -> Time {
                        return current_time - last_time;
                    })
                    .value_or(0.0_sec);

    last_time = current_time;

    return result;
}

// returns time since program started
// uses pros::millis to get the information
Time now() {
    return from_msec(pros::millis());
}

Divided<Number, Angle> sinc(Angle theta) {
    if (units::abs(theta) < 1e-6 * rad) {
        return (1.0 - theta.internal() * theta.internal() / 6.0) / rad;
    } else {
        return units::sin(theta) / theta;
    }
};

bool timeoutDone(std::optional<Time> timeout, Time start_time) {
    return timeout
      .transform([start_time](Time timeout) -> bool {
          return now() - start_time > timeout;
      })
      .value_or(false);
}

} // namespace blazing
