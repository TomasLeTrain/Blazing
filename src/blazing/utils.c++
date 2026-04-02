#include "blazing/utils.hpp"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"

namespace blazing {

Angle angleError(Angle target,
                 Angle heading,
                 std::optional<AngularDirection> direction) {
    target = units::constrainAngle2pi(target);
    heading = units::constrainAngle2pi(heading);
    Angle error = units::constrainAngle180(target - heading);
    // 0 - 270 = -270 = 90? != -90????

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
    Time delta_time = last_time
                        .transform([current_time](Time last_time) -> Time {
                            return current_time - last_time;
                        })
                        .value_or(0.0_sec);

    last_time = current_time;

    return delta_time;
}

// returns time since program started
// uses pros::millis to get the information
Time now() {
    return from_msec(pros::millis());
}

FTime Fnow() {
    return from_Fmsec(FNumber(pros::millis()));
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

FAngularVelocity gearingToVelocity(pros::MotorGears gearing) {
    if (gearing == pros::MotorGears::rpm_600)
        return 600_Frpm;
    else if (gearing == pros::MotorGears::rpm_200)
        return 200_Frpm;
    else if (gearing == pros::MotorGears::rpm_100)
        return 100_Frpm;
    // if encoder units are not set then it defaults to 200?
    return 200_Frpm;
}

// gets the average angular velocity of the motor group
FAngularVelocity getGroupVelocity(pros::MotorGroup* motors,
                                    FAngularVelocity final_rpm) {
    FAngularVelocity average_rpm = 0_rpm;

    for (std::int8_t motor_i = 0; motor_i < motors->size(); motor_i++) {
        auto zero_indexed_port = abs(motors->get_port(motor_i)) - 1;
        bool installed = pros::DeviceType::motor ==
                         (pros::DeviceType)pros::c::registry_get_plugged_type(
                           zero_indexed_port);
        if (!installed) continue;

        double velocity = motors->get_actual_velocity(motor_i);
        pros::MotorGears encoder_units = motors->get_gearing(motor_i);
        FAngularVelocity start_rpm = gearingToVelocity(encoder_units);
        FAngularVelocity actual_rpm =
          (velocity * Frpm) * (final_rpm / start_rpm);

        average_rpm += actual_rpm;
    }

    average_rpm /= static_cast<float>(motors->size());

    return average_rpm;
};

FLinearVelocity getGroupVelocity(pros::MotorGroup* motors,
                                   FLength wheel_diameter,
                                   FAngularVelocity final_rpm) {
    return toLinear(getGroupVelocity(motors, final_rpm), wheel_diameter);
};

FVoltage getGroupVoltage(pros::MotorGroup* motors) {
    FVoltage result = 0_Fvolt;

    for (std::int8_t motor_i = 0; motor_i < motors->size(); motor_i++) {
        auto zero_indexed_port = abs(motors->get_port(motor_i)) - 1;
        bool installed = pros::DeviceType::motor ==
                         (pros::DeviceType)pros::c::registry_get_plugged_type(
                           zero_indexed_port);
        if (!installed) continue;

        result += from_Fmvolt(FNumber(motors->get_voltage(motor_i))) / 12.f;
    }

    result /= static_cast<float>(motors->size());
    return result;
};

} // namespace blazing
