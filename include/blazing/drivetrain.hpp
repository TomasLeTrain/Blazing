#pragma once

#include "pros/motor_group.hpp"
#include "units/units.hpp"

namespace blazing {

template<typename Q>
concept ArcadeDrivetrain =
  requires(Q q, Voltage linear_output, Voltage angular_output) {
      q.moveArcade(linear_output, angular_output);
  };
template<typename Q>
concept TankDrivetrain =
  requires(Q q, Voltage left_voltage, Voltage right_voltage) {
      q.moveTank(left_voltage, right_voltage);
  };

class DifferentialDrivetrain {
  private:
    pros::MotorGroup* left_motors;
    pros::MotorGroup* right_motors;

  public:
    DifferentialDrivetrain(pros::MotorGroup* left_motors,
                           pros::MotorGroup* right_motors)
        : left_motors(left_motors),
          right_motors(right_motors) {}

    // move robot based on left and right velocities
    void moveTank(Voltage left_voltage, Voltage right_voltage) {
        if (left_motors != nullptr && right_motors != nullptr) {
            left_motors->move_voltage(to_Mvolt(left_voltage));
            right_motors->move_voltage(to_Mvolt(right_voltage));
        }
    }

    // move robot based on left and right velocities
    void moveArcade(Voltage linear_output, Voltage angular_output) {
        // TODO: implement
    }
};

} // namespace blazing
