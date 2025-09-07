#pragma once

#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/util.hpp"
#include "pros/motor_group.hpp"
#include "units/units.hpp"
#include <array>

namespace blazing {

class DifferentialDrivetrain : public ChainableDrivetrain {
  private:
    pros::MotorGroup* left_motors;
    pros::MotorGroup* right_motors;

    std::vector<Voltage> voltages { 0_volt, 0_volt };

  public:
    DifferentialDrivetrain(pros::MotorGroup* left_motors,
                           pros::MotorGroup* right_motors)
        : left_motors(left_motors),
          right_motors(right_motors) {}

    void moveVoltage(std::vector<Voltage> voltages) override {
        // if voltages are invalid then the .at should throw an error
        moveTank(voltages.at(0), voltages.at(1));
    }

    std::vector<Voltage> getVoltages() override {
        return voltages;
    }

    // move robot based on left and right velocities
    void moveTank(Voltage left_voltage, Voltage right_voltage) {
        if (left_motors != nullptr && right_motors != nullptr && enabled) {
            left_motors->move_voltage(to_Mvolt(left_voltage));
            right_motors->move_voltage(to_Mvolt(right_voltage));
        }
        voltages = { left_voltage, right_voltage };
    }

    // move robot based on left and right velocities
	// positive angular -> turns left
    void moveArcade(Voltage linear_output, Voltage angular_output) {
        std::array<Voltage, 2> saturated_voltages {
            linear_output - angular_output,
            linear_output + angular_output
        };

        // normalizes to [-1, 1]
        auto [left_voltage, right_voltage] =
          desaturate(saturated_voltages, 1_volt);

        moveTank(left_voltage, right_voltage);
    }
};
} // namespace blazing
