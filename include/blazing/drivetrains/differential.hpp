#pragma once

#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/utils.hpp"
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

    void moveVoltages(std::vector<Voltage> voltages) override {
        // if voltages are invalid then the .at should throw an error
        moveTank(voltages.at(0), voltages.at(1));
    }

    std::vector<Voltage> getVoltages() override {
        return voltages;
    }

    // move robot based on left and right velocities
    void moveTank(Voltage left_voltage, Voltage right_voltage) {
        voltages = { left_voltage, right_voltage };
        if (!enabled) return;

        if (left_motors != nullptr && right_motors != nullptr) {

            // std::cout << "[diff] volts: " << to_mvolt(12 * left_voltage)
            //           << " " << to_mvolt(12 * right_voltage) << std::endl;

            left_motors->move_voltage(to_mvolt(12 * left_voltage));
            right_motors->move_voltage(to_mvolt(12 * right_voltage));
        }
    }

    // move robot based on left and right velocities
    // positive angular -> turns left
    void moveArcade(Voltage linear_output, Voltage angular_output) {
        // std::cout << "[differential] arcade lin/ang: " << linear_output << " "
        //           << angular_output << std::endl;

        std::array<Voltage, 2> saturated_voltages {
            linear_output - angular_output,
            linear_output + angular_output
        };

        // std::cout << "[differential] saturated: " << saturated_voltages.at(0)
        //           << " " << saturated_voltages.at(1) << std::endl;

        // normalizes voltages to [-1, 1]
        auto [left_voltage, right_voltage] =
          desaturate(saturated_voltages, 1_volt);

        // std::cout << "[differential] desaturated: " << left_voltage << " "
        //           << right_voltage << std::endl;

        moveTank(left_voltage, right_voltage);
    }
};
} // namespace blazing
