#include "main.h"
#include "blazing/chassis.hpp"
#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/feedback/pid.hpp"
#include "blazing/controllers/slew.hpp"
#include "blazing/controllers/voltage_clamp.hpp"
#include "blazing/drivetrains/differential.hpp"
#include "blazing/executor.hpp"
#include "blazing/motion_builder.hpp"
#include "blazing/motions/distance_at_heading.hpp"
#include "blazing/motions/moveTo.hpp"
#include "blazing/motions/turnTo.hpp"
#include "blazing/trackers/simple_odom.hpp"
#include "blazing/utils.hpp"
#include "pros/motor_group.hpp"
#include <cstdio>
#include <iostream>
#include <ostream>
#include <span>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

pros::MotorGroup left_motors({ -11, -14, 13 });
pros::MotorGroup right_motors({ 15, 16, -10 });
pros::Imu imu(1);
// ScaledImu imu(1, (360.0 + 3.8) / 360.0);

using namespace blazing;

PID<Length, Voltage>
  lateral_pid(6, 0, 3, std::nullopt, 1, 50_msec, 1_in, (1.0 / 127.0) * volt);

PID<Angle, Voltage> angular_pid(2.8,
                                0.0,
                                5,
                                std::nullopt,
                                1,
                                50_msec,
                                (1_stDeg),
                                (1.0 / 127.0) * volt);

Controllers controllers {
    // pid controllers
    PIDLinearController(lateral_pid),
    PIDAngularController(angular_pid),
    // slew controllers

    LinearSlewController(0.3_volt),

    AngularSlewController(0.4_volt),

    // // min/max voltage controllers
    LinearVoltageClampController(1.0_volt),
    AngularVoltageClampController(1.0_volt),
};

DifferentialDrivetrain drivetrain(&left_motors, &right_motors);

Length track_width = 10.5_in;
Length wheel_diameter = 3.25_in;
AngularVelocity final_rpm = 450_rpm;

SimpleOdomTracker pose_tracker(&left_motors,
                               &right_motors,
                               &imu,
                               track_width,
                               wheel_diameter,
                               final_rpm);

Tolerances linearTolerances(200_msec,
                            ErrorTolerance { 3_in },
                            VelocityTolerance { 10_inps });
// HalfCircleTolerance { 1_in });

Tolerances angularTolerances(150_msec,
                             ErrorTolerance { 3_stDeg },
                             VelocityTolerance { 30_degps });

Tolerances largeLinearTolerances(500_msec,
                                 // ErrorTolerance { 6_in },
                                 ErrorTolerance { 14_in },
                                 VelocityTolerance { 50_inps });
// HalfCircleTolerance { 10_in });

Tolerances largeAngularTolerances(1_sec,
                                  // ErrorTolerance { 5_stDeg },
                                  // VelocityTolerance { 40_degps });
                                  ErrorTolerance { 30_stDeg },
                                  VelocityTolerance { 1000_degps });

DefaultTolerances tolerances(linearTolerances,
                             angularTolerances,
                             largeLinearTolerances,
                             largeAngularTolerances);

Chassis chassis(drivetrain, pose_tracker, tolerances);

RunExecutor run;
AsyncExecutor async;

auto chain_lerp = [](Voltage a, Voltage b, double t) -> Voltage {
    return (1 - t) * a + t * b;
};

auto angular_linear_func = [](Angle angle) -> double {
    // reduces the domain to [0,pi]
    angle = units::abs(units::constrainAngle180(angle));

    // defined on the range [0,pi/2]
    auto func = [](double x) -> double {
        if (x < 1.224747) {
            // simple polynomial that delays linear output until angle error is
            // small
            return 1.0 - 2.0 * (x * x) + 1.08866 * (x * x * x);
        }
        return 0.00001;
    };

    // makes this function apply on the range [0,pi]
    if (angle <= rot / 2.0) {
        return func(angle.internal());
    } else {
        return -func(M_PI - angle.internal());
    }
};

MotionBuilder mb(chassis, controllers);

ChainedExecutor chain(300_msec, chain_lerp);

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");

    pros::lcd::register_btn1_cb(on_center_button);

    imu.reset(true);
}

void opcontrol() {
    // needed for async/chain motions to run
    async.init();
    chain.init();

    pros::Task([&]() {
        while (true) {
            pose_tracker.update();
            pros::delay(10);
        }
    });

    // change all moveTo movements to use custom angularLinear function and go
    // in reverse
    mb.setMoveToModifier([=](auto moveTo) {
        return moveTo.customAngularLinearFunc(angular_linear_func).reverse();
    });

    pros::delay(100);

    // pose_tracker.setPose({ 0_in, 0_in, 90_stDeg });

    //   while (true) {
    //       auto position = pose_tracker.getPosition();
    //       auto angle = pose_tracker.getAngle();
    //       std::cout << "x: " << position.x << ", y: " << position.y
    //                 << ", theta: " << angle << std::endl;
    // pros::delay(50);
    //   }

    pose_tracker.setPose({ 12_in, -12_in, 135_stDeg });

    mb.moveTo(20, -20).setChainTime(10_msec) | run;
    mb.moveTo(20, -20) | run;

    mb.moveTo(20, -20) | chain;
    mb.moveTo(24, 24) | chain;
}
