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

PID<Length, Voltage> lateral_pid(1,
                                 0,
                                 0,
                                 std::nullopt,
                                 std::nullopt,
                                 50_msec,
                                 1_in,
                                 (1.0 / 127.0) * volt);

PID<Angle, Voltage> angular_pid(3,
                                0.0,
                                5,
                                std::nullopt,
                                std::nullopt,
                                50_msec,
                                (1_stDeg),
                                (1.0 / 127.0) * volt);

Controllers controllers {
    // pid controllers
    PIDLinearController(lateral_pid),
    PIDAngularController(angular_pid),
    // slew controllers
    LinearSlewController(0.5_volt, 0.5_volt),
    AngularSlewController(0.8_volt, 0.9_volt),
    // min/max voltage controllers
    LinearVoltageClampController(0.9_volt),
    AngularVoltageClampController(0.8_volt),
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

Tolerances linearTolerances(300_msec,
                            ErrorTolerance { 1_in },
                            VelocityTolerance { 1_inps },
                            HalfCircleTolerance { 1_in });

Tolerances angularTolerances(200_msec,
                             ErrorTolerance { 3_stDeg },
                             VelocityTolerance { 20_degps });

Tolerances largeLinearTolerances(10_msec,
                                 ErrorTolerance { 10_in },
                                 VelocityTolerance { 10_inps },
                                 HalfCircleTolerance { 10_in });

Tolerances largeAngularTolerances(1_sec,
                                  ErrorTolerance { 5_stDeg },
                                  VelocityTolerance { 40_degps });

DefaultTolerances tolerances(linearTolerances,
                             angularTolerances,
                             largeLinearTolerances,
                             largeAngularTolerances);

Chassis chassis(drivetrain, pose_tracker, tolerances);

MotionBuilder mb(chassis, controllers);

RunExecutor run;
AsyncExecutor async;
ChainedExecutor chain(300_msec);

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");

    pros::lcd::register_btn1_cb(on_center_button);

	imu.reset(true);

}

void opcontrol() {
    // needed for async motions to run
    async.init();
    chain.init();

    pros::Task([&]() {
        while (true) {
            pose_tracker.update();
            pros::delay(10);
        }
    });

    pros::delay(100);

    pose_tracker.setPose({ 0_in, 0_in, 0_stDeg });
	imu.set_rotation(90);

    mb.turnTo(90_stDeg) | chain;
    mb.turnTo(180_stDeg) | chain;

    // mb.turnTo(90_stDeg).withTimeout(10_sec) | run;
    // mb.turnTo(90_stDeg).withTimeout(1_sec) | run;

    // mb.moveTo(2_in, 3_in)
    //     .lateral_kD(12 * lateral_pid.UKD)
    //     .angular_kI(0.02)
    //     .reverse()
    //
    //     .angularErrorTolerance(4_stDeg)
    //     .linearErrorTolerance(4_in)
    //     .largeLinearErrorTolerance(4_in)
    //     .largeAngularErrorTolerance(4_stDeg)
    //
    //     .angularVelocityTolerance(4_degps)
    //     .linearVelocityTolerance(4_inps)
    //     .largeLinearVelocityTolerance(4_inps)
    //     .largeAngularVelocityTolerance(4_degps)
    //
    //     .linearToleranceDuration(500_msec)
    //     .angularToleranceDuration(700_msec)
    //     .largeLinearToleranceDuration(400_msec)
    //     .largeAngularToleranceDuration(700_msec)
    //     .withTimeout(2_sec) |
    //   run;
    //
    // std::cout << "erm!" << std::endl;
    //
    // moveTo(controllers, chassis, 2, 3)
    //     .lateral_kD(12)
    //     .angular_kI(0.02 * angular_pid.UKI)
    //     .reverse() |
    //   async;
    //
    // moveTo(controllers, chassis, 2, 3).reverse() | async;
    //
    // // wait until all async movements are done
    // async.wait();
    //
    // mb.moveTo(2_in, 3_in) | chain;
    // mb.moveTo(4_in, 3_in) | chain;
    // chain.wait();
    //
    // turnTo(controllers, chassis, 2_stDeg) | run;
    // mb.turnTo(2_stDeg) | run;
    //
    // distanceAtHeading(controllers, chassis, 0_in, 2_stDeg) | run;
    // distanceAtHeading(controllers, chassis, 10_in) | run;
    // mb.distanceAtHeading(10_in) | run;
    //
    // mb.turnTo(2_stDeg);
}
