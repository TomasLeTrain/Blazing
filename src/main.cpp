#include "main.h"
#include "blazing/chassis.hpp"
#include "blazing/controllers.hpp"
#include "blazing/drivetrain.hpp"
#include "blazing/executor.hpp"
#include "blazing/feedback/pid.hpp"
#include "blazing/motion_builder.hpp"
#include "blazing/motions/moveTo.hpp"
#include "pros/motor_group.hpp"
#include <iostream>
#include <ostream>

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
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");

    pros::lcd::register_btn1_cb(on_center_button);
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

using namespace blazing;

PID<Length, Voltage> lateral_pid(4, 2, 4, 4, 10, 10, 1_sec, 1_in, 1_volt);
PID<Angle, Voltage> angular_pid(4,
                                2,
                                4,
                                std::nullopt,
                                std::nullopt,
                                std::nullopt,
                                1_sec,
                                1_stDeg,
                                1_volt);

Controllers<PIDLinearController, PIDAngularController> controllers(lateral_pid,
                                                                   angular_pid);

// TODO: make sure a motion won't run at the same time as another one
// could be implemented by using mutexes on the drivetrain

pros::MotorGroup left_motors({ 1 });
pros::MotorGroup right_motors({ 2 });

DifferentialDrivetrain drivetrain(&left_motors, &right_motors);

PoseTracker pose_tracker;

Tolerances linearTolerances(300_msec, ErrorTolerance<Length> { 1_in });
// VelocityTolerance<Length> { 1_inps },
// HalfCircleTolerance { 1_in });
Tolerances angularTolerances(300_msec,
                             ErrorTolerance<Angle> { 3_stDeg },
                             VelocityTolerance<Angle> { 2_degps });

Tolerances largeLinearTolerances(10_msec,
                                 ErrorTolerance<Length> { 10_in },
                                 VelocityTolerance<Length> { 10_inps },
                                 HalfCircleTolerance { 10_in });
Tolerances largeAngularTolerances(10_msec,
                                  ErrorTolerance<Angle> { 6_stDeg },
                                  VelocityTolerance<Angle> { 4_degps });

DefaultTolerances tolerances(linearTolerances,
                             angularTolerances,
                             largeLinearTolerances,
                             largeAngularTolerances);

Chassis chassis(drivetrain, pose_tracker, tolerances);

MotionBuilder mb(chassis, controllers);

RunExecutor run;
AsyncExecutor async;
ChainedExecutor chain(0.3_sec);

void opcontrol() {
    // needed for async motions to run
    async.init();
    chain.init();

    std::cout << "hello world!" << std::endl;
    mb.moveTo(2_in, 3_in)
        .reverse()
        .lateral_kD(12 * lateral_pid.UKD)
        .angular_kI(0.02)
        .reverse()

        .angularErrorTolerance(4_stDeg)
        .linearErrorTolerance(4_in)
        .largeLinearErrorTolerance(4_in)
        .largeAngularErrorTolerance(4_stDeg)

        .angularVelocityTolerance(4_stDeg / 1_sec)
        // .linearVelocityTolerance(4_in / 1_sec)
        .largeLinearVelocityTolerance(4_in / 1_sec)
        .largeAngularVelocityTolerance(4_stDeg / 1_sec)

        .linearToleranceDuration(500_msec)
        .angularToleranceDuration(700_msec)
        .largeLinearToleranceDuration(400_msec)
        .largeAngularToleranceDuration(700_msec)
        .withTimeout(2_sec) |
      run;

    std::cout << "erm!" << std::endl;

    moveTo(controllers, chassis, 2, 3)
        .lateral_kD(12)
        .angular_kI(0.02 * angular_pid.UKI)
        .reverse() |
      async;

    std::cout << "this is async" << std::endl;

    moveTo(controllers, chassis, 2, 3).reverse() | async;

    std::cout << "still async!" << std::endl;

    // wait until all async movements are done
    async.wait();

    std::cout << "wowskers!" << std::endl;

    mb.moveTo(2_in, 3_in) | chain;
    mb.moveTo(4_in, 3_in) | chain;
    chain.wait();

    turnTo(controllers, chassis, 2_stDeg) | run;
	mb.turnTo(2_stDeg) | run;

    // mb.turnTo(2_stDeg);
}
