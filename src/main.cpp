#include "main.h"
#include "blazing/chassis.hpp"
#include "blazing/controllers.hpp"
#include "blazing/drivetrain.hpp"
#include "blazing/feedback/pid.hpp"
#include "blazing/motion_builder.hpp"
#include "blazing/executor.hpp"
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

const KP_t<Length, Voltage> LKP = (1_volt / 1_in);
const KI_t<Length, Voltage> LKI = (1_volt / 1_sec / 1_in);
const KD_t<Length, Voltage> LKD = (1_volt * 1_sec / 1_in);
const Length LWindup = 1_in;

const KP_t<Angle, Voltage> AKP = (1_volt / 1_stDeg);
const KI_t<Angle, Voltage> AKI = (1_volt / 1_sec / 1_stDeg);
const KD_t<Angle, Voltage> AKD = (1_volt * 1_sec / 1_stDeg);
const Angle AWindup = 1_stDeg;

PID<Length, Voltage>
  lateral_pid(4 * LKP, 2 * LKI, 4 * LKD, 4_in, 10_volt, 10_volt);
PID<Angle, Voltage> angular_pid(4 * AKP, 2 * AKI, 4 * AKD, std::nullopt);

Controllers<PIDLinearController, PIDAngularController> controllers(lateral_pid,
                                                                   angular_pid);

// TODO: make sure a motion won't run at the same time as another one
// could be implemented by using mutexes on the drivetrain

pros::MotorGroup left_motors({ 1 });
pros::MotorGroup right_motors({ 2 });

DifferentialDrivetrain drivetrain(&left_motors, &right_motors);

PoseTracker pose_tracker;

Tolerances linearTolerances(300_msec,
                            ErrorTolerance<Length> { 1_in },
                            VelocityTolerance<Length> { 1_inps },
                            HalfCircleTolerance { 1_in });
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

void opcontrol() {
	std::cout << "hello world!" << std::endl;
    mb.moveTo(2_in, 3_in)
      .reverse()
      .lateral_kD(12 * LKD)
      .angular_kI(0.02 * AKI)
      .reverse()

      .angularErrorTolerance(4_stDeg)
      .linearErrorTolerance(4_in)
      .largeLinearErrorTolerance(4_in)
      .largeAngularErrorTolerance(4_stDeg)

      .angularVelocityTolerance(4_stDeg / 1_sec)
      .linearVelocityTolerance(4_in / 1_sec)
      .largeLinearVelocityTolerance(4_in / 1_sec)
      .largeAngularVelocityTolerance(4_stDeg / 1_sec)

      .linearToleranceDuration(400_msec)
      .angularToleranceDuration(700_msec)
      .largeLinearToleranceDuration(400_msec)
      .largeAngularToleranceDuration(700_msec)
		| run;

	std::cout << "erm!" << std::endl;

    moveTo(controllers, chassis, 2, 3)
      .lateral_kD(12 * LKD)
      .angular_kI(0.02 * AKI)
      .reverse()
      .async();
	std::cout << "more!" << std::endl;

    moveTo(controllers, chassis, 2, 3).reverse().run();
	std::cout << "wowskers!" << std::endl;
}
