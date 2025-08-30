#include "main.h"
#include "blazing/drivetrain.hpp"
#include "blazing/moveTo.hpp"
#include "blazing/pid.hpp"
#include "pros/motor_group.hpp"

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

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    using namespace blazing;

    const auto LKP = (1_volt / 1_in);
    const auto LKI = (1_volt / 1_sec / 1_in);
    const auto LKD = (1_volt * 1_sec / 1_in);
    const auto LWindup = (1_in * 1_sec);

    const auto AKP = (1_volt / 1_stDeg);
    const auto AKI = (1_volt / 1_sec / 1_stDeg);
    const auto AKD = (1_volt * 1_sec / 1_stDeg);
    const auto AWindup = (1_stDeg * 1_sec);

    PID<Length, Voltage> lateral_pid(4 * LKP,
                                     2 * LKI,
                                     4 * LKD,
                                     4_in,
                                     10_volt,
                                     10_volt);
    PID<Angle, Voltage> angular_pid(4 * AKP, 2 * AKI, 4 * AKD, std::nullopt);

    // TODO: make sure a motion won't run at the same time as another one
    // could be implemented by using mutexes on the drivetrain

    PoseTracker pose_tracker;
    PositionOnlyTracker position_tracker;

    Tolerances customTolerances(10_sec,
                                ErrorTolerance<Length> { 10_in },
                                VelocityTolerance<Length> { 10_inps },
                                HalfCircleTolerance { 1_m });

    pros::MotorGroup left_motors({ 1 });
    pros::MotorGroup right_motors({ 2 });
    DifferentialDrivetrain drivetrain(&left_motors, &right_motors);

    Chassis chassis(drivetrain, pose_tracker, customTolerances);

    moveTo(lateral_pid, angular_pid, chassis, 2, 3)
      .lateral_kD(12 * LKD)
      .reverse()
      .async();
    moveTo(lateral_pid, angular_pid, chassis, 2, 3).reverse().run();
}
