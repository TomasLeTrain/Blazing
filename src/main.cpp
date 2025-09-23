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
#include "blazing/tolerances.hpp"
#include "blazing/trackers/arc_odom.hpp"
#include "blazing/trackers/simple_odom.hpp"
#include "blazing/utils.hpp"
#include "pros/misc.h"
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
pros::Controller master(pros::E_CONTROLLER_MASTER);
// ScaledImu imu(1, (360.0 + 3.8) / 360.0);

using namespace blazing;

// tracker stuff
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

ForwardsTracker
  left_motor_tracker(&left_motors, -track_width / 2, wheel_diameter, 450_rpm);

ForwardsTracker right_motor_tracker(&right_motors,
                                    track_width / 2,
                                    wheel_diameter,
                                    final_rpm);

pros::Rotation forwards_rotation_sensor(6);
pros::Rotation sideways_rotation_sensor(7);

ForwardsTracker forwards_tracker(&forwards_rotation_sensor, 0_in, 1.96_in);
SidewaysTracker sideways_tracker(&sideways_rotation_sensor, 0_in, 1.96_in);

ArcOdomTracker arc_pose_tracker({ forwards_tracker,
                                  left_motor_tracker,
                                  right_motor_tracker },
                                { sideways_tracker },
                                { TrackingImu(&imu) });

// controller stuff
PID<Length, Voltage> lateral_pid(6,
                                 0,
                                 3,
                                 5,
                                 // std::nullopt,
                                 127,
                                 50_msec,
                                 1_in,
                                 (1.0 / 127.0) * volt);

PID<Angle, Voltage>
  angular_pid(2.8, 0.0, 5, 10, 127, 50_msec, (1_stDeg), (1.0 / 127.0) * volt);

Controllers controllers(
  // pid controllers
  PIDLinearController(lateral_pid),
  PIDAngularController(angular_pid),

  // slew controllers
  LinearSlewController(0.3_volt),
  AngularSlewController(0.3_volt),

  // voltage constraints controllers
  LinearVoltageClampController(),
  AngularVoltageClampController());

// tolerance stuff
Tolerances linearTolerances(200_msec,
                            ErrorTolerance { 3_in },
                            VelocityTolerance { 10_inps });
// HalfCircleTolerance { 1_in });

Tolerances angularTolerances(150_msec,
                             ErrorTolerance { 4_stDeg },
                             VelocityTolerance { 40_degps });

// large tolerances
Tolerances largeLinearTolerances(1_sec,
                                 ErrorTolerance { 5_in },
                                 VelocityTolerance { 30_inps });

Tolerances largeAngularTolerances(1_sec,
                                  ErrorTolerance { 30_stDeg },
                                  VelocityTolerance { 1000_degps });

// chain tolerances
Tolerances chainLinearTolerances(1_sec, ErrorTolerance { 14_in });

Tolerances chainAngularTolerances(1_sec, ErrorTolerance { 30_stDeg });

normalLargeChainTolerances tolerances(linearTolerances,
                                      angularTolerances,
                                      largeLinearTolerances,
                                      largeAngularTolerances,

                                      chainLinearTolerances,
                                      chainAngularTolerances);

Chassis chassis(drivetrain, arc_pose_tracker, tolerances);

RunExecutor run;
AsyncExecutor async;

auto chain_lerp = [](Voltage a, Voltage b, double t) -> Voltage {
    return (1 - t) * a + t * b;

    // return b;

    // if (t >= 0.5) {
    //     return b;
    // } else {
    //     return a;
    // }
};

auto angular_linear_func = [](Angle angle) -> double {
    // reduces the domain to [0,pi]
    angle = units::abs(units::constrainAngle180(angle));

    // defined on the range [0,pi/2]
    auto func = [](double x) -> double {
        double poly = 0.0001;
        if (x < 1.224747) {
            // simple polynomial that delays linear output until angle error is
            // small
            poly = 1.0 - 2.0 * (x * x) + 1.08866 * (x * x * x);
        }
        // return 0.00001;
        return 0.7 * poly + std::cos(x) * 0.3;
    };

    // makes this function apply on the range [0,pi]
    if (angle <= rot / 2.0) {
        return func(angle.internal());
    } else {
        return -func(M_PI - angle.internal());
    }
};

MotionBuilder mb(chassis, controllers);

ChainedExecutor chain(100_msec, chain_lerp);

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
    mb.setMoveToModifier([](auto moveTo) {
        return moveTo.customAngularLinearFunc(angular_linear_func);
    });

    mb.setBoomerangModifier([](auto boomerang) {
        return boomerang.customAngularLinearFunc(angular_linear_func);
    });

    pros::delay(100);

    std::vector<units::Pose> poses;

    pros::Task([&]() {
        bool disabled = false;
        while (true) {
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
                disabled = true;

            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
                // print results
                std::cout << "\n\nposes:\n";
                for (auto pose : poses) {
                    std::cout << pose.x << "," << pose.y << ","
                              << pose.orientation << '\n';
                }
                break;
            }

            if (!disabled) {
                poses.push_back(
                  { pose_tracker.getPosition(), pose_tracker.getAngle() });
            }

            pros::delay(20);
        }
    });

    // pose_tracker.setPose({ 0_in, 0_in, 0_stDeg });
    arc_pose_tracker.setPose({ 0_in, 0_in, 0_stDeg });

    mb.turnTo(90) | run;
    // mb.moveTo(24, 24).reverse() | chain;
    // mb.moveTo(-24, 24) | chain;
    // mb.moveTo(0, 0).reverse() | chain;
    mb.boomerang(24, 24, 0).withLead(0.4).closeThreshold(14_in) | run;

    // chain.wait();
    // chain.waitUntil([&]() -> bool {
    // 	return pose_tracker.getDistanceTraveled() > 80_in;
    // });

    // mb.moveTo(20, -20).setChainTime(10_msec) | run;
    // mb.moveTo(20, -20) | run;
    //
    // mb.moveTo(20, -20) | run;
    // mb.moveTo(24, 24) | run;

    while (true) {
        Voltage dir = from_volt(master.get_analog(ANALOG_LEFT_Y) / 127.0);
        Voltage turn = -from_volt(master.get_analog(ANALOG_RIGHT_X) / 127.0);

        drivetrain.moveArcade(dir, turn);

        pros::delay(20);
    }
}
