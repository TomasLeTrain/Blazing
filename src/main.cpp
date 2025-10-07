// this should always be first
#include "blazing/utils.hpp"
#include "pch.h"
//
#include "blazing/api.hpp"
#include "main.h"

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

class ScaledIMU : public pros::IMU {
  public:
    ScaledIMU(int port, double scalar = 1.0)
        : pros::IMU(port),
          m_scalar(scalar),
          m_port() {}

    ScaledIMU(const pros::IMU& other, double scalar = 1.0)
        : pros::IMU(other),
          m_scalar(scalar),
          m_port(other.get_port()) {}

    virtual int32_t reset(bool blocking = false) {
        std::lock_guard lock(m_mutex);

        m_offset = 0;
        return pros::IMU::reset(blocking);
    }

    virtual double get_rotation() const {
        std::lock_guard lock(m_mutex);

        double raw = pros::c::imu_get_rotation(m_port);
        if (raw == INFINITY) return INFINITY;
        return raw * m_scalar + m_offset;
    }

    virtual int set_rotation(double new_rotation) {
        std::lock_guard lock(m_mutex);

        double curr_raw = this->get_rotation();
        if (curr_raw == INFINITY) return INT32_MAX;

        m_offset += new_rotation - curr_raw;
        return 0;
    }

  private:
    const double m_scalar;
    int m_port;

    mutable pros::Mutex m_mutex;

    double m_offset = 0;
};

pros::MotorGroup left_motors({ -11, -14, 13 });
pros::MotorGroup right_motors({ 15, 16, -10 });
ScaledIMU imu(1, (360.0 + 3.8) / 360.0);
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
  left_motor_tracker(&left_motors, -track_width / 2, wheel_diameter, final_rpm);

ForwardsTracker right_motor_tracker(&right_motors,
                                    track_width / 2,
                                    wheel_diameter,
                                    final_rpm);

pros::Rotation forwards_rotation_sensor(-20);
pros::Rotation sideways_rotation_sensor(5);

ForwardsTracker forwards_tracker(&forwards_rotation_sensor, -0.55_in, 1.996_in);
SidewaysTracker sideways_tracker(&sideways_rotation_sensor, -0.2_in, 1.96_in);

ArcOdomTracker arc_pose_tracker({ forwards_tracker,
                                  left_motor_tracker,
                                  right_motor_tracker },
                                { sideways_tracker },
                                { TrackingImu(&imu) });

// controller stuff
PID<Length, Voltage> linear_pid(4.7,
                                0.0,
                                1,
                                5,
                                // std::null8opt,
                                127,
                                50_msec,
                                1_in,
                                (1.0 / 127.0) * volt);

PID<Angle, Voltage>
  angular_pid(2.8, 0.0, 5, 10, 127, 50_msec, (1_stDeg), (1.0 / 127.0) * volt);

Controllers controllers(
  // pid controllers
  PIDLinearController(linear_pid),
  PIDAngularController(angular_pid),

  // slew controllers
  LinearSlewController(0.2_volt),
  AngularSlewController(0.3_volt),

  // voltage constraints controllers
  LinearVoltageClampController(),
  AngularVoltageClampController());

// tolerance stuff
Tolerances linearTolerances(150_msec, ErrorTolerance { 2.5_in }
                            // VelocityTolerance { 20_inps });
);
// HalfCircleTolerance { 1_in });

Tolerances angularTolerances(150_msec, ErrorTolerance { 4_stDeg }
                             // VelocityTolerance { 20_degps });
);

// large tolerances
Tolerances largeLinearTolerances(1_sec, ErrorTolerance { 6_in }
                                 // VelocityTolerance { 40_inps }
);

Tolerances largeAngularTolerances(1_sec, ErrorTolerance { 15_stDeg });

// chain tolerances
Tolerances chainLinearTolerances(1_sec, ErrorTolerance { 6_in });
Tolerances chainAngularTolerances(1_sec, ErrorTolerance { 15_stDeg });

// Tolerances chainLinearTolerances(1_sec, ErrorTolerance { 0_in });
// Tolerances chainAngularTolerances(1_sec, ErrorTolerance { 0_stDeg });

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

    pros::Task([&]() {
        while (true) {
            arc_pose_tracker.update();
            pros::delay(10);
        }
    });

    // default a timeout
    mb.setTurnToModifier([](auto turnTo) {
        return turnTo.timeout(5_sec);
    });

    mb.setDistanceAtHeadingModifier([](auto distanceAtHeading) {
        return distanceAtHeading.timeout(5_sec);
    });

    mb.setMoveToModifier([](auto moveTo) {
        // return moveTo.customAngularLinearFunc(angular_linear_func);
        return moveTo.k_lat(0.3 * rad / m).timeout(5_sec);
    });

    mb.setBoomerangModifier([](auto boomerang) {
        // return boomerang.customAngularLinearFunc(angular_linear_func);
        // return boomerang.k_lat();
        return boomerang.timeout(7_sec);
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
                    std::cout << to_in(pose.x) << "," << to_in(pose.y) << ","
                              << to_stDeg(pose.orientation) << '\n';
                }
                break;
            }

            if (!disabled) {
                poses.push_back({ arc_pose_tracker.getPosition(),
                                  arc_pose_tracker.getAngle() });
            }

            pros::delay(20);
        }
    });

    pros::delay(100);

    // mb.arc(24_in, arc_pose_tracker.getAngle(), 90_stDeg)
    //     .angular_clampMaxVoltage(0.5_volt) |
    //     // .angular_clampMaxVoltage(0.5_volt) |
    //   run;

    // mb.boomerang(-24, 48, 180)
    //     .lead(0.4, 0.38)
    //     .lead2DistThreshold(7_in)
    //     .closeThreshold(7_in)
    //     .timeout(7_sec)
    //     .linear_kd(linear_pid.get_kd() * 0.7) |
    //   run;

    Time start_time = now();

    arc_pose_tracker.setPose({ -63_in, -16.7_in, 90_stDeg });

    // pull matchloader down
    mb.moveTo(-63, 18) | chain;

    // chain.wait();

    // while (true) {
    //     pros::lcd::print(0,
    //                      "%f %f %f",
    //                      to_in(arc_pose_tracker.getPosition().x),
    //                      to_in(arc_pose_tracker.getPosition().y),
    //                      to_stDeg(arc_pose_tracker.getAngle()));
    //     pros::delay(10);
    // }

    // go towards top left ball cluster
    mb.boomerang(-32, 31.7, 315).lead(0.35).linear_clampMaxVoltage(0.7_volt) |
      chain;
    size_t top_left_cluster = chain.getCurrentIndex();

    // go to top center
    // mb.boomerang(-13, 12, 315) | chain;
    mb.moveTo(-13, 12) | chain;

    // pros::delay(1000);
    // pull matchloader up

    // wait for boomerang to finish
    chain.waitUntilIndex(top_left_cluster);

    // finished the boomerang, pull matchloader down

    // wait to get to goal
    chain.wait();

    // while (true) {
    //     pros::lcd::print(0,
    //                      "%f %f %f",
    //                      to_in(arc_pose_tracker.getPosition().x),
    //                      to_in(arc_pose_tracker.getPosition().y),
    //                      to_stDeg(arc_pose_tracker.getAngle()));
    //     pros::delay(10);
    // }

    // score top center

    // back up and go to bottom right cluster
    mb.arc(-10_in,
           arc_pose_tracker.getAngle(),
           270_stDeg,
           AngularDirection::RIGHT)
        .angular_clampMaxVoltage(0.6_volt) |
      chain;

    // bottom-left middle ball cluster
    mb.boomerang(-21.5, -13.4, 260) | chain;

    size_t bottom_left_cluster_index = chain.getCurrentIndex();

    // go to matchloader
    mb.boomerang(-52.4, -46.7, 180).linear_clampMaxVoltage(0.5_volt) | chain;
    // the matchloader is already pulled down at this point,
    // don't have to worry about it
    mb.moveTo(-57, -46.7) | chain;

    // waits until gets to cluster to pull matchloader down
    chain.waitUntilIndex(bottom_left_cluster_index);

    // pull matchloader down

    // wait until all queued motions stop
    chain.wait();

    pros::delay(1000);

    mb.moveTo(-30.8, -47.1).reverse().linear_accelSlew(0.1_volt) | run;
    //
    // score on long goal
    //

    // turn around and go towards matchloader
    mb.arc(track_width,
           arc_pose_tracker.getAngle(),
           0_stDeg,
           AngularDirection::LEFT)
        .angular_clampMaxVoltage(1.0_volt) |
      chain;

    chain.wait();

    // while (true) {
    //     pros::lcd::print(0,
    //                      "%f %f %f",
    //                      to_in(arc_pose_tracker.getPosition().x),
    //                      to_in(arc_pose_tracker.getPosition().y),
    //                      to_stDeg(arc_pose_tracker.getAngle()));
    //     pros::delay(10);
    // }

    // go to other side of the field, close to the wall
    mb.moveTo(22.41, -60) | chain;

    // go to matchloader
    mb.boomerang(52.4, -46.7, 0) | chain;
    mb.moveTo(57, -46.7) | chain;
    chain.wait();

    // matchload

    mb.moveTo(30.8, -47.1).reverse() | run;
    //
    // score
    //

    mb.boomerang(62.2, -17, 90) | chain;
    // matchload down?
    mb.moveTo(63.4, -17) | chain;

    mb.boomerang(30.5, 27, 200) | chain;
    size_t top_right_cluster = chain.getCurrentIndex();

    chain.waitUntilIndex(top_right_cluster);

    // matchload down
    //
    // wait a bit, matchload up
    // pros::delay(100);

    // wait for all motions to complete
    chain.wait();

    // score bottom

    // back up
    mb.moveTo(30, 33) | chain;
    mb.boomerang(52.4, 46.7, 0) | chain;
    size_t top_right_matchloader = chain.getCurrentIndex();

    mb.moveTo(57, 46.7) | chain;

    chain.waitUntilIndex(top_right_matchloader);
    // pull matchloader down

    chain.wait();
    // get matchloader
    // pros::delay(1000);

    mb.moveTo(30.8, 47.1).reverse() | run;

    // go score on long goal, pull matchloader up
    // pros::delay(1000);

    // go to other side of long goal and matchloader
    mb.arc(track_width / 2 + 2_in,
           arc_pose_tracker.getAngle(),
           180_stDeg,
           AngularDirection::LEFT)
        .angular_clampMaxVoltage(0.9_volt) |
      chain;

    // go to other side of the field, close to the wall
    mb.moveTo(-22.41, 60) | chain;

    // go to matchloader
    mb.boomerang(-52.4, 46.7, 180) | chain;
    size_t top_left_matchloader = chain.getCurrentIndex();

    mb.moveTo(-57, 46.7) | chain;

    chain.waitUntilIndex(top_left_matchloader);
    // pull matchloader down

    chain.wait();

    // matchload

    mb.moveTo(-30.8, 47.1).reverse() | run;
    //
    // score
    //

    // finish, go to park :)
    mb.boomerang(-63, 24.5, 270) | chain;
    mb.moveTo(-63, 0) | chain;

    chain.wait();

    std::cout << "finished run in time: " << now() - start_time << std::endl;

    pros::delay(30);

    while (true) {
        Voltage dir = from_volt(
          master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0);
        Voltage turn = -from_volt(
          master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0);

        drivetrain.moveArcade(dir, turn);

        pros::lcd::print(0,
                         "%f %f %f",
                         to_in(arc_pose_tracker.getPosition().x),
                         to_in(arc_pose_tracker.getPosition().y),
                         to_stDeg(arc_pose_tracker.getAngle()));

        pros::delay(20);
    }
}
