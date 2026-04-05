#include "main.h"
#include "api.h"
#include "blazing/api.hpp"
#include "blazing/controllers/controllers.hpp"
#include "blazing/executor.hpp"
#include "blazing/utils.hpp"
#include "units/units.hpp"

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
void disabled() {
    std::cout << "disabled ran!" << std::endl;
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    std::cout << "competition initialize!" << std::endl;
}

class ScaledIMU : public pros::IMU {
  public:
    ScaledIMU(int port, double scalar = 1.0)
        : pros::IMU(port),
          m_scalar(scalar),
          m_port(port) {}

    ScaledIMU(const pros::IMU& other, double scalar = 1.0)
        : pros::IMU(other),
          m_scalar(scalar),
          m_port(other.get_port()) {}

    int32_t reset(bool blocking = false) {
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

Length track_width = 10.5_in;
Length wheel_diameter = 3.25_in;
AngularVelocity final_rpm = 450_rpm;

// tracker stuff
DifferentialDrivetrain
  drivetrain(&left_motors, &right_motors, wheel_diameter, final_rpm);

// SimpleOdomTracker pose_tracker(&left_motors,
//                                &right_motors,
//                                &imu,
//                                track_width,
//                                wheel_diameter,
//                                final_rpm);

ForwardsTracker
  left_motor_tracker(&left_motors, -track_width / 2, wheel_diameter, final_rpm);

ForwardsTracker right_motor_tracker(&right_motors,
                                    track_width / 2,
                                    wheel_diameter,
                                    final_rpm);

pros::Rotation forwards_rotation_sensor(-20);
pros::Rotation sideways_rotation_sensor(5);

ForwardsTracker forwards_tracker(&forwards_rotation_sensor, -0.44_in, 1.996_in);
SidewaysTracker sideways_tracker(&sideways_rotation_sensor, -0.15_in, 1.96_in);

TrackingImu tracking_imu(&imu);

ArcOdomTracker arc_pose_tracker({ &forwards_tracker,
                                  &left_motor_tracker,
                                  &right_motor_tracker },
                                { &sideways_tracker },
                                { &tracking_imu });

// controller stuff
PID<Length, Voltage> linear_pid(4.7,
                                0.0,
                                1,
                                5,
                                // std::nullopt,
                                127,
                                0.9,
                                50_msec,
                                1_in,
                                (1.0 / 127.0) * volt);

PID<Angle, Voltage> angular_pid(2.8,
                                0.0,
                                5,
                                10,
                                127,
                                0.9,
                                50_msec,
                                (1_stDeg),
                                (1.0 / 127.0) * volt);

PID<Length, LinearVelocity> linear_vel_pid(2.8,
                                           0.0,
                                           5,
                                           10,
                                           127,
                                           0.9,
                                           50_msec,
                                           (1_in),
                                           (1.0 / 127.0) * inps);

// PID<LinearVelocity, Voltage> vel_voltage_pid(2.8,
//                                              0.0,
//                                              5,
//                                              10,
//                                              127,
//                                              50_msec,
//                                              (1_inps),
//                                              (1.0 / 127.0) * volt);

// LinearVelocityFeedbackController<typename Controller>

struct dummyVelFeedforward : ControllerBase {
    Voltage update(LinearVelocity target, Time duration) {
        return 1_volt;
    }
};

dummyVelFeedforward vel_feedforward;

CascadedControllers<decltype(linear_vel_pid),
                    decltype(vel_feedforward),
                    Length,
                    LinearVelocity,
                    Voltage>
  cascadedControl(linear_vel_pid, vel_feedforward);

Controllers controllers(
  // PIDLinearController(linear_pid),
  PIDAngularController(angular_pid),

  LinearFeedbackController<decltype(cascadedControl)>(cascadedControl),

  // slew controllers
  LinearSlewController(0.2_volt, 0.08_volt),
  AngularSlewController(0.3_volt),

  // voltage constraints controllers
  // (included just so they can be set per motion)
  LinearVoltageClampController(),
  AngularVoltageClampController());

// tolerance stuff
Tolerances linearTolerances(150_msec,
                            ErrorTolerance { 2.5_in },
                            VelocityTolerance { 20_inps });
// HalfCircleTolerance { 1_in });

Tolerances angularTolerances(150_msec,
                             ErrorTolerance { 4_stDeg },
                             VelocityTolerance { 20_degps });

// large tolerances
Tolerances largeLinearTolerances(1_sec, ErrorTolerance { 6_in });
Tolerances largeAngularTolerances(1_sec, ErrorTolerance { 15_stDeg });

// chain tolerances
Tolerances chainLinearTolerances(1_sec, ErrorTolerance { 6_in });
Tolerances chainAngularTolerances(1_sec, ErrorTolerance { 15_stDeg });

normalLargeChainTolerances tolerances(linearTolerances,
                                      angularTolerances,
                                      largeLinearTolerances,
                                      largeAngularTolerances,

                                      chainLinearTolerances,
                                      chainAngularTolerances);

Chassis chassis(&drivetrain, &arc_pose_tracker, tolerances);

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
    // std::cout << "started initialize!" << std::endl;
    //
    // pros::lcd::initialize();
    // pros::lcd::set_text(1, "Hello PROS User!");
    //
    // pros::lcd::register_btn1_cb(on_center_button);
    //
    // imu.reset(true);
    //
    // // needed for async/chain motions to run
    // async.init();
    // chain.init();
}

void opcontrol() {
    std::cout << "running opcontrol!" << std::endl;

    auto test = [](Number angle_num) {
        Angle angle = angle_num * deg;
        auto target = angle;
        auto heading = 0 * deg;

        target = units::constrainAngle2pi(target);
        heading = units::constrainAngle2pi(heading);
        Angle error = units::constrainAngle180(target - heading);

        std::cout << "test: " << angle_num << std::endl;
        std::cout << "target: " << target << std::endl;
        std::cout << "heading:" << heading << std::endl;
        std::cout << "error:" << error << std::endl;

        auto first_test = angleError(angle, 0 * deg);
        auto test_direction =
          angleError(angle, 0 * deg, AngularDirection::RIGHT);
        std::cout << "test: " << angle_num << std::endl;
        std::cout << "no direction:" << first_test << std::endl;
        std::cout << "direction:" << test_direction << std::endl;
    };

    test(0);
    test(-90);
    test(90);
    test(-180);
    test(180);
    test(-270);
    test(270);

    // pros::Task([&]() {
    //     while (true) {
    //         pose_tracker.update();
    //         pros::delay(10);
    //     }
    // });

    pros::Task([&]() {
        while (true) {
            arc_pose_tracker.update();
            pros::delay(10);
        }
    });

    // default a timeout
    mb.setTurnToModifier([](auto turnTo) {
        // return turnTo.timeout(5_sec);
        return turnTo;
    });

    mb.setDistanceAtHeadingModifier([](auto distanceAtHeading) {
        // return distanceAtHeading.timeout(5_sec);
        return distanceAtHeading;
    });

    mb.setMoveToModifier([](auto moveTo) {
        // return moveTo.customAngularLinearFunc(angular_linear_func);
        // return moveTo.k_lat(0.3 * rad / m).timeout(5_sec);
        return moveTo;
    });

    mb.setBoomerangModifier([](auto boomerang) {
        // return boomerang.customAngularLinearFunc(angular_linear_func);
        // return boomerang.k_lat();
        return std::move(boomerang.k_lat(0.2, true).timeout(7_sec));
        // return boomerang;
    });

    arc_pose_tracker.setPose({ -63_in, -16.7_in, 90_stDeg });

    mb.arc(90, 2).turn_maxVolt(0.5_volt) |
      // .angular_clampMaxVoltage(0.5_volt) |
      run;

    mb.moveTo(50, 50_in)
        // .withLinearFeedbackController(linear_pid)
        .closeThreshold(10_in) |
      run;

    mb.boomerang(-24, 48, 180)
        .lead(0.4, 0.38)
        .lead2DistThreshold(7_in)
        .closeThreshold(7_in)
        .timeout(7_sec)
        .executeBeforeMotion([] {
            printf("executed before motion!\n");
        })
        .executeAfterMotion([] {
            printf("ended motion!\n");
        })|
        // .drive_vel_kp(linear_vel_pid.get_kd() * 0.7) |
      run;

    // pull matchloader down
    mb.moveTo(-63, 18) | chain;

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
void autonomous() {

    // std::cout << "auto starts, starting motion!" << std::endl;

    // mb.moveTo(50, 50_in)
    //     .timeout(4_sec)
    //     .closeThreshold(10_in)
    //     .only_x(true)
    //     .only_y(true)
    //     .k_lat(0) |
    //   chain;

    // std::cout << "ended motion" << std::endl;
}
