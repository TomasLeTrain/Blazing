#pragma once

#include "pros/motor_group.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <concepts>
#include <cstddef>

template<typename Q>
concept angleTracker = requires(Q q) {
    { q.getAngle() } -> std::same_as<Angle>;
};

template<typename Q>
concept positionTracker = requires(Q q) {
    { q.getPosition() } -> std::same_as<units::V2Position>;
};

template<typename Q>
concept poseTracker = positionTracker<Q> && angleTracker<Q>;

template<typename Q>
concept ArcadeDrivetrain = requires(Q q) { q.moveArcade(); };
template<typename Q>
concept TankDrivetrain = requires(Q q) { q.moveTank(); };

namespace blazing {
class PoseTracker {
  public:
    PoseTracker() {}

    Angle getAngle() {}

    units::V2Position getPosition() {}
};

class PositionOnlyTracker {
  public:
    units::V2Position getPosition() {}
};

template<typename TrackerType>
class DifferentialDrivetrain {
  private:
    pros::MotorGroup* left_motors;
    pros::MotorGroup* right_motors;

  public:
    DifferentialDrivetrain(pros::MotorGroup* left_motors,
                           pros::MotorGroup* right_motors,
                           TrackerType tracker)
        : left_motors(left_motors),
          right_motors(right_motors) {}

    // move robot based on left and right velocities
    void moveTank(Voltage left_voltage, Voltage right_voltage) {
        if (left_motors != nullptr && right_motors != nullptr) {
            left_motors->move_voltage(to_Mvolt(left_voltage));
            right_motors->move_voltage(to_Mvolt(right_voltage));
        }
    }

    // move robot based on left and right velocities
    void moveArcade(Voltage linear_output, Voltage angular_output) {
        // TODO: implement
    }
};

// holds both a drivetrain and a drivetrain
// main hardware abstraction for motions to use
template<typename DrivetrainType, typename TrackerType>
class Chassis {
    DrivetrainType drivetrain;
    TrackerType tracker;

    Chassis(DrivetrainType drivetrain, TrackerType tracker)
        : drivetrain(drivetrain),
          tracker(tracker) {}
};

} // namespace blazing
