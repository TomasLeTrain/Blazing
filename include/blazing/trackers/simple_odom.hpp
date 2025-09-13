#pragma once

#include "pros/device.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cmath>
#include <iterator>
#include <optional>

class SimpleOdomTracker {
  private:
    pros::MotorGroup* left_motors;
    pros::MotorGroup* right_motors;
    pros::Imu* imu;

    Length track_width;
    Length wheel_diameter;
    AngularVelocity final_rpm;

    units::Pose pose {};
    Length forward_travel;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;

    std::optional<Length> last_left_dist = std::nullopt;
    std::optional<Length> last_right_dist = std::nullopt;
    std::optional<Angle> last_heading = std::nullopt;
    std::optional<Time> last_time = std::nullopt;

  public:
    SimpleOdomTracker(pros::MotorGroup* left_motors,
                      pros::MotorGroup* right_motors,
                      pros::Imu* imu,
                      Length track_width,
                      Length wheel_diameter,
                      AngularVelocity final_rpm)
        : left_motors(left_motors),
          right_motors(right_motors),
          track_width(track_width),
          wheel_diameter(wheel_diameter),
          final_rpm(final_rpm),
          imu(imu) {}

    Angle getAngle() {
        // std::cout << "returned angle: " << pose.orientation << std::endl;
        return pose.orientation;
    }

    units::V2Position getPosition() {
        return pose;
    }

    LinearVelocity getLinearVelocity() {
        return linear_velocity;
    }

    AngularVelocity getAngularVelocity() {
        return angular_velocity;
    }

    Length getForwardTravel() {
        return forward_travel;
    }

    void setPose(units::Pose new_pose) {
        pose = new_pose;
    }

    void update() {
        Time current_time = from_msec(pros::millis());

        const Time delta_time =
          last_time
            .transform([current_time](Time last_time) -> Time {
                return current_time - last_time;
            })
            .value_or(0.0_sec);
        last_time = current_time;

        auto get_dist = [this](pros::MotorGroup* motors) -> Length {
            Length res = 0_in;
            double count = 0;
            for (auto position : motors->get_raw_position_all(NULL)) {
                // number of rotations
                Number rotations =
                  (final_rpm * static_cast<double>(position)) / (3600_rpm * 50);
                res += rotations * wheel_diameter;
                count++;
            }
            return res / count;
        };

        const Length left_dist = get_dist(left_motors);
        const Length right_dist = get_dist(right_motors);

        if (!last_left_dist) last_left_dist = left_dist;
        if (!last_right_dist) last_right_dist = right_dist;

        const Length left_delta = left_dist - *last_left_dist;
        const Length right_delta = right_dist - *last_right_dist;

        last_left_dist = left_dist;
        last_right_dist = right_dist;

        const Length average_distance = (left_delta + right_delta) / 2;

        // NOTE: this is not super accurate, might return 0 due to the polling
        // rate
        linear_velocity = average_distance / delta_time;

        forward_travel += average_distance;

        const Angle heading = from_cDeg(imu->get_rotation());
        if (!last_heading) last_heading = heading;

        // std::cout << "[odom] heading " << heading << std::endl;

        Angle heading_theta = heading - *last_heading;
        angular_velocity = heading_theta / delta_time;
        last_heading = heading;

        // update pose
        auto change_vector =
          units::V2Position::fromPolar(heading, average_distance);

        units::Pose new_pose = { change_vector.x, change_vector.y, heading };

        // std::cout << "set? " << new_pose.orientation << std::endl;

        pose = new_pose;
        // std::cout << "set2? " << pose.orientation << std::endl;
    }
};
