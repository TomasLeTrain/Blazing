#pragma once

#include "pros/device.hpp"
#include "pros/error.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cmath>
#include <optional>
#include <variant>

namespace blazing {
enum TrackerOrientation {
    Sideways,
    Forwards
};

template<TrackerOrientation orientation>
class TrackingWheel {
  private:
    std::variant<pros::Rotation*, pros::MotorGroup*> sensor;
    Length offset;
    Length wheel_diameter;
    std::optional<AngularVelocity> final_rpm = std::nullopt;

    Length last_distance = INFINITY * m;
    Length m_delta = INFINITY * m;

  public:
    TrackingWheel(std::variant<pros::Rotation*, pros::MotorGroup*> sensor,
                  Length offset,
                  Length wheel_diameter,
                  std::optional<AngularVelocity> final_rpm = std::nullopt)
        requires
      // either holds a rotation sensor
      (std::holds_alternative<pros::Rotation*>(sensor) ||
       // or holds a motor group and specifis a final rpm
       (std::holds_alternative<pros::MotorGroup*>(sensor) &&
        final_rpm.has_value()))
        : sensor(sensor),
          offset(offset),
          wheel_diameter(wheel_diameter),
          final_rpm(final_rpm) {}

    Length getOffset() {
        return offset;
    }

    Length getDelta() {
        return m_delta;
    }

    Length update() {
        auto motor_get_dist = [this](pros::MotorGroup* motors) -> Length {
            Length res = 0_in;
            double count = 0;
            for (auto position : motors->get_raw_position_all(NULL)) {
                if (position == PROS_ERR) continue;

                Number rotations =
                  (*final_rpm * static_cast<double>(position)) /
                  (3600_rpm * 50.0);

                res += rotations * (wheel_diameter * M_PI);
                count += 1.0;
            }

            Length current = INFINITY * m, delta = INFINITY * m;

            if (count != 0) {
                current = res / count;
            }

            if (std::isfinite(current.internal()) &&
                std::isfinite(last_distance.internal())) {
                delta = current - last_distance;
            }

            last_distance = current;
            return delta;
        };

        auto rotation_get_dist = [this](pros::Rotation* sensor) -> Length {
            if (sensor == nullptr || !sensor->is_installed()) {
                last_distance = INFINITY * m;
                return INFINITY * m;
            }

            Length current = INFINITY * m, delta = INFINITY * m;

            int32_t current_deg = sensor->get_position();

            if (current_deg != PROS_ERR) {
                current = static_cast<double>(current_deg) * wheel_diameter *
                          M_PI / 36000.0;
            }

            if (current_deg != PROS_ERR &&
                std::isfinite(last_distance.internal())) {
                delta = current - last_distance;
            }

            last_distance = current;

            return delta;
        };

        if (std::holds_alternative<pros::MotorGroup*>(sensor)) {
            m_delta = motor_get_dist(std::get<pros::MotorGroup*>(sensor));
        } else {
            m_delta = rotation_get_dist(std::get<pros::Rotation*>(sensor));
        }
    }
};

using SidewaysTracker = TrackingWheel<TrackerOrientation::Sideways>;
using ForwardsTracker = TrackingWheel<TrackerOrientation::Forwards>;

class TrackingImu {
  private:
    double last_heading;
    Angle m_delta = INFINITY * rad;

  public:
    pros::Imu* sensor;

    Angle getDelta() {
        return m_delta;
    }

    void update() {
        if (sensor == nullptr || !sensor->is_installed()) {
            last_heading = INFINITY;
            m_delta = INFINITY * rad;
            return;
        }

        double current = sensor->get_rotation();
        double result = INFINITY;

        if (std::isfinite(current) && std::isfinite(last_heading)) {
            result = -1.0 * (current - last_heading);
        }

        last_heading = current;

        m_delta = from_stDeg(result);
    }

    TrackingImu(pros::Imu* sensor)
        : sensor(sensor) {}
};

class SimpleOdomTracker {
  private:
    std::vector<ForwardsTracker> forwards_trackers;
    std::vector<SidewaysTracker> sideways_trackers;

    std::vector<TrackingImu> imus;

    Length track_width;
    Length wheel_diameter;
    AngularVelocity final_rpm;

    units::Pose pose {};
    Length forward_travel = 0_in;
    Length distance_traveled = 0_in;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;

    std::optional<Length> last_left_dist = std::nullopt;
    std::optional<Length> last_right_dist = std::nullopt;
    std::optional<Angle> last_heading = std::nullopt;
    std::optional<Time> last_time = std::nullopt;

  public:
    SimpleOdomTracker(
      std::initializer_list<SidewaysTracker> sideways_trackers,
      std::initializer_list<ForwardsTracker> forwards_trackers,
      std::initializer_list<TrackingImu> imus)
        : sideways_trackers(sideways_trackers),
          forwards_trackers(forwards_trackers),
          imus(imus) {}

    Angle getAngle() {
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

    Length getDistanceTraveled() {
        return distance_traveled;
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

        for (auto& tracker : imus) {
            tracker.update();
        }
        for (auto& tracker : sideways_trackers) {
            tracker.update();
        }
        for (auto& tracker : forwards_trackers) {
            tracker.update();
        }

        Angle heading_delta = 0_stDeg;
        int imu_count = 0;

        for (auto& imu : imus) {
            Angle current = imu.getDelta();
            if (std::isfinite(current.internal())) {
                heading_delta += current;
                imu_count++;
            }
        }

		if(imu_count == 0){
			heading_delta = INFINITY * rad;
		}else {
			heading_delta /= static_cast<double>(imu_count);
		}

		if(!std::isfinite(heading_delta.internal())){
			// try to calculate heading from forward trackers
		}

		// use first tracker that gives good delta
        for (auto& tracker : forwards_trackers) {
			Length current = tracker.getDelta();
			if(!std::isfinite(current.internal())) continue;


			break;
		}


        // NOTE: this is not super accurate, might return 0 due to the
        // polling rate
        linear_velocity = average_distance / delta_time;

        forward_travel += average_distance;
        distance_traveled += units::abs(average_distance);

        pose.orientation += heading_delta;

        if (!last_heading) last_heading = heading;

        // std::cout << "[odom] heading " << heading << std::endl;

        Angle heading_theta = heading - *last_heading;
        angular_velocity = heading_theta / delta_time;
        last_heading = heading;

        // update pose
        units::V2Position change_vector = {
            average_distance * units::cos(heading),
            average_distance * units::sin(heading)
        };

        pose = { pose + change_vector, heading };
    }
};
} // namespace blazing
