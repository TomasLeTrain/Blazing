#pragma once

#include "blazing/motions/distance_at_heading.hpp"
#include "blazing/motions/moveTo.hpp"
#include "blazing/motions/turnTo.hpp"

namespace blazing {

template<typename Chassis, typename Controllers>
class MotionBuilder {
  private:
    Chassis chassis;
    Controllers controllers;

  public:
    MotionBuilder(Chassis chassis, Controllers controllers)
        : chassis(chassis),
          controllers(controllers) {}

    auto moveTo(Length x, Length y) {
        return blazing::moveTo(controllers, chassis, x, y);
    }

    auto moveTo(double x, double y) {
        return blazing::moveTo(controllers, chassis, x, y);
    }

    auto turnTo(Length x, Length y) {
        return blazing::turnTo(controllers, chassis, x, y);
    }

    auto turnTo(double x, double y) {
        return blazing::turnTo(controllers, chassis, x, y);
    }

    auto turnTo(Angle heading) {
        return blazing::turnTo(controllers, chassis, heading);
    }

    auto turnTo(double heading) {
        return blazing::turnTo(controllers, chassis, heading);
    }

    auto distanceAtHeading(Length target_distance) {
        return blazing::distanceAtHeading(controllers,
                                          chassis,
                                          target_distance);
    }

    auto distanceAtHeading(double target_distance) {
        return blazing::turnTo(controllers, chassis, target_distance);
    }

    auto distanceAtHeading(Length target_distance, Angle target_heading) {
        return blazing::turnTo(controllers,
                               chassis,
                               target_distance,
                               target_heading);
    }

    auto distanceAtHeading(double target_distance, double target_heading) {
        return blazing::turnTo(controllers,
                               chassis,
                               target_distance,
                               target_heading);
    }
};
} // namespace blazing
