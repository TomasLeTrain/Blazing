#pragma once

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

    auto moveTo(float x, float y) {
        return blazing::moveTo(controllers, chassis, x, y);
    }

    auto turnTo(Length x, Length y) {
        return blazing::turnTo(controllers, chassis, x, y);
    }

    auto turnTo(float x, float y) {
        return blazing::turnTo(controllers, chassis, x, y);
    }

    auto turnTo(Angle heading) {
        return blazing::turnTo(controllers, chassis, heading);
    }

    auto turnTo(double heading) {
        return blazing::turnTo(controllers, chassis, heading);
    }
};
} // namespace blazing
