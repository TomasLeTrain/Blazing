#pragma once

#include "blazing/motions/moveTo.hpp"

namespace blazing {

template<typename Chassis,
typename Controllers>
class MotionBuilder {
  private:
    Chassis chassis;
	Controllers controllers;

  public:
    MotionBuilder(Chassis chassis,
				  Controllers controllers)
        : chassis(chassis),
		controllers(controllers) {}

    auto moveTo(Length x, Length y) {
        return blazing::moveTo(controllers, chassis, x, y);
    }

    auto moveTo(float x, float y) {
        return blazing::moveTo(controllers, chassis, x, y);
    }
};
} // namespace blazing
