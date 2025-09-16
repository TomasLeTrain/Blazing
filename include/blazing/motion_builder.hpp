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

    using moveToType = blazing::moveTo<Controllers,
                                       typename Chassis::drivetrainType,
                                       typename Chassis::trackerType,
                                       typename Chassis::tolerancesType>;
    using turnToType = blazing::turnTo<Controllers,
                                       typename Chassis::drivetrainType,
                                       typename Chassis::trackerType,
                                       typename Chassis::tolerancesType>;

    using MoveToModifier = std::function<moveToType(moveToType)>;
    using TurnToModifier = std::function<turnToType(turnToType)>;

    MoveToModifier moveToModifier = [](moveToType moveTo) {
        return moveTo;
    };
    TurnToModifier turnToModifier = [](turnToType turnTo) {
        return turnTo;
    };

  public:
    MotionBuilder(Chassis chassis, Controllers controllers)
        : chassis(chassis),
          controllers(controllers) {}

	void setMoveToModifier(MoveToModifier customModifier){
		moveToModifier = customModifier;
	}

	void setTurnToModifier(TurnToModifier customModifier){
		turnToModifier = customModifier;
	}

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    moveToType moveTo(Length x, Length y) {
        return moveToModifier(blazing::moveTo(controllers, chassis, x, y));
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    moveToType moveTo(double x, double y) {
        return moveToModifier(blazing::moveTo(controllers, chassis, x, y));
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    turnToType turnTo(Length x, Length y) {
        return turnToModifier(blazing::turnTo(controllers, chassis, x, y));
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    turnToType turnTo(double x, double y) {
        return turnToModifier(blazing::turnTo(controllers, chassis, x, y));
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    turnToType turnTo(Angle heading) {
        return turnToModifier(blazing::turnTo(controllers, chassis, heading));
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    turnToType turnTo(double heading) {
        return turnToModifier(blazing::turnTo(controllers, chassis, heading));
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    auto distanceAtHeading(Length target_distance) {
        return blazing::distanceAtHeading(controllers,
                                          chassis,
                                          target_distance);
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    auto distanceAtHeading(double target_distance) {
        return blazing::turnTo(controllers, chassis, target_distance);
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    auto distanceAtHeading(Length target_distance, Angle target_heading) {
        return blazing::turnTo(controllers,
                               chassis,
                               target_distance,
                               target_heading);
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    auto distanceAtHeading(double target_distance, double target_heading) {
        return blazing::turnTo(controllers,
                               chassis,
                               target_distance,
                               target_heading);
    }
};
} // namespace blazing
