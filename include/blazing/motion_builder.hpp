#pragma once

#include "blazing/motions/distance_at_heading.hpp"
#include "blazing/motions/moveTo.hpp"
#include "blazing/motions/turnTo.hpp"
#include "blazing/utils.hpp"
#include "motions/boomerang.hpp"
#include <iterator>

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
    using distanceAtHeadingType =
      blazing::distanceAtHeading<Controllers,
                                 typename Chassis::drivetrainType,
                                 typename Chassis::trackerType,
                                 typename Chassis::tolerancesType>;
    using boomerangType = blazing::boomerang<Controllers,
                                             typename Chassis::drivetrainType,
                                             typename Chassis::trackerType,
                                             typename Chassis::tolerancesType>;

    using MoveToModifier = std::function<moveToType(moveToType)>;
    using TurnToModifier = std::function<turnToType(turnToType)>;
    using DistanceAtHeadingModifier =
      std::function<distanceAtHeadingType(distanceAtHeadingType)>;
    using BoomerangModifier = std::function<boomerangType(boomerangType)>;

    MoveToModifier moveToModifier = [](moveToType moveTo) {
        return moveTo;
    };
    TurnToModifier turnToModifier = [](turnToType turnTo) {
        return turnTo;
    };
    DistanceAtHeadingModifier distanceAtHeadingModifier =
      [](distanceAtHeadingType distanceAtHeading) {
          return distanceAtHeading;
      };
    BoomerangModifier boomerangModifier = [](boomerangType boomerang) {
        return boomerang;
    };

  public:
    MotionBuilder(Chassis chassis, Controllers controllers)
        : chassis(chassis),
          controllers(controllers) {}

    void setMoveToModifier(MoveToModifier customModifier) {
        moveToModifier = customModifier;
    }

    void setTurnToModifier(TurnToModifier customModifier) {
        turnToModifier = customModifier;
    }

    void
    setDistanceAtHeadingModifier(DistanceAtHeadingModifier customModifier) {
        distanceAtHeadingModifier = customModifier;
    }

    void setBoomerangModifier(BoomerangModifier customModifier) {
        boomerangModifier = customModifier;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    moveToType moveTo(Length x, Length y) {
        return moveToModifier(blazing::moveTo(controllers, chassis, x, y));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    moveToType moveTo(double x, double y) {
        return moveToModifier(blazing::moveTo(controllers, chassis, x, y));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnToType turnTo(Length x, Length y) {
        return turnToModifier(blazing::turnTo(controllers, chassis, x, y));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnToType turnTo(double x, double y) {
        return turnToModifier(blazing::turnTo(controllers, chassis, x, y));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnToType turnTo(Angle heading) {
        return turnToModifier(blazing::turnTo(controllers, chassis, heading));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnToType turnTo(double heading) {
        return turnToModifier(blazing::turnTo(controllers, chassis, heading));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    distanceAtHeadingType distanceAtHeading(Length target_distance) {
        return distanceAtHeadingModifier(
          blazing::distanceAtHeading(controllers, chassis, target_distance));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    distanceAtHeadingType distanceAtHeading(double target_distance) {
        return distanceAtHeadingModifier(
          blazing::distanceAtHeading(controllers, chassis, target_distance));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    distanceAtHeadingType distanceAtHeading(Length target_distance,
                                            Angle target_heading) {
        return distanceAtHeadingModifier(
          blazing::distanceAtHeading(controllers,
                                     chassis,
                                     target_distance,
                                     target_heading));
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    distanceAtHeadingType distanceAtHeading(double target_distance,
                                            double target_heading) {
        return distanceAtHeadingModifier(
          blazing::distanceAtHeading(controllers,
                                     chassis,
                                     target_distance,
                                     target_heading));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    distanceAtHeadingType
    arc(Length radius, Angle current_heading, Angle target_heading) {
        // calculate target distance from radius
        Length distance =
          units::abs(radius * angleError(target_heading, current_heading)) /
          rad;
        return distanceAtHeadingModifier(
          blazing::distanceAtHeading(controllers,
                                     chassis,
                                     distance,
                                     target_heading));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    boomerangType boomerang(units::Pose pose) {
        return boomerangModifier(
          blazing::boomerang(controllers, chassis, pose));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    boomerangType boomerang(Length x, Length y, Angle heading) {
        return boomerangModifier(
          blazing::boomerang(controllers, chassis, x, y, heading));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    boomerangType boomerang(double x, double y, double heading) {
        return boomerangModifier(
          blazing::boomerang(controllers, chassis, x, y, heading));
    }
};
} // namespace blazing
