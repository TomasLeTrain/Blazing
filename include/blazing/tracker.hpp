#pragma once

#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include <concepts>

namespace blazing {
template<typename Q>
concept angleTracker = requires(Q q) {
    { q.getAngle() } -> std::same_as<Angle>;
};

template<typename Q>
concept positionTracker = requires(Q q) {
    { q.getPosition() } -> std::same_as<units::V2Position>;
};

template<typename Q>
concept distanceTraveledTracker = requires(Q q) {
    { q.getDistanceTraveled() } -> std::same_as<Length>;
};

template<typename Q>
concept poseTracker = positionTracker<Q> && angleTracker<Q>;

template<typename Q>
concept velocityTracker = requires(Q q) {
    { q.getVelocity() } -> std::same_as<LinearVelocity>;
    { q.getAngularVelocity() } -> std::same_as<AngularVelocity>;
};

class PoseTracker {
  public:
    PoseTracker() {}

    Angle getAngle() {
        return 0_stDeg;
    }

    units::V2Position getPosition() {
        return { 2_in, 3_in };
    }

    LinearVelocity getVelocity() {
        return 1_inps;
    }
};

// class PositionOnlyTracker {
//   public:
//     units::V2Position getPosition() {}
// };

} // namespace blazing
