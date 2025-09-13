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
concept forwardTravelTracker = requires(Q q) {
    { q.getForwardTravel() } -> std::same_as<Length>;
};

template<typename Q>
concept poseTracker = positionTracker<Q> && angleTracker<Q>;

template<typename Q>
concept linearVelocityTracker = requires(Q q) {
    { q.getLinearVelocity() } -> std::same_as<LinearVelocity>;
};

template<typename Q>
concept angularVelocityTracker = requires(Q q) {
    { q.getAngularVelocity() } -> std::same_as<AngularVelocity>;
};
template<typename Q>
concept velocityTracker = linearVelocityTracker<Q> && angularVelocityTracker<Q>;

class PoseTracker {
  public:
    PoseTracker() {}

    Angle getAngle() {
        return 67_stDeg;
    }

    units::V2Position getPosition() {
        return { 2_in, 3_in };
    }

    LinearVelocity getLinearVelocity() {
        return 1_inps;
    }

    AngularVelocity getAngularVelocity() {
        return 1_degps;
    }
	Length getForwardTravel() {
		return 1_in;
	}
};

// class PositionOnlyTracker {
//   public:
//     units::V2Position getPosition() {}
// };

} // namespace blazing
