#pragma once

#include "blazing/drivetrain.hpp"
#include "blazing/motion.hpp"
#include "blazing/tracker.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <optional>

namespace blazing {

struct TurnToState {
    Length initial_distance_traveled;

    Time start_time;
    std::optional<Time> last_time;

    bool linear_settled;
    bool angular_settled;
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires poseTracker<TrackerType> && velocityTracker<TrackerType> &&
             distanceTraveledTracker<TrackerType> &&
             ArcadeDrivetrain<DrivetrainType>
class turnTo : public Motion<ControllersType,
                             DrivetrainType,
                             TrackerType,
                             TolerancesType> {
  private:
    units::V2Position point;

    // moveTo-specific properties
    std::optional<Time> timeout = std::nullopt;
    bool reversed = false;

    std::optional<TurnToState> m_state;

    int getLoopDelayTime() override {
        return 10;
    }

    motionExecutionResult execute() override {
        if (!m_state.has_value()) {
            m_state = {
                .initial_distance_traveled =
                  this->tracker.getDistanceTraveled(),
                .start_time = from_msec(pros::millis()),
                .last_time = from_msec(pros::millis()),
                .linear_settled = false,
                .angular_settled = false,
            };
        }

        TurnToState& state = m_state.value();
        motionExecutionResult result;

        Time current_time = from_msec(pros::millis());

        Time delta_time = state.last_time
                            .transform([current_time](Time last_time) -> Time {
                                return current_time - last_time;
                            })
                            .value_or(0.0_sec);

        state.last_time = current_time;

        units::V2Position position = this->tracker.getPosition();
        Angle heading = this->tracker.getAngle();
        Length distance_traveled = this->tracker.getDistanceTraveled();

        Angle target_heading = position.angleTo(point);

        Length linear_error =
          state.initial_distance_traveled - distance_traveled;
        Angle angular_error =
          units::constrainAngle180(heading - target_heading);

        // TODO: make it able to specify left / right direction of rotation

        if (reversed) {
            angular_error = units::constrainAngle180(rot / 2 - angular_error);
        }

        // update tolerances if they are included
        if constexpr (hasLinearErrorTolerance<TolerancesType>) {
            this->tolerances.linear.errorToleranceUpdate(linear_error);
        }
        if constexpr (hasLinearVelocityTolerance<TolerancesType>) {
            this->tolerances.linear.velocityToleranceUpdate(
              this->tracker.getVelocity());
        }

        if constexpr (hasLargeLinearErrorTolerance<TolerancesType>) {
            this->tolerances.large_linear.errorToleranceUpdate(linear_error);
        }
        if constexpr (hasLargeLinearVelocityTolerance<TolerancesType>) {
            this->tolerances.large_linear.velocityToleranceUpdate(
              this->tracker.getVelocity());
        }

        // angular tolerances
        if constexpr (hasAngularErrorTolerance<TolerancesType>) {
            this->tolerances.angular.errorToleranceUpdate(angular_error);
        }
        if constexpr (hasAngularVelocityTolerance<TolerancesType>) {
            this->tolerances.angular.velocityToleranceUpdate(
              this->tracker.getAngularVelocity());
        }

        if constexpr (hasLargeAngularErrorTolerance<TolerancesType>) {
            this->tolerances.large_angular.errorToleranceUpdate(linear_error);
        }
        if constexpr (hasLargeAngularVelocityTolerance<TolerancesType>) {
            this->tolerances.large_angular.velocityToleranceUpdate(
              this->tracker.getAngularVelocity());
        }

        result.finished = false;
        result.inLargeTolerance = true;
        result.inSmallTolerance = true;

        // check tolerances
        if constexpr (hasLinearTolerance<TolerancesType>) {
            result.inSmallTolerance &=
              this->tolerances.linear.withinTolerance();
            state.linear_settled |= this->tolerances.linear.finished();
            this->tolerances.linear.reset();
        }
        if constexpr (hasLargeLinearTolerance<TolerancesType>) {
            result.inLargeTolerance &=
              this->tolerances.large_linear.withinTolerance();
            state.linear_settled |= this->tolerances.large_linear.finished();
            this->tolerances.large_linear.reset();
        }
        if constexpr (hasAngularTolerance<TolerancesType>) {
            result.inSmallTolerance &=
              this->tolerances.angular.withinTolerance();
            state.angular_settled |= this->tolerances.angular.finished();
            this->tolerances.angular.reset();
        }
        if constexpr (hasLargeAngularTolerance<TolerancesType>) {
            result.inLargeTolerance &=
              this->tolerances.large_angular.withinTolerance();
            state.angular_settled |= this->tolerances.large_angular.finished();
            this->tolerances.large_angular.reset();
        }

        // the previous results in:
        // inSmallTolerance = linear.inTolerance() && angular.inTolerance();
        // inLargeTolerance = large_linear.inTolerance() &&
        // large_angular.inTolerance();
        // state.linear_settled = linear.settled() | large_linear.settled();
        // state.angular_settled = angular.settled() | large_angular.settled();

        result.finished |= state.linear_settled && state.angular_settled;

        // check timeout
        result.finished |=
          timeout
            .transform([state](Time timeout) -> bool {
                std::cout << "dt "
                          << from_msec(pros::millis()) - state.start_time
                          << " timeout:  " << timeout << std::endl;
                return from_msec(pros::millis()) - state.start_time > timeout;
            })
            .value_or(false);

        // finished if any of the available tolerances or timeout are triggered
        if (result.finished) {
            this->drivetrain.moveArcade(0_volt, 0_volt);
            // returns immediately to avoid more movement
            return result;
        }

        Voltage angular_output =
          this->controllers.angular_feedback_controller.update(-angular_error,
                                                               0_stRad,
                                                               delta_time);

        Voltage linear_output =
          this->controllers.linear_feedback_controller.update(
            distance_traveled,
            state.initial_distance_traveled,
            delta_time);

        this->drivetrain.moveArcade(linear_output, angular_output);

        return result;
    }

  public:
    turnTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           Length x,
           Length y)
        : Motion<ControllersType, DrivetrainType, TrackerType, TolerancesType>(
            controllers,
            chassis),
          point(x, y) {}

    turnTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           double x,
           double y)
        : turnTo(controllers, chassis, from_in(x), from_in(y)) {}

    turnTo& getReference() {
        return *this;
    }

    // changer methods

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    auto reverse() {
        this->reversed = true;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    auto withTimeout(Time timeout) {
        this->timeout = timeout;

        return this->getReference();
    }
};
} // namespace blazing
