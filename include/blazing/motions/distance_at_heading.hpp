#pragma once

#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/motions/motion.hpp"
#include "blazing/trackers/tracker.hpp"
#include "blazing/tolerances.hpp"
#include "blazing/util.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <optional>

namespace blazing {

struct DistanceAtHeadingState {
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
    requires velocityTracker<TrackerType> &&
             distanceTraveledTracker<TrackerType> &&
             ArcadeDrivetrain<DrivetrainType> &&
             hasAngularFeedbackController<ControllersType> &&
             hasLinearFeedbackController<ControllersType>
class distanceAtHeading : public Motion<ControllersType,
                                        DrivetrainType,
                                        TrackerType,
                                        TolerancesType> {
  private:
    Length target_distance;
    std::optional<Angle> given_target_heading = std::nullopt;

    // moveTo-specific properties
    bool reversed = false;
    std::optional<Time> timeout = std::nullopt;
    std::optional<AngularDirection> direction = std::nullopt;

    std::optional<DistanceAtHeadingState> m_state;

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

        DistanceAtHeadingState& state = m_state.value();
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

        Angle target_heading =
          // use given target heading
          given_target_heading
            // or just target current angle so that the angle doesn't move at
            // all
            .value_or(heading);

        if (reversed) {
            target_heading = 180_stDeg - target_heading;
            target_distance *= -1.0;
        }

        Length linear_error =
          (target_distance + state.initial_distance_traveled) -
          distance_traveled;

        Angle angular_error = angleError(target_heading, heading, direction);

        // update tolerances if they are included
        if constexpr (hasLinearErrorTolerance<TolerancesType>) {
            this->tolerances.linear.errorToleranceUpdate(linear_error);
        }
        if constexpr (hasLinearVelocityTolerance<TolerancesType>) {
            this->tolerances.linear.velocityToleranceUpdate(
              this->tracker.getLinearVelocity());
        }

        if constexpr (hasLargeLinearErrorTolerance<TolerancesType>) {
            this->tolerances.large_linear.errorToleranceUpdate(linear_error);
        }
        if constexpr (hasLargeLinearVelocityTolerance<TolerancesType>) {
            this->tolerances.large_linear.velocityToleranceUpdate(
              this->tracker.getLinearVelocity());
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
            this->tolerances.large_angular.errorToleranceUpdate(angular_error);
        }
        if constexpr (hasLargeAngularVelocityTolerance<TolerancesType>) {
            this->tolerances.large_angular.velocityToleranceUpdate(
              this->tracker.getAngularVelocity());
        }

        state.angular_settled = false;
        state.linear_settled = false;

        // check tolerances
        if constexpr (hasLinearTolerance<TolerancesType>) {
            bool curr_in_tolerance = this->tolerances.linear.withinTolerance();

            result.inSmallTolerance =
              result.inSmallTolerance
                .transform([curr_in_tolerance](auto inSmallTolerance) {
                    return inSmallTolerance & curr_in_tolerance;
                })
                .value_or(curr_in_tolerance);

            state.linear_settled |= this->tolerances.linear.finished();
            this->tolerances.linear.reset();
        }
        if constexpr (hasLargeLinearTolerance<TolerancesType>) {
            bool curr_in_tolerance =
              this->tolerances.large_linear.withinTolerance();

            result.inLargeTolerance =
              result.inLargeTolerance
                .transform([curr_in_tolerance](auto inLargeTolerance) {
                    return inLargeTolerance & curr_in_tolerance;
                })
                .value_or(curr_in_tolerance);

            state.linear_settled |= this->tolerances.large_linear.finished();
            this->tolerances.large_linear.reset();
        }
        if constexpr (hasAngularTolerance<TolerancesType>) {
            bool curr_in_tolerance = this->tolerances.angular.withinTolerance();

            result.inSmallTolerance =
              result.inSmallTolerance
                .transform([curr_in_tolerance](auto inSmallTolerance) {
                    return inSmallTolerance & curr_in_tolerance;
                })
                .value_or(curr_in_tolerance);

            state.angular_settled |= this->tolerances.angular.finished();
            this->tolerances.angular.reset();
        }
        if constexpr (hasLargeAngularTolerance<TolerancesType>) {
            bool curr_in_tolerance =
              this->tolerances.large_angular.withinTolerance();

            result.inLargeTolerance =
              result.inLargeTolerance
                .transform([curr_in_tolerance](auto inLargeTolerance) {
                    return inLargeTolerance & curr_in_tolerance;
                })
                .value_or(curr_in_tolerance);

            state.angular_settled |= this->tolerances.large_angular.finished();
            this->tolerances.large_angular.reset();
        }

        result.finished = state.linear_settled && state.angular_settled;

        // check timeout
        result.finished |=
          timeout
            .transform([state](Time timeout) -> bool {
                return from_msec(pros::millis()) - state.start_time > timeout;
            })
            .value_or(false);

        // finished if any of the available tolerances or timeout are
        // triggered
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
          this->controllers.linear_feedback_controller.update(-linear_error,
                                                              0_in,
                                                              delta_time);

        this->drivetrain.moveArcade(linear_output, angular_output);

        return result;
    }

  public:
    distanceAtHeading(
      ControllersType controllers,
      Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
      Length target_distance)
        : Motion<ControllersType, DrivetrainType, TrackerType, TolerancesType>(
            controllers,
            chassis),
          target_distance(target_distance) {}

    distanceAtHeading(
      ControllersType controllers,
      Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
      double target_distance)
        : distanceAtHeading(controllers, chassis, from_in(target_distance)) {}

    distanceAtHeading(
      ControllersType controllers,
      Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
      Length target_distance,
      Angle target_heading)
        : Motion<ControllersType, DrivetrainType, TrackerType, TolerancesType>(
            controllers,
            chassis),
          target_distance(target_distance),
          given_target_heading(target_heading) {}

    distanceAtHeading(
      ControllersType controllers,
      Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
      double target_distance,
      double target_heading)
        : distanceAtHeading(controllers,
                            chassis,
                            from_in(target_distance),
                            from_stDeg(target_heading)) {}

    distanceAtHeading& getReference() {
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

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    auto withDirection(std::optional<AngularDirection> direction) {
        this->direction = direction;

        return this->getReference();
    }
};
} // namespace blazing
