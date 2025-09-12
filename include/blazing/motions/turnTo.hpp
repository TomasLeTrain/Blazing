#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/motions/motion.hpp"
#include "blazing/tolerances.hpp"
#include "blazing/trackers/tracker.hpp"
#include "blazing/util.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <optional>
#include <variant>

namespace blazing {

struct TurnToState {
    Length initial_distance_traveled;

    Time start_time;
    std::optional<Time> last_time;

    bool settled;
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires angleTracker<TrackerType> && angularVelocityTracker<TrackerType> &&
             ArcadeDrivetrain<DrivetrainType> &&
             hasAngularFeedbackController<ControllersType>
class turnTo : public Motion<ControllersType,
                             DrivetrainType,
                             TrackerType,
                             TolerancesType> {
  private:
    // std::optional<units::V2Position> target_point = std::nullopt;
    // std::optional<Angle> given_target_heading = std::nullopt;
    std::variant<Angle, units::V2Position> target;

    // turnTo-specific properties
    std::optional<Time> timeout = std::nullopt;
    bool reversed = false;
    std::optional<AngularDirection> direction = std::nullopt;

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
                .settled = false,
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

        const Angle heading = [this] {
            const Angle heading = this->tracker.getAngle();
            return reversed ? reverseAngle(heading) : heading;
        }();

        // defalts to std::nullopt if tracker does not implements getPosition
        const std::optional<units::V2Position> position = [this] {
            if constexpr (positionTracker<TrackerType>)
                return this->tracker.getPosition();
            else
                return std::nullopt;
        }();

        if (std::holds_alternative<units::V2Position>(target) &&
            !position.has_value()) {
            // should not be possible
            printf("invalid configuration! target is point but tracker doesn't "
                   "track position!\n");
        }

        const Angle target_heading =
          std::holds_alternative<Angle>(target) ?
            std::get<Angle>(target) :
            position.value().angleTo(std::get<units::V2Position>(target));

        const Angle angular_error =
          angleError(target_heading, heading, direction);

        // update tolerances if they are included
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

        state.settled = false;

        // check tolerances
        if constexpr (hasAngularTolerance<TolerancesType>) {
            result.inSmallTolerance =
              this->tolerances.angular.withinTolerance();
            state.settled |= this->tolerances.angular.finished();
            this->tolerances.angular.reset();
        }
        if constexpr (hasLargeAngularTolerance<TolerancesType>) {
            result.inLargeTolerance =
              this->tolerances.large_angular.withinTolerance();
            state.settled |= this->tolerances.large_angular.finished();
            this->tolerances.large_angular.reset();
        }

        result.finished = state.settled;

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

        Voltage linear_output = 0_volt;

        this->drivetrain.moveArcade(linear_output, angular_output);

        return result;
    }

  public:
    turnTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           Length x,
           Length y)
      // requires tracker to be able to track position without making it a
      // requirement for target heading
        requires positionTracker<TrackerType>
        : Motion<ControllersType, DrivetrainType, TrackerType, TolerancesType>(
            controllers,
            chassis),
          target(units::V2Position(x, y)) {}

    turnTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           double x,
           double y)
        : turnTo(controllers, chassis, from_in(x), from_in(y)) {}

    turnTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           Angle target_heading)
        : Motion<ControllersType, DrivetrainType, TrackerType, TolerancesType>(
            controllers,
            chassis),
          target(target_heading) {}

    turnTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           double target_heading)
        : turnTo(controllers, chassis, from_stDeg(target_heading)) {}

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

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    auto withDirection(std::optional<AngularDirection> direction) {
        this->direction = direction;

        return this->getReference();
    }
};
} // namespace blazing
