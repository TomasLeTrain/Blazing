#pragma once

#include "blazing/chassis.hpp"
#include "blazing/drivetrain.hpp"
#include "blazing/feedback/feedback.hpp"
#include "blazing/motion.hpp"
#include "blazing/tolerances.hpp"
#include "blazing/tracker.hpp"
#include "blazing/util.hpp"
#include "units/Vector2D.hpp"
#include <iostream>

namespace blazing {
struct MoveToState {
    bool close;
    std::optional<Time> last_time;
    Time start_time;
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires poseTracker<TrackerType> && linearVelocityTracker<TrackerType> &&
             ArcadeDrivetrain<DrivetrainType> &&
             hasAngularFeedbackController<ControllersType> &&
             hasLinearFeedbackController<ControllersType>
class moveTo : public Motion<ControllersType,
                             DrivetrainType,
                             TrackerType,
                             TolerancesType> {
  private:
    units::V2Position target;

    // moveTo-specific properties
    std::optional<Time> timeout = std::nullopt;
    bool reversed = false;

    std::optional<MoveToState> m_state;

    int getLoopDelayTime() override {
        return 10;
    }

    motionExecutionResult execute() override {
        if (!m_state.has_value()) {
            m_state = { .close = false,
                        .last_time = from_msec(pros::millis()),
                        .start_time = from_msec(pros::millis()) };
        }

        MoveToState& state = m_state.value();
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

        Length lateral_error = (target - position).magnitude();

        if (units::abs(lateral_error) < 4_in && !state.close) {
            state.close = true;
        }

        Angle target_heading = position.angleTo(target);

        if (reversed) {
            lateral_error *= -1.0;
            target_heading = 180_stDeg - target_heading;
        }

        Angle angular_error = angleError(target_heading, heading);

        // If the turn error exceeds 90 degrees, then the point is behind
        // the robot, so it's more efficient to travel to the point
        // backwards. only applies when the robot is close to the point so
        // that it doesn't accidently go to the point backwards at the
        // beginning
        if (state.close && units::abs(angular_error) >= 90.0_stDeg) {
            lateral_error *= -1.0;
            angular_error = units::constrainAngle180(rot / 2 - angular_error);
        }

        // update tolerances if they are included
        if constexpr (hasLinearErrorTolerance<TolerancesType>) {
            this->tolerances.linear.errorToleranceUpdate(lateral_error);
        }
        if constexpr (hasLinearVelocityTolerance<TolerancesType>) {
            this->tolerances.linear.velocityToleranceUpdate(
              this->tracker.getVelocity());
        }
        if constexpr (hasHalfcircleTolerance<TolerancesType>) {
            this->tolerances.linear.halfcircleToleranceUpdate(position,
                                                              target,
                                                              heading);
        }

        if constexpr (hasLargeLinearErrorTolerance<TolerancesType>) {
            this->tolerances.large_linear.errorToleranceUpdate(lateral_error);
        }
        if constexpr (hasLargeLinearVelocityTolerance<TolerancesType>) {
            this->tolerances.large_linear.velocityToleranceUpdate(
              this->tracker.getVelocity());
        }
        if constexpr (hasLargeHalfcircleTolerance<TolerancesType>) {
            this->tolerances.large_linear.halfcircleToleranceUpdate(position,
                                                                    target,
                                                                    heading);
        }

        result.finished = false;

        // check tolerances
        if constexpr (hasLinearTolerance<TolerancesType>) {
            result.inSmallTolerance = this->tolerances.linear.withinTolerance();
            result.finished |= this->tolerances.linear.finished();
            this->tolerances.linear.reset();
        }
        if constexpr (hasLargeLinearTolerance<TolerancesType>) {
            result.inLargeTolerance =
              this->tolerances.large_linear.withinTolerance();
            result.finished |= this->tolerances.large_linear.finished();
            this->tolerances.large_linear.reset();
        }

        // check timeout
        result.finished |=
          timeout
            .transform([state](Time timeout) -> bool {
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
          this->controllers.linear_feedback_controller.update(-lateral_error,
                                                              0.0_in,
                                                              delta_time) *
          units::cos(angular_error);

        std::cout << "calculate voltages" << std::endl;

        this->drivetrain.moveArcade(linear_output, angular_output);

        std::cout << "moved arcade" << std::endl;

        return result;
    }

  public:
    moveTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           Length x,
           Length y)
        : Motion<ControllersType, DrivetrainType, TrackerType, TolerancesType>(
            controllers,
            chassis),
          target(x, y) {}

    moveTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           double x,
           double y)
        : moveTo(controllers, chassis, from_in(x), from_in(y)) {}

    moveTo& getReference() {
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
