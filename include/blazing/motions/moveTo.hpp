#pragma once

#include "blazing/chassis.hpp"
#include "blazing/drivetrain.hpp"
#include "blazing/feedback/feedback.hpp"
#include "blazing/motion.hpp"
#include "blazing/tolerances.hpp"
#include "blazing/tracker.hpp"
#include "units/Vector2D.hpp"

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
    requires poseTracker<TrackerType> && velocityTracker<TrackerType> &&
             ArcadeDrivetrain<DrivetrainType>
class moveTo : public Motion<ControllersType,
                             DrivetrainType,
                             TrackerType,
                             TolerancesType> {
  private:
    units::V2Position target;
    bool reversed;

    std::optional<Time> timeout;

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

        Time delta_time = state.last_time.has_value() ?
                            from_msec(pros::millis()) - *state.last_time :
                            0.0_sec;

        std::optional<Time> timeout;

        units::V2Position position = this->tracker.getPosition();
        Angle heading = this->tracker.getAngle();

        auto local_target = target - position;
        Length distance_error = local_target.magnitude();

        if (units::abs(distance_error) < 4_in && !state.close) {
            state.close = true;
        }

        Angle angle_error = heading - position.angleTo(target);

        if (reversed) {
            distance_error *= -1.0;
            angle_error = (rot / 2) - angle_error;
        }

        // If the turn error exceeds 90 degrees, then the point is behind
        // the robot, so it's more efficient to travel to the point
        // backwards. only applies when the robot is close to the point so
        // that it doesn't accidently go to the point backwards at the
        // beginning
        if (state.close && units::abs(angle_error) >= 90.0_stDeg) {
            distance_error *= -1.0;
            angle_error = (rot / 2) - angle_error;
        }

        // update tolerances if they are included
        if constexpr (hasLinearErrorTolerance<TolerancesType>) {
            this->tolerances.linear.errorToleranceUpdate(distance_error);
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
            this->tolerances.large_linear.errorToleranceUpdate(distance_error);
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
          this->controllers.angular_feedback_controller.update(-angle_error,
                                                               0_stRad,
                                                               delta_time);

        Voltage linear_output =
          this->controllers.linear_feedback_controller.update(-distance_error,
                                                              0.0_in,
                                                              delta_time) *
          units::cos(angle_error);

        this->drivetrain.moveArcade(linear_output, angular_output);
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

    // functions which alter the motion conditions
    [[nodiscard("motion won't be executed!")]]
    moveTo& reverse() {
        // alter current state
        this->reversed = true;

        return this->getReference();
    }
};
} // namespace blazing
