#pragma once

#include "blazing/chassis.hpp"
#include "blazing/controllers/feedback/feedback.hpp"
#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/motions/motion.hpp"
#include "blazing/tolerances.hpp"
#include "blazing/trackers/tracker.hpp"
#include "blazing/utils.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include <iostream>

namespace blazing {
struct MoveToState {
    std::optional<Time> last_time;
    Time start_time;

    bool close;
    units::V2FPosition prev_position;
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires poseTracker<TrackerType> && linearVelocityTracker<TrackerType> &&
             ArcadeDrivetrain<DrivetrainType> &&
             hasAngularFeedbackController<ControllersType> &&
             hasLinearFeedbackController<ControllersType>
class boomerang : public Motion<ControllersType,
                                DrivetrainType,
                                TrackerType,
                                TolerancesType> {
  private:
    units::Pose target;

    // boomerang-specific properties
    std::optional<Time> timeout = std::nullopt;
    bool reversed = false;
    double lead = 0.5;
    Length close_threshold = 4_in;

    std::optional<MoveToState> m_state;

    int getLoopDelayTime() override {
        return 10;
    }

    motionExecutionResult execute() override {
        if (!m_state.has_value()) {
            m_state = { .last_time = from_msec(pros::millis()),
                        .start_time = from_msec(pros::millis()),
                        .close = false,
                        .prev_position = this->tracker.getPosition() };
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

        const units::V2Position position = this->tracker.getPosition();

        const Angle heading = [this] {
            const Angle heading = this->tracker.getAngle();
            return reversed ? reverseAngle(heading) : heading;
        }();

        const Length pose_target_distance = position.distanceTo(target);

        // when close gets activated it switches to move to point behavior

        if (units::abs(pose_target_distance) < close_threshold &&
            !state.close) {
            state.close = true;
        }

        const units::V2Position carrot =
          state.close ?
            target :
            target - units::V2Position::fromPolar(target.orientation,
                                                  pose_target_distance * lead);

        const Angle target_heading =
          state.close ? target.orientation : position.angleTo(carrot);

        Length linear_error =
          position.distanceTo(carrot) * (reversed ? -1.0 : 1.0);

        Angle angular_error = angleError(target_heading, heading);

        if (state.close && units::abs(angular_error) >= 90.0_stDeg) {
            linear_error *= -1.0;
            angular_error =
              units::constrainAngle180(reverseAngle(angular_error));
        }

        // update tolerances if they are included
        if constexpr (hasLinearErrorTolerance<TolerancesType>) {
            this->tolerances.linear.errorToleranceUpdate(linear_error);
        }
        if constexpr (hasLinearVelocityTolerance<TolerancesType>) {
            this->tolerances.linear.velocityToleranceUpdate(
              this->tracker.getLinearVelocity());
        }
        if constexpr (hasHalfcircleTolerance<TolerancesType>) {
            this->tolerances.linear.halfcircleToleranceUpdate(position,
                                                              target,
                                                              heading);
        }

        if constexpr (hasLargeLinearErrorTolerance<TolerancesType>) {
            this->tolerances.large_linear.errorToleranceUpdate(linear_error);
        }
        if constexpr (hasLargeLinearVelocityTolerance<TolerancesType>) {
            this->tolerances.large_linear.velocityToleranceUpdate(
              this->tracker.getLinearVelocity());
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
          this->controllers.linear_feedback_controller.update(-linear_error,
                                                              0.0_in,
                                                              delta_time) *
          units::cos(angular_error);

        this->drivetrain.moveArcade(linear_output, angular_output);

        return result;
    }

  public:
    boomerang(ControllersType controllers,
              Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
              units::Pose pose)
        : Motion<ControllersType, DrivetrainType, TrackerType, TolerancesType>(
            controllers,
            chassis),
          target(pose) {}

    boomerang(ControllersType controllers,
              Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
              Length x,
              Length y,
              Angle heading)
        : boomerang(controllers, chassis, { x, y, heading }) {}

    boomerang(ControllersType controllers,
              Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
              double x,
              double y,
              double heading)
        : boomerang(controllers,
                    chassis,
                    from_in(x),
                    from_in(y),
                    from_stDeg(heading)) {}

    boomerang& getReference() {
        return *this;
    }

    // changer methods

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    auto reverse() {
        this->reversed = true;

        return this->getReference();
    }

    auto closeThreshold(Length threshold) {
        this->close_threshold = threshold;
        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    auto withTimeout(Time timeout) {
        this->timeout = timeout;

        return this->getReference();
    }
};
} // namespace blazing
