#pragma once

#include "blazing/chassis.hpp"
#include "blazing/controllers/feedback/feedback.hpp"
#include "blazing/controllers/slew.hpp"
#include "blazing/controllers/voltage_clamp.hpp"
#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/motions/motion.hpp"
#include "blazing/tolerances.hpp"
#include "blazing/trackers/tracker.hpp"
#include "blazing/utils.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include <iostream>

namespace blazing {
struct MoveToState {
    bool close;
    std::optional<Time> last_time;
    Time start_time;
    std::optional<Angle> locked_heading;
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
    Length close_threshold = 4_in;
    bool overturn = false;

    std::optional<MoveToState> m_state;

    int getLoopDelayTime() override {
        return 10;
    }

    motionExecutionResult execute() override {
        if (!m_state.has_value()) {
            m_state = { .close = false,
                        .last_time = from_msec(pros::millis()),
                        .start_time = from_msec(pros::millis()),
                        .locked_heading = std::nullopt };
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
        const Angle heading = [&] -> Angle {
            const Angle heading = this->tracker.getAngle();
            return reversed ? reverseAngle(heading) : heading;
        }();

        Length linear_error =
          (target - position).magnitude() * (reversed ? -1.0 : 1.0);

        Angle position_target_heading = position.angleTo(target);

        if (units::abs(linear_error) < close_threshold && !state.close) {
            state.locked_heading = position_target_heading;
            state.close = true;
        }

        // switches to locked heading when close
        Angle target_heading = state.locked_heading ? *state.locked_heading :
                                                      position_target_heading;

        // used to determine sign and cosine scaling of linear output
        Angle position_target_error =
          angleError(position_target_heading, heading);

        Angle angular_error = angleError(target_heading, heading);

        auto angle_func = [](Angle angle) -> double {
            // return units::cos(angle);

            angle = units::abs(units::constrainAngle360(angle));

            // defined on the range [0,pi/2]
            auto func = [](double x) -> double {
                if (x < 1.224747) {
					// simple polynomial that delays linear output
                    return 1.0 - 2.0 * (x * x) + 1.08866 * (x * x * x);
                }
                return 0.00001;
            };

            if (angle <= rot / 2.0) {
                return func(angle.internal());
            } else {
                return -func(M_PI - angle.internal());
            }
        };

        // used for cosine scaling and applying correct sign for linear
        // error/output
        Number lin_multiplier = angle_func(position_target_error);

        // applies sign component here so that sign of error is accurate
		// NOTE: sgn can be zero, which can set linear error to zero as well!
        linear_error *= units::sgn(lin_multiplier) == 0 ?
                          Number(1.0) :
                          units::sgn(lin_multiplier);

        this->tolerances.linearErrorToleranceUpdate(linear_error);
        this->tolerances.linearVelocityToleranceUpdate(
          this->tracker.getLinearVelocity());
        this->tolerances.linearHalfcircleToleranceUpdate(position,
                                                         target,
                                                         heading);

        result.finished = false;

        // check tolerances
        if constexpr (hasLinearTolerance<TolerancesType>) {
            result.inSmallTolerance = this->tolerances.linear.withinTolerance();
            result.finished |= this->tolerances.linear.finished();
        }
        if constexpr (hasLargeLinearTolerance<TolerancesType>) {
            result.inLargeTolerance =
              this->tolerances.large_linear.withinTolerance();
            result.finished |= this->tolerances.large_linear.finished();
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

        // calculate outputs
        Voltage angular_output =
          this->controllers.angular_feedback_controller.update(-angular_error,
                                                               0_stRad,
                                                               delta_time);

        Voltage linear_output =
          this->controllers.linear_feedback_controller.update(-linear_error,
                                                              0.0_in,
                                                              delta_time);

        // sign was already applied to error, only applies cosine scaling
        // component
        linear_output *= units::abs(lin_multiplier);

        // here the robot would attempt to move backwards, when instead the
        // robot should turn around until it should start moving towards the
        // target
        // the reason that this is done to linear_output and not linear_error is
        // because that would trigger error tolerances
        if (!state.close && lin_multiplier < 0) {
            linear_output = 0_volt;
        }

        // apply min voltage constraints
        if constexpr (hasLinearVoltageClampController<ControllersType>) {
            linear_output =
              this->controllers.linear_voltage_clamp_controller.applyMin(
                linear_output);
        }
        if constexpr (hasAngularVoltageClampController<ControllersType>) {
            angular_output =
              this->controllers.angular_voltage_clamp_controller.applyMin(
                angular_output);
        }

        Voltage max_output = 1_volt;

        // apply overturn
        Voltage overturn_value =
          units::abs(linear_output) + units::abs(angular_output) - max_output;

        if (overturn_value > 0_volt && overturn) {
            linear_output -= overturn_value * units::sgn(linear_output);
        }

        // apply max voltage constraints
        if constexpr (hasLinearVoltageClampController<ControllersType>) {
            linear_output =
              this->controllers.linear_voltage_clamp_controller.applyMax(
                linear_output);
        }
        if constexpr (hasAngularVoltageClampController<ControllersType>) {
            angular_output =
              this->controllers.angular_voltage_clamp_controller.applyMax(
                angular_output);
        }

        // apply slew
        if constexpr (hasLinearSlewController<ControllersType>) {
            linear_output =
              this->controllers.linear_slew_controller.apply(linear_output,
                                                             delta_time);
        }
        if constexpr (hasAngularSlewController<ControllersType>) {
            angular_output =
              this->controllers.angular_slew_controller.apply(angular_output,
                                                              delta_time);
        }

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

    // changer methods

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    auto reverse() {
        this->reversed = true;

        return this->getReference();
    }

    auto withOverturn() {
        this->overturn = true;

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
