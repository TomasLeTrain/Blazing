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
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include <iostream>

namespace blazing {
struct BoomerangState {
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
    std::optional<Voltage> max_overturn_output = std::nullopt;

    // defaults to cosine of angle
    std::function<double(Angle)> angular_linear_func =
      [](Angle angle) -> double {
        return units::cos(angle);
    };

    std::optional<BoomerangState> m_state;

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

        BoomerangState& state = m_state.value();
        motionExecutionResult result;

        Time delta_time = deltaTime(state.last_time);

        const units::V2Position position = this->tracker.getPosition();

        const Angle heading = [&] {
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

        Angle position_carrot_heading = position.angleTo(carrot);

        const Angle target_heading =
          state.close ? target.orientation :
			(0.3 * target.orientation + 0.7 * position.angleTo(carrot));

        Length linear_error =
          position.distanceTo(carrot) * (reversed ? -1.0 : 1.0);

        Angle angular_error = angleError(target_heading, heading);

        Angle position_carrot_error =
          angleError(position_carrot_heading, heading);

        // used for cosine scaling and applying correct sign for linear
        // error/output
        Number lin_multiplier = angular_linear_func(position_carrot_error);

        // applies sign component here so that sign of error is accurate
        linear_error *= signed_sgn(lin_multiplier);

        // update tolerances if they are included
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
        // dont use to check if we have finished
        if constexpr (hasChainLinearTolerance<TolerancesType>) {
            result.inChainTolerance =
              this->tolerances.chain_linear.withinTolerance();
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

        if (max_overturn_output) {
            // apply overturn
            Voltage overturn_value = units::abs(linear_output) +
                                     units::abs(angular_output) -
                                     *max_overturn_output;

            if (overturn_value > 0_volt) {
                linear_output -= overturn_value * units::sgn(linear_output);
            }
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

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto reverse() {
        this->reversed = true;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto withOverturn(Voltage max_overturn_output = 1_volt) {
        this->max_overturn_output = max_overturn_output;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto closeThreshold(Length threshold) {
        this->close_threshold = threshold;
        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto withLead(double lead) {
        this->lead = lead;
        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto customAngularLinearFunc(
      std::function<double(Angle)> custom_angular_linear_func) {
        angular_linear_func = custom_angular_linear_func;
        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto withTimeout(Time timeout) {
        this->timeout = timeout;

        return this->getReference();
    }
};
} // namespace blazing
