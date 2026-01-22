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
#include "units/units.hpp"
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <variant>

namespace blazing {
struct MoveToState {
    bool close;
    std::optional<Time> last_time;
    Time start_time;
    std::optional<Angle> locked_heading;
};

// abstracts all interactions with controllers into one class
template<typename Input, typename Output>
class FeedbackClampSlewClass {
  public:
    // controller update
    // needs to be implemented
    virtual Output update(Input measurement, Input target, Time duration) = 0;

    // voltage clamp functions
    // leaves output untouched if not implemented
    virtual Output clampApplyMin(Output output) {
        return output;
    };

    virtual Output clampApplyMax(Output output) {
        return output;
    };

    // slew
    // leaves output untouched if not implemented
    virtual Output slewApply(Output output, Time delta_time) {
        return output;
    }

    virtual ~FeedbackClampSlewClass() = default;
};

// implements interactions, and allows using custom controller type while still
// allowing motions to hold pointers
template<typename Controller, typename Input, typename Output>
    requires Feedback<Controller, Input, Output>
class ControllerFeedbackClampSlewClass
    : public FeedbackClampSlewClass<Input, Output> {
  public:
    Controller controller;
    VoltageClampController voltage_clamp;
    SlewController slew;

    ControllerFeedbackClampSlewClass(Controller controller)
        : controller(controller) {}

    ControllerFeedbackClampSlewClass(Controller controller,
                                     VoltageClampController voltage_clamp,
                                     SlewController slew)
        : controller(controller),
          voltage_clamp(voltage_clamp),
          slew(slew) {}

    // controller update
    Output update(Input measurement, Input target, Time duration) override {
        return controller.update(measurement, target, duration);
    }

    // voltage clamp
    virtual Output clampApplyMin(Output output) override {
        return voltage_clamp.applyMin(output);
    }

    virtual Output clampApplyMax(Output output) override {
        return voltage_clamp.applyMax(output);
    }

    // slew
    virtual Output slewApply(Output output, Time delta_time) override {
        return slew.apply(output, delta_time);
    }

    // creates a copy of itself as a unique_ptr
    std::unique_ptr<ControllerFeedbackClampSlewClass<Controller, Input, Output>>
    copy() {
        return std::make_unique<
          ControllerFeedbackClampSlewClass<Controller, Input, Output>>(*this);
    }
};

template<typename DrivetrainType, typename TrackerType, typename TolerancesType>
    requires poseTracker<TrackerType> && linearVelocityTracker<TrackerType> &&
             ArcadeDrivetrain<DrivetrainType>
class moveTo : public Motion<DrivetrainType, TrackerType> {
  private:
    using LinearController = FeedbackClampSlewClass<Length, Voltage>;
    using AngularController = FeedbackClampSlewClass<Angle, Voltage>;

  public:
    std::shared_ptr<LinearController> linear_controller;
    std::shared_ptr<AngularController> angular_controller;
    std::shared_ptr<TolerancesType> tolerances;

  private:
    using point_func_t = std::function<units::V2Position()>;
    std::variant<units::V2Position, point_func_t> target;

    // moveTo-specific properties
    std::optional<Time> m_timeout = std::nullopt;
    bool reversed = false;
    Length close_threshold = 4_in;
    std::optional<Voltage> max_overturn_output = std::nullopt;

    std::optional<Divided<Angle, Length>> m_k_lat = std::nullopt;

    bool m_only_x = false;
    bool m_only_y = false;

    // defaults to cosine of angle
    std::function<double(Angle)> angular_linear_func =
      [](Angle angle) -> double {
        return units::cos(angle);
    };

    std::optional<MoveToState> m_state;

  public:
    int getLoopDelayTime() override {
        return 10;
    }

    std::optional<motionExecutionResult> execute() override {
        if (!m_state.has_value()) {
            m_state = { .close = false,
                        .last_time = now(),
                        .start_time = now(),
                        .locked_heading = std::nullopt };
            // done to prevent values like delta_time being 0
            return std::nullopt;
        }

        MoveToState& state = m_state.value();
        motionExecutionResult result;

        // should never equal 0_sec
        Time delta_time = deltaTime(state.last_time);

        const units::V2Position position = this->tracker.getPosition();
        const Angle heading = [&] -> Angle {
            const Angle heading = this->tracker.getAngle();
            return reversed ? reverseAngle(heading) : heading;
        }();

        auto target_point = std::holds_alternative<units::V2Position>(target) ?
                              // either we have a point
                              std::get<units::V2Position>(target) :
                              // or a function returning a point
                              std::get<point_func_t>(target)();

        Length linear_error = [&] -> Length {
            double reverse_multiplier = reversed ? -1.0 : 1.0;

            if (m_only_x) {
                return units::abs(target_point.x - position.x) *
                       reverse_multiplier;
            }
            if (m_only_y) {
                return units::abs(target_point.y - position.y) *
                       reverse_multiplier;
            }

            // none active, error like normal
            return (target_point - position).magnitude() * reverse_multiplier;
        }();

        Angle position_target_heading = position.angleTo(target_point);

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

        // used for cosine scaling and applying correct sign for linear
        // error/output
        Number lin_multiplier = angular_linear_func(position_target_error);

        // applies sign component here so that sign of error is accurate
        // NOTE: sgn can be zero, which can set linear error to zero as well!
        linear_error *= signed_sgn(lin_multiplier);

        tolerances->linearErrorToleranceUpdate(linear_error);
        tolerances->linearVelocityToleranceUpdate(
          this->tracker.getLinearVelocity());
        // TODO: does half circle exit make sense here?
        tolerances->linearHalfcircleToleranceUpdate(position,
                                                    target_point,
                                                    target_heading);

        result.finished = false;

        // check tolerances
        if constexpr (hasLinearTolerance<TolerancesType>) {
            result.inSmallTolerance = tolerances->linear.withinTolerance();
            result.finished |= tolerances->linear.finished();
        }
        if constexpr (hasLargeLinearTolerance<TolerancesType>) {
            result.inLargeTolerance =
              tolerances->large_linear.withinTolerance();
            result.finished |= tolerances->large_linear.finished();
        }
        // dont use to check if we have finished
        if constexpr (hasChainLinearTolerance<TolerancesType>) {
            result.inChainTolerance =
              tolerances->chain_linear.withinTolerance();
        }

        // check timeout
        result.finished |= timeoutDone(m_timeout, state.start_time);

        // finished if any of the available tolerances or timeout are triggered
        if (result.finished) {
            this->drivetrain.moveArcade(0_volt, 0_volt);
            // returns immediately to avoid more movement
            return result;
        }

        // calculate outputs
        Voltage angular_output =
          angular_controller->update(-angular_error, 0_stRad, delta_time);

        Voltage linear_output =
          linear_controller->update(-linear_error, 0.0_in, delta_time);

        if (m_k_lat) {
            angular_output = angular_output +
                             *m_k_lat * linear_output *
                               (target_point - position).rotatedBy(-heading).y *
                               sinc(angular_error);
        }

        // sign was already applied to error, only applies cosine scaling
        // component
        linear_output *= units::abs(lin_multiplier);

        // here the robot would attempt to move backwards, when instead the
        // robot should turn around until it should start moving towards the
        // target
        // the reason that this is done to linear_output and not linear_error is
        // because otherwise linear_error would be zero and tolerances would
        // trigger
        if (!state.close && lin_multiplier < 0) {
            linear_output = 0_volt;
        }

        // apply min voltage constraints
        linear_output = linear_controller->clampApplyMin(linear_output);
        angular_output = angular_controller->clampApplyMin(angular_output);

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
        linear_output = linear_controller->clampApplyMax(linear_output);
        angular_output = angular_controller->clampApplyMax(angular_output);

        // apply slew
        linear_output = linear_controller->slewApply(linear_output, delta_time);
        angular_output =
          angular_controller->slewApply(angular_output, delta_time);

        this->drivetrain.moveArcade(linear_output, angular_output);

        return result;
    }

  public:
    [[nodiscard("motion won't be executed unless an executor is used!")]]
    moveTo(std::shared_ptr<LinearController> linear_controller,
           std::shared_ptr<AngularController> angular_controller,
           std::shared_ptr<TolerancesType> tolerances,
           DrivetrainType drivetrain,
           TrackerType tracker,
           units::V2Position point)
        : Motion<DrivetrainType, TrackerType>(drivetrain, tracker),
          linear_controller(std::move(linear_controller)),
          angular_controller(std::move(angular_controller)),
          tolerances(std::move(tolerances)),
          target(point) {}

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    moveTo(std::shared_ptr<LinearController> linear_controller,
           std::shared_ptr<AngularController> angular_controller,
           std::shared_ptr<TolerancesType> tolerances,
           DrivetrainType drivetrain,
           TrackerType tracker,
           Length x,
           Length y)
        : Motion<DrivetrainType, TrackerType>(drivetrain, tracker),
          linear_controller(std::move(linear_controller)),
          angular_controller(std::move(angular_controller)),
          tolerances(std::move(tolerances)),
          target(units::V2Position(x, y)) {}

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    moveTo(std::shared_ptr<LinearController> linear_controller,
           std::shared_ptr<AngularController> angular_controller,
           std::shared_ptr<TolerancesType> tolerances,
           DrivetrainType drivetrain,
           TrackerType tracker,
           point_func_t point_func)
        : Motion<DrivetrainType, TrackerType>(drivetrain, tracker),
          linear_controller(std::move(linear_controller)),
          angular_controller(std::move(angular_controller)),
          tolerances(std::move(tolerances)),
          target(point_func) {}

    moveTo& getReference() {
        return *this;
    }

    // changer methods

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto reverse() {
        this->reversed = true;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto overturn(Voltage max_overturn_output = 1_volt) {
        this->max_overturn_output = max_overturn_output;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto k_lat(std::optional<std::variant<Divided<Angle, Length>, double, int>>
                 k_lat = std::nullopt) {
        if (!k_lat)
            this->m_k_lat = std::nullopt;
        else {
            const auto& variant = k_lat.value();
            if (std::holds_alternative<Divided<Angle, Length>>(variant)) {
                this->m_k_lat = std::get<Divided<Angle, Length>>(variant);
            } else if (std::holds_alternative<double>(variant)) {
                this->m_k_lat = std::get<double>(variant) * (rad / m);
            } else if (std::holds_alternative<int>(variant)) {
                this->m_k_lat = std::get<int>(variant) * (rad / m);
            }
        }

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto closeThreshold(Length threshold) {
        this->close_threshold = threshold;
        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto customAngularLinearFunc(
      std::function<double(Angle)> custom_angular_linear_func) {
        angular_linear_func = custom_angular_linear_func;
        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto timeout(Time timeout) {
        this->m_timeout = timeout;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto only_x(bool only_x) {
        this->m_only_x = only_x;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto only_y(bool only_y) {
        this->m_only_y = only_y;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto modify(
      std::function<void(moveTo<DrivetrainType, TrackerType, TolerancesType>*)>
        func) {
        func(this);

        return this->getReference();
    }

    auto modify_lin(std::function<void(decltype(linear_controller))> func) {
        func(linear_controller);

        return this->getReference();
    }

    auto modify_ang(std::function<void(decltype(angular_controller))> func) {
        func(angular_controller);

        return this->getReference();
    }
};
} // namespace blazing
