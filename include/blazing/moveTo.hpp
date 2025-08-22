#pragma once

#include "blazing/drivetrain.hpp"
#include "blazing/motion.hpp"
#include "blazing/pid.hpp"
#include "blazing/tolerances.hpp"
#include "units/Vector2D.hpp"

// pid is just another input
// trackers are inputs
// drivetrain is the output

// controllers just control how some target -> some output
// we could make defaults by using singletons or smth
// therefore when the motion gets initalized it gets initialized with some
// defaults, which can then be changed

namespace blazing {

template<typename T, typename Input, typename Output>
concept Feedback =
  requires(T controller, Input measurement, Input setpoint, Time duration) {
      {
          controller.update(measurement, setpoint, duration)
      } -> std::same_as<Output>;
  };

template<typename T, typename Input, typename Output>
concept Feedforward = requires(T controller, Input input, Time duration) {
    { controller.update(input, duration) } -> std::same_as<Output>;
};

struct MoveToState {
    bool close;
    std::optional<Time> last_time;
    Time start_time;
};

template<typename LinearController,
         typename AngularController,
         typename Drivetrain,
         typename Tracker>
    requires Feedback<LinearController, Length, Voltage> &&
             Feedback<AngularController, Angle, Voltage> &&
             poseTracker<Tracker> && TankDrivetrain<Drivetrain>
class moveTo : public Motion {
  private:
    units::V2Position target;
    bool reversed;

    LinearController linear_controller;
    AngularController angular_controller;

    Chassis<Drivetrain, Tracker> drivetrain;
    Tolerances<Length> tolerances;

    std::optional<Time> timeout;

    std::optional<MoveToState> m_state;

    int getLoopDelayTime() override {
        return 10;
    }

    void execute() override {
        if (!m_state.has_value()) {
            m_state = { .close = false,
                        .last_time = from_msec(pros::millis()),
                        .start_time = from_msec(pros::millis()) };
        }

        MoveToState& state = m_state.value();

        Time delta_time = state.last_time.has_value() ?
                            from_msec(pros::millis()) - *state.last_time :
                            0.0_sec;

        std::optional<Time> timeout;

        units::V2Position position = drivetrain.tracker.getPosition();
        Angle heading = drivetrain.tracker.getPosition();

        auto local_target = target - position;
        Length distance_error = local_target.magnitude();

        if (units::abs(distance_error) < 7.5_in && !state.close) {
            state.close = true;
        }

        Angle angle_error = heading - local_target.getAngle();

        if (reversed) {
            distance_error *= -1.0;
            angle_error = (rot / 2) - angle_error;
        }

        // check tolerances and timeout
        if (tolerances.check(distance_error,
                             drivetrain.tracker.getVelocity()) ||
            timeout.transform([&](Time timeout) -> bool {
                return from_msec(pros::millis()) - state.start_time > timeout;
            })) {
            drivetrain.drivetrain.moveArcade(0_volt, 0_volt);
        }

        Voltage angular_output =
          state.close ?
            0.0 :
            angular_controller.update(-angle_error, 0_stRad, delta_time);

        Voltage linear_output =
          linear_controller.update(-distance_error, 0.0_in, delta_time) *
          units::cos(angle_error);

        drivetrain.drivetrain.moveArcade(linear_output, angular_output);
    }

  public:
    moveTo(LinearController lateral_controller,
           AngularController angular_controller,
           Chassis<Drivetrain, Tracker> drivetrain,
           double x,
           double y)
        : linear_controller(lateral_controller),
          angular_controller(angular_controller),
          drivetrain(drivetrain),
          target(from_in(x), from_in(y)) {}

    moveTo(LinearController lateral_controller,
           AngularController angular_controller,
           Chassis<Drivetrain, Tracker> drivetrain,
           Length x,
           Length y)
        : linear_controller(lateral_controller),
          angular_controller(angular_controller),
          drivetrain(drivetrain),
          target(x, y) {}

    // functions which alter the motion conditions
    [[nodiscard("motion won't be executed!")]]
    moveTo& reverse() {
        // alter current state

        this->reversed = true;

        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    moveTo& lateral_kP(KP_t<Length, Voltage> kP)
        requires hasKP<LinearController, Length, Voltage>
    {
        linear_controller.set_kP(kP);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    moveTo& angular_kP(KP_t<Angle, Voltage> kP)
        requires hasKP<AngularController, Angle, Voltage>
    {
        angular_controller.set_kP(kP);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    moveTo& lateral_kI(KI_t<Length, Voltage> kI)
        requires hasKI<LinearController, Length, Voltage>
    {
        linear_controller.set_kI(kI);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    moveTo& angular_kI(KI_t<Angle, Voltage> kI)
        requires hasKI<AngularController, Angle, Voltage>
    {

        angular_controller.set_kI(kI);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    moveTo& lateral_kD(KD_t<Length, Voltage> kD)
        requires hasKD<LinearController, Length, Voltage>
    {
        linear_controller.set_kD(kD);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    moveTo& angular_kD(KD_t<Angle, Voltage> kD)
        requires hasKD<AngularController, Angle, Voltage>
    {
        angular_controller.set_kD(kD);
        return *this;
    }
};
} // namespace blazing
