#pragma once

#include "blazing/drivetrain.hpp"
#include "blazing/motion.hpp"
#include "blazing/pid.hpp"
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

template<typename T, typename Input, typename Output>
concept hasKP =
  requires(T t, KP_t<Input, Output> kp_value) { t.set_kP(kp_value); };

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

    LinearController lateral_controller;
    AngularController angular_controller;

    Chassis<Drivetrain, Tracker> drivetrain;

    int getLoopDelayTime() override {
        return 10;
    }

    void execute() override {
        // actual logic
    }

  public:
    moveTo(LinearController lateral_controller,
           AngularController linear_controller,
           Chassis<Drivetrain, Tracker> drivetrain,
           double x,
           double y)
        : lateral_controller(lateral_controller),
          angular_controller(angular_controller),
          drivetrain(drivetrain),
          target(from_in(x), from_in(y)) {}

    moveTo(LinearController lateral_pid,
           AngularController angular_pid,
           Chassis<Drivetrain, Tracker> drivetrain,
           Length x,
           Length y)
        : lateral_controller(lateral_pid),
          angular_controller(angular_pid),
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
        requires requires(LinearController t, KP_t<Length, Voltage> val) {
            t.set_kP(val);
        }
    {
        lateral_controller.set_kP(kP);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    moveTo& angular_kP(KP_t<Angle, Voltage> kP)
        requires requires(AngularController t, KP_t<Angle, Voltage> val) {
            t.set_kP(val);
        }
    {
        angular_controller.set_kP(kP);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    moveTo& lateral_kI(KI_t<Length, Voltage> kI)
        requires requires(LinearController t, KI_t<Length, Voltage> val) {
            t.set_kI(val);
        }
    {
        lateral_controller.set_kI(kI);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    moveTo& angular_kI(KI_t<Angle, Voltage> kI)
        requires requires(AngularController t, KI_t<Angle, Voltage> val) {
            t.set_kI(val);
        }
    {

        angular_controller.set_kI(kI);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    moveTo& lateral_kD(KD_t<Length, Voltage> kD)
        requires requires(LinearController t, KD_t<Length, Voltage> val) {
            t.set_kD(val);
        }
    {
        lateral_controller.set_kD(kD);
        return *this;
    }

    [[nodiscard("motion won't be executed!")]]
    moveTo& angular_kD(KD_t<Angle, Voltage> kD)
        requires requires(AngularController t, KD_t<Angle, Voltage> val) {
            t.set_kD(val);
        }
    {
        angular_controller.set_kD(kD);
        return *this;
    }
};
} // namespace blazing
