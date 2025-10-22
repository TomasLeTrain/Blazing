#pragma once

#include "blazing/chassis.hpp"
#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/slew.hpp"
#include "blazing/controllers/voltage_clamp.hpp"
#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/tolerances.hpp"
#include "units/units.hpp"
#include <concepts>
#include <functional>
#include <optional>
#include <vector>

namespace blazing {

// used to make the motion changer methods more readable
#define motionChanger                                                          \
    template<typename Self>                                                    \
    [[nodiscard("motion won't be executed unless an executor is used!")]] auto

#define motionChangerT                                                         \
    template<typename Self, typename T>                                        \
    [[nodiscard("motion won't be executed unless an executor is used!")]] auto

#define motionChangerTU                                                        \
    template<typename Self, typename T, typename U>                            \
    [[nodiscard("motion won't be executed unless an executor is used!")]] auto

struct motionExecutionResult {
    std::optional<bool> inLargeTolerance = std::nullopt;
    std::optional<bool> inSmallTolerance = std::nullopt;
    std::optional<bool> inChainTolerance = std::nullopt;
    bool finished = false;
};

// untemplated class to allow pointers
class MotionBase {
  public:
    std::function<void()> during_motion_func;
    std::function<void()> after_motion_func;

    virtual int getLoopDelayTime() = 0;
    virtual std::optional<motionExecutionResult> execute() = 0;

    // functions meant to be used for chaining motions
    virtual bool setEnabledDrivetrain(bool enabled) {
        return false;
    }

    virtual std::optional<std::vector<Voltage>> getVoltagesDrivetrain() {
        return std::nullopt;
    }

    virtual bool moveVoltagesDrivetrain(std::vector<Voltage> voltages) {
        return false;
    }

    virtual std::optional<Time> getChainTime() {
        return std::nullopt;
    }

    virtual ~MotionBase() = default;
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires std::derived_from<TolerancesType, TolerancesGroup>
class Motion : public MotionBase {
  protected:
    // these are assumed to have no issues being copied
    ControllersType controllers;
    TolerancesType tolerances;

    TrackerType& tracker;
    DrivetrainType& drivetrain;

    std::optional<Time> chain_time = std::nullopt;

  public:
    Motion(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis)
        : controllers(controllers),
          tolerances(chassis.tolerances),
          tracker(chassis.tracker),
          drivetrain(chassis.drivetrain) {}

    // attempt to override chain functions
    bool setEnabledDrivetrain(bool enabled) override {
        if constexpr (MotionChainableDrivetrain<DrivetrainType>) {
            drivetrain.setEnabled(enabled);
            return true;
        }
        return false;
    };

    std::optional<std::vector<Voltage>> getVoltagesDrivetrain() override {
        if constexpr (MotionChainableDrivetrain<DrivetrainType>) {
            return drivetrain.getVoltages();
        }
        return std::nullopt;
    };

    bool moveVoltagesDrivetrain(std::vector<Voltage> voltages) override {
        if constexpr (MotionChainableDrivetrain<DrivetrainType>) {
            drivetrain.moveVoltages(voltages);
            return true;
        }
        return false;
    };

    std::optional<Time> getChainTime() override {
        return chain_time;
    };

    motionChanger setChainTime(this Self&& self, Time chain_time) {
        self.chain_time = chain_time;
        return self.getReference();
    };

    // tracker
    motionChanger executeBeforeMotion(this Self&& self,
                                      std::function<void()> func) {
        // immediately executes
        func();
        return self.getReference();
    }

    // gets repeatedly executed while a motion is in motion
    motionChanger executeDuringMotion(this Self&& self,
                                      std::function<void()> func) {
        // immediately executes
        self.during_motion_func = func;
        return self.getReference();
    }

    motionChanger executeAfterMotion(this Self&& self,
                                     std::function<void()> func) {
        // immediately executes
        self.after_motion_func = func;
        return self.getReference();
    }

    // tolerance duration changers

    motionChanger linearToleranceDuration(this Self&& self, Time duration) {
        self.tolerances.linear.setDuration(duration);
        return self.getReference();
    }

    motionChanger angularToleranceDuration(this Self&& self, Time duration) {
        self.tolerances.angular.setDuration(duration);
        return self.getReference();
    }

    motionChanger largeLinearToleranceDuration(this Self&& self,
                                               Time duration) {
        self.tolerances.large_linear.setDuration(duration);
        return self.getReference();
    }

    motionChanger largeAngularToleranceDuration(this Self&& self,
                                                Time duration) {
        self.tolerances.large_angular.setDuration(duration);
        return self.getReference();
    }

    // not really relevant to how its supposed to be used
    //
    // motionChanger chainLinearToleranceDuration(this Self&& self,
    //                                            Time duration) {
    //     self.tolerances.chain_linear.setDuration(duration);
    //     return self.getReference();
    // }
    //
    // motionChanger chainAngularToleranceDuration(this Self&& self,
    //                                             Time duration) {
    //     self.tolerances.chain_angular.setDuration(duration);
    //     return self.getReference();
    // }

    // Error tolerance changers
    motionChanger linearErrorTolerance(this Self&& self, Length tolerance) {
        self.tolerances.linear.setErrorTolerance(tolerance);
        return self.getReference();
    }

    motionChanger angularErrorTolerance(this Self&& self, Angle tolerance) {
        self.tolerances.angular.setErrorTolerance(tolerance);
        return self.getReference();
    }

    motionChanger largeLinearErrorTolerance(this Self&& self,
                                            Length tolerance) {
        self.tolerances.large_linear.setErrorTolerance(tolerance);
        return self.getReference();
    }

    motionChanger largeAngularErrorTolerance(this Self&& self,
                                             Angle tolerance) {
        self.tolerances.large_angular.setErrorTolerance(tolerance);
        return self.getReference();
    }

    motionChanger chainLinearErrorTolerance(this Self&& self,
                                            Length tolerance) {
        self.tolerances.chain_linear.setErrorTolerance(tolerance);
        return self.getReference();
    }

    motionChanger chainAngularErrorTolerance(this Self&& self,
                                             Angle tolerance) {
        self.tolerances.chain_angular.setErrorTolerance(tolerance);
        return self.getReference();
    }

    // velocity tolerance changers
    motionChanger linearVelocityTolerance(this Self&& self,
                                          LinearVelocity tolerance) {
        self.tolerances.linear.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    motionChanger angularVelocityTolerance(this Self&& self,
                                           AngularVelocity tolerance) {
        self.tolerances.angular.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    motionChanger largeLinearVelocityTolerance(this Self&& self,
                                               LinearVelocity tolerance) {
        self.tolerances.large_linear.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    motionChanger largeAngularVelocityTolerance(this Self&& self,
                                                AngularVelocity tolerance) {
        self.tolerances.large_angular.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    motionChanger chainLinearVelocityTolerance(this Self&& self,
                                               LinearVelocity tolerance) {
        self.tolerances.change_linear.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    motionChanger chainAngularVelocityTolerance(this Self&& self,
                                                AngularVelocity tolerance) {
        self.tolerances.chain_angular.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    // half circle tolerances
    motionChanger halfcircleTolerance(this Self&& self, Length tolerance) {
        self.tolerances.linear.setHalfcircleTolerance(tolerance);
        return self.getReference();
    }

    motionChanger largeHalfcircleTolerance(this Self&& self, Length tolerance) {
        self.tolerances.large_linear.setHalfcircleTolerance(tolerance);
        return self.getReference();
    }

    motionChanger chainHalfcircleTolerance(this Self&& self, Length tolerance) {
        self.tolerances.chain_linear.setHalfcircleTolerance(tolerance);
        return self.getReference();
    }

    // linear pid changers
    motionChangerT linear_kp(this Self&& self, T kp)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback.set_kp(kp);
        return self.getReference();
    }

    motionChangerT linear_ki(this Self&& self, T ki)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback.set_ki(ki);
        return self.getReference();
    }

    motionChangerT linear_kd(this Self&& self, T kd)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback.set_kd(kd);
        return self.getReference();
    }

    motionChangerT linear_windupRange(this Self&& self, T windupRange)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback.set_windupRange(windupRange);
        return self.getReference();
    }

    motionChangerT linear_PIDmaxVolt(this Self&& self, T maxVoltage)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback.set_maxVoltage(maxVoltage);
        return self.getReference();
    }

    // angular pid changers
    motionChangerT angular_kp(this Self&& self, T kp)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback.set_kp(kp);
        return self.getReference();
    }

    motionChangerT angular_ki(this Self&& self, T ki)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback.set_ki(ki);
        return self.getReference();
    }

    motionChangerT angular_kd(this Self&& self, T kd)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback.set_kd(kd);
        return self.getReference();
    }

    motionChangerT angular_windupRange(this Self&& self, T windupRange)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback.set_windupRange(windupRange);
        return self.getReference();
    }

    motionChangerT angular_PIDmaxVolt(this Self&& self, T maxVoltage)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback.set_maxVoltage(maxVoltage);
        return self.getReference();
    }

    // linear voltage constraints
    motionChangerTU
    linear_minMaxVolt(this Self&& self, T minVoltage, U maxVoltage)
        requires std::derived_from<ControllersType,
                                   LinearVoltageClampController>
    {
        self.controllers.linear_voltage_clamp.setMin(minVoltage);
        self.controllers.linear_voltage_clamp.setMax(maxVoltage);
        return self.getReference();
    }

    motionChangerT linear_minVolt(this Self&& self, T minVoltage)
        requires std::derived_from<ControllersType,
                                   LinearVoltageClampController>
    {
        self.controllers.linear_voltage_clamp.setMin(minVoltage);
        return self.getReference();
    }

    motionChangerT linear_maxVolt(this Self&& self, T maxVoltage)
        requires std::derived_from<ControllersType,
                                   LinearVoltageClampController>
    {
        self.controllers.linear_voltage_clamp.setMax(maxVoltage);
        return self.getReference();
    }

    // angular voltage constraints
    motionChangerTU
    angular_minMaxVolt(this Self&& self, T minVoltage, U maxVoltage)
        requires std::derived_from<ControllersType,
                                   AngularVoltageClampController>
    {
        self.controllers.angular_voltage_clamp.setMin(minVoltage);
        self.controllers.angular_voltage_clamp.setMax(maxVoltage);
        return self.getReference();
    }

    motionChangerT angular_minVolt(this Self&& self, T minVoltage)
        requires std::derived_from<ControllersType,
                                   AngularVoltageClampController>
    {
        self.controllers.angular_voltage_clamp.setMin(minVoltage);
        return self.getReference();
    }

    motionChangerT angular_maxVolt(this Self&& self, T maxVoltage)
        requires std::derived_from<ControllersType,
                                   AngularVoltageClampController>
    {
        self.controllers.angular_voltage_clamp.setMax(maxVoltage);
        return self.getReference();
    }

    // linear slew changers
    motionChangerT linear_slew(this Self&& self, LinearSlewController new_slew)
        requires hasLinearSlew<ControllersType>
    {
        self.controllers.linear_slew = new_slew;
        return self.getReference();
    }

    motionChangerT linear_accelSlew(this Self&& self, T accelSlew)
        requires hasLinearSlew<ControllersType>
    {
        self.controllers.linear_slew.set_accel(accelSlew);
        return self.getReference();
    }

    motionChangerT linear_decelSlew(this Self&& self, T decelSlew)
        requires hasLinearSlew<ControllersType>
    {
        self.controllers.linear_slew.set_decel(decelSlew);
        return self.getReference();
    }

    // angular slew changers
    motionChangerT angular_slew(this Self&& self,
                                AngularSlewController new_slew)
        requires hasAngularSlew<ControllersType>
    {
        self.controllers.angular_slew = new_slew;
        return self.getReference();
    }

    motionChangerT angular_accelSlew(this Self&& self, T accelSlew)
        requires hasAngularSlew<ControllersType>
    {
        self.controllers.angular_slew.set_accel(accelSlew);
        return self.getReference();
    }

    motionChangerT angular_backwardsAccelSlew(this Self&& self,
                                              T backwardsAccelSlew)
        requires hasAngularSlew<ControllersType>
    {
        self.controllers.angular_slew.set_backwards_accel(backwardsAccelSlew);
        return self.getReference();
    }

    motionChangerT angular_decelSlew(this Self&& self, T decelSlew)
        requires hasAngularSlew<ControllersType>
    {
        self.controllers.angular_slew.set_decel(decelSlew);
        return self.getReference();
    }

    motionChangerT angular_backwardsDecelSlew(this Self&& self,
                                              T backwardsDecelSlew)
        requires hasAngularSlew<ControllersType>
    {
        self.controllers.angular_slew.set_backwards_decel(backwardsDecelSlew);
        return self.getReference();
    }
};

} // namespace blazing
