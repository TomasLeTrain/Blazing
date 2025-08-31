#pragma once

#include "blazing/chassis.hpp"
#include "blazing/controllers.hpp"
#include "blazing/feedback/pid.hpp"
#include "blazing/tolerances.hpp"
#include "pros/rtos.hpp"
#include "units/units.hpp"
#include <concepts>

namespace blazing {

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
class Motion {
  protected:
    virtual int getLoopDelayTime() = 0;
    virtual void execute() = 0;

    DrivetrainType drivetrain;
    TrackerType tracker;
    TolerancesType tolerances;

    ControllersType controllers;

  public:
    Motion(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis)
        : controllers(controllers),
          drivetrain(chassis.drivetrain),
          tracker(chassis.tracker),
          tolerances(chassis.tolerances) {}

    // Motion(ControllersType controllers,
    //        DrivetrainType drivetrain,
    //        TrackerType tracker,
    //        TolerancesType tolerances)
    //     : controllers(controllers),
    //       drivetrain(drivetrain),
    //       tracker(tracker),
    //       tolerances(tolerances) {}

    // waits until finishes
    void run() {
        while (true) {
            this->execute();
            pros::delay(getLoopDelayTime());
        }
    };

    // runs async
    void async() {
        // spawn a task to run this in
        pros::Task([this]() {
            this->run();
        });
    };

    // the current api allows all the change functions to be specified here
    // without having to repeat them for every motion

    // tolerance functions
    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto linearToleranceDuration(this Self&& self, Time duration)
    {
        self.tolerances.linear.setDuration(duration);
        return self.getReference();
    }
    // tolerance functions
    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto angularToleranceDuration(this Self&& self, Time duration)
    {
        self.tolerances.angular.setDuration(duration);
        return self.getReference();
    }
	
    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto linearErrorTolerance(this Self&& self, Length tolerance)
        requires hasLinearErrorTolerance<TolerancesType>
    {
        self.tolerances.linear.setErrorTolerance(tolerance);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto angularErrorTolerance(this Self&& self, Angle tolerance)
        requires hasAngularErrorTolerance<TolerancesType>
    {
        self.tolerances.angular.setErrorTolerance(tolerance);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto linearVelocityTolerance(this Self&& self, LinearVelocity tolerance)
        requires hasLinearVelocityTolerance<TolerancesType>
    {
        self.tolerances.linear.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto angularVelocityTolerance(this Self&& self, AngularVelocity tolerance)
        requires hasAngularVelocityTolerance<TolerancesType>
    {
        self.tolerances.angular.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto halfcircleTolerance(this Self&& self, Length tolerance)
        requires hasHalfcircleTolerance<TolerancesType>
    {
        self.tolerances.linear.setHalfcircleTolerance(tolerance);
        return self.getReference();
    }

    // pid functions
    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto lateral_kP(this Self&& self, KP_t<Length, Voltage> kP)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_kP(kP);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto lateral_kI(this Self&& self, KI_t<Length, Voltage> kI)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_kI(kI);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto lateral_kD(this Self&& self, KD_t<Length, Voltage> kD)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_kD(kD);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto lateral_windupRange(this Self&& self,
                              std::optional<Length> windupRange)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_windupRange(
          windupRange);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto lateral_positiveSlew(this Self&& self,
                               std::optional<Voltage> positiveSlew)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_positiveSlew(
          positiveSlew);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto lateral_negativeSlew(this Self&& self,
                               std::optional<Voltage> negativeSlew)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_negativeSlew(
          negativeSlew);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto angular_kP(this Self&& self, KP_t<Angle, Voltage> kP)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_kP(kP);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto angular_kI(this Self&& self, KI_t<Angle, Voltage> kI)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_kI(kI);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto angular_kD(this Self&& self, KD_t<Angle, Voltage> kD)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_kD(kD);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto angular_windupRange(this Self&& self,
                              std::optional<Angle> windupRange)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_windupRange(
          windupRange);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto angular_positiveSlew(this Self&& self,
                               std::optional<Voltage> positiveSlew)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_positiveSlew(
          positiveSlew);
        return self.getReference();
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    auto angular_negativeSlew(this Self&& self,
                               std::optional<Voltage> negativeSlew)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_negativeSlew(
          negativeSlew);
        return self.getReference();
    }
};

} // namespace blazing
