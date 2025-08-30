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
    Self& errorTolerance(this Self&& self, Length tolerance)
        requires hasErrorTolerance<TolerancesType, Length>
    {
        self.tolerances.setErrorTolerance(tolerance);
        return self;
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& velocityTolerance(this Self&& self, LinearVelocity tolerance)
        requires hasVelocityTolerance<TolerancesType, Length>
    {
        self.tolerances.setVelocityTolerance(tolerance);
        return self;
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& halfcircleTolerance(this Self&& self, Length tolerance)
        requires hasHalfcircleTolerance<TolerancesType>
    {
        self.tolerances.setHalfcircleTolerance(tolerance);
        return self;
    }

    // pid functions
    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& lateral_kP(this Self&& self, KP_t<Length, Voltage> kP)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_kP(kP);
        return self;
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& lateral_kI(this Self&& self, KI_t<Length, Voltage> kI)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_kI(kI);
        return self;
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& lateral_kD(this Self&& self, KD_t<Length, Voltage> kD)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_kD(kD);
        return self;
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& lateral_windupRange(this Self&& self,
                              std::optional<Length> windupRange)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_windupRange(
          windupRange);
        return self;
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& lateral_positiveSlew(this Self&& self,
                               std::optional<Voltage> positiveSlew)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_positiveSlew(
          positiveSlew);
        return self;
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& lateral_negativeSlew(this Self&& self,
                               std::optional<Voltage> negativeSlew)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_negativeSlew(
          negativeSlew);
        return self;
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& angular_kP(this Self&& self, KP_t<Angle, Voltage> kP)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_kP(kP);
        return self;
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& angular_kI(this Self&& self, KI_t<Angle, Voltage> kI)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_kI(kI);
        return self;
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& angular_kD(this Self&& self, KD_t<Angle, Voltage> kD)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_kD(kD);
        return self;
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& angular_windupRange(this Self&& self,
                              std::optional<Angle> windupRange)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_windupRange(
          windupRange);
        return self;
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& angular_positiveSlew(this Self&& self,
                               std::optional<Voltage> positiveSlew)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_positiveSlew(
          positiveSlew);
        return self;
    }

    template<typename Self>
    [[nodiscard("motion won't be executed!")]]
    Self& angular_negativeSlew(this Self&& self,
                               std::optional<Voltage> negativeSlew)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_negativeSlew(
          negativeSlew);
        return self;
    }
};

} // namespace blazing
