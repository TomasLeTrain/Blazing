#pragma once

#include "blazing/chassis.hpp"
#include "blazing/controllers.hpp"
#include "blazing/feedback/pid.hpp"
#include "blazing/tolerances.hpp"
#include "pros/rtos.hpp"
#include "units/units.hpp"
#include <concepts>

namespace blazing {

// used to make the motion changer methods more readable
#define motionChanger                                                  \
    template<typename Self>                                            \
    [[nodiscard(                                                       \
      "motion won't be executed unless run or async are used!")]] auto

#define motionChangerT                                                 \
    template<typename Self, typename T>                                \
    [[nodiscard(                                                       \
      "motion won't be executed unless run or async are used!")]] auto

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

    motionChanger halfcircleTolerance(this Self&& self, Length tolerance) {
        self.tolerances.linear.setHalfcircleTolerance(tolerance);
        return self.getReference();
    }

    // lateral pid changers
    motionChangerT lateral_kP(this Self&& self, T kP)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_kP(kP);
        return self.getReference();
    }

    motionChangerT lateral_kI(this Self&& self, T kI)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_kI(kI);
        return self.getReference();
    }

    motionChangerT lateral_kD(this Self&& self, T kD)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_kD(kD);
        return self.getReference();
    }

    motionChangerT lateral_windupRange(this Self&& self, T windupRange)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_windupRange(
          windupRange);
        return self.getReference();
    }

    motionChangerT lateral_positiveSlew(this Self&& self, T positiveSlew)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_positiveSlew(
          positiveSlew);
        return self.getReference();
    }

    motionChangerT lateral_negativeSlew(this Self&& self, T negativeSlew)
        requires std::derived_from<ControllersType, PIDLinearController>
    {
        self.controllers.linear_feedback_controller.set_negativeSlew(
          negativeSlew);
        return self.getReference();
    }

    // angular pid changers
    motionChangerT angular_kP(this Self&& self, T kP)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_kP(kP);
        return self.getReference();
    }

    motionChangerT angular_kI(this Self&& self, T kI)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_kI(kI);
        return self.getReference();
    }

    motionChangerT angular_kD(this Self&& self, T kD)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_kD(kD);
        return self.getReference();
    }

    motionChangerT angular_windupRange(this Self&& self, T windupRange)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_windupRange(
          windupRange);
        return self.getReference();
    }

    motionChangerT angular_positiveSlew(this Self&& self, T positiveSlew)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_positiveSlew(
          positiveSlew);
        return self.getReference();
    }

    motionChangerT angular_negativeSlew(this Self&& self, T negativeSlew)
        requires std::derived_from<ControllersType, PIDAngularController>
    {
        self.controllers.angular_feedback_controller.set_negativeSlew(
          negativeSlew);
        return self.getReference();
    }
};

} // namespace blazing
