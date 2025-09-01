#pragma once

#include "pros/rtos.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <optional>

namespace blazing {

class Tolerance {
  protected:
    std::optional<bool> in_tolerance = std::nullopt;

    void update_in_tolerance(bool tolerance) {
        if (in_tolerance.has_value()) {
            in_tolerance = in_tolerance && tolerance;
        } else {
            in_tolerance = tolerance;
        }
    }
};

template<typename T>
class ErrorTolerance : virtual Tolerance {
  private:
    std::optional<T> error_tolerance = std::nullopt;

  public:
    ErrorTolerance(T error_tolerance)
        : error_tolerance(error_tolerance) {}

    void setErrorTolerance(T error_tolerance) {
        this->error_tolerance = error_tolerance;
    }

    void errorToleranceUpdate(T error) {
        bool curr_tolerance_active = error_tolerance
                                       .transform([error](T tolerance) -> bool {
                                           return units::abs(error) < tolerance;
                                       })
                                       .value_or(false);
        update_in_tolerance(curr_tolerance_active);
    }
};

template<typename T>
class VelocityTolerance : virtual Tolerance {
  private:
    std::optional<Divided<T, Time>> velocity_tolerance = std::nullopt;

  public:
    VelocityTolerance(Divided<T, Time> velocity_tolerance)
        : velocity_tolerance(velocity_tolerance) {}

    void setVelocityTolerance(Divided<T, Time> velocity_tolerance) {
        this->velocity_tolerance = velocity_tolerance;
    }

    void velocityToleranceUpdate(Divided<T, Time> velocity) {
        bool curr_tolerance_active =
          velocity_tolerance
            .transform([velocity](Divided<T, Time> tolerance) -> bool {
                return units::abs(velocity) < tolerance;
            })
            .value_or(false);

        update_in_tolerance(curr_tolerance_active);
    }
};

class HalfCircleTolerance : virtual Tolerance {
  private:
    std::optional<Length> radius_tolerance = std::nullopt;

  public:
    HalfCircleTolerance(Length radius_tolerance)
        : radius_tolerance(radius_tolerance) {}

    void setHalfcircleTolerance(Length radius_tolerance) {
        this->radius_tolerance = radius_tolerance;
    }

    void halfcircleToleranceUpdate(units::V2Position pose,
                                   units::V2Position target,
                                   Angle theta) {
        bool curr_tolerance_active =
          radius_tolerance
            .transform([pose, target, theta](Length tolerance) -> bool {
                return (pose.y - target.y) * -units::cos(theta) >=
                       units::sin(theta) * (pose.x - target.y) + tolerance;
            })
            .value_or(false);

        update_in_tolerance(curr_tolerance_active);
    }
};

// allows inheriting all the functions from the tolerances
template<typename... ToleranceTypes>
    requires(std::is_base_of_v<Tolerance, ToleranceTypes> && ...)
class Tolerances : virtual Tolerance,
                   public ToleranceTypes... {
  private:
    std::optional<Time> tolerance_timestamp = std::nullopt;
    std::optional<Time> duration = std::nullopt;

  public:
    Tolerances(Time duration, ToleranceTypes&&... bases)
        : duration(duration),
          ToleranceTypes(std::move(bases))... {}

    void setDuration(Time duration) {
        this->duration = duration;
    }

	bool withinTolerance(){
        return in_tolerance.value_or(false);
	}

    // assumes each tolerance check has been performed
    bool finished() {
        if (withinTolerance()) {
            // set timestamp if it doesn't have one
            if (!tolerance_timestamp.has_value()) {
                tolerance_timestamp = from_msec(pros::millis());
            }
            if (duration.transform([&](Time time) -> bool {
                    return from_msec(pros::millis()) - *tolerance_timestamp >
                           time;
                })) {
                tolerance_timestamp = std::nullopt;
                return true;
            }
        } else if (tolerance_timestamp.has_value()) {
            // reset tolerance value
            tolerance_timestamp = std::nullopt;
        }

        return false;
    }
	// if not called then withinTolerance will always remain on after becoming true once
	bool reset(){
        // reset in_tolerance
        in_tolerance = std::nullopt;
	}
};

template<typename LinearTolerances, typename AngularTolerances>
struct LinearAndAngularTolerances {
    LinearTolerances linear;
    AngularTolerances angular;
};

template<typename LinearTolerances,
         typename AngularTolerances,
         typename LargeLinearTolerances,
         typename LargeAngularTolerances>
struct DefaultTolerances {
    LinearTolerances linear;
    AngularTolerances angular;
    LargeLinearTolerances large_linear;
    LargeAngularTolerances large_angular;
};

// tolerance concepts
template<typename TolerancesType>
concept hasLinearTolerance =
  requires(TolerancesType tolerances) {
      tolerances.linear;
  };
template<typename TolerancesType>
concept hasLargeLinearTolerance =
  requires(TolerancesType tolerances) {
      tolerances.large_linear;
  };
template<typename TolerancesType>
concept hasAngularTolerance =
  requires(TolerancesType tolerances) {
      tolerances.angular;
  };
template<typename TolerancesType>
concept hasLargeAngularTolerance =
  requires(TolerancesType tolerances) {
      tolerances.large_angular;
  };

template<typename TolerancesType>
concept hasLinearErrorTolerance =
  requires(TolerancesType tolerances, Length error) {
      tolerances.linear.setErrorTolerance(error);
      tolerances.linear.errorToleranceUpdate(error);
  };

template<typename TolerancesType>
concept hasLargeLinearErrorTolerance =
  requires(TolerancesType tolerances, Length error) {
      tolerances.large_linear.setErrorTolerance(error);
      tolerances.large_linear.errorToleranceUpdate(error);
  };

template<typename TolerancesType>
concept hasAngularErrorTolerance =
  requires(TolerancesType tolerances, Angle error) {
      tolerances.angular.setErrorTolerance(error);
      tolerances.angular.errorToleranceUpdate(error);
  };

template<typename TolerancesType>
concept hasLargeAngularErrorTolerance =
  requires(TolerancesType tolerances, Angle error) {
      tolerances.large_angular.setErrorTolerance(error);
      tolerances.large_angular.errorToleranceUpdate(error);
  };

template<typename TolerancesType>
concept hasLinearVelocityTolerance =
  requires(TolerancesType tolerances, LinearVelocity velocity) {
      tolerances.linear.setVelocityTolerance(velocity);
      tolerances.linear.velocityToleranceUpdate(velocity);
  };
template<typename TolerancesType>
concept hasLargeLinearVelocityTolerance =
  requires(TolerancesType tolerances, LinearVelocity velocity) {
      tolerances.large_linear.setVelocityTolerance(velocity);
      tolerances.large_linear.velocityToleranceUpdate(velocity);
  };

template<typename TolerancesType>
concept hasAngularVelocityTolerance =
  requires(TolerancesType tolerances, AngularVelocity velocity) {
      tolerances.angular.setVelocityTolerance(velocity);
      tolerances.angular.velocityToleranceUpdate(velocity);
  };

template<typename TolerancesType>
concept hasLargeAngularVelocityTolerance =
  requires(TolerancesType tolerances, AngularVelocity velocity) {
      tolerances.large_angular.setVelocityTolerance(velocity);
      tolerances.large_angular.velocityToleranceUpdate(velocity);
  };

template<typename TolerancesType>
concept hasHalfcircleTolerance = requires(TolerancesType tolerances,
                                          Length tolerance,
                                          units::V2Position pose,
                                          units::V2Position target,
                                          Angle theta) {
    tolerances.linear.setHalfcircleTolerance(tolerance);
    tolerances.linear.halfcircleToleranceUpdate(pose, target, theta);
};

template<typename TolerancesType>
concept hasLargeHalfcircleTolerance = requires(TolerancesType tolerances,
                                          Length tolerance,
                                          units::V2Position pose,
                                          units::V2Position target,
                                          Angle theta) {
    tolerances.large_linear.setHalfcircleTolerance(tolerance);
    tolerances.large_linear.halfcircleToleranceUpdate(pose, target, theta);
};
} // namespace blazing
