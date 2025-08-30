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

    // assumes each tolerance check has been performed
    bool check() {
        if (in_tolerance.value_or(false)) {
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

        // reset in_tolerance
        in_tolerance = std::nullopt;

        return false;
    }
};

// tolerance concepts
template<typename TolerancesType, typename T>
concept hasErrorTolerance = requires(TolerancesType tolerances, T error) {
    tolerances.setErrorTolerance(error);
    tolerances.errorToleranceUpdate(error);
};

template<typename TolerancesType, typename T>
concept hasVelocityTolerance =
  requires(TolerancesType tolerances, Divided<T, Time> velocity) {
      tolerances.setVelocityTolerance(velocity);
      tolerances.velocityToleranceUpdate(velocity);
  };

template<typename TolerancesType, typename T>
concept hasHalfcircleTolerance = requires(TolerancesType tolerances,
                                          Length tolerance,
                                          units::V2Position pose,
                                          units::V2Position target,
                                          Angle theta) {
    tolerances.setHalfcircleTolerance(tolerance);
    tolerances.halfcircleToleranceUpdate(pose, target, theta);
};
} // namespace blazing
