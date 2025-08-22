#include "pros/rtos.hpp"
#include "units/units.hpp"
#include <optional>

template<typename T>
class Tolerances {
  private:
    std::optional<Time> tolerance_timestamp = std::nullopt;
    std::optional<Time> duration = std::nullopt;
    std::optional<T> error_tolerance = std::nullopt;
    std::optional<Divided<T, Time>> velocity_tolerance = std::nullopt;

  public:
    Tolerances(Time duration,
               T error_tolerance,
               Divided<T, Time> velocity_tolerance)
        : duration(duration),
          error_tolerance(error_tolerance),
          velocity_tolerance(velocity_tolerance) {}

    void setErrorTolerance(T error_tolerance) {
        this->error_tolerance = error_tolerance;
    }

    void setVelocityTolerance(Divided<T, Time> velocity_tolerance) {
        this->velocity_tolerance = velocity_tolerance;
    }

    void setDuration(Time duration) {
        this->duration = duration;
    }

    bool check(T error, Divided<T, Time> velocity) {
        bool in_tolerance =
          error_tolerance.transform([&](T tolerance) -> bool {
              return units::abs(error) < tolerance;
          }) &&
          velocity_tolerance.transform([&](Divided<T, Time> tolerance) -> bool {
              return units::abs(velocity) < tolerance;
          });

        if (in_tolerance) {
            if (tolerance_timestamp.has_value()) {
                tolerance_timestamp = from_msec(pros::millis());
            }
            if (duration.transform([&](Time time) -> bool {
                    return tolerance_timestamp > time;
                })) {
                tolerance_timestamp = std::nullopt;
                return true;
            }
        } else if (tolerance_timestamp.has_value()) {
            tolerance_timestamp = std::nullopt;
        }
    }
};
