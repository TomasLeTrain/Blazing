#pragma once

#include "blazing/controllers/controllers.hpp"

namespace blazing {

class VoltageClampController {
    std::optional<Voltage> min_voltage = std::nullopt;
    std::optional<Voltage> max_voltage = std::nullopt;

  public:
    VoltageClampController(std::optional<Voltage> max_voltage = std::nullopt,
                           std::optional<Voltage> min_voltage = std::nullopt)
        : max_voltage(max_voltage),
          min_voltage(min_voltage) {}

    void setMin(Voltage min) {
        this->min_voltage = min;
    }

    void setMax(Voltage max) {
        this->max_voltage = max;
    }

    Voltage applyMin(Voltage output) {
        if (!min_voltage) {
            return output;
        } else {
            return units::sgn(output) *
                   units::max(units::abs(output), *min_voltage);
        }
    }

    Voltage applyMax(Voltage output) {
        if (!max_voltage) {
            return output;
        } else {
            return units::sgn(output) *
                   units::min(units::abs(output), *max_voltage);
        }
    }

    // output should be signed, indicating its direction of travel
    Voltage apply(Voltage output) {

        Voltage adjusted_output = output;
        adjusted_output = applyMin(adjusted_output);
        adjusted_output = applyMax(adjusted_output);

        return adjusted_output;
    }
};

class LinearVoltageClampController : virtual ControllerBase {
  public:
    VoltageClampController linear_voltage_clamp_controller;

    LinearVoltageClampController(
      VoltageClampController linear_voltage_clamp_controller)
        : linear_voltage_clamp_controller(linear_voltage_clamp_controller) {}

    LinearVoltageClampController(
      std::optional<Voltage> max_voltage = std::nullopt,
      std::optional<Voltage> min_voltage = std::nullopt)
        : linear_voltage_clamp_controller(max_voltage, min_voltage) {}
};

class AngularVoltageClampController : virtual ControllerBase {
  public:
    VoltageClampController angular_voltage_clamp_controller;

    AngularVoltageClampController(
      VoltageClampController angular_voltage_clamp_controller)
        : angular_voltage_clamp_controller(angular_voltage_clamp_controller) {}

    AngularVoltageClampController(
      std::optional<Voltage> max_voltage = std::nullopt,
      std::optional<Voltage> min_voltage = std::nullopt)
        : angular_voltage_clamp_controller(max_voltage, min_voltage) {}
};

// voltage clamp constraints
template<typename Controller>
concept hasLinearVoltageClampController = requires(Controller controller) {
    controller.linear_voltage_clamp_controller;
};
template<typename Controller>
concept hasAngularVoltageClampController = requires(Controller controller) {
    controller.angular_voltage_clamp_controller;
};

} // namespace blazing
