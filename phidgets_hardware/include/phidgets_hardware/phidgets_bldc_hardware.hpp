#pragma once

#include <string>
#include <vector>
#include <cstdint>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Phidgets C API (Phidget22)
#include <phidget22.h>

namespace phidgets_hardware
{

class PhidgetsBldcHardware : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PhidgetsBldcHardware)  // <-- no trailing semicolon

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  int32_t device_serial_{766944};  // VINT hub serial (for VINT devices)
  int hub_port_{0};           // VINT hub port
  int channel_{0};            // channel number
  double command_limit_{1.0}; // clamp for duty cycle command

  // Optional rescale support
  double acceleration_{5.0};
  double stall_velocity_{0.0};
  double gear_ratio_{106.0};
  int commutations_per_motor_rev_{24};
  double rescale_factor_rot_{1.0};  // rotations per commutation (output shaft)

  // Phidgets handle
  PhidgetBLDCMotorHandle motor_{nullptr};
  bool attached_{false};


  // ROS2 control values
  double pos_rad_{0.0};
  double vel_state_{0.0};  // we will store the Phidgets-reported velocity (duty cycle)
  double cmd_{0.0};        // commanded "velocity" (interpreted as duty cycle)

  static double clamp(double x, double lo, double hi)
  {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }

  void close_phidget();
};

}  // namespace phidgets_hardware
