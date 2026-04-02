#pragma once

#include <string>
#include <vector>
#include <cstdint>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"

// Phidgets C API (Phidget22)
#include <phidget22.h>

namespace phidgets_hardware
{

class PhidgetsBldcHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PhidgetsBldcHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:

  static const int NUM_MOTORS = 4;

  int32_t device_serial_[NUM_MOTORS] = {765818,765818,765818,765818};
  int hub_port_[NUM_MOTORS] = {3,5,2,0};
  int channel_[NUM_MOTORS] = {0,0,0,0};
  double direction_sign_[NUM_MOTORS] = {-1.0, -1.0, 1.0, 1.0};

  double command_limit_{1.0};
  double attachment_retry_period_sec_{1.0};

  double acceleration_{5.0};
  double stall_velocity_{0.0};
  double gear_ratio_{106.0};
  int commutations_per_motor_rev_{24};

  double rescale_factor_rot_{1.0};

  PhidgetBLDCMotorHandle motors_[NUM_MOTORS] = {nullptr};
  PhidgetTemperatureSensorHandle temperature_sensors_[NUM_MOTORS] = {nullptr};

  bool attached_[NUM_MOTORS] = {false};
  bool temperature_attached_[NUM_MOTORS] = {false};
  double last_attachment_attempt_sec_[NUM_MOTORS] = {0};

  double pos_rad_[NUM_MOTORS] = {0};
  double vel_state_[NUM_MOTORS] = {0};
  double temperature_c_[NUM_MOTORS] = {0};
  double cmd_[NUM_MOTORS] = {0};

  rclcpp::Node::SharedPtr telemetry_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
  rclcpp::Time last_telemetry_publish_time_{0, 0, RCL_ROS_TIME};
  double telemetry_publish_period_sec_{0.2};

  static double clamp(double x, double lo, double hi)
  {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }

  void close_phidget(int i);
  void close_temperature_sensor(int i);
  void try_attach_motor(int i);
  void try_attach_temperature_sensor(int i);
  void retry_unattached_devices(const rclcpp::Time & time);
  bool all_motors_attached() const;
  void publish_motor_telemetry(const rclcpp::Time & time);
};

}  // namespace phidgets_hardware
