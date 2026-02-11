#include "phidgets_hardware/phidgets_bldc_hardware.hpp"

#include <cmath>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace phidgets_hardware
{

hardware_interface::CallbackReturn
PhidgetsBldcHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsBldcHardware"),
                 "Expected exactly 1 joint, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto get_param = [&](const std::string & key, const std::string & def) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    return (it == info_.hardware_parameters.end()) ? def : it->second;
  };

  device_serial_ = std::stoi(get_param("device_serial", "0"));
  hub_port_      = std::stoi(get_param("hub_port", "0"));
  channel_       = std::stoi(get_param("channel", "0"));

  gear_ratio_ = std::stod(get_param("gear_ratio", "106.0"));
  commutations_per_motor_rev_ = std::stoi(get_param("commutations_per_motor_rev", "24"));
  acceleration_ = std::stod(get_param("acceleration", "5.0"));
  stall_velocity_ = std::stod(get_param("stall_velocity", "0.0"));
  command_limit_ = std::stod(get_param("command_limit", "1.0"));

  // Rotations per commutation at OUTPUT shaft
  rescale_factor_rot_ = 1.0 / (gear_ratio_ * static_cast<double>(commutations_per_motor_rev_));

  cmd_ = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"),
              "Init: serial=%d hub_port=%d channel=%d gear_ratio=%.3f commutations_per_motor_rev=%d command_limit=%.3f",
              device_serial_, hub_port_, channel_, gear_ratio_, commutations_per_motor_rev_, command_limit_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PhidgetsBldcHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &pos_rad_);
  state_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &vel_state_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PhidgetsBldcHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &cmd_);
  return command_interfaces;
}

hardware_interface::CallbackReturn
PhidgetsBldcHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"), "Activating...");

  close_phidget();
  attached_ = false;

  PhidgetReturnCode rc = PhidgetBLDCMotor_create(&motor_);
  if (rc != EPHIDGET_OK || motor_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsBldcHardware"),
                 "PhidgetBLDCMotor_create failed: %d", rc);
    return hardware_interface::CallbackReturn::ERROR;
  }

  rc = Phidget_setDeviceSerialNumber((PhidgetHandle)motor_, device_serial_);
  if (rc != EPHIDGET_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsBldcHardware"),
                 "Phidget_setDeviceSerialNumber failed: %d", rc);
    close_phidget();
    return hardware_interface::CallbackReturn::ERROR;
  }

  rc = Phidget_setHubPort((PhidgetHandle)motor_, hub_port_);
  if (rc != EPHIDGET_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsBldcHardware"),
                 "Phidget_setHubPort failed: %d", rc);
    close_phidget();
    return hardware_interface::CallbackReturn::ERROR;
  }

  rc = Phidget_setChannel((PhidgetHandle)motor_, channel_);
  if (rc != EPHIDGET_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsBldcHardware"),
                 "Phidget_setChannel failed: %d", rc);
    close_phidget();
    return hardware_interface::CallbackReturn::ERROR;
  }

  rc = Phidget_openWaitForAttachment((PhidgetHandle)motor_, 5000);
  if (rc != EPHIDGET_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsBldcHardware"),
                 "Phidget_openWaitForAttachment failed: %d", rc);
    close_phidget();
    return hardware_interface::CallbackReturn::ERROR;
  }

  attached_ = true;

  // Optional: rescale factor (only if your phidget22.h provides it)
  // If this line fails to compile on your system, delete this block entirely.
  rc = PhidgetBLDCMotor_setRescaleFactor(motor_, rescale_factor_rot_);
  if (rc != EPHIDGET_OK) {
    RCLCPP_WARN(rclcpp::get_logger("PhidgetsBldcHardware"),
                "PhidgetBLDCMotor_setRescaleFactor failed (continuing): %d", rc);
  }

  rc = PhidgetBLDCMotor_setAcceleration(motor_, acceleration_);
  if (rc != EPHIDGET_OK) {
    const char * err = nullptr;
    Phidget_getErrorDescription(rc, &err);
    RCLCPP_WARN(
      rclcpp::get_logger("PhidgetsBldcHardware"),
      "Failed to set acceleration %.3f (rc=%d, %s)",
      acceleration_, rc, err ? err : "unknown"
    );
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("PhidgetsBldcHardware"),
      "BLDC acceleration set to %.3f duty/s",
      acceleration_
    );
  }

  rc = PhidgetBLDCMotor_setStallVelocity(motor_, stall_velocity_);

  // Start stopped using the correct API function you referenced
  rc = PhidgetBLDCMotor_setTargetVelocity(motor_, 0.0);
  if (rc != EPHIDGET_OK) {
    RCLCPP_WARN(rclcpp::get_logger("PhidgetsBldcHardware"),
                "PhidgetBLDCMotor_setTargetVelocity(0.0) failed: %d", rc);
  }

  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"), "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PhidgetsBldcHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"), "Deactivating...");

  if (motor_) {
    (void)PhidgetBLDCMotor_setTargetVelocity(motor_, 0.0);
  }

  close_phidget();
  attached_ = false;

  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"), "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
PhidgetsBldcHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!attached_ || !motor_) {
    return hardware_interface::return_type::OK;
  }

  // Velocity unit in BLDCMotor API is duty cycle
  double vel_duty = 0.0;
  double pos_val = 0.0;

  auto rc_v = PhidgetBLDCMotor_getVelocity(motor_, &vel_duty);
  if (rc_v == EPHIDGET_OK) {
    vel_state_ = vel_duty;
  }

  // Position: if rescaleFactor is set, pos_val is in rotations (as we intended).
  // If rescaleFactor is not supported, pos_val will be in commutations.
  auto rc_p = PhidgetBLDCMotor_getPosition(motor_, &pos_val);
  if (rc_p == EPHIDGET_OK) {
    // Assume pos_val is rotations (if rescale worked); convert to radians.
    // If you removed rescale support, this will not be meaningful until you convert commutations->rot.
    pos_rad_ = pos_val * 2.0 * M_PI;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PhidgetsBldcHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!attached_ || !motor_) {
    return hardware_interface::return_type::OK;
  }

  // Treat command as duty cycle; clamp
  const double duty = clamp(cmd_, -command_limit_, command_limit_);

  const auto rc = PhidgetBLDCMotor_setTargetVelocity(motor_, duty);

  
  if (rc != EPHIDGET_OK) {
    RCLCPP_WARN(rclcpp::get_logger("PhidgetsBldcHardware"),
                "PhidgetBLDCMotor_setTargetVelocity(%f) failed: %d", duty, rc);
  }

  return hardware_interface::return_type::OK;
}

void PhidgetsBldcHardware::close_phidget()
{
  if (motor_) {
    (void)Phidget_close((PhidgetHandle)motor_);
    (void)PhidgetBLDCMotor_delete(&motor_);
    motor_ = nullptr;
  }
}

}  // namespace phidgets_hardware

PLUGINLIB_EXPORT_CLASS(phidgets_hardware::PhidgetsBldcHardware, hardware_interface::ActuatorInterface)
