#include "phidgets_hardware/phidgets_bldc_hardware.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace phidgets_hardware
{

hardware_interface::CallbackReturn
PhidgetsBldcHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != NUM_MOTORS) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsBldcHardware"),
                 "Expected %d joints, got %zu", NUM_MOTORS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto get_param = [&](const std::string & key, const std::string & def) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    return (it == info_.hardware_parameters.end()) ? def : it->second;
  };

  gear_ratio_ = std::stod(get_param("gear_ratio", "106.0"));
  commutations_per_motor_rev_ = std::stoi(get_param("commutations_per_motor_rev", "24"));
  acceleration_ = std::stod(get_param("acceleration", "5.0"));
  stall_velocity_ = std::stod(get_param("stall_velocity", "0.0"));
  command_limit_ = std::stod(get_param("command_limit", "1.0"));
  attachment_retry_period_sec_ = std::stod(get_param("attachment_retry_period_sec", "1.0"));

  rescale_factor_rot_ =
    1.0 / (gear_ratio_ * static_cast<double>(commutations_per_motor_rev_));

  for (int i = 0; i < NUM_MOTORS; i++) {
    cmd_[i] = 0.0;
    temperature_c_[i] = 0.0;
    last_attachment_attempt_sec_[i] = -attachment_retry_period_sec_;
  }

  telemetry_node_ = rclcpp::Node::make_shared("phidgets_motor_telemetry_bridge");
  telemetry_pub_ = telemetry_node_->create_publisher<std_msgs::msg::String>(
    "/rover/drive/motor_telemetry",
    10);

  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"),
              "Init complete for %d motors", NUM_MOTORS);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PhidgetsBldcHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (int i = 0; i < NUM_MOTORS; i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &pos_rad_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &vel_state_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PhidgetsBldcHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (int i = 0; i < NUM_MOTORS; i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &cmd_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
PhidgetsBldcHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"), "Activating...");

  for (int i = 0; i < NUM_MOTORS; i++) {
    close_phidget(i);
    close_temperature_sensor(i);
    attached_[i] = false;
    temperature_attached_[i] = false;
    try_attach_motor(i);
    if (attached_[i]) {
      try_attach_temperature_sensor(i);
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("PhidgetsBldcHardware"),
    "Activated. Motors that are powered later will be attached on retry.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PhidgetsBldcHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"), "Deactivating...");

  for (int i = 0; i < NUM_MOTORS; i++) {

    if (motors_[i]) {
      PhidgetBLDCMotor_setTargetVelocity(motors_[i], 0.0);
    }

    close_phidget(i);
    close_temperature_sensor(i);
    attached_[i] = false;
    temperature_attached_[i] = false;
  }

  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"), "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
PhidgetsBldcHardware::read(const rclcpp::Time & time, const rclcpp::Duration &)
{
  retry_unattached_devices(time);

  for (int i = 0; i < NUM_MOTORS; i++) {

    if (!attached_[i] || !motors_[i])
      continue;

    double vel_duty = 0.0;
    double pos_val = 0.0;

    auto rc_v = PhidgetBLDCMotor_getVelocity(motors_[i], &vel_duty);

    if (rc_v == EPHIDGET_OK) {
      vel_state_[i] = vel_duty * direction_sign_[i];
    } else {
      close_phidget(i);
      close_temperature_sensor(i);
      attached_[i] = false;
      temperature_attached_[i] = false;
      vel_state_[i] = 0.0;
      continue;
    }

    auto rc_p = PhidgetBLDCMotor_getPosition(motors_[i], &pos_val);

    if (rc_p == EPHIDGET_OK) {
      pos_rad_[i] = pos_val * 2.0 * M_PI * direction_sign_[i];
    } else {
      close_phidget(i);
      close_temperature_sensor(i);
      attached_[i] = false;
      temperature_attached_[i] = false;
      vel_state_[i] = 0.0;
      continue;
    }

    if (temperature_attached_[i] && temperature_sensors_[i]) {
      double temperature_val = 0.0;
      auto rc_t = PhidgetTemperatureSensor_getTemperature(temperature_sensors_[i], &temperature_val);
      if (rc_t == EPHIDGET_OK) {
        temperature_c_[i] = temperature_val;
      } else {
        close_temperature_sensor(i);
        temperature_attached_[i] = false;
      }
    }
  }

  publish_motor_telemetry(time);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PhidgetsBldcHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!all_motors_attached()) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (motors_[i]) {
        PhidgetBLDCMotor_setTargetVelocity(motors_[i], 0.0);
      }
    }
    return hardware_interface::return_type::OK;
  }

  for (int i = 0; i < NUM_MOTORS; i++) {

    if (!attached_[i] || !motors_[i])
      continue;

    const double duty = clamp(cmd_[i] * direction_sign_[i], -command_limit_, command_limit_);

    auto rc = PhidgetBLDCMotor_setTargetVelocity(motors_[i], duty);

    if (rc != EPHIDGET_OK) {
      RCLCPP_WARN(rclcpp::get_logger("PhidgetsBldcHardware"),
                  "Motor %d velocity command failed", i);
      close_phidget(i);
      close_temperature_sensor(i);
      attached_[i] = false;
      temperature_attached_[i] = false;
    }
  }

  return hardware_interface::return_type::OK;
}

void PhidgetsBldcHardware::close_phidget(int i)
{
  if (motors_[i]) {
    Phidget_close((PhidgetHandle)motors_[i]);
    PhidgetBLDCMotor_delete(&motors_[i]);
    motors_[i] = nullptr;
  }
}

void PhidgetsBldcHardware::close_temperature_sensor(int i)
{
  if (temperature_sensors_[i]) {
    Phidget_close((PhidgetHandle)temperature_sensors_[i]);
    PhidgetTemperatureSensor_delete(&temperature_sensors_[i]);
    temperature_sensors_[i] = nullptr;
  }
}

void PhidgetsBldcHardware::try_attach_motor(int i)
{
  close_phidget(i);
  attached_[i] = false;

  PhidgetReturnCode rc = PhidgetBLDCMotor_create(&motors_[i]);
  if (rc != EPHIDGET_OK || motors_[i] == nullptr) {
    RCLCPP_WARN(
      rclcpp::get_logger("PhidgetsBldcHardware"),
      "Motor %d create failed; will retry",
      i);
    close_phidget(i);
    return;
  }

  Phidget_setDeviceSerialNumber((PhidgetHandle)motors_[i], device_serial_[i]);
  Phidget_setHubPort((PhidgetHandle)motors_[i], hub_port_[i]);
  Phidget_setChannel((PhidgetHandle)motors_[i], channel_[i]);

  rc = Phidget_openWaitForAttachment((PhidgetHandle)motors_[i], 500);
  if (rc != EPHIDGET_OK) {
    close_phidget(i);
    return;
  }

  attached_[i] = true;
  PhidgetBLDCMotor_setRescaleFactor(motors_[i], rescale_factor_rot_);
  PhidgetBLDCMotor_setAcceleration(motors_[i], acceleration_);
  PhidgetBLDCMotor_setStallVelocity(motors_[i], stall_velocity_);

  rc = PhidgetBLDCMotor_setTargetVelocity(motors_[i], 0.0);
  if (rc != EPHIDGET_OK) {
    RCLCPP_WARN(rclcpp::get_logger("PhidgetsBldcHardware"),
                "Motor %d start velocity failed", i);
  }

  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"),
              "Motor %d attached successfully", i);
}

void PhidgetsBldcHardware::try_attach_temperature_sensor(int i)
{
  close_temperature_sensor(i);
  temperature_attached_[i] = false;

  PhidgetReturnCode rc = PhidgetTemperatureSensor_create(&temperature_sensors_[i]);
  if (rc != EPHIDGET_OK || temperature_sensors_[i] == nullptr) {
    return;
  }

  Phidget_setDeviceSerialNumber((PhidgetHandle)temperature_sensors_[i], device_serial_[i]);
  Phidget_setHubPort((PhidgetHandle)temperature_sensors_[i], hub_port_[i]);
  Phidget_setChannel((PhidgetHandle)temperature_sensors_[i], channel_[i]);

  rc = Phidget_openWaitForAttachment((PhidgetHandle)temperature_sensors_[i], 500);
  if (rc != EPHIDGET_OK) {
    close_temperature_sensor(i);
    return;
  }

  temperature_attached_[i] = true;
}

void PhidgetsBldcHardware::retry_unattached_devices(const rclcpp::Time & time)
{
  const double now_sec = time.seconds();
  for (int i = 0; i < NUM_MOTORS; i++) {
    if ((now_sec - last_attachment_attempt_sec_[i]) < attachment_retry_period_sec_) {
      continue;
    }
    last_attachment_attempt_sec_[i] = now_sec;

    if (!attached_[i]) {
      try_attach_motor(i);
    }
    if (attached_[i] && !temperature_attached_[i]) {
      try_attach_temperature_sensor(i);
    }
  }
}

bool PhidgetsBldcHardware::all_motors_attached() const
{
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!attached_[i] || !motors_[i]) {
      return false;
    }
  }
  return true;
}

void PhidgetsBldcHardware::publish_motor_telemetry(const rclcpp::Time & time)
{
  if (!telemetry_pub_) {
    return;
  }

  if (last_telemetry_publish_time_.nanoseconds() != 0 &&
      (time - last_telemetry_publish_time_).seconds() < telemetry_publish_period_sec_)
  {
    return;
  }
  last_telemetry_publish_time_ = time;

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4);
  stream << "{\"motors\":[";
  for (int i = 0; i < NUM_MOTORS; i++) {
    const bool motor_attached = attached_[i] && motors_[i] != nullptr;
    const bool temp_attached = temperature_attached_[i] && temperature_sensors_[i] != nullptr;
    const std::string health = motor_attached ? (temp_attached ? "healthy" : "warning") : "offline";
    const std::string fault = motor_attached ? (temp_attached ? "None" : "Temperature sensor unavailable") : "Motor not attached";

    if (i > 0) {
      stream << ",";
    }
    stream << "{"
           << "\"name\":\"" << info_.joints[i].name << "\","
           << "\"attached\":" << (motor_attached ? "true" : "false") << ","
           << "\"position\":" << pos_rad_[i] << ","
           << "\"velocity\":" << vel_state_[i] << ","
           << "\"temperature\":" << temperature_c_[i] << ","
           << "\"health\":\"" << health << "\","
           << "\"fault\":\"" << fault << "\","
           << "\"last_update\":\"" << std::to_string(time.seconds()) << "\""
           << "}";
  }
  stream << "]}";

  std_msgs::msg::String msg;
  msg.data = stream.str();
  telemetry_pub_->publish(msg);
}

}  // namespace phidgets_hardware

PLUGINLIB_EXPORT_CLASS(
  phidgets_hardware::PhidgetsBldcHardware,
  hardware_interface::SystemInterface
)
