#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <cmath>
#include <string>

class Ps4Teleop : public rclcpp::Node
{
public:
  Ps4Teleop() : Node("ps4_teleop")
  {
    // Declare parameters with defaults
    this->declare_parameter<int>("axis_linear_x", 1);
    this->declare_parameter<int>("axis_angular_z", 2);
    this->declare_parameter<double>("deadzone", 0.18);
    this->declare_parameter<double>("expo", 1.8);
    this->declare_parameter<double>("speed_scale_normal", 0.5);
    this->declare_parameter<double>("speed_scale_turbo", 1.0);
    this->declare_parameter<double>("turn_scale_normal", 0.5);
    this->declare_parameter<double>("turn_scale_turbo", 1.0);
    this->declare_parameter<double>("max_linear", 0.0925);
    this->declare_parameter<double>("max_angular", 0.474);
    this->declare_parameter<int>("turbo_button", 6);
    this->declare_parameter<int>("turbo_axis", -1);        // NEW: -1 means disabled
    this->declare_parameter<double>("turbo_threshold", -0.5); // NEW

    // Get parameters
    axis_linear_x_ = this->get_parameter("axis_linear_x").as_int();
    axis_angular_z_ = this->get_parameter("axis_angular_z").as_int();
    deadzone_ = this->get_parameter("deadzone").as_double();
    expo_ = this->get_parameter("expo").as_double();
    speed_scale_normal_ = this->get_parameter("speed_scale_normal").as_double();
    speed_scale_turbo_ = this->get_parameter("speed_scale_turbo").as_double();
    turn_scale_normal_ = this->get_parameter("turn_scale_normal").as_double();
    turn_scale_turbo_ = this->get_parameter("turn_scale_turbo").as_double();
    max_linear_ = this->get_parameter("max_linear").as_double();
    max_angular_ = this->get_parameter("max_angular").as_double();
    turbo_button_ = this->get_parameter("turbo_button").as_int();
    turbo_axis_ = this->get_parameter("turbo_axis").as_int();           // NEW
    turbo_threshold_ = this->get_parameter("turbo_threshold").as_double(); // NEW

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy",
        10,
        std::bind(&Ps4Teleop::joy_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive_controller/cmd_vel_unstamped",
        10);

    RCLCPP_INFO(this->get_logger(), "PS4 teleop node started");
    RCLCPP_INFO(this->get_logger(), "  axis_linear_x: %d", axis_linear_x_);
    RCLCPP_INFO(this->get_logger(), "  axis_angular_z: %d", axis_angular_z_);
    RCLCPP_INFO(this->get_logger(), "  deadzone: %.3f", deadzone_);
    RCLCPP_INFO(this->get_logger(), "  max_linear: %.4f", max_linear_);
    RCLCPP_INFO(this->get_logger(), "  max_angular: %.4f", max_angular_);
    RCLCPP_INFO(this->get_logger(), "  turbo_button: %d", turbo_button_);
    RCLCPP_INFO(this->get_logger(), "  turbo_axis: %d", turbo_axis_);           // NEW
    RCLCPP_INFO(this->get_logger(), "  turbo_threshold: %.2f", turbo_threshold_); // NEW
  }

private:
  double apply_deadzone(double value, double deadzone, double expo) const
  {
    const double magnitude = std::fabs(value);
    if (magnitude <= deadzone) {
      return 0.0;
    }

    const double normalized = (magnitude - deadzone) / (1.0 - deadzone);
    const double shaped = std::pow(normalized, expo);
    return std::copysign(shaped, value);
  }

  // NEW: Check if turbo is enabled from either button or axis
  bool is_turbo_enabled(const sensor_msgs::msg::Joy::SharedPtr msg) const
  {
    // Check button first
    bool button_active = false;
    if (turbo_button_ >= 0 && msg->buttons.size() > static_cast<size_t>(turbo_button_)) {
      button_active = (msg->buttons[turbo_button_] == 1);
    }

    // Check axis second
    bool axis_active = false;
    if (turbo_axis_ >= 0 && msg->axes.size() > static_cast<size_t>(turbo_axis_)) {
      double axis_value = msg->axes[turbo_axis_];
      
      // For axis that goes from 1 (released) to -1 (pressed)
      if (turbo_threshold_ < 0) {
        axis_active = (axis_value <= turbo_threshold_);
      } 
      // For axis that goes from 0 to 1 (like trigger)
      else {
        axis_active = (axis_value >= turbo_threshold_);
      }
    }

    // Turbo is enabled if EITHER button or axis is active
    return button_active || axis_active;
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    geometry_msgs::msg::Twist cmd;

    // Get joystick inputs
    const double forward_input = apply_deadzone(msg->axes[axis_linear_x_], deadzone_, expo_);
    const double turn_input = apply_deadzone(msg->axes[axis_angular_z_], deadzone_, expo_);

    // Check turbo (button OR axis)
    const bool boost_enabled = is_turbo_enabled(msg);  // CHANGED: now uses both
    
    // Apply speed scaling
    const double speed_scale = boost_enabled ? speed_scale_turbo_ : speed_scale_normal_;
    const double turn_scale = boost_enabled ? turn_scale_turbo_ : turn_scale_normal_;

    // Scale maximum velocities
    const double max_linear = max_linear_ * speed_scale;
    const double max_angular = max_angular_ * turn_scale;

    // Publish commands
    cmd.linear.x = forward_input * max_linear;
    cmd.angular.z = turn_input * max_angular;

    cmd_pub_->publish(cmd);

    // Debug output (prints every 10 messages to avoid spam)
    static int counter = 0;
    if (++counter % 10 == 0) {
      // NEW: Show both button and axis status for debugging
      std::string turbo_source = "OFF";
      if (boost_enabled) {
        if (turbo_button_ >= 0 && msg->buttons.size() > static_cast<size_t>(turbo_button_) && msg->buttons[turbo_button_] == 1) {
          turbo_source = "BUTTON";
        } else if (turbo_axis_ >= 0 && msg->axes.size() > static_cast<size_t>(turbo_axis_)) {
          turbo_source = "AXIS (val=" + std::to_string(msg->axes[turbo_axis_]) + ")";
        }
      }
      
      RCLCPP_INFO(this->get_logger(), 
                  "Forward: %.3f, Turn: %.3f, Turbo: %s, Linear: %.4f, Angular: %.4f",
                  forward_input, 
                  turn_input, 
                  turbo_source.c_str(),
                  cmd.linear.x,
                  cmd.angular.z);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  
  // Parameters
  int axis_linear_x_;
  int axis_angular_z_;
  double deadzone_;
  double expo_;
  double speed_scale_normal_;
  double speed_scale_turbo_;
  double turn_scale_normal_;
  double turn_scale_turbo_;
  double max_linear_;
  double max_angular_;
  int turbo_button_;
  int turbo_axis_;           
  double turbo_threshold_;   
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ps4Teleop>());
  rclcpp::shutdown();
  return 0;
}