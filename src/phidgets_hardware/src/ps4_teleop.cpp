#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <nlohmann/json.hpp>
#include <cmath>
#include <string>
#include <functional>
#include <chrono>
#include <algorithm>

class Ps4Teleop : public rclcpp::Node
{
public:
  Ps4Teleop() : Node("ps4_teleop")
  {
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
    this->declare_parameter<int>("turn_on_button", 1);
    this->declare_parameter<int>("kill_button", 2);
    this->declare_parameter<int>("deadman_button", 7);
    this->declare_parameter<int>("turbo_button", 6);
    this->declare_parameter<int>("turbo_axis", -1);
    this->declare_parameter<double>("turbo_threshold", -0.5);

    // Headlight controls
    this->declare_parameter<int>("left_headlight_button", 4);
    this->declare_parameter<int>("right_headlight_button", 5);
    this->declare_parameter<int>("headlight_brightness_axis", 7);
    this->declare_parameter<double>("headlight_axis_threshold", 0.5);
    this->declare_parameter<int>("headlight_step", 1);
    this->declare_parameter<int>("headlight_update_period_ms", 50);


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

    turn_on_button_ = this->get_parameter("turn_on_button").as_int();
    kill_button_ = this->get_parameter("kill_button").as_int();
    deadman_button_ = this->get_parameter("deadman_button").as_int();

    turbo_button_ = this->get_parameter("turbo_button").as_int();
    turbo_axis_ = this->get_parameter("turbo_axis").as_int();
    turbo_threshold_ = this->get_parameter("turbo_threshold").as_double();

    //Headlights parameters
    left_headlight_button_ =
      this->get_parameter("left_headlight_button").as_int();

    right_headlight_button_ =
      this->get_parameter("right_headlight_button").as_int();

    headlight_brightness_axis_ =
      this->get_parameter("headlight_brightness_axis").as_int();

    headlight_axis_threshold_ =
      this->get_parameter("headlight_axis_threshold").as_double();

    headlight_step_ =
      this->get_parameter("headlight_step").as_int();

    headlight_update_period_ms_ =
      this->get_parameter("headlight_update_period_ms").as_int();


    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      10,
      std::bind(&Ps4Teleop::joy_callback, this, std::placeholders::_1));


    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diff_drive_controller/cmd_vel_unstamped",
      10);

    headlight_pub_ =
      this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/drive/headlight_command",
        10);

    headlight_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(headlight_update_period_ms_),
      std::bind(&Ps4Teleop::headlight_timer_callback, this));

    command_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/rover/relay_board/command",
      10);

    publish_headlight_command();


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
    double magnitude = std::fabs(value);

    if (magnitude <= deadzone)
      return 0.0;

    double normalized = (magnitude - deadzone) / (1.0 - deadzone);
    double shaped = std::pow(normalized, expo);

    return std::copysign(shaped, value);
  }


  bool is_turbo_enabled(
    const sensor_msgs::msg::Joy::SharedPtr msg) const
  {
    bool button_active = false;

    if (turbo_button_ >= 0 &&
        msg->buttons.size() > static_cast<size_t>(turbo_button_))
    {
      button_active = msg->buttons[turbo_button_] == 1;
    }


    bool axis_active = false;

    if (turbo_axis_ >= 0 &&
        msg->axes.size() > static_cast<size_t>(turbo_axis_))
    {
      double axis_value = msg->axes[turbo_axis_];

      if (turbo_threshold_ < 0)
        axis_active = axis_value <= turbo_threshold_;
      else
        axis_active = axis_value >= turbo_threshold_;
    }


    return button_active || axis_active;
  }

  bool button_pressed(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    int button_index) const
  {
    return button_index >= 0 &&
           msg->buttons.size() > static_cast<size_t>(button_index) &&
           msg->buttons[button_index] == 1;
  }

  void publish_headlight_command()
  {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data.resize(2);

    uint8_t enable_flags = 0;

    if (left_headlight_enabled_) {
      enable_flags |= 0x01;
    }

    if (right_headlight_enabled_) {
      enable_flags |= 0x02;
    }

    msg.data[0] = enable_flags;
    msg.data[1] = static_cast<uint8_t>(brightness_percent_);

    headlight_pub_->publish(msg);
  }

  void update_headlight_controls(
    const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const bool left_pressed =
      button_pressed(msg, left_headlight_button_);

    const bool right_pressed =
      button_pressed(msg, right_headlight_button_);

    bool state_changed = false;

    // Toggle only when the button changes from released to pressed.
    if (left_pressed && !previous_left_button_) {
      left_headlight_enabled_ = !left_headlight_enabled_;
      state_changed = true;
    }

    if (right_pressed && !previous_right_button_) {
      right_headlight_enabled_ = !right_headlight_enabled_;
      state_changed = true;
    }

    previous_left_button_ = left_pressed;
    previous_right_button_ = right_pressed;

    brightness_direction_ = 0;

    if (
      headlight_brightness_axis_ >= 0 &&
      msg->axes.size() >
        static_cast<size_t>(headlight_brightness_axis_))
    {
      const double dpad_value =
        msg->axes[headlight_brightness_axis_];

      if (dpad_value > headlight_axis_threshold_) {
        brightness_direction_ = 1;
      } else if (dpad_value < -headlight_axis_threshold_) {
        brightness_direction_ = -1;
      }
    }

    if (state_changed) {
      publish_headlight_command();

      RCLCPP_INFO(
        this->get_logger(),
        "Headlights: left=%s, right=%s, brightness=%d%%",
        left_headlight_enabled_ ? "ON" : "OFF",
        right_headlight_enabled_ ? "ON" : "OFF",
        brightness_percent_);
    }
  }


  void headlight_timer_callback()
  {
    if (brightness_direction_ != 0) {
      brightness_percent_ = std::clamp(
        brightness_percent_ +
          brightness_direction_ * headlight_step_,
        0,
        100);
    }

    // Publish continuously as a heartbeat for science_serial_bridge.
    // The bridge turns the headlights off if this topic becomes stale.
    publish_headlight_command();
  }

  void send_relay_command(const std::string &type)
  {
    nlohmann::json cmd;
    cmd["type"] = type;

    std_msgs::msg::String msg;
    msg.data = cmd.dump();

    command_pub_->publish(msg);

    RCLCPP_WARN(
      this->get_logger(),
      "Relay command sent: %s",
      type.c_str());
  }


  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {

    bool turn_on_pressed = false;
    bool kill_pressed = false;


    if (turn_on_button_ >= 0 &&
        msg->buttons.size() > static_cast<size_t>(turn_on_button_))
    {
      turn_on_pressed =
        msg->buttons[turn_on_button_] == 1;
    }


    if (kill_button_ >= 0 &&
        msg->buttons.size() > static_cast<size_t>(kill_button_))
    {
      kill_pressed =
        msg->buttons[kill_button_] == 1;
    }


    // ONE-TIME PRESS
    if (turn_on_pressed && !last_turn_on_state_)
    {
      send_relay_command("turn_on_all");
    }


    if (kill_pressed && !last_kill_state_)
    {
      send_relay_command("software_kill");
    }


    last_turn_on_state_ = turn_on_pressed;
    last_kill_state_ = kill_pressed;



    bool deadman_pressed = false;

    if (deadman_button_ >= 0 &&
        msg->buttons.size() > static_cast<size_t>(deadman_button_))
    {
      deadman_pressed =
        msg->buttons[deadman_button_] == 1;
    }


    geometry_msgs::msg::Twist cmd;


    if (!deadman_pressed)
    {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      return;
    }

    double forward_input = 0.0;
    double turn_input = 0.0;


    if (msg->axes.size() > static_cast<size_t>(axis_linear_x_))
    {
      forward_input =
        apply_deadzone(
          msg->axes[axis_linear_x_],
          deadzone_,
          expo_);
    }


    if (msg->axes.size() > static_cast<size_t>(axis_angular_z_))
    {
      turn_input =
        apply_deadzone(
          msg->axes[axis_angular_z_],
          deadzone_,
          expo_);
    }


    bool boost_enabled = is_turbo_enabled(msg);


    double speed_scale =
      boost_enabled ? speed_scale_turbo_ : speed_scale_normal_;

    double turn_scale =
      boost_enabled ? turn_scale_turbo_ : turn_scale_normal_;


    cmd.linear.x =
      forward_input * max_linear_ * speed_scale;

    cmd.angular.z =
      turn_input * max_angular_ * turn_scale;


    cmd_pub_->publish(cmd);
  }



  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr headlight_pub_;
  rclcpp::TimerBase::SharedPtr headlight_timer_;


  int axis_linear_x_;
  int axis_angular_z_;

  int turn_on_button_;
  int kill_button_;
  int deadman_button_;

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

  // Headlight parameters
  int left_headlight_button_;
  int right_headlight_button_;
  int headlight_brightness_axis_;
  double headlight_axis_threshold_;
  int headlight_step_;
  int headlight_update_period_ms_;

  // Headlight state
  bool left_headlight_enabled_{false};
  bool right_headlight_enabled_{false};
  bool previous_left_button_{false};
  bool previous_right_button_{false};
  int brightness_percent_{0};
  int brightness_direction_{0};

  // Button edge detection
  bool last_turn_on_state_{false};
  bool last_kill_state_{false};
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ps4Teleop>());
  rclcpp::shutdown();
  return 0;
}