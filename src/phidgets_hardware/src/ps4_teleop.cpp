#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp>
#include <cmath>
#include <string>
#include <functional>

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


    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      10,
      std::bind(&Ps4Teleop::joy_callback, this, std::placeholders::_1));


    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diff_drive_controller/cmd_vel_unstamped",
      10);


    command_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/rover/relay_board/command",
      10);


    RCLCPP_INFO(this->get_logger(), "PS4 teleop node started");
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