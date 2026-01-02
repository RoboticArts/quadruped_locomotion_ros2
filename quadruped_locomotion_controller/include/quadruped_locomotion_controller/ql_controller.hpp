#pragma once

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <quadruped_locomotion/types.hpp>

using ControllerInterface = controller_interface::ControllerInterface;
using CallbackReturn = controller_interface::CallbackReturn;
using InterfaceConfiguration = controller_interface::InterfaceConfiguration;
using ReturnType = controller_interface::return_type;
using LoanedCommandInterface = hardware_interface::LoanedCommandInterface;
using LoanedStateInterface = hardware_interface::LoanedStateInterface;

using JointEnum = quadruped_locomotion::types::JointEnum;
using LegEnum = quadruped_locomotion::types::LegEnum;

namespace ql_controller
{

class QLController : public ControllerInterface
{
public:

  static constexpr const char* BASE_CMD_RESOURCE = "base_cmd";
  static constexpr const char* IF_LINEAR_X = "linear_x";
  static constexpr const char* IF_ANGULAR_Z = "angular_z";

  CallbackReturn on_init() override;
  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  InterfaceConfiguration command_interface_configuration() const override;
  InterfaceConfiguration state_interface_configuration() const override;

  ReturnType update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  struct LoanedJointState {
    std::optional<std::reference_wrapper<const LoanedStateInterface>> state;
  };

  struct LoanedAxisCommand {
    std::optional<std::reference_wrapper<LoanedCommandInterface>> axis;
  };

  struct IConfig {
    std::string name;
    std::string resource;  
    std::string interface;
  };

  struct Config {
    quadruped_locomotion::types::QuadrupedJointT<IConfig> joint_state_info;
    quadruped_locomotion::types::QuadrupedAxisT<IConfig> twist_command_info;
  };

  Config config_;
  quadruped_locomotion::types::QuadrupedJointT<LoanedJointState> quadruped_state_;
  quadruped_locomotion::types::QuadrupedAxisT<LoanedAxisCommand> quadruped_twist_command_;
  
  double linear_x_{0.0};
  double angular_z_{0.0};  

  rclcpp::Logger logger_{rclcpp::get_logger("QLController")};

};

}  // namespace ql_controller