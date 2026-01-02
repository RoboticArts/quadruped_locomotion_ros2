#pragma once

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <quadruped_locomotion/types.hpp>
#include <quadruped_locomotion/api/client.hpp>


namespace ql_system{

using CallbackReturn = hardware_interface::CallbackReturn;
using HardwareComponentInterfaceParams = hardware_interface::HardwareComponentInterfaceParams;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;
using ReturnType = hardware_interface::return_type;

using QuadrupedTwist = quadruped_locomotion::types::QuadrupedTwist;
using QuadrupedState = quadruped_locomotion::types::QuadrupedState;

using JointEnum = quadruped_locomotion::types::JointEnum;
using LegEnum = quadruped_locomotion::types::LegEnum;


class QLSystem : public hardware_interface::SystemInterface {

  public:

		static constexpr const char* BASE_CMD_RESOURCE = "base_cmd";
		static constexpr const char* IF_LINEAR_X = "linear_x";
		static constexpr const char* IF_ANGULAR_Z = "angular_z";

		CallbackReturn on_init(
      const HardwareComponentInterfaceParams& params) override;
		CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

		std::vector<StateInterface> export_state_interfaces() override;
		std::vector<CommandInterface> export_command_interfaces() override;

		CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
		CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
		
    ReturnType read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
		ReturnType write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  private:

    struct IConfig {
      std::string name;
      std::string resource;  
      std::string interface;
    };

    struct Config {
      std::string port;
      quadruped_locomotion::types::QuadrupedJointT<IConfig> joint_state_info;
      quadruped_locomotion::types::QuadrupedAxisT<IConfig> twist_command_info;
    };

    Config config_;
    QuadrupedState quadruped_state_;
		QuadrupedTwist quadruped_twist_command_;

    quadruped_locomotion::api::Client quadruped_locomotion_client_;

    rclcpp::Logger logger_{rclcpp::get_logger("QLSystem")};

};

}
