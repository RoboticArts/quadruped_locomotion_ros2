#include "quadruped_locomotion_system/ql_system.hpp"

namespace ql_system
{

using namespace quadruped_locomotion::types;

CallbackReturn QLSystem::on_init(const HardwareComponentInterfaceParams& params)
{

  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "On init...");

  auto get_param = [&](const std::string& name)-> std::string {
      auto it = params.hardware_info.hardware_parameters.find(name);
      if (it == params.hardware_info.hardware_parameters.end()){
        RCLCPP_ERROR(logger_, "Missing required hardware parameter: %s", name.c_str());  
        throw std::runtime_error("Missing hardware paramter");
      };
      return it->second;
  };

  // Left front
  config_.joint_state_info[LF][HAA].name = get_param("lf_haa_joint_name");
  config_.joint_state_info[LF][HFE].name = get_param("lf_hfe_joint_name");
  config_.joint_state_info[LF][KFE].name = get_param("lf_kfe_joint_name");

  // Left hind
  config_.joint_state_info[LH][HAA].name = get_param("lh_haa_joint_name");
  config_.joint_state_info[LH][HFE].name = get_param("lh_hfe_joint_name");
  config_.joint_state_info[LH][KFE].name = get_param("lh_kfe_joint_name");

  // Right front
  config_.joint_state_info[RF][HAA].name = get_param("rf_haa_joint_name");
  config_.joint_state_info[RF][HFE].name = get_param("rf_hfe_joint_name");
  config_.joint_state_info[RF][KFE].name = get_param("rf_kfe_joint_name");

  // Right hind
  config_.joint_state_info[RH][HAA].name = get_param("rh_haa_joint_name");
  config_.joint_state_info[RH][HFE].name = get_param("rh_hfe_joint_name");
  config_.joint_state_info[RH][KFE].name = get_param("rh_kfe_joint_name");

  for (LegEnum leg: {LF, LH, RF, RH}){

    for (JointEnum joint: {HAA, HFE, KFE})
    {
      config_.joint_state_info[leg][joint].resource = config_.joint_state_info[leg][joint].name;
      config_.joint_state_info[leg][joint].interface = hardware_interface::HW_IF_POSITION;
    }
  }

  // Matches with gpio/command_interface of quadruped_control.urdf.xacro
  // Matches with ql_controller::command_interface_configuration
  for (VectorEnum vector: {LINEAR, ANGULAR})
  {
    for (AxisEnum axis: {X, Y, Z})
    {
      config_.twist_command_info[vector][axis].resource = "twist";
    }
  }
  config_.twist_command_info[LINEAR][X].interface = "linear_x";
  config_.twist_command_info[LINEAR][Y].interface = "linear_y";
  config_.twist_command_info[LINEAR][Z].interface = "linear_z";
  config_.twist_command_info[ANGULAR][X].interface = "angular_x";
  config_.twist_command_info[ANGULAR][Y].interface = "angular_y";
  config_.twist_command_info[ANGULAR][Z].interface = "angular_z";

  RCLCPP_INFO(logger_, "Finished On init.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn QLSystem::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  
  RCLCPP_INFO(logger_, "On configure...");

  RCLCPP_INFO(logger_, "Finished Configuration");

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<StateInterface> QLSystem::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;

  for (LegEnum leg: {LF, LH, RF, RH}){

    for (JointEnum joint: {HAA, HFE, KFE}){
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          config_.joint_state_info[leg][joint].resource,
          config_.joint_state_info[leg][joint].interface,
          &quadruped_state_[leg][joint].position)
      );

      RCLCPP_INFO(logger_, "Exporting state interface: %s/%s",
        config_.joint_state_info[leg][joint].resource.c_str(),
        config_.joint_state_info[leg][joint].interface.c_str());
    }
  }

  return state_interfaces;
}

std::vector<CommandInterface> QLSystem::export_command_interfaces()
{

  std::vector<CommandInterface> command_interfaces;

  for (VectorEnum vector: {LINEAR, ANGULAR}){

    for (AxisEnum axis: {X, Y, Z}){
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          config_.twist_command_info[vector][axis].resource,
          config_.twist_command_info[vector][axis].interface,
          &quadruped_twist_command_[vector][axis].velocity)
      );

      RCLCPP_INFO(logger_, "Exporting command interface: %s/%s",
        config_.twist_command_info[vector][axis].resource.c_str(),
        config_.twist_command_info[vector][axis].interface.c_str());
    }
  }

  return command_interfaces;
}


CallbackReturn QLSystem::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
{
  
  RCLCPP_INFO(logger_, "On activate...");

  //anymal_driver_ = std::make_unique<ql_bridge::QLBridge>();
  quadruped_locomotion_client_.startBackgroundSpin();

  RCLCPP_INFO(logger_, "Finished Activation");

  return CallbackReturn::SUCCESS;
}

CallbackReturn QLSystem::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */)
{
  
  RCLCPP_INFO(logger_, "On deactivate...");
  RCLCPP_INFO(logger_, "Finished Deactivation");

  return CallbackReturn::SUCCESS;
}

ReturnType QLSystem::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  quadruped_state_ = quadruped_locomotion_client_.getQuadrupedStates();
  //RCLCPP_INFO(logger_, "Read hardware interface! %f", quadruped_state_[LF][HAA].position);

  return hardware_interface::return_type::OK;
}

ReturnType QLSystem::write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  //RCLCPP_INFO(logger_, "Write hardware interface!  %f", quadruped_twist_command_[LINEAR][X].velocity);
  quadruped_locomotion_client_.setQuadrupedTwist(quadruped_twist_command_);

  return hardware_interface::return_type::OK;
}


}

PLUGINLIB_EXPORT_CLASS(ql_system::QLSystem, hardware_interface::SystemInterface)