#include "quadruped_locomotion_controller/ql_controller.hpp"


namespace ql_controller
{

using controller_interface::interface_configuration_type;
using namespace quadruped_locomotion::types;

CallbackReturn QLController::on_init()
{

  auto node = get_node();

  // Left front
  node->declare_parameter<std::string>("left_front_leg.haa_joint.name");
  node->declare_parameter<std::string>("left_front_leg.hfe_joint.name");
  node->declare_parameter<std::string>("left_front_leg.kfe_joint.name");

  // Left hind
  node->declare_parameter<std::string>("left_hind_leg.haa_joint.name");
  node->declare_parameter<std::string>("left_hind_leg.hfe_joint.name");
  node->declare_parameter<std::string>("left_hind_leg.kfe_joint.name");

  // Right front
  node->declare_parameter<std::string>("right_front_leg.haa_joint.name");
  node->declare_parameter<std::string>("right_front_leg.hfe_joint.name");
  node->declare_parameter<std::string>("right_front_leg.kfe_joint.name");

  // Right hind
  node->declare_parameter<std::string>("right_hind_leg.haa_joint.name");
  node->declare_parameter<std::string>("right_hind_leg.hfe_joint.name");
  node->declare_parameter<std::string>("right_hind_leg.kfe_joint.name");

  return CallbackReturn::SUCCESS;
}


CallbackReturn QLController::on_configure(const rclcpp_lifecycle::State &)
{

  auto node = get_node();

  auto get_param = [&node](const std::string& name) -> std::string {
    return node->get_parameter(name).as_string();
  };

  config_.joint_state_info[LF][HAA].name = get_param("left_front_leg.haa_joint.name");
  config_.joint_state_info[LF][HFE].name = get_param("left_front_leg.hfe_joint.name");
  config_.joint_state_info[LF][KFE].name = get_param("left_front_leg.kfe_joint.name");

  config_.joint_state_info[LH][HAA].name = get_param("left_hind_leg.haa_joint.name");
  config_.joint_state_info[LH][HFE].name = get_param("left_hind_leg.hfe_joint.name");
  config_.joint_state_info[LH][KFE].name = get_param("left_hind_leg.kfe_joint.name");

  config_.joint_state_info[RF][HAA].name = get_param("right_front_leg.haa_joint.name");
  config_.joint_state_info[RF][HFE].name = get_param("right_front_leg.hfe_joint.name");
  config_.joint_state_info[RF][KFE].name = get_param("right_front_leg.kfe_joint.name");

  config_.joint_state_info[RH][HAA].name = get_param("right_hind_leg.haa_joint.name");
  config_.joint_state_info[RH][HFE].name = get_param("right_hind_leg.hfe_joint.name");
  config_.joint_state_info[RH][KFE].name = get_param("right_hind_leg.kfe_joint.name");

  for (LegEnum leg: {LF, LH, RF, RH}){

    for (JointEnum joint: {HAA, HFE, KFE})
    {
      config_.joint_state_info[leg][joint].resource = config_.joint_state_info[leg][joint].name;
      config_.joint_state_info[leg][joint].interface = hardware_interface::HW_IF_POSITION;
    }
  }

  // Matches with ql_system::export_command_interfaces
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

  cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&QLController::cmdVelCallback, this, std::placeholders::_1));

  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  return CallbackReturn::SUCCESS;
}


InterfaceConfiguration
QLController::state_interface_configuration() const
{
  std::vector<std::string> config_names;

  for (LegEnum leg: {LF, LH, RF, RH}){

    for (JointEnum joint: {HAA, HFE, KFE}){
      config_names.push_back(
        config_.joint_state_info[leg][joint].resource + "/" + 
        config_.joint_state_info[leg][joint].interface
      );
   
      RCLCPP_INFO(logger_, "Register state interface: %s/%s",
        config_.joint_state_info[leg][joint].resource.c_str(),
        config_.joint_state_info[leg][joint].interface.c_str());
    }
  }

  return { interface_configuration_type::INDIVIDUAL, config_names};
}


InterfaceConfiguration
QLController::command_interface_configuration() const
{
  std::vector<std::string> config_names;

  for (VectorEnum vector: {LINEAR, ANGULAR})
  {
    for (AxisEnum axis: {X, Y, Z})
    {
      config_names.push_back(
        config_.twist_command_info[vector][axis].resource + "/" + 
        config_.twist_command_info[vector][axis].interface
      );
   
      RCLCPP_INFO(logger_, "Register command interface: %s/%s",
        config_.twist_command_info[vector][axis].resource.c_str(),
        config_.twist_command_info[vector][axis].interface.c_str());
    }
  }

  return {interface_configuration_type::INDIVIDUAL,config_names};
}


CallbackReturn QLController::on_activate(const rclcpp_lifecycle::State &)
{
  // Create an array of references from state_interfaces_ for legs and joints clasifiaction 
  // to be used in update() to get states from hardware interface:
  //   - state_interface_name definition depends on quadruped_controller.yaml joint name
  //     (set in state_interface_configuration())
  //   - state_interfaces_ list names defined on quadruped_control.urdf.xacro ros2_joint tag 
  //     (set in hardware_interface::export_state_interfaces())

  RCLCPP_INFO(logger_, "Listing state interfaces found in hardware_interface::export_state_interfaces (state_interfaces_)");
  for (const auto &state_interface : state_interfaces_) {
    RCLCPP_INFO(logger_, " - %s", state_interface.get_name().c_str());
  }

  for (LegEnum leg: {LF, LH, RF, RH}){

    for (JointEnum joint: {HAA, HFE, KFE})
    {
      bool found = false;
      
      std::string state_interface_name = 
        config_.joint_state_info[leg][joint].resource + "/" +
        config_.joint_state_info[leg][joint].interface; 

      for (auto& state_interface: state_interfaces_)
      {
        if (state_interface.get_name() == state_interface_name)
        {
          RCLCPP_INFO(logger_, 
            "State interface '%s' found in hardware_interface::export_state_interfaces (state_interfaces_)", 
            state_interface_name.c_str());

          quadruped_state_[leg][joint] = LoanedJointState{{std::ref(state_interface)}};
          found = true;
          break;
        }
      }

      if (!found)
      {
          RCLCPP_ERROR(logger_, 
            "Failed to bind state interface for '%s' (not found in state_interfaces_ list)", 
            state_interface_name.c_str());
      }
    }
  }
  
  // Create an array of references from command_interfaces_ for linear and axes clasifiaction 
  // to be used in update() to set commands for hardware interface:
  //   - command_interface_name definition depends on quadruped_control.urdf.xacro gpio tag
  //     (set in command_interface_configuration())
  //   - command_interfaces_ list names defined on quadruped_control.urdf.xacro gpio tag 
  //     (set in hardware_interface::export_command_interfaces())

  RCLCPP_INFO(logger_, "Listing command interfaces found in hardware_interface::export_command_interfaces (command_interfaces_)");
  for (const auto &command_interface : command_interfaces_) {
    RCLCPP_INFO(logger_, " - %s", command_interface.get_name().c_str());
  }

  for (VectorEnum vector: {LINEAR, ANGULAR}){

    for (AxisEnum axis: {X, Y, Z}){

      bool found = false;

      std::string command_interface_name = 
        config_.twist_command_info[vector][axis].resource + "/" +
        config_.twist_command_info[vector][axis].interface; 

      for (auto& command_interface: command_interfaces_)
      {
        if (command_interface.get_name() == command_interface_name)
        {
          RCLCPP_INFO(logger_, 
            "Command interface '%s' found in hardware_interface::export_command_interfaces (command_interfaces_)", 
            command_interface_name.c_str());

          quadruped_twist_command_[vector][axis] = LoanedAxisCommand{{std::ref(command_interface)}};
          found = true;
          break;
        }
      }

      if (!found)
      {
          RCLCPP_ERROR(logger_, 
            "Failed to bind command interface for '%s' (not found in command_interfaces_ list)", 
            command_interface_name.c_str());
      }
    }
  }
  
  return CallbackReturn::SUCCESS;
}


void QLController::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  
  linear_x_ = msg->linear.x;
  angular_z_ = msg->angular.z;
}

ReturnType QLController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  
  // WRITE → hardware
  if (!quadruped_twist_command_[LINEAR][X].axis->get().set_value<double>(linear_x_)) {
    
    //RCLCPP_INFO(logger_, "Control write: %f", linear_x_);
    return controller_interface::return_type::ERROR;
  }
  if (!quadruped_twist_command_[ANGULAR][Z].axis->get().set_value<double>(angular_z_)) {
    return controller_interface::return_type::ERROR;
  }

  // READ ← hardware
  for (LegEnum leg: {LF, LH, RF, RH}){

    for (JointEnum joint: {HAA, HFE, KFE}){

      const auto &opt_state = quadruped_state_[leg][joint].state;
      if (!opt_state) {
        RCLCPP_ERROR(logger_, "Missing state interface for leg %zu joint %zu",
                     static_cast<std::size_t>(leg),
                     static_cast<std::size_t>(joint));
        return ReturnType::ERROR;
      }

      const auto &handle = opt_state->get();
      auto val_opt = handle.get_optional<double>();
      if (val_opt.has_value()) {
        double val = val_opt.value();
        (void)val; // use or store value as needed
      } else {
        RCLCPP_WARN(logger_, "State interface has no value for leg %zu joint %zu",
                    static_cast<std::size_t>(leg),
                    static_cast<std::size_t>(joint));
      }

    }
  }

  // ODOM
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = 17; //state_;

  odom_pub_->publish(odom);

  return ReturnType::OK;
}

}  // namespace ql_controller

PLUGINLIB_EXPORT_CLASS(ql_controller::QLController, ControllerInterface)
