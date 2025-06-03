#include "mujoco_ros2_simulation/mujoco_system_interface.hpp"
#include "mujoco_ros2_simulation/mujoco_simulate.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>


namespace mujoco_ros2_simulation
{

MujocoSystemInterface::MujocoSystemInterface() = default;

MujocoSystemInterface::~MujocoSystemInterface()
{

}

hardware_interface::CallbackReturn MujocoSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Load the model path from hardware parameters
  if (info.hardware_parameters.count("mujoco_model") == 0) {
    RCLCPP_FATAL(rclcpp::get_logger("MujocoSystemInterface"),
      "Missing 'mujoco_model' in <hardware_parameters>.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::string model_path = info.hardware_parameters.at("mujoco_model");

  RCLCPP_INFO_STREAM(rclcpp::get_logger("MujocoSystemInterface"),
    "Loading 'mujoco_model' from: " << model_path);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MujocoSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MujocoSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  return command_interfaces;
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"),
              "Activating MuJoCo hardware interface and starting Simulate threads...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"),
              "Deactivating MuJoCo hardware interface and shutting down Simulate...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoSystemInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystemInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

} // end namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mujoco_ros2_simulation::MujocoSystemInterface,
  hardware_interface::SystemInterface
);
