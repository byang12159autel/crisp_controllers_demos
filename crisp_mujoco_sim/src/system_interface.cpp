#include "crisp_mujoco_sim/system_interface.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crisp_mujoco_sim
{

Simulator::CallbackReturn Simulator::on_init(const hardware_interface::HardwareInfo & info)
{
  // Keep an internal copy of the given configuration
  if (hardware_interface::SystemInterface::on_init(info) != Simulator::CallbackReturn::SUCCESS)
  {
    return Simulator::CallbackReturn::ERROR;
  }

  info_ = info;

  if (info_.joints.size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Simulator"), "No joints found");
    return CallbackReturn::ERROR;
  }

  // Get the clock
  clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  // Get MuJoCo model path
  m_mujoco_model = info_.hardware_parameters["mujoco_model"];

  // Initialize state vectors
  m_positions.resize(info_.joints.size(), 0.0);
  m_velocities.resize(info_.joints.size(), 0.0);
  m_efforts.resize(info_.joints.size(), 0.0);
  m_effort_commands.resize(info_.joints.size(), 0.0);

  // Create ROS2 node for service clients
  m_node = std::make_shared<rclcpp::Node>("mujoco_sim_client");
  m_init_client = m_node->create_client<crisp_mujoco_sim_msgs::srv::Init>("mujoco_init");
  m_step_client = m_node->create_client<crisp_mujoco_sim_msgs::srv::Step>("mujoco_step");
  m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  m_executor->add_node(m_node);

  // Wait for Init service
  RCLCPP_INFO(rclcpp::get_logger("Simulator"), "Waiting for /mujoco_init service...");
  while (!m_init_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"), "Interrupted while waiting for service");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("Simulator"), "Service not available, waiting...");
  }

  // Call Init service
  auto init_request = std::make_shared<crisp_mujoco_sim_msgs::srv::Init::Request>();
  init_request->model_path = m_mujoco_model;
  
  RCLCPP_INFO(rclcpp::get_logger("Simulator"), "Calling Init service with model: %s", m_mujoco_model.c_str());
  auto init_future = m_init_client->async_send_request(init_request);
  
  // Wait for Init response
  if (m_executor->spin_until_future_complete(init_future, std::chrono::seconds(10)) == 
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = init_future.get();
    if (response->success) {
      // Initialize state from response
      for (size_t i = 0; i < info_.joints.size() && i < response->position.size(); ++i) {
        m_positions[i] = response->position[i];
        m_velocities[i] = response->velocity[i];
        m_efforts[i] = response->effort[i];
      }
      RCLCPP_INFO(rclcpp::get_logger("Simulator"), "MuJoCo simulator initialized via service");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"), "Init service returned failure");
      return CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("Simulator"), "Failed to call Init service");
    return CallbackReturn::ERROR;
  }

  int joint_idx = 0;
  for (const hardware_interface::ComponentInfo & joint : info_.joints){
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Simulator"), "Found joint: " << joint.name);
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"), "Joint '%s' needs 1 command interface.",
                   joint.name.c_str());
      return Simulator::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"), "Joint '%s' needs the following command interface: %s.",
                   joint.name.c_str(), hardware_interface::HW_IF_EFFORT);
      return Simulator::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"), "Joint '%s' needs 3 state interfaces.",
                   joint.name.c_str());

      return Simulator::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"),
                   "Joint '%s' needs the following state interfaces in that order: %s, %s, and %s.",
                   joint.name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);

      return Simulator::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].initial_value.empty())) {
      m_positions[joint_idx] = joint.state_interfaces[0].initial_value.at(0);
    }
    joint_idx++;
  }

  return Simulator::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Simulator::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_positions[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &m_velocities[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &m_efforts[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Simulator::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &m_effort_commands[i]));
  }

  return command_interfaces;
}

Simulator::return_type Simulator::prepare_command_mode_switch(
  [[maybe_unused]] const std::vector<std::string> & start_interfaces,
  [[maybe_unused]] const std::vector<std::string> & stop_interfaces)
{
  return return_type::OK;
}

Simulator::return_type Simulator::read([[maybe_unused]] const rclcpp::Time & time,
                                       [[maybe_unused]] const rclcpp::Duration & period)
{
  // State already updated in write() - nothing to do
  return return_type::OK;
}

Simulator::return_type Simulator::write([[maybe_unused]] const rclcpp::Time & time,
                                        [[maybe_unused]] const rclcpp::Duration & period)
{
  // Create Step request
  auto step_request = std::make_shared<crisp_mujoco_sim_msgs::srv::Step::Request>();
  step_request->effort_commands = m_effort_commands;
  
  // Call Step service (async)
  auto step_future = m_step_client->async_send_request(step_request);
  
  // Wait for response with timeout
  auto status = m_executor->spin_until_future_complete(
    step_future, 
    std::chrono::milliseconds(10)
  );
  
  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = step_future.get();
    
    // Update stored state for next read()
    for (size_t i = 0; i < m_positions.size() && i < response->position.size(); ++i) {
      m_positions[i] = response->position[i];
      m_velocities[i] = response->velocity[i];
      m_efforts[i] = response->effort[i];
    }
    
    return return_type::OK;
  }
  
  RCLCPP_WARN_THROTTLE(
    rclcpp::get_logger("Simulator"), 
    *clock, 
    1000, 
    "Step service call timeout or failed"
  );
  return return_type::ERROR;
}

}  // namespace crisp_mujoco_sim

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(crisp_mujoco_sim::Simulator,
                       hardware_interface::SystemInterface)
