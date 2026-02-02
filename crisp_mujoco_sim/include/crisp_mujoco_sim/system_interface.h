#pragma once

#include <map>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

// Service message includes
#include "crisp_mujoco_sim_msgs/srv/init.hpp"
#include "crisp_mujoco_sim_msgs/srv/step.hpp"

namespace crisp_mujoco_sim
{

class Simulator : public hardware_interface::SystemInterface
{
public:
  using return_type = hardware_interface::return_type;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  RCLCPP_SHARED_PTR_DEFINITIONS(Simulator)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  rclcpp::Clock::SharedPtr clock;

private:
  std::vector<double> m_effort_commands;
  std::vector<double> m_positions;
  std::vector<double> m_velocities;
  std::vector<double> m_efforts;

  // Commands sent to Python sim in the previous write(); used for next Step request
  std::vector<double> m_stored_commands;

  // Node and client for Python MuJoCo sim service (Step, Init)
  rclcpp::Node::SharedPtr m_node;
  rclcpp::Client<crisp_mujoco_sim_msgs::srv::Step>::SharedPtr m_step_client;
  rclcpp::Client<crisp_mujoco_sim_msgs::srv::Init>::SharedPtr m_init_client;
  rclcpp::Executor::SharedPtr m_executor;

  std::string m_mujoco_model;
};

}  // namespace crisp_mujoco_sim
