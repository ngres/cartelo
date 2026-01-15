// Copyright 2025 Nicolas Gres
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CARTELO__HOMING_HANDLER_HPP_
#define CARTELO__HOMING_HANDLER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

namespace cartelo
{
/**
 * @brief Homing handler class.
 *
 * Handles the logic for homing the robot by switching controllers and moving to a target joint configuration.
 */
class HomingHandler
{
public:
  /**
   * @brief Construct a new Homing Handler object
   *
   * @param node ROS 2 node pointer.
   * @param joint_controller_name Name of the joint controller to switch to (and used for trajectory action).
   * @param cartesian_controller_name Name of the cartesian controller to switch back to.
   * @param joint_names List of joint names for the home position.
   * @param joint_positions List of joint positions for the home position.
   */
  HomingHandler(rclcpp::Node* node, const std::string& joint_controller_name,
                const std::string& cartesian_controller_name, const std::vector<std::string>& joint_names,
                const std::vector<double>& joint_positions);

  /**
   * @brief Trigger the homing sequence.
   *
   * @param callback Optional callback to be called when homing completes (true = success, false = failure).
   */
  void trigger_homing(std::function<void(bool)> callback = nullptr);

private:
  rclcpp::Node* node_;
  std::string joint_controller_name_;
  std::string cartesian_controller_name_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;

  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_trajectory_client_;
};

}  // namespace cartelo

#endif  // CARTELO__HOMING_HANDLER_HPP_
