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

#include "cartelo/homing_handler.hpp"

namespace cartelo
{

HomingHandler::HomingHandler(rclcpp::Node* node, const std::string& joint_controller_name,
                             const std::string& cartesian_controller_name, const std::vector<std::string>& joint_names,
                             const std::vector<double>& joint_positions)
  : node_(node)
  , joint_controller_name_(joint_controller_name)
  , cartesian_controller_name_(cartesian_controller_name)
  , joint_names_(joint_names)
  , joint_positions_(joint_positions)
{
  switch_controller_client_ =
      node_->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller");
  follow_joint_trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      node_, joint_controller_name_ + "/follow_joint_trajectory");
}

void HomingHandler::trigger_homing(std::function<void(bool)> callback)
{
  if (!switch_controller_client_->service_is_ready())
  {
    RCLCPP_ERROR(node_->get_logger(), "Switch controller service not available");
    if (callback)
      callback(false);
    return;
  }

  if (!follow_joint_trajectory_client_->action_server_is_ready())
  {
    RCLCPP_ERROR(node_->get_logger(), "Follow joint trajectory action server not available");
    if (callback)
      callback(false);
    return;
  }

  // 1. Switch to joint controller
  auto switch_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  switch_req->activate_controllers.push_back(joint_controller_name_);
  switch_req->deactivate_controllers.push_back(cartesian_controller_name_);
  switch_req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  switch_req->activate_asap = true;

  switch_controller_client_->async_send_request(
      switch_req,
      [this, callback](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
        try
        {
          auto response = future.get();
          if (!response->ok)
          {
            RCLCPP_ERROR(node_->get_logger(), "Failed to switch controllers (to joint): %s", response->message.c_str());
            if (callback)
              callback(false);
            return;
          }
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", e.what());
          if (callback)
            callback(false);
          return;
        }

        // 2. Move to home
        auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_positions_;
        point.time_from_start = rclcpp::Duration::from_seconds(5.0);  // [TODO] make configurable
        goal_msg.trajectory.points.push_back(point);

        auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.result_callback =
            [this, callback](
                const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult&
                    result) {
              if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
              {
                RCLCPP_ERROR(node_->get_logger(), "Homing trajectory failed");
              }

              // 3. Switch back to cartesian controller
              auto switch_req_back = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
              switch_req_back->activate_controllers.push_back(cartesian_controller_name_);
              switch_req_back->deactivate_controllers.push_back(joint_controller_name_);
              switch_req_back->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
              switch_req_back->activate_asap = true;

              switch_controller_client_->async_send_request(
                  switch_req_back,
                  [this,
                   callback](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_back) {
                    try
                    {
                      auto response_back = future_back.get();
                      if (!response_back->ok)
                      {
                        RCLCPP_ERROR(node_->get_logger(), "Failed to switch controllers (to cartesian): %s",
                                     response_back->message.c_str());
                        if (callback)
                          callback(false);  // Even though homing succeeded, switching back failed
                        return;
                      }
                      RCLCPP_INFO(node_->get_logger(), "Homing completed successfully");
                      if (callback)
                        callback(true);
                    }
                    catch (const std::exception& e)
                    {
                      RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", e.what());
                      if (callback)
                        callback(false);
                    }
                  });
            };

        follow_joint_trajectory_client_->async_send_goal(goal_msg, send_goal_options);
      });
}

}  // namespace cartelo
