// Copyright 2026 Nicolas Gres
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

#include "cartelo/cartesian_bounds.hpp"
#include <algorithm>
#include <memory>
#include <functional>
#include "rclcpp_components/register_node_macro.hpp"

namespace cartelo
{

CartesianBounds::CartesianBounds(const rclcpp::NodeOptions& options) : Node("cartesian_bounds", options)
{
  param_listener_ = std::make_shared<cartesian_bounds::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  clipped_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("clipped_pose", 3);
  
  input_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "input_pose", 1, std::bind(&CartesianBounds::input_pose_callback, this, std::placeholders::_1));
      
  RCLCPP_INFO(this->get_logger(), "CartesianBounds node initialized.");
}

CartesianBounds::~CartesianBounds()
{
}

void CartesianBounds::input_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Refresh parameters
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  geometry_msgs::msg::PoseStamped clipped_msg = *msg;

  // Apply bounds
  clipped_msg.pose.position.x = std::clamp(msg->pose.position.x, params_.bounds.x_min, params_.bounds.x_max);
  clipped_msg.pose.position.y = std::clamp(msg->pose.position.y, params_.bounds.y_min, params_.bounds.y_max);
  clipped_msg.pose.position.z = std::clamp(msg->pose.position.z, params_.bounds.z_min, params_.bounds.z_max);

  clipped_pose_pub_->publish(clipped_msg);
}

}  // namespace cartelo

RCLCPP_COMPONENTS_REGISTER_NODE(cartelo::CartesianBounds)
