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

#include "cartelo/twist_teleoperation.hpp"

#include <chrono>
#include <exception>
#include <cmath>
#include <tf2/LinearMath/Vector3.hpp>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace cartelo
{

TwistTeleoperation::TwistTeleoperation(const rclcpp::NodeOptions& options) : Node("twist_teleoperation", options)
{
  param_listener_ = std::make_shared<twist_teleoperation::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Joystick setup
  joystick_handler_ = std::make_shared<JoystickHandler>(this, params_.controller_frame_id);
  joystick_handler_->register_on_press(params_.joystick.home_button, [this]() {
    try
    {
      RCLCPP_INFO(this->get_logger(), "Trigger homing");
      trigger_homing();
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to trigger homing: %s", e.what());
    }
  });

  homing_handler_ =
      std::make_shared<HomingHandler>(this, params_.home.joint_controller_name, params_.cartesian_controller_name,
                                      params_.home.joint_names, params_.home.joint_positions);

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 3);
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      params_.twist_topic, 1, std::bind(&TwistTeleoperation::twist_callback, this, std::placeholders::_1));

  double period = 1.0 / params_.update_rate;
  publish_timer_ = this->create_wall_timer(std::chrono::duration<double>(period),
                                           std::bind(&TwistTeleoperation::publish_target_pose, this));

  // Start startup check
  startup_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TwistTeleoperation::startup, this));
}

TwistTeleoperation::~TwistTeleoperation()
{
}

void TwistTeleoperation::startup()
{
  if (startup_done_)
  {
    startup_timer_->cancel();
    return;
  }

  // Reload params to get correct frame ids if they changed
  params_ = param_listener_->get_params();

  try
  {
    geometry_msgs::msg::TransformStamped start;
    start = tf_buffer_->lookupTransform(params_.base_frame_id, params_.end_effector_frame_id, tf2::TimePointZero);

    tf2::fromMsg(start.transform.translation, pos_);
    tf2::fromMsg(start.transform.rotation, rot_);

    frame_rot_.setRPY(params_.frame_rotation[0], params_.frame_rotation[1], params_.frame_rotation[2]);
    
    startup_done_ = true;
    RCLCPP_INFO(this->get_logger(), "TwistTeleoperation startup successful. Initialized at robot pose.");
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(), "Startup failed, retrying: %s", ex.what());
  }
}

void TwistTeleoperation::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!startup_done_)
  {
    return;
  }

  rclcpp::Time now = this->now();
  if (!first_twist_received_)
  {
    last_twist_time_ = now;
    first_twist_received_ = true;
    return;  // Do not integrate on first packet to avoid large dt issues
  }

  double dt = (now - last_twist_time_).seconds();
  last_twist_time_ = now;

  // Position update
  tf2::Vector3 linear;
  tf2::fromMsg(msg->linear, linear);
  pos_ += frame_rot_ * linear * dt;

  if (params_.bounds.enabled) {
    pos_.setX(std::clamp(pos_.x(), params_.bounds.x_min, params_.bounds.x_max));
    pos_.setY(std::clamp(pos_.y(), params_.bounds.y_min, params_.bounds.y_max));
    pos_.setZ(std::clamp(pos_.z(), params_.bounds.z_min, params_.bounds.z_max));
  }

  // Orientation update
  tf2::Vector3 angular;
  tf2::fromMsg(msg->angular, angular);
  angular = frame_rot_ * angular;

  double angle = angular.length() * dt;
  if (angle > 1e-6)
  {
    angular.normalize();
    tf2::Quaternion delta_q(angular, angle);
    rot_ = delta_q * rot_;
    rot_.normalize();
  }

  is_homed_ = false;
}

void TwistTeleoperation::publish_target_pose()
{
  if (!startup_done_)
  {
    return;
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = params_.base_frame_id;
  msg.pose.position.x = pos_.x();
  msg.pose.position.y = pos_.y();
  msg.pose.position.z = pos_.z();

  msg.pose.orientation = tf2::toMsg(rot_);

  pose_pub_->publish(msg);
}

void TwistTeleoperation::trigger_homing()
{
  if (is_homed_)
  {
    return;
  }

  // Re-startup/reset state to current actual robot pose
  startup_done_ = false;
  first_twist_received_ = false;

  homing_handler_->trigger_homing([this](bool success) {
    if (success)
    {
      is_homed_ = true;
      // Restart startup timer to recapture pose
      startup_timer_->reset();
    }
  });
}

}  // namespace cartelo

RCLCPP_COMPONENTS_REGISTER_NODE(cartelo::TwistTeleoperation)
