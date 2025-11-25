#include "cartelo/pose_teleoperation.hpp"

#include <chrono>
#include <exception>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

namespace cartelo
{

PoseTeleoperation::PoseTeleoperation(const rclcpp::NodeOptions & options)
: Node("pose_teleoperation", options)
{
  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    params_.pose_teleoperation.joy_topic, 1,
    std::bind(&PoseTeleoperation::joystick_event_callback, this, std::placeholders::_1));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    params_.pose_teleoperation.target_pose_topic, 3);

  double period = 1.0 / params_.pose_teleoperation.publishing_rate;
  pose_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&PoseTeleoperation::publish_target_pose, this));

  frame_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PoseTeleoperation::broadcast_frame_transform, this));
}

PoseTeleoperation::~PoseTeleoperation() {}

void PoseTeleoperation::joystick_event_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons.empty()) {
    return;
  }

  if (last_button_cmds_.empty()) {
    last_button_cmds_ = msg->buttons;
    return;
  }

  // Log pressed buttons
  for (size_t i = 0; i < msg->buttons.size(); ++i) {
    if (msg->buttons[i] == 1 && last_button_cmds_[i] == 0) {
      RCLCPP_INFO(this->get_logger(), "Button %ld pressed", i);
    }
  }

  // Button 6: Calibrate frame
  if (msg->buttons.size() > 6 && msg->buttons[6] == 1 && last_button_cmds_[6] == 0) {
    try {
      RCLCPP_INFO(this->get_logger(), "Calibrate frame");
      calibrate_frame();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to calibrate frame: %s", e.what());
    }
  }

  // Button 7: Start/Stop teleoperation
  if (msg->buttons.size() > 7) {
    if (msg->buttons[7] == 0 && last_button_cmds_[7] == 1) {
      try {
        RCLCPP_INFO(this->get_logger(), "Stop teleoperation");
        stop_teleoperation();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop teleoperation: %s", e.what());
      }
    } else if (msg->buttons[7] == 1 && last_button_cmds_[7] == 0) {
      try {
        RCLCPP_INFO(this->get_logger(), "Start teleoperation");
        start_teleoperation();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start teleoperation: %s", e.what());
      }
    }
  }

  last_button_cmds_ = msg->buttons;
}

void PoseTeleoperation::calibrate_frame()
{
  auto frame_transform = get_frame_transform();
  if (!frame_transform) {
    RCLCPP_ERROR(this->get_logger(), "Could not get frame transform");
    return;
  }
  frame_transform_ = frame_transform;
  broadcast_frame_transform();
}

void PoseTeleoperation::start_teleoperation()
{
  params_ = param_listener_->get_params();
  geometry_msgs::msg::TransformStamped b_T_e_msg, b_T_c_msg;

  try {
    b_T_e_msg = tf_buffer_->lookupTransform(
      params_.pose_teleoperation.base_frame_id,
      params_.pose_teleoperation.end_effector_frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform %s to %s: %s",
      params_.pose_teleoperation.base_frame_id.c_str(), params_.pose_teleoperation.end_effector_frame_id.c_str(), ex.what());
    return;
  }

  try {
    b_T_c_msg = tf_buffer_->lookupTransform(
      params_.pose_teleoperation.base_frame_id,
      params_.pose_teleoperation.controller_frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform %s to %s: %s",
      params_.pose_teleoperation.base_frame_id.c_str(), params_.pose_teleoperation.controller_frame_id.c_str(), ex.what());
    return;
  }

  tf2::Transform b_T_e, b_T_c;
  tf2::fromMsg(b_T_e_msg.transform, b_T_e);
  tf2::fromMsg(b_T_c_msg.transform, b_T_c);

  tf2::Transform delta;
  delta.setOrigin(b_T_c.getOrigin() - b_T_e.getOrigin());
  delta.setRotation(b_T_e.getRotation().inverse() * b_T_c.getRotation());
  transform_delta_ = delta;
}

void PoseTeleoperation::stop_teleoperation()
{
  transform_delta_.reset();
}

std::optional<geometry_msgs::msg::TransformStamped> PoseTeleoperation::get_frame_transform()
{
  params_ = param_listener_->get_params();
  geometry_msgs::msg::TransformStamped v_T_c_msg;

  try {
    v_T_c_msg = tf_buffer_->lookupTransform(
      params_.pose_teleoperation.virtual_world_frame_id,
      params_.pose_teleoperation.controller_frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform %s to %s: %s",
      params_.pose_teleoperation.virtual_world_frame_id.c_str(), params_.pose_teleoperation.controller_frame_id.c_str(), ex.what());
    return std::nullopt;
  }

  geometry_msgs::msg::TransformStamped v_T_b;
  v_T_b.header.stamp = v_T_c_msg.header.stamp;
  v_T_b.header.frame_id = v_T_c_msg.header.frame_id;
  v_T_b.child_frame_id = params_.pose_teleoperation.base_frame_id;

  tf2::Transform v_T_c;
  tf2::fromMsg(v_T_c_msg.transform, v_T_c);

  tf2::Matrix3x3 rot_mat(
     0, -1,  0,
     0,  0,  1,
    -1,  0,  0);
  tf2::Vector3 trans_vec(0, 0, 0);
  tf2::Transform conversion_transform(rot_mat, trans_vec);

  tf2::Transform v_T_b_transform = v_T_c * conversion_transform;
  v_T_b.transform = tf2::toMsg(v_T_b_transform);

  return v_T_b;
}

void PoseTeleoperation::broadcast_frame_transform()
{
  if (!frame_transform_) {
    return;
  }

  frame_transform_->header.stamp = this->now();
  tf_broadcaster_->sendTransform(*frame_transform_);
}

void PoseTeleoperation::publish_target_pose()
{
  if (!frame_transform_ || !transform_delta_) {
    return;
  }

  params_ = param_listener_->get_params();
  geometry_msgs::msg::TransformStamped b_T_c_msg;

  try {
    b_T_c_msg = tf_buffer_->lookupTransform(
      params_.pose_teleoperation.base_frame_id,
      params_.pose_teleoperation.controller_frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform %s to %s: %s",
      params_.pose_teleoperation.base_frame_id.c_str(), params_.pose_teleoperation.controller_frame_id.c_str(), ex.what());
    return;
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = params_.pose_teleoperation.base_frame_id;

  tf2::Transform b_T_c;
  tf2::fromMsg(b_T_c_msg.transform, b_T_c);

  // Apply the inverse of the delta transform to get the target pose
  tf2::Transform target_transform = b_T_c * transform_delta_.value().inverse();

  msg.pose.position.x = target_transform.getOrigin().x();
  msg.pose.position.y = target_transform.getOrigin().y();
  msg.pose.position.z = target_transform.getOrigin().z();
  msg.pose.orientation = tf2::toMsg(target_transform.getRotation());

  pose_pub_->publish(msg);
}

}  // namespace cartelo

RCLCPP_COMPONENTS_REGISTER_NODE(cartelo::PoseTeleoperation)
