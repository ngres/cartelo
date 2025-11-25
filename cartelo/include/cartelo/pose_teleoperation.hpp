#ifndef CARTELO__POSE_TELEOPERATION_HPP_
#define CARTELO__POSE_TELEOPERATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "cartelo/pose_teleoperation_parameters.hpp"

namespace cartelo
{

class PoseTeleoperation : public rclcpp::Node
{
public:
  explicit PoseTeleoperation(const rclcpp::NodeOptions & options);
  virtual ~PoseTeleoperation();

private:
  /**
   * @brief Callback function for joystick events.
   * 
   * @param msg The joystick message.
   */
  void joystick_event_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  /**
   * @brief Calibrate the reference frame for teleoperation by holding the controller close to the robot base.
   * 
   */
  void calibrate_frame();

  /**
   * @brief Start teleoperation by calculating the initial transform delta between the controller and the end-effector.
   * 
   */
  void start_teleoperation();

  /**
   * @brief Stop teleoperation by clearing the transform delta.
   * 
   */
  void stop_teleoperation();

  /**
   * @brief Publish the target end-effector pose based on the controller's current pose and the transform delta.
   * 
   */
  void publish_target_pose();

  /**
   * @brief Broadcast the frame transform between the virtual world and the controller.
   * 
   */
  void broadcast_frame_transform();

  std::optional<geometry_msgs::msg::TransformStamped> get_frame_transform();

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr frame_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<int> last_button_cmds_;
  std::optional<tf2::Transform> transform_delta_;
  std::optional<geometry_msgs::msg::TransformStamped> frame_transform_;
};

}  // namespace cartelo

#endif  // CARTELO__POSE_TELEOPERATION_HPP_
