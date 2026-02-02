#ifndef CARTELO__CARTESIAN_BOUNDS_HPP_
#define CARTELO__CARTESIAN_BOUNDS_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cartesian_bounds_parameters.hpp"

namespace cartelo
{

class CartesianBounds : public rclcpp::Node
{
public:
  explicit CartesianBounds(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  virtual ~CartesianBounds();

private:
  void input_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  std::shared_ptr<cartesian_bounds::ParamListener> param_listener_;
  cartesian_bounds::Params params_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr input_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr clipped_pose_pub_;
};

}  // namespace cartelo

#endif  // CARTELO__CARTESIAN_BOUNDS_HPP_
