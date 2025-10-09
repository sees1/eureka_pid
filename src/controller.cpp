#include <chrono>
#include "eureka_jerk/controller.hpp"

namespace nav2_eureka_controller
{

void Controller::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<mppi::ParametersHandler>(parent);

  auto node = parent_.lock();
  clock_ = node->get_clock();
  last_time_called_ = clock_->now();
  auto getParam = parameters_handler_->getParamGetter(name_);
  
  pid_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());

  RCLCPP_INFO(logger_, "Configured Eureka Controller: %s", name_.c_str());
}

void Controller::cleanup()
{
  parameters_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaned up Eureka Controller: %s", name_.c_str());
}

void Controller::activate()
{
  parameters_handler_->start();
  RCLCPP_INFO(logger_, "Activated Eureka Controller: %s", name_.c_str());
}

void Controller::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivated Eureka Controller: %s", name_.c_str());
}

void Controller::reset()
{
  RCLCPP_INFO(logger_, "Reset Eureka Controller: %s", name_.c_str());
}

geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{  
  rclcpp::Duration dt = clock_->now() - last_time_called_;
  last_time_called_ = clock_->now();

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  geometry_msgs::msg::TwistStamped cmd;

  if (goal_checker->isGoalReached(robot_pose.pose, path_handler_.getPath().poses.back().pose, robot_speed))
  {
    cmd.twist.linear.x = 0;
    cmd.twist.angular.z = 0;
  }
  else
  {
    cmd = pid_.evalControl(robot_pose, transformed_plan, dt);
  }

  return cmd;
}

void Controller::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_.setPath(path);
}

void Controller::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  double speed_limit_s = speed_limit;
  speed_limit_s++;
  double percentage_s = percentage;
  percentage_s++;
  RCLCPP_INFO(logger_, "set Speed Controller: %s", name_.c_str());
}

} // namespace nav2_eureka_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_eureka_controller::Controller, nav2_core::Controller)