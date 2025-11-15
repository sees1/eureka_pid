
#ifndef NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
#define NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_

#include <chrono>

#include "eureka_jerk/pid.hpp"
#include "eureka_jerk/path_handler.hpp"

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_eureka_controller
{

class Controller : public nav2_core::Controller
{
public:
  Controller() = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                 std::string name,
                 const std::shared_ptr<tf2_ros::Buffer> tf,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void reset();

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) ;

protected:
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("EurekaController")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::unique_ptr<mppi::ParametersHandler> parameters_handler_;
  PID pid_;
  PathHandler path_handler_;

  std::chrono::time_point<std::chrono::steady_clock> last_time_called_;
};

} // namespace nav2_eureka_controller

#endif // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_