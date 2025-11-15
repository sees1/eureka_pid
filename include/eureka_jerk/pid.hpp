#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include <string>
#include <fstream>

class Segment {
public:
  Segment(geometry_msgs::msg::PoseStamped s,
          geometry_msgs::msg::PoseStamped e,
          double y,
          bool bs,
          bool inv)
  : start_segment_(s),
    end_segment_(e),
    yaw_segment_(y),
    bad_segment_(bs),
    inverse_move_(inv)
  {}

public:
  geometry_msgs::msg::PoseStamped start_segment_;
  geometry_msgs::msg::PoseStamped end_segment_;
  double yaw_segment_;
  bool bad_segment_;
  bool inverse_move_;
};

class PID {
public:
  PID() = default;

  void initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
                  const std::string & name,
                  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
                  mppi::ParametersHandler * dynamic_parameters_handler);

  void deactivate();

  geometry_msgs::msg::TwistStamped evalControl(const geometry_msgs::msg::PoseStamped & robot_pose,
                                               const nav_msgs::msg::Path & transformed_plan,
                                               double dt_ms);

  double getMaxCostmapDist();

private:
  double wrapToPi(double thetha);

  Segment computeSegment(const geometry_msgs::msg::PoseStamped & robot_pose, const nav_msgs::msg::Path & transformed_plan);

  double computeYaw(const geometry_msgs::msg::PoseStamped& pose);
  double computeYaw(const geometry_msgs::msg::PoseStamped& first_pose, const geometry_msgs::msg::PoseStamped& second_pose);
  geometry_msgs::msg::PoseStamped carrotRobotPose(const geometry_msgs::msg::PoseStamped& robot_pose, bool inverse_move);

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  mppi::ParametersHandler * parameters_handler_;
  rclcpp::Logger logger_{rclcpp::get_logger("EurekaController")};

  double max_robot_pose_search_dist_;

  // work var
  double e_y_prev;
  double integral_e_y;
  
  // config var
  double v_max_;
  size_t segment_size_;
  double carrot_length_;
  double k_p;
  double k_i;
  double k_d;

  // log
  std::ofstream log;
};