#include "eureka_jerk/pid.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "cmath"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

void PID::initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
                     const std::string & name,
                     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
                     mppi::ParametersHandler * param_handler)
{
  parent_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  parameters_handler_ = param_handler;

  auto node = parent_.lock();
  logger_ = node->get_logger();

  max_robot_pose_search_dist_ = getMaxCostmapDist();

  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(segment_size_, "segment_size", 4);
  getParam(v_max_, "v_max", 0.25);
  getParam(carrot_length_, "carrot_length", 1.0);
  getParam(k_p, "k_p", 2.0);
  getParam(k_d, "k_d", 0.0);
  getParam(k_i, "k_i", 0.0);

#ifdef DEBUG
  std::cout << "segment_size = " << segment_size_ << std::endl;
  std::cout << "v_max = " << v_max_ << std::endl;
  std::cout << "carrot_length = " << carrot_length_ << std::endl;
  std::cout << "k_p = " << k_p << std::endl;
  std::cout << "k_d = " << k_d << std::endl;
  std::cout << "k_i = " << k_i << std::endl;

  log.open("/home/sees/CommLineProg/work/catkin2_ws/work_log/pid_log.txt");

  if (!log.is_open()) {
    std::cerr << "Can't open log!\n";
  }
#endif // DEBUG
}

void PID::deactivate()
{
#ifdef DEBUG
  log.close();
#endif // DEBUG
}

Segment PID::computeSegment(const geometry_msgs::msg::PoseStamped& robot_pose, const nav_msgs::msg::Path& plan)
{
  using nav2_util::geometry_utils::euclidean_distance;

  auto begin = plan.poses.begin();

  // Limit the search for the closest pose up to max_robot_pose_search_dist on the path
  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    plan.poses.begin(), plan.poses.end(),
    max_robot_pose_search_dist_);

  // Find closest point to the robot
  auto start_pose = nav2_util::geometry_utils::min_by(
    begin, closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  auto next_pose = start_pose;
  double segment_yaw = 0;
  double robot_yaw = 0;
  
  bool bad_segment = false;
  bool inverse_move = false;

  size_t count = 0;
  if (count != segment_size_ || next_pose != plan.poses.end() - 1)
  {  
    next_pose++;
    count++;
  }
  else
  {
    bad_segment = true;
  }

  if(start_pose != next_pose)
  {
    segment_yaw = computeYaw(*start_pose, *next_pose);
    robot_yaw = computeYaw(robot_pose);

#ifdef DEBUG
    std::cout << "SEGMENT_YAW = " << segment_yaw << std::endl;
    std::cout << "ROBOT_YAW = " << robot_yaw << std::endl;
    log << "SEGMENT_YAW = " << segment_yaw << "\n";
    log << "ROBOT_YAW = " << robot_yaw << "\n";
#endif // DEBUG

    double local_segment_yaw = std::acos(std::cos(robot_yaw) * std::cos(segment_yaw) + std::sin(robot_yaw) * std::sin(segment_yaw));
    
    if (local_segment_yaw > M_PI / 2)
      inverse_move = true;
  }

  return Segment(*start_pose, *next_pose, segment_yaw, bad_segment, inverse_move);
}

geometry_msgs::msg::TwistStamped PID::evalControl(const geometry_msgs::msg::PoseStamped & robot_pose,
                                                  const nav_msgs::msg::Path & transformed_plan,
                                                  double dt_ms)
{
  Segment segment = computeSegment(robot_pose, transformed_plan);

  geometry_msgs::msg::TwistStamped result_vel;

  if (!segment.bad_segment_)
  {
    double dt_s = dt_ms / 1000.0;

    double robot_yaw = computeYaw(robot_pose);

    geometry_msgs::msg::PoseStamped carrot_robot_pose = carrotRobotPose(robot_pose, segment.inverse_move_);

    double e_y = -(carrot_robot_pose.pose.position.x - segment.start_segment_.pose.position.x) * std::sin(segment.yaw_segment_) +
                  (carrot_robot_pose.pose.position.y - segment.start_segment_.pose.position.y) * std::cos(segment.yaw_segment_);

#ifdef DEBUG
    std::cout << "e_y " << e_y << std::endl;
    std::cout << "Delta x = " << carrot_robot_pose.pose.position.x - segment.start_segment_.pose.position.x << std::endl;
    std::cout << "Delta y = " << carrot_robot_pose.pose.position.y - segment.start_segment_.pose.position.y << std::endl;

    log << "e_y " << e_y << "\n";
    log << "Delta x = " << carrot_robot_pose.pose.position.x - segment.start_segment_.pose.position.x << "\n";
    log << "Delta y = " << carrot_robot_pose.pose.position.y - segment.start_segment_.pose.position.y << "\n";
#endif // DEBUG

    double e_theta = wrapToPi(std::abs(robot_yaw) - std::abs(computeYaw(segment.end_segment_)));

#ifdef DEBUG
    std::cout << "e_theta = " << e_theta << std::endl;
    log << "e_theta = " << e_theta << "\n";
#endif // DEBUG

    double de_y = (e_y - e_y_prev) / dt_s;
    integral_e_y += e_y * dt_s;
    double omega = k_p * (-1.0) * e_y + k_d * de_y + k_i * integral_e_y;

    double v = v_max_ * std::cos(e_theta);

#ifdef DEBUG
    std::cout << "V = " << v << std::endl;
    std::cout << "Omega = " << omega << std::endl;
    std::cout << "Inverse? " << segment.inverse_move_ << std::endl;
    log << "V = " << v << "\n";
    log << "Omega = " << omega << "\n";
    log << "Inverse? " << segment.inverse_move_ << "\n";
#endif // DEBUG

    result_vel.twist.linear.x = segment.inverse_move_ ? -v : v;
    result_vel.twist.angular.z = omega;

    e_y_prev = e_y;
  }
  else
  {
    result_vel.twist.linear.x = 0.0;
    result_vel.twist.angular.z = 0.0;
  }

  return result_vel;
}

double PID::wrapToPi(double theta)
{
  theta = std::fmod(theta + M_PI, 2.0 * M_PI);
  if (theta < 0)
      theta += 2.0 * M_PI;
  return theta - M_PI;
}

double PID::getMaxCostmapDist()
{
  return static_cast<double>(std::max(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY())) *
         costmap_->getResolution() * 0.50;
}

double PID::computeYaw(const geometry_msgs::msg::PoseStamped& pose)
{
  geometry_msgs::msg::Quaternion pose_q_msg = pose.pose.orientation;
  tf2::Quaternion pose_q(pose_q_msg.x, pose_q_msg.y, pose_q_msg.z, pose_q_msg.w);
  tf2::Matrix3x3 pose_m(pose_q);
  double pose_roll, pose_pitch, pose_yaw;
  pose_m.getRPY(pose_roll, pose_pitch, pose_yaw);
  
  return pose_yaw;
}

double PID::computeYaw(const geometry_msgs::msg::PoseStamped& first_pose, const geometry_msgs::msg::PoseStamped& second_pose)
{
  return std::atan2(second_pose.pose.position.y - first_pose.pose.position.y, second_pose.pose.position.x - first_pose.pose.position.x);
}

geometry_msgs::msg::PoseStamped PID::carrotRobotPose(const geometry_msgs::msg::PoseStamped& robot_pose, bool inverse_move)
{
  double robot_yaw = computeYaw(robot_pose);

  geometry_msgs::msg::PoseStamped res = robot_pose;

#ifdef DEBUG
  log << "Robot yaw in carrot = " << robot_yaw << "\n";
  log << "Robot pos x before carrot = " << res.pose.position.x << "\n";
  log << "Robot pos y before carrot = " << res.pose.position.y << "\n";
#endif // DEBUG

  double delta_x = inverse_move ? (-1) * carrot_length_ * std::cos(robot_yaw) : carrot_length_ * std::cos(robot_yaw);
  double delta_y = inverse_move ? (-1) * carrot_length_ * std::sin(robot_yaw) : carrot_length_ * std::sin(robot_yaw);

  res.pose.position.x += delta_x;
  res.pose.position.y += delta_y;

#ifdef DEBUG
  log << "is Inverse now after carrot? = " << inverse_move << "\n";
  log << "Delta x in carrot process = " << delta_x << "\n";
  log << "Delta y in carrot process = " << delta_y << "\n";
  log << "Robot pos x after carrot = " << res.pose.position.x << "\n";
  log << "Robot pos y after carrot = " << res.pose.position.y << "\n";
#endif // DEBUG

  return res;
}