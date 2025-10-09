#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "nav2_util/geometry_utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

nav_msgs::msg::Path current_path;
geometry_msgs::msg::PoseStamped robot_pose, start_pose, close_pose;
double path_yaw, close_yaw;
float max_robot_pose_search_dist_ = 0.1;
double e_y_prev;
double v_max = 0.2;
double integral_e_y = 0;
double k_p = 1.0;
double k_i = 0.0;
double k_d = 0.0;

double wrapToPi(double theta)
{
  theta = std::fmod(theta + M_PI, 2.0 * M_PI);
  if (theta < 0)
      theta += 2.0 * M_PI;
  return theta - M_PI;
}

void computeSegment()
{
  using nav2_util::geometry_utils::euclidean_distance;

  if (current_path.poses.size() != 0)
  {
    auto begin = current_path.poses.begin();

    // Limit the search for the closest pose up to max_robot_pose_search_dist on the path
    auto closest_pose_upper_bound =
      nav2_util::geometry_utils::first_after_integrated_distance(
      current_path.poses.begin(), current_path.poses.end(),
      max_robot_pose_search_dist_);

    // Find closest point to the robot
    auto start_poser = nav2_util::geometry_utils::min_by(
      begin, closest_pose_upper_bound,
      [](const geometry_msgs::msg::PoseStamped & ps) {
        return euclidean_distance(robot_pose, ps);
      });

    close_pose = *(current_path.poses.end() - 1);

    start_pose = *start_poser;

    path_yaw = std::atan2(close_pose.pose.position.y - start_pose.pose.position.y, close_pose.pose.position.x - start_pose.pose.position.x);
      
    geometry_msgs::msg::Quaternion close_q_msg = close_pose.pose.orientation;
    tf2::Quaternion close_q(close_q_msg.x, close_q_msg.y, close_q_msg.z, close_q_msg.w);
    tf2::Matrix3x3 close_m(close_q);
    double closest_roll, closest_pitch, closest_yaw;
    close_m.getRPY(closest_roll, closest_pitch, closest_yaw);
    close_yaw = closest_yaw;
  }
}

void computeVel()
{
  double dt = 0.1;

  if (current_path.poses.size() != 0)
  {
    geometry_msgs::msg::Quaternion robot_q_msg = robot_pose.pose.orientation;
    tf2::Quaternion robot_q(robot_q_msg.x, robot_q_msg.y, robot_q_msg.z, robot_q_msg.w);
    tf2::Matrix3x3 robot_m(robot_q);
    double robot_roll, robot_pitch, robot_yaw;
    robot_m.getRPY(robot_roll, robot_pitch, robot_yaw);

    double e_y = -(robot_pose.pose.position.x - start_pose.pose.position.x) * std::sin(path_yaw) +
                  (robot_pose.pose.position.y - start_pose.pose.position.y) * std::cos(path_yaw);

    std::cout << "e_y " << e_y << std::endl;

    double e_theta = wrapToPi(std::abs(robot_yaw) - std::abs(close_yaw));
    std::cout << "e_theta = " << e_theta << std::endl;

    double de_y = (e_y - e_y_prev) / dt;
    integral_e_y += e_y * dt;
    double omega = k_p * e_y + k_d * de_y + k_i * integral_e_y;

    double v = v_max * std::cos(e_theta);
    std::cout << "V = " << v << std::endl;
    std::cout << "Omega = " << omega << std::endl;

    // result_vel.twist.linear.x = v;
    // result_vel.twist.angular.z = omega;

    e_y_prev = e_y;
  }
}

void plan_callback(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
  current_path = *msg;
}

void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  robot_pose.pose.position = msg->pose.pose.position;
  robot_pose.pose.orientation = msg->pose.pose.orientation;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("controller_dopler");

  auto timer_ = node->create_wall_timer(std::chrono::milliseconds(100), computeVel);
  auto path_sub = node->create_subscription<nav_msgs::msg::Path>("plan", rclcpp::QoS(10), plan_callback);  
  auto pose_sub = node->create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", rclcpp::QoS(10), pose_callback);  

  rclcpp::spin(node);

  return 0;
}