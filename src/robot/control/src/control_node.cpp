#include "control_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <optional>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  lookahead_distance_ = 1.0;
  goal_tolerance_ = 0.1;
  linear_speed_ = 0.5;

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() { controlLoop(); });
}

void ControlNode::controlLoop() //Edge cases
{
  if (!current_path_ || !robot_odom_) {
    return;
  }

  // Empty path (planner cleared it) or goal reached: stop once, then forget the path.
  // The sim keeps executing the last /cmd_vel, so an explicit stop is required.
  // Clearing the path means we don't keep publishing zeros (which would block teleop).
  if (current_path_->poses.empty()) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    current_path_.reset();
    return;
  }

  const auto &robot_position = robot_odom_->pose.pose.position;
  const auto &goal_position = current_path_->poses.back().pose.position;
  double distance_to_goal = computeDistance(robot_position, goal_position);

  if (distance_to_goal < goal_tolerance_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    current_path_.reset();
    return;
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    return;
  }

  auto cmd_vel = computeVelocity(*lookahead_point);
  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  if (current_path_->poses.empty()) {
    return std::nullopt;
  }

  const auto &robot_position = robot_odom_->pose.pose.position;
  const auto &poses = current_path_->poses;

  // Start searching from the path point closest to the robot, so we never
  // chase points the robot has already passed (e.g. the start of the path)
  size_t closest = 0;
  double closest_distance = computeDistance(robot_position, poses[0].pose.position);
  for (size_t i = 1; i < poses.size(); ++i) {
    double distance = computeDistance(robot_position, poses[i].pose.position);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest = i;
    }
  }

  for (size_t i = closest; i < poses.size(); ++i) {
    double distance = computeDistance(robot_position, poses[i].pose.position);
    if (distance >= lookahead_distance_) {
      return poses[i];
    }
  }

  return poses.back();
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd_vel;

  const auto &robot_position = robot_odom_->pose.pose.position;
  double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

  double dx = target.pose.position.x - robot_position.x;
  double dy = target.pose.position.y - robot_position.y;

  // Rotate the world-frame offset into the robot's own frame of reference
  double local_x = dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);
  double local_y = -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);

  // Target far off to the side or behind: pure pursuit would drive away from it,
  // so rotate in place toward it first
  double heading_error = std::atan2(local_y, local_x);
  if (std::abs(heading_error) > M_PI / 4.0) {
    // Pick a turn direction once and stick to it until we're facing the target.
    // With the target almost directly behind, the error flips between +180 and -180 deg
    // on tiny movements; re-deciding every tick makes the robot wiggle in place.
    if (turn_direction_ == 0) {
      turn_direction_ = (heading_error > 0.0) ? 1 : -1;
    }
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = turn_direction_ * 1.0;
    return cmd_vel;
  }
  turn_direction_ = 0;  // facing the target: back to normal pursuit

  double distance_to_target = std::hypot(local_x, local_y);
  if (distance_to_target < 1e-6) {
    return cmd_vel;  // on top of the target: nothing to steer toward
  }
  double curvature = (2.0 * local_y) / (distance_to_target * distance_to_target);

  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = linear_speed_ * curvature;

  return cmd_vel;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}