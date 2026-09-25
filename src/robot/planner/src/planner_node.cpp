#include <chrono>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // Parameters
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  goal_timeout_ = this->declare_parameter<double>("goal_timeout", 60.0);
  int obstacle_threshold = this->declare_parameter<int>("obstacle_threshold", 50);
  double cost_weight = this->declare_parameter<double>("cost_weight", 1.0);
  planner_.setParameters(obstacle_threshold, cost_weight);
  no_progress_timeout_ = this->declare_parameter<double>("no_progress_timeout", 10.0);
  progress_epsilon_ = this->declare_parameter<double>("progress_epsilon", 0.2);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer: checks goal reached / timeout (Step 3) and replanning (Step 6)
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

// Store the latest global map from map memory. Replanning happens in the timer,
// so several maps arriving between ticks still cause only one replan.
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  have_map_ = true;
  map_updated_ = true;
}

// New goal clicked in Foxglove: WAITING_FOR_GOAL -> WAITING_FOR_ROBOT_TO_REACH_GOAL
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (msg->header.frame_id != "sim_world") {
    RCLCPP_WARN(this->get_logger(), "Goal is in frame '%s', expected 'sim_world'", msg->header.frame_id.c_str());
  }

  goal_ = *msg;
  goal_start_time_ = this->now();
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  resetProgress();
  map_updated_ = false;  // we're about to plan on the latest map anyway

  RCLCPP_INFO(this->get_logger(), "New goal (%.2f, %.2f), %.2f m away. State: WAITING_FOR_ROBOT_TO_REACH_GOAL",
    goal_.point.x, goal_.point.y, distanceToGoal());

  planAndPublish();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
}

// Periodic checks while driving to a goal
void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL || !have_odom_) {
    return;
  }

  double distance = distanceToGoal();
  double elapsed = (this->now() - goal_start_time_).seconds();

  if (distance < goal_tolerance_) {
    RCLCPP_INFO(this->get_logger(), "Goal reached (%.2f m away). State: WAITING_FOR_GOAL", distance);
    publishEmptyPath();
    state_ = State::WAITING_FOR_GOAL;
    return;
  }

  if (elapsed > goal_timeout_) {
    RCLCPP_WARN(this->get_logger(), "Goal timed out after %.0f s (%.2f m away). State: WAITING_FOR_GOAL", elapsed, distance);
    publishEmptyPath();
    state_ = State::WAITING_FOR_GOAL;
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "Driving to goal: %.2f m left, %.0f s elapsed", distance, elapsed);

  // Progress = getting meaningfully closer than the best distance so far
  if (distance < best_distance_ - progress_epsilon_) {
    best_distance_ = distance;
    last_progress_time_ = this->now();
  }
  bool no_progress = (this->now() - last_progress_time_).seconds() > no_progress_timeout_;

  if (map_updated_ || no_progress) {
    if (no_progress) {
      RCLCPP_WARN(this->get_logger(), "No progress for %.0f s, replanning", no_progress_timeout_);
      resetProgress();  // give the new plan a fresh window
    } else {
      RCLCPP_INFO(this->get_logger(), "Map updated, replanning");
    }
    map_updated_ = false;
    planAndPublish();
  }
}

// Start a fresh "is the robot getting closer?" window
void PlannerNode::resetProgress() {
  best_distance_ = distanceToGoal();
  last_progress_time_ = this->now();
}

// Plan from the robot to the goal with A* on the latest map, and publish it
void PlannerNode::planAndPublish() {
  if (!have_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry yet, can't plan");
    return;
  }
  if (!have_map_) {
    RCLCPP_WARN(this->get_logger(), "No map yet, can't plan");
    return;
  }

  std::vector<std::pair<double, double>> waypoints;
  if (!planner_.planPath(current_map_, robot_x_, robot_y_, goal_.point.x, goal_.point.y, waypoints)) {
    // No path: stop the robot. We stay in WAITING_FOR_ROBOT_TO_REACH_GOAL, so the
    // goal can still be reached after a replan (Step 6) or give up at the timeout.
    publishEmptyPath();
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "sim_world";

  for (const auto & [x, y] : waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  path_pub_->publish(path);
  RCLCPP_INFO(this->get_logger(), "Published A* path with %zu poses", path.poses.size());
}

// An empty path tells the control node to stop
void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "sim_world";
  path_pub_->publish(path);
}

double PlannerNode::distanceToGoal() const {
  return std::hypot(goal_.point.x - robot_x_, goal_.point.y - robot_y_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
