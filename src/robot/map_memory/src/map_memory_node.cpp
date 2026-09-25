#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Parameters
  distance_threshold_ = this->declare_parameter<double>("distance_threshold", 1.5);
  double map_resolution = this->declare_parameter<double>("map_resolution", 0.1);
  int map_width = this->declare_parameter<int>("map_width", 300);
  int map_height = this->declare_parameter<int>("map_height", 300);

  // Empty global map
  map_memory_.initMap(map_resolution, static_cast<unsigned int>(map_width), static_cast<unsigned int>(map_height));

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received costmap: %u x %u", msg->info.width, msg->info.height);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  const auto & q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0-2.0 * (q.y * q.y+q.z*q.z));

  double distance = std::hypot(robot_x_ - last_x_, robot_y_ - last_y_);
  if (distance >= distance_threshold_) {
    last_x_ = robot_x_;
    last_y_ = robot_y_;
    should_update_map_ = true;
  }
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Odom: x=%.2f y=%.2f yaw=%.2f", robot_x_, robot_y_, robot_yaw_);
}

void MapMemoryNode::updateMap() {
  if (!should_update_map_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Update triggered at (%.2f, %.2f)", last_x_, last_y_);

  // (Step 5: merge the latest costmap here)

  auto map = map_memory_.getMap();
  map.header.stamp = this->now();
  map_pub_->publish(map);

  should_update_map_ = false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
