#include <chrono>
#include <memory>
#include <cmath>
#include "costmap_node.hpp"

using namespace std;
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  lidar_sub_= this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10,
  std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::initializeCostmap() {
  resolution_ = 0.1;       
  width_ = 30;             
  height_ = 30;            
  origin_x_ = -1.5;         
  origin_y_ = -1.5;

  inflation_radius_ = 1.0;
  max_cost_ = 100;

  grid_.assign(height_, std::vector<int8_t>(width_, 0));
}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  x_grid = static_cast<int>((x - origin_x_) / resolution_);
  y_grid = static_cast<int>((y - origin_y_) / resolution_);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
    grid_[y_grid][x_grid] = 100;
  }
}

void CostmapNode::inflateObstacles() {
  int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);

  std::vector<std::pair<int, int>> obstacles;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (grid_[y][x] == 100) {
        obstacles.emplace_back(x, y);
      }
    }
  }
    for (const auto &obs : obstacles) {
    int ox = obs.first, oy = obs.second;

    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        int nx = ox + dx, ny = oy + dy;
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

        double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
        if (distance > inflation_radius_) continue;

        int8_t cost = static_cast<int8_t>(max_cost_ * (1.0 - (distance / inflation_radius_)));
        if (cost > grid_[ny][nx]) {
          grid_[ny][nx] = cost;
        }
      }
    }
  }
}

void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "robot/chassis/lidar"; 

  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;

  msg.data.resize(width_ * height_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      msg.data[y * width_ + x] = grid_[y][x];
    }
  }

  costmap_pub_->publish(msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initializeCostmap();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min) {
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }

  inflateObstacles();
  publishCostmap();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}