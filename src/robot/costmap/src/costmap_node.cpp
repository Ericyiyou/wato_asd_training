#include <chrono>
#include <memory>
#include <algorithm>
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
  width_ = 200;             // 200 cells * 0.1 m = 20 m (+/-10 m around the robot)
  height_ = 200;
  origin_x_ = -10.0;        // robot (lidar) at the center
  origin_y_ = -10.0;

  inflation_radius_ = 1.5;  // m: robot is 2 m x 1 m, so it needs ~1.1 m+ of clearance
  max_cost_ = 100;

  // Start unknown (-1): only cells a lidar beam actually passes through become free.
  // Otherwise space behind obstacles would be reported as free, and map memory would
  // overwrite obstacles it remembers with that fake free space.
  grid_.assign(height_, std::vector<int8_t>(width_, -1));
}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  // floor, not truncation, so negative coordinates land in the right cell
  x_grid = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  y_grid = static_cast<int>(std::floor((y - origin_y_) / resolution_));
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
    grid_[y_grid][x_grid] = 100;
  }
}

void CostmapNode::markFree(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_ && grid_[y_grid][x_grid] == -1) {
    grid_[y_grid][x_grid] = 0;
  }
}

// Mark every cell the beam passes through (from the lidar out to `range`) as free
void CostmapNode::raytraceFree(double range, double angle) {
  const double step = resolution_ / 2.0;  // half-cell steps so no cell is skipped
  for (double r = 0.0; r < range; r += step) {
    int x_grid, y_grid;
    convertToGrid(r, angle, x_grid, y_grid);
    markFree(x_grid, y_grid);
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
        if (cost > 0 && cost > grid_[ny][nx]) {
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
  msg.info.origin.orientation.w = 1.0;  // no rotation (an all-zero quaternion is invalid)

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

  // The window's corner is ~14 m away, so there's no point tracing beams further
  const double max_trace = std::hypot(width_ * resolution_, height_ * resolution_) / 2.0;

  // Pass 1: free space along every beam
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (std::isnan(range)) {
      continue;  // no information
    }
    if (range > scan->range_min && range < scan->range_max) {
      raytraceFree(range - resolution_, angle);  // stop just short of the hit
    } else if (range >= scan->range_max) {
      raytraceFree(std::min(static_cast<double>(scan->range_max), max_trace), angle);  // inf: nothing hit, all free
    }
  }

  // Pass 2: obstacles (after free space, so a hit always wins over a neighbouring beam's free cell)
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