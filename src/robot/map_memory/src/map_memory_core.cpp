#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}
  void MapMemoryCore::initMap(double resolution, unsigned int width, unsigned int height, int8_t fill_value) {
  global_map_.header.frame_id = "sim_world";

  global_map_.info.resolution = resolution;
  global_map_.info.width = width;
  global_map_.info.height = height;

  // Origin = world position of cell (0,0), the bottom-left corner.
  // Shift by half the map size so the map is centered on (0,0).
  global_map_.info.origin.position.x = -(width * resolution) / 2.0;
  global_map_.info.origin.position.y = -(height * resolution) / 2.0;
  global_map_.info.origin.orientation.w = 1.0;

  // One value per cell, all starting as fill_value
  global_map_.data.assign(static_cast<size_t>(width) * height, fill_value);

  RCLCPP_INFO(logger_, "Global map initialized: %u x %u cells at %.2f m/cell", width, height, resolution);
  }

const nav_msgs::msg::OccupancyGrid& MapMemoryCore::getMap() const {
  return global_map_;
  }

void MapMemoryCore::mergeCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_yaw) {
  const auto& local = costmap.info;
  const auto& global = global_map_.info;

  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  for (unsigned int j = 0; j < local.height; ++j) {      // rows (y)
    for (unsigned int i = 0; i < local.width; ++i) {     // columns (x)
      int8_t value = costmap.data[j * local.width + i];
      if (value < 0) {
        continue;  // unknown: keep what the global map already has
      }

      // Cell center in the costmap's frame (relative to the robot)
      double local_x = local.origin.position.x + (i + 0.5) * local.resolution;
      double local_y = local.origin.position.y + (j + 0.5) * local.resolution;

      // Robot-relative -> world: rotate by robot yaw, then translate by robot position
      double world_x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
      double world_y = robot_y + local_x * sin_yaw + local_y * cos_yaw;

      // World -> global map cell
      int gi = static_cast<int>(std::floor((world_x - global.origin.position.x) / global.resolution));
      int gj = static_cast<int>(std::floor((world_y - global.origin.position.y) / global.resolution));

      if (gi < 0 || gj < 0 || gi >= static_cast<int>(global.width) || gj >= static_cast<int>(global.height)) {
        continue;  // outside the global map
      }

      // Newest data wins
      global_map_.data[gj * global.width + gi] = value;
    }
  }
}

} 
