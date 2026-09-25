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

} 
