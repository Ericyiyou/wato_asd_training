#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    void initMap(double resolution, unsigned int width, unsigned int height, int8_t fill_value = -1);

    const nav_msgs::msg::OccupancyGrid& getMap() const;

    // Copy known cells from a robot-relative costmap into the global map
    void mergeCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_yaw);

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
};

}  

#endif  
