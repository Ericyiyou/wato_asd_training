#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Cells with cost >= obstacle_threshold are blocked. Unknown (-1) cells are
    // treated as free, so we can plan through space the robot hasn't seen yet.
    // cost_weight adds a penalty for driving through high-cost (inflated) cells.
    void setParameters(int obstacle_threshold, double cost_weight);

    // A* from start to goal (world coordinates) on the occupancy grid.
    // On success, fills `path` with world (x, y) waypoints and returns true.
    bool planPath(const nav_msgs::msg::OccupancyGrid& map,
                  double start_x, double start_y,
                  double goal_x, double goal_y,
                  std::vector<std::pair<double, double>>& path);

  private:
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double x, double y, int& gx, int& gy) const;
    bool isBlocked(const nav_msgs::msg::OccupancyGrid& map, int gx, int gy) const;

    rclcpp::Logger logger_;
    int obstacle_threshold_ = 25;
    double cost_weight_ = 1.0;
};

}

#endif
