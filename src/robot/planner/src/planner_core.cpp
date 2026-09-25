#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

void PlannerCore::setParameters(int obstacle_threshold, double cost_weight) {
  obstacle_threshold_ = obstacle_threshold;
  cost_weight_ = cost_weight;
}

bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double x, double y, int& gx, int& gy) const {
  gx = static_cast<int>(std::floor((x - map.info.origin.position.x) / map.info.resolution));
  gy = static_cast<int>(std::floor((y - map.info.origin.position.y) / map.info.resolution));
  return gx >= 0 && gy >= 0 && gx < static_cast<int>(map.info.width) && gy < static_cast<int>(map.info.height);
}

bool PlannerCore::isBlocked(const nav_msgs::msg::OccupancyGrid& map, int gx, int gy) const {
  if (gx < 0 || gy < 0 || gx >= static_cast<int>(map.info.width) || gy >= static_cast<int>(map.info.height)) {
    return true;  // outside the map
  }
  int8_t value = map.data[gy * map.info.width + gx];
  return value >= obstacle_threshold_;  // unknown (-1) counts as free
}

bool PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map,
                           double start_x, double start_y,
                           double goal_x, double goal_y,
                           std::vector<std::pair<double, double>>& path) {
  path.clear();

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double res = map.info.resolution;

  int sx, sy, gx, gy;
  if (!worldToGrid(map, start_x, start_y, sx, sy)) {
    RCLCPP_WARN(logger_, "A*: start (%.2f, %.2f) is outside the map", start_x, start_y);
    return false;
  }
  if (!worldToGrid(map, goal_x, goal_y, gx, gy)) {
    RCLCPP_WARN(logger_, "A*: goal (%.2f, %.2f) is outside the map", goal_x, goal_y);
    return false;
  }
  if (isBlocked(map, gx, gy)) {
    RCLCPP_WARN(logger_, "A*: goal (%.2f, %.2f) is inside an obstacle", goal_x, goal_y);
    return false;
  }
  // The start cell is not checked: the robot may be sitting in an inflated cell
  // next to a wall, and it still needs a way out.

  const int start = sy * width + sx;
  const int goal = gy * width + gx;

  // h(n): straight-line (Euclidean) distance to the goal, in meters.
  // Never overestimates with 8-direction moves, so A* stays optimal.
  auto heuristic = [&](int idx) {
    return std::hypot(idx % width - gx, idx / width - gy) * res;
  };

  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> g_cost(width * height, INF);   // g(n): best known cost from start
  std::vector<int> came_from(width * height, -1);    // parent of each cell on the best path
  std::vector<bool> closed(width * height, false);   // closed list: already expanded

  // Open list: (f, cell), smallest f first
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

  g_cost[start] = 0.0;
  open.push({heuristic(start), start});

  // 8 neighbors: 4 straight (cost 1 cell) + 4 diagonal (cost sqrt(2) cells)
  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  // Can we step from cell (fx, fy) into cell (tx, ty)?
  // Normally only unblocked cells. But if the robot starts inside an inflated zone
  // (e.g. pushed up against a box), every neighbor is blocked and it would be trapped.
  // So from a blocked cell we may also step to a non-lethal cell (< 100) that is no
  // more costly: the robot can back out down the cost gradient, never deeper in.
  auto passable = [&](int fx, int fy, int tx, int ty) {
    if (!isBlocked(map, tx, ty)) {
      return true;
    }
    if (tx < 0 || ty < 0 || tx >= width || ty >= height || !isBlocked(map, fx, fy)) {
      return false;
    }
    int8_t from_value = map.data[fy * width + fx];
    int8_t to_value = map.data[ty * width + tx];
    return to_value < 100 && to_value <= from_value;
  };

  while (!open.empty()) {
    int current = open.top().second;
    open.pop();

    if (closed[current]) {
      continue;  // stale entry: this cell was already expanded with a better cost
    }
    closed[current] = true;

    if (current == goal) {
      break;
    }

    int cx = current % width;
    int cy = current / width;

    for (int k = 0; k < 8; ++k) {
      int nx = cx + dx[k];
      int ny = cy + dy[k];
      if (!passable(cx, cy, nx, ny)) {
        continue;
      }
      // No corner cutting: a diagonal move needs both side cells passable
      if (dx[k] != 0 && dy[k] != 0 &&
          (!passable(cx, cy, cx + dx[k], cy) || !passable(cx, cy, cx, cy + dy[k]))) {
        continue;
      }

      int neighbor = ny * width + nx;
      if (closed[neighbor]) {
        continue;
      }

      // Step cost = distance, plus a penalty for high-cost (inflated) cells
      double step = (dx[k] != 0 && dy[k] != 0) ? std::sqrt(2.0) * res : res;
      int8_t value = map.data[neighbor];
      double penalty = (value > 0) ? cost_weight_ * (value / 100.0) * step : 0.0;
      double tentative_g = g_cost[current] + step + penalty;

      if (tentative_g < g_cost[neighbor]) {
        g_cost[neighbor] = tentative_g;
        came_from[neighbor] = current;
        open.push({tentative_g + heuristic(neighbor), neighbor});   // f = g + h
      }
    }
  }

  if (!closed[goal]) {
    RCLCPP_WARN(logger_, "A*: no path found to (%.2f, %.2f)", goal_x, goal_y);
    return false;
  }

  // Walk back from goal to start, then reverse
  for (int idx = goal; idx != -1; idx = came_from[idx]) {
    double wx = map.info.origin.position.x + (idx % width + 0.5) * res;
    double wy = map.info.origin.position.y + (idx / width + 0.5) * res;
    path.emplace_back(wx, wy);
  }
  std::reverse(path.begin(), path.end());

  // End exactly on the requested goal rather than its cell center
  path.back() = {goal_x, goal_y};
  return true;
}

}
