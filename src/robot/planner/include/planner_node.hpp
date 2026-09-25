#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // Helpers
    void planAndPublish();
    void publishEmptyPath();
    void resetProgress();
    double distanceToGoal() const;

    robot::PlannerCore planner_;

    // Subscribers, publisher, timer
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State machine
    State state_ = State::WAITING_FOR_GOAL;
    geometry_msgs::msg::PointStamped goal_;
    rclcpp::Time goal_start_time_;

    // Latest map (used by A* in Step 5)
    nav_msgs::msg::OccupancyGrid current_map_;
    bool have_map_ = false;
    bool map_updated_ = false;   // set by mapCallback, cleared when the timer replans

    // Progress tracking (replan if the robot stops getting closer)
    double best_distance_ = 0.0;
    rclcpp::Time last_progress_time_;

    // Robot position
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool have_odom_ = false;

    // Parameters
    double goal_tolerance_ = 0.5;   // m
    double goal_timeout_ = 120.0;   // s
    double no_progress_timeout_ = 10.0;  // s without getting closer before replanning
    double progress_epsilon_ = 0.2;      // m closer that counts as progress
};

#endif
