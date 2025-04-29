#ifndef WAYPOINT_NAVIGATION__WAYPOINT_NAVIGATOR_NODE_HPP
#define WAYPOINT_NAVIGATION__WAYPOINT_NAVIGATOR_NODE_HPP

#include <string>
#include <vector>
#include <fstream>
#include <cmath>
#include <limits>
#include <queue>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"

struct PathNode {
    int id;
    double dist;
    bool operator>(const PathNode& other) const {
        return dist > other.dist;
    }
};

class WaypointNavigator : public rclcpp::Node
{
public:
    // Actions
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WaypointNavigator();

private:
    // Parameters
    std::string odom_topic_, json_path_, waypoint_goal_;
    int waypoint_limit_, num_waypoint_;

    // Vectors for waypoint data
    std::vector<std::string> names_;
    std::vector<int> ids_;
    std::vector<std::vector<std::string>> neighbours_;
    std::vector<std::vector<double>> weights_;
    std::vector<double> x_pos_, y_pos_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Action
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    // Odometry
    geometry_msgs::msg::Pose2D current_location_;
    bool odom_received_ = false;
    bool goal_done_ = false;
    bool goal_succeeded_ = false;


    // Core methods
    void callback();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void wait_for_odometry();
    bool load_waypoints_from_json();
    double distance(double x1, double y1, double x2, double y2);
    void calculate_weights();
    int find_nearest_waypoint(double x_pos, double y_pos);
    std::vector<std::vector<int>> convert_name_to_index(
        const std::vector<std::vector<std::string>>& name_neighbours,
        const std::vector<std::string>& names);
    std::vector<int> dijkstra(
        const std::vector<std::vector<double>>& weights_,
        const std::vector<std::vector<int>>& neighbours_,
        int start, int goal);
    geometry_msgs::msg::PoseStamped create_pose(double x, double y, double yaw);
    bool nav_to_goal(const geometry_msgs::msg::PoseStamped &goal_pose);
    void result_callback(const GoalHandleNavigateToPose::WrappedResult &result);
};

#endif  // WAYPOINT_NAVIGATION__WAYPOINT_NAVIGATOR_NODE_HPP
