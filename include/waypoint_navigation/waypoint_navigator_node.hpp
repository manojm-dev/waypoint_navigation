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

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"

/**
 * @struct PathNode
 * @brief A structure to represent a node in the path with its ID and distance.
 * 
 * Used in Dijkstra's algorithm to store the node ID and the distance to the source node.
 */
struct PathNode {
    int id;           ///< ID of the node
    double dist;      ///< Distance from the source node

    /**
     * @brief Comparison operator to compare nodes based on distance.
     * 
     * @param other Another PathNode to compare with
     * @return True if the current node's distance is greater than the other node's distance
     */
    bool operator>(const PathNode& other) const {
        return dist > other.dist;
    }
};

/**
 * @class WaypointNavigator
 * @brief A ROS2 node responsible for navigating through waypoints based on odometry and pathfinding algorithms.
 * 
 * This node integrates waypoint navigation, pathfinding using Dijkstra's algorithm, and goal navigation.
 * It listens for odometry data, computes shortest paths between waypoints, and sends navigation goals.
 */
class WaypointNavigator : public rclcpp::Node
{
public:
    // Action definitions for goal navigation
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /**
     * @brief Constructor for the WaypointNavigator node.
     * Initializes the node and sets up necessary parameters and subscriptions.
     */
    WaypointNavigator();

private:
    /** ---------------------------- Parameters ---------------------------- **/

    std::string odom_topic_;        ///< Topic name for odometry data
    std::string json_path_;         ///< Path to the JSON file with waypoint data
    std::string waypoint_goal_;     ///< Goal waypoint to navigate to
    int waypoint_limit_;            ///< Limit on the number of waypoints
    int num_waypoint_;              ///< Total number of waypoints available

    /** ---------------------------- Waypoint Data ---------------------------- **/

    std::vector<std::string> names_;               ///< Names of the waypoints
    std::vector<int> ids_;                         ///< IDs of the waypoints
    std::vector<std::vector<std::string>> neighbours_;  ///< Adjacency list of neighbours for each waypoint
    std::vector<std::vector<double>> weights_;         ///< Weights (distances) between neighbouring waypoints
    std::vector<double> x_pos_, y_pos_;                 ///< X and Y coordinates of the waypoints

    /** ---------------------------- Subscribers ---------------------------- **/

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  ///< Odometry subscriber to receive robot's current position

    /** ---------------------------- Action Client ---------------------------- **/

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;  ///< Action client to send navigation goals

    /** ---------------------------- Odometry ---------------------------- **/

    geometry_msgs::msg::Pose2D current_location_;  ///< The robot's current location from odometry
    bool odom_received_ = false;                    ///< Flag to indicate if odometry data has been received
    bool goal_done_ = false;                        ///< Flag to check if the goal navigation has been completed
    bool goal_succeeded_ = false;                   ///< Flag to indicate if the goal was successfully reached

    /** ---------------------------- Core Methods ---------------------------- **/

    /**
     * @brief Main callback function to handle waypoint navigation and pathfinding.
     */
    void callback();

    /**
     * @brief Callback function to update the robot's position using odometry data.
     * 
     * @param msg Odometry message containing robot's position.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Waits for odometry data to be received before proceeding.
     */
    void wait_for_odometry();

    /**
     * @brief Loads waypoint data from a JSON file.
     * 
     * @return True if loading was successful, false otherwise.
     */
    bool load_waypoints_from_json();

    /**
     * @brief Computes the Euclidean distance between two points (x1, y1) and (x2, y2).
     * 
     * @param x1 X coordinate of the first point
     * @param y1 Y coordinate of the first point
     * @param x2 X coordinate of the second point
     * @param y2 Y coordinate of the second point
     * @return The Euclidean distance between the two points.
     */
    double distance(double x1, double y1, double x2, double y2);

    /**
     * @brief Calculates the weights (distances) between neighbouring waypoints.
     */
    void calculate_weights();

    /**
     * @brief Finds the nearest waypoint to a given position.
     * 
     * @param x_pos X coordinate of the robot's position
     * @param y_pos Y coordinate of the robot's position
     * @return The ID of the nearest waypoint.
     */
    int find_nearest_waypoint(double x_pos, double y_pos);

    /**
     * @brief Converts waypoint names and their neighbours to indices for easier processing in pathfinding.
     * 
     * @param name_neighbours A list of neighbouring waypoint names for each waypoint
     * @param names A list of all waypoint names
     * @return A 2D vector of indices representing the neighbours of each waypoint.
     */
    std::vector<std::vector<int>> convert_name_to_index(
        const std::vector<std::vector<std::string>>& name_neighbours,
        const std::vector<std::string>& names);

    /**
     * @brief Implements Dijkstra's algorithm to find the shortest path from a start waypoint to a goal waypoint.
     * 
     * @param weights_ A 2D vector of weights (distances) between waypoints
     * @param neighbours_ A 2D vector of indices representing the neighbours of each waypoint
     * @param start The starting waypoint's index
     * @param goal The goal waypoint's index
     * @return A vector of indices representing the shortest path from start to goal.
     */
    std::vector<int> dijkstra(
        const std::vector<std::vector<double>>& weights_,
        const std::vector<std::vector<int>>& neighbours_,
        int start, int goal);

    /**
     * @brief Creates a PoseStamped message for a given waypoint's coordinates and orientation.
     * 
     * @param x X coordinate of the goal
     * @param y Y coordinate of the goal
     * @param yaw Orientation (yaw angle) of the goal
     * @return A PoseStamped message representing the goal pose.
     */
    geometry_msgs::msg::PoseStamped create_pose(double x, double y, double yaw);

    /**
     * @brief Sends a navigation goal to the action server to navigate the robot to the specified goal pose.
     * 
     * @param goal_pose The goal pose to navigate to
     * @return True if the goal was successfully sent, false otherwise.
     */
    bool nav_to_goal(const geometry_msgs::msg::PoseStamped &goal_pose);

    /**
     * @brief Callback function to handle the result of the navigation goal once the robot reaches the goal.
     * 
     * @param result The result of the navigation action
     */
    void result_callback(const GoalHandleNavigateToPose::WrappedResult &result);
};

#endif  // WAYPOINT_NAVIGATION__WAYPOINT_NAVIGATOR_NODE_HPP
