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

#include "waypoint_navigation/waypoint_navigator_node.hpp"

using json = nlohmann::json;

int main(int argc, char * argv[])
{
    // Initialize ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a shared pointer to the WaypointNavigator node and start spinning (process callbacks)
    rclcpp::spin(std::make_shared<WaypointNavigator>());

    // Shutdown the ROS 2 system gracefully
    rclcpp::shutdown();

    // Return 0 to indicate successful execution
    return 0;
}

WaypointNavigator::WaypointNavigator()
: Node("waypoint_navigator")  // Initialize node with name "waypoint_navigator"
{
    // Get full path to the default waypoint JSON file inside the package
    std::string package_path = ament_index_cpp::get_package_share_directory("waypoint_navigation");
    std::string default_json_path = package_path + "/data/waypoints.json";

    // Declare ROS parameters with default values
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("waypoint_limit", 100);
    this->declare_parameter("json_path", default_json_path);
    this->declare_parameter("waypoint_goal", "F");

    // Retrieve parameter values and store them in member variables
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    waypoint_limit_ = this->get_parameter("waypoint_limit").as_int();
    json_path_ = this->get_parameter("json_path").as_string();
    waypoint_goal_ = this->get_parameter("waypoint_goal").as_string();

    // Pre-allocate space in vectors based on max waypoint limit
    names_.resize(waypoint_limit_);
    ids_.resize(waypoint_limit_);
    neighbours_.resize(waypoint_limit_);
    x_pos_.resize(waypoint_limit_);
    y_pos_.resize(waypoint_limit_);
    weights_.resize(waypoint_limit_);

    // Subscribe to odometry topic to get robot position updates
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&WaypointNavigator::odom_callback, this, std::placeholders::_1)
    );

    // Create action client to send navigation goals
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Log that the node has started
    RCLCPP_INFO(this->get_logger(), "Waypoint navigator node started");

    // Call custom callback function to maybe load/process waypoints
    callback();
}


void WaypointNavigator::callback()
{
    // Wait until odometry is received and store robot's current position
    wait_for_odometry();
    double robot_x = current_location_.x;
    double robot_y = current_location_.y;

    RCLCPP_INFO(this->get_logger(), "Current robot position: x = %.2f, y = %.2f", robot_x, robot_y);

    // Load waypoint data from JSON file
    if (!load_waypoints_from_json()) {
        RCLCPP_ERROR(this->get_logger(), "Could not load waypoints from file: %s", json_path_.c_str());
        return;
    }

    // Find the closest waypoint to the robot's current position
    int current_waypoint_id = find_nearest_waypoint(robot_x, robot_y);
    std::string current_waypoint_name = names_[current_waypoint_id];

    RCLCPP_INFO(this->get_logger(), "Closest waypoint: %s (ID: %d)", current_waypoint_name.c_str(), current_waypoint_id + 1);

    // Compute edge weights for pathfinding
    calculate_weights();

    // Convert goal waypoint name to index
    int goal_index = -1;
    for (int i = 0; i < names_.size(); ++i) {
        if (names_[i] == waypoint_goal_) {
            goal_index = i;
            break;
        }
    }

    // Fail early if goal not found
    if (goal_index == -1) {
        RCLCPP_ERROR(this->get_logger(), "Goal waypoint '%s' not found in waypoint list.", waypoint_goal_.c_str());
        return;
    }

    // Convert string-based neighbor names to indices
    auto index_neighbours = convert_name_to_index(neighbours_, names_);

    // Run Dijkstra's algorithm to find shortest path from current to goal
    std::vector<int> path = dijkstra(weights_, index_neighbours, current_waypoint_id, goal_index);

    // Navigate through each waypoint in the planned path
    for (int i = 0; i < path.size(); ++i)
    {
        int index = path[i];
        double x = x_pos_[index];
        double y = y_pos_[index];
        double yaw = 0.0;  // Assuming facing forward for now

        std::string wp_name = names_[index];

        RCLCPP_INFO(this->get_logger(),
            "[%d/%zu] Navigating to waypoint '%s' at (x = %.2f, y = %.2f)",
            i + 1, path.size(), wp_name.c_str(), x, y);

        // Create and send navigation goal
        geometry_msgs::msg::PoseStamped goal_pose = create_pose(x, y, yaw);

        if (!nav_to_goal(goal_pose)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal to waypoint '%s'", wp_name.c_str());
            return;
        }

        // Wait until goal result is received
        RCLCPP_DEBUG(this->get_logger(), "Waiting for goal result...");
        while (!goal_done_ && rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        if (goal_succeeded_) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint '%s' successfully.", wp_name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Navigation to waypoint '%s' failed.", wp_name.c_str());
            return;
        }

        // Reset flags for next goal
        goal_done_ = false;
        goal_succeeded_ = false;
    }

    RCLCPP_INFO(this->get_logger(), "Navigation sequence complete.");
}


geometry_msgs::msg::PoseStamped WaypointNavigator::create_pose(double x, double y, double yaw)
{
    geometry_msgs::msg::PoseStamped pose_msg;

    // Set frame to 'map' for global navigation
    pose_msg.header.frame_id = "map";

    // Timestamp the pose with current time
    pose_msg.header.stamp = this->get_clock()->now();

    // Set position coordinates (z is flat)
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = 0.0;

    // Convert yaw angle to quaternion for orientation
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);  // Roll = 0, Pitch = 0, Yaw = input

    // Set orientation from quaternion
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    return pose_msg;
}


bool WaypointNavigator::load_waypoints_from_json()
{
    // Open the JSON file from the specified path
    std::ifstream file(json_path_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open JSON file: %s", json_path_.c_str());
        return false;
    }

    // Parse the JSON content safely
    json data;
    try {
        file >> data;
    } catch (const json::parse_error &e) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing JSON: %s", e.what());
        return false;
    }

    // Limit number of waypoints based on the configured max
    num_waypoint_ = std::min(static_cast<int>(data.size()), waypoint_limit_);

    // Resize internal storage containers accordingly
    names_.resize(num_waypoint_);
    ids_.resize(num_waypoint_);
    neighbours_.resize(num_waypoint_);
    x_pos_.resize(num_waypoint_);
    y_pos_.resize(num_waypoint_);
    weights_.resize(num_waypoint_);

    // Populate waypoint data
    for (int i = 0; i < num_waypoint_; ++i)
    {
        const auto& wp = data[i];
        
        ids_[i] = wp["id"];
        names_[i] = wp["name"];
        x_pos_[i] = wp["x"];
        y_pos_[i] = wp["y"];

        // Copy neighbour names for this waypoint
        for (const auto& n : wp["neighbours"]) {
            neighbours_[i].push_back(n);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Successfully loaded %d waypoints from JSON.", num_waypoint_);
    return true;
}


void WaypointNavigator::calculate_weights()
{
    // Iterate over each waypoint
    for (int a = 0; a < num_waypoint_; ++a)
    {
        const std::vector<std::string>& neighbour_list = neighbours_[a];
        weights_[a].resize(neighbour_list.size());  // Resize weight vector for current waypoint
        
        // Iterate over each neighbour for the current waypoint
        for (int b = 0; b < neighbour_list.size(); ++b)
        {
            const std::string& neighbour_name = neighbour_list[b];

            // Find the index of the neighbour in the names list
            for (int c = 0; c < names_.size(); ++c)
            {
                if (neighbour_name == names_[c])
                {
                    // Calculate distance between the current waypoint and the neighbour
                    weights_[a][b] = distance(x_pos_[a], y_pos_[a], x_pos_[c], y_pos_[c]);

                    // Log the distance between waypoints for debugging (optional in production)
                    RCLCPP_DEBUG(this->get_logger(),
                        "Distance between %s(%.2f, %.2f) and %s(%.2f, %.2f) is %.2f",
                        names_[a].c_str(), x_pos_[a], y_pos_[a],
                        names_[c].c_str(), x_pos_[c], y_pos_[c],
                        weights_[a][b]);
                }
            }
        }
    }
}


int WaypointNavigator::find_nearest_waypoint(double x, double y)
{
    // Initialize variables to track the closest waypoint
    double min_distance = std::numeric_limits<double>::max();  // Start with a very high value
    int min_index = -1;  // Will store the index of the nearest waypoint

    // Iterate over all waypoints to find the nearest one
    for (int i = 0; i < num_waypoint_; ++i) {
        // Calculate distance to current waypoint
        double d = distance(x, y, x_pos_[i], y_pos_[i]);

        // Update the closest waypoint if this one is closer
        if (d < min_distance) {
            min_distance = d;
            min_index = i;
        }
    }

    // Return the index of the closest waypoint
    return min_index;
}


bool WaypointNavigator::nav_to_goal(const geometry_msgs::msg::PoseStamped &goal_pose)
{
    // Wait for the action server to be available (timeout: 10 seconds)
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return false;
    }

    // Set up the goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    // Set result callback and send the goal asynchronously
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&WaypointNavigator::result_callback, this, std::placeholders::_1);
    nav_client_->async_send_goal(goal_msg, send_goal_options);

    return true;  // Goal sent successfully
}


void WaypointNavigator::result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
{
    goal_done_ = true;  // Mark goal as done

    // Handle different result codes from the action server
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        goal_succeeded_ = true;
        RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
        break;
    case rclcpp_action::ResultCode::ABORTED:
        goal_succeeded_ = false;
        RCLCPP_ERROR(this->get_logger(), "Goal aborted!");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        goal_succeeded_ = false;
        RCLCPP_WARN(this->get_logger(), "Goal canceled!");
        break;
    default:
        goal_succeeded_ = false;
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
}


std::vector<std::vector<int>> WaypointNavigator::convert_name_to_index(
    const std::vector<std::vector<std::string>>& name_neighbours,
    const std::vector<std::string>& names)
{
    std::vector<std::vector<int>> index_neighbours(name_neighbours.size());

    // Iterate over each waypoint's neighbours
    for (size_t i = 0; i < name_neighbours.size(); ++i) {
        // Convert each neighbour name to its index
        for (const std::string& neigh_name : name_neighbours[i]) {
            auto it = std::find(names.begin(), names.end(), neigh_name);
            if (it != names.end()) {
                // Found the neighbour, store its index
                index_neighbours[i].push_back(std::distance(names.begin(), it));
            } else {
                // Handle missing neighbour name (optional for debugging)
                RCLCPP_ERROR(this->get_logger(), "Neighbour '%s' not found in names list!", neigh_name.c_str());
            }
        }
    }
    return index_neighbours;
}

// Dijkstra's Algorithm to find the shortest path from start to goal.
std::vector<int> WaypointNavigator::dijkstra(
    const std::vector<std::vector<double>>& weights_,  // Weights between nodes
    const std::vector<std::vector<int>>& neighbours_,  // Neighbour nodes for each node
    int start, int goal)  // Start and goal node indices
{
    int num_nodes = weights_.size();  // Total number of nodes in the graph

    // Distance array: initialized to maximum values except for the start node
    std::vector<double> dist(num_nodes, std::numeric_limits<int>::max());
    dist[start] = 0;

    // Previous node array for path reconstruction
    std::vector<int> prev(num_nodes, -1);

    // Min-heap priority queue to explore the node with the smallest distance first
    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> pq;
    pq.push(PathNode{start, 0});  // Start node with distance 0

    // Main loop: explore nodes until the priority queue is empty or goal is reached
    while (!pq.empty()) {
        PathNode current = pq.top();
        pq.pop();

        // If the goal is reached, exit the loop
        if (current.id == goal) {
            break;
        }

        // Explore all neighbours of the current node
        for (size_t i = 0; i < neighbours_[current.id].size(); ++i) {
            int neighbor_id = neighbours_[current.id][i];  // Neighbour node index
            double weight = weights_[current.id][i];  // Weight of the edge to the neighbour

            // If a shorter path to the neighbour is found, update distance and previous node
            if (dist[current.id] + weight < dist[neighbor_id]) {
                dist[neighbor_id] = dist[current.id] + weight;
                prev[neighbor_id] = current.id;
                pq.push(PathNode{neighbor_id, dist[neighbor_id]});
            }
        }
    }

    // Reconstruct the shortest path from start to goal using the previous node array
    std::vector<int> path;
    for (int at = goal; at != -1; at = prev[at]) {
        path.push_back(at);
    }

    // Reverse the path to get it from start to goal
    std::reverse(path.begin(), path.end());

    // Output the shortest path and its total distance (for debugging/logging)
    std::string path_str;
    for (size_t i = 0; i < path.size(); ++i) {
        path_str += "\"" + names_[path[i]] + "\"";
        if (i != path.size() - 1) {
            path_str += " > ";
        }
    }
    RCLCPP_INFO(this->get_logger(), "Shortest path from %s to %s: %s",
            names_[start].c_str(), names_[goal].c_str(), path_str.c_str());

    RCLCPP_INFO(this->get_logger(), "Total distance: %.2f", dist[goal]);

    return path;  // Return the shortest path from start to goal
}


void WaypointNavigator::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Extract current robot position from the Odometry message
    current_location_.x = msg->pose.pose.position.x;
    current_location_.y = msg->pose.pose.position.y;

    // Flag indicating that the odometry has been received
    odom_received_ = true;
}

void WaypointNavigator::wait_for_odometry()
{
    // Set the rate of checking for odometry to 10Hz
    rclcpp::Rate rate(10);

    // Wait until odometry data is received or ROS is shut down
    while (!odom_received_ && rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());  // Process incoming messages
        rate.sleep();  
    }
}

// Calculate the Euclidean distance between two points (x1, y1) and (x2, y2)
double WaypointNavigator::distance(double x1, double y1, double x2, double y2)
{
    // Return the Euclidean distance using the standard distance formula
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
