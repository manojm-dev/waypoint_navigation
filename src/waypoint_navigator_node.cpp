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

#include "waypoint_navigation/waypoint_navigator_node.hpp"

using json = nlohmann::json;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNavigator>());
    rclcpp::shutdown();
    return 0;
}

WaypointNavigator::WaypointNavigator()
: Node("waypoint_navigator")
{
    std::string package_path = ament_index_cpp::get_package_share_directory("waypoint_navigation");
    std::string default_json_path = package_path + "/data/waypoints.json";

    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("waypoint_limit", 100);
    this->declare_parameter("json_path", default_json_path);
    this->declare_parameter("goal_waypoint", "F");

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    waypoint_limit_ = this->get_parameter("waypoint_limit").as_int();
    json_path_ = this->get_parameter("json_path").as_string();
    waypoint_goal_ = this->get_parameter("goal_waypoint").as_string();

    // Resize vectors to max possible initially
    names_.resize(waypoint_limit_);
    ids_.resize(waypoint_limit_);
    neighbours_.resize(waypoint_limit_);
    x_pos_.resize(waypoint_limit_);
    y_pos_.resize(waypoint_limit_);
    weights_.resize(waypoint_limit_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&WaypointNavigator::odom_callback, this, std::placeholders::_1)
    );

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    RCLCPP_INFO(this->get_logger(), "Waypoint navigator node started");
    callback();
}

void WaypointNavigator::callback()
{
    // Get robots current location
    wait_for_odometry();
    double robot_x = current_location_.x;
    double robot_y = current_location_.y;
    RCLCPP_INFO(this->get_logger(), "The Robot is currently located at position x: %f y: %f", robot_x, robot_y);

    // Load the waypoint data
    if (!load_waypoints_from_json()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints.");
        return;
    }

    // Finding the current waypoint 
    int current_waypoint_id = find_nearest_waypoint(robot_x, robot_y);
    std::string current_waypoint_name = names_[current_waypoint_id];
    RCLCPP_INFO(this->get_logger(), "Current at waypoint %s (ID: %d )",  current_waypoint_name.c_str(), current_waypoint_id);

    // Calculating weights
    calculate_weights();

    // Find shortest path

    // Converting goal name to index
    int goal_index = -1;
    for (int i = 0; i < names_.size(); ++i) {
        if (names_[i] == waypoint_goal_) {
            goal_index = i;
            break;
        }
    }
    // Converting neighbours name into index
    auto index_neighbours = convert_name_to_index(neighbours_, names_);
    std::vector<int> path = dijkstra(weights_, index_neighbours, current_waypoint_id, goal_index);
    std::reverse(path.begin(), path.end());

    for (int i = 0; i < path.size(); ++i)
    {
        int index = path[i];
        double x = x_pos_[index];
        double y = y_pos_[index];
        double yaw = 0.0;
    
        std::string wp_name = names_[index];
        RCLCPP_INFO(this->get_logger(), "[%d/%zu] Navigating to waypoint '%s' (x: %.2f, y: %.2f)",
                    i + 1, path.size(), wp_name.c_str(), x, y);
    
        geometry_msgs::msg::PoseStamped goal_pose = create_pose(x, y, yaw);
    
        if (!nav_to_goal(goal_pose)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal to '%s' (index: %d)", wp_name.c_str(), index);
            return;
        }
    
        RCLCPP_INFO(this->get_logger(), "Goal sent. Waiting for result...");
    
        while (!goal_done_ && rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    
        if (goal_succeeded_) {
            RCLCPP_INFO(this->get_logger(), "Successfully reached waypoint '%s'", wp_name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach waypoint '%s'", wp_name.c_str());
            return;
        }
    
        goal_done_ = false;
        goal_succeeded_ = false;
    }
}

geometry_msgs::msg::PoseStamped WaypointNavigator::create_pose(double x, double y, double yaw)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = this->get_clock()->now();

    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw); // Roll, Pitch, Yaw
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    return pose_msg;
}

bool WaypointNavigator::load_waypoints_from_json()
{
    std::ifstream file(json_path_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't open JSON file: %s", json_path_.c_str());
        return false;
    }

    json data;
    try {
        file >> data;
    } catch (const json::parse_error &e) {
        RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
        return false;
    }

    num_waypoint_ = std::min((int)data.size(), waypoint_limit_);
    names_.resize(num_waypoint_);
    ids_.resize(num_waypoint_);
    neighbours_.resize(num_waypoint_);
    x_pos_.resize(num_waypoint_);
    y_pos_.resize(num_waypoint_);
    weights_.resize(num_waypoint_);

    for (int i = 0; i < num_waypoint_; ++i)
    {
        const auto& wp = data[i];
        ids_[i] = wp["id"];
        names_[i] = wp["name"];
        x_pos_[i] = wp["x"];
        y_pos_[i] = wp["y"];

        for (const auto& n : wp["neighbours"]) {
            neighbours_[i].push_back(n);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %d waypoints from JSON", num_waypoint_);
    return true;
}

void WaypointNavigator::calculate_weights()
{
    for (int a = 0; a < num_waypoint_; ++a)
    {
        const std::vector<std::string>& neighbour_list = neighbours_[a];
        weights_[a].resize(neighbour_list.size());
            
        for (int b = 0; b < neighbour_list.size(); ++b)
        {
            const std::string& neighbour_name = neighbour_list[b];

            for (int c = 0; c < names_.size(); ++c)
            {
                if (neighbour_name == names_[c])
                {
                    weights_[a][b] = distance(x_pos_[a], y_pos_[a], x_pos_[c], y_pos_[c]);

                    RCLCPP_DEBUG(this->get_logger(),
                        "Distance between %s(%f,%f) and %s(%f,%f) is %f",
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
    double min_distance = std::numeric_limits<double>::max();
    int min_index = -1;

    for (int i = 0; i < num_waypoint_; ++i) {
        double d = distance(x, y, x_pos_[i], y_pos_[i]);
        if (d < min_distance) {
            min_distance = d;
            min_index = i;
        }
    }

    return min_index;
}

bool WaypointNavigator::nav_to_goal(const geometry_msgs::msg::PoseStamped &goal_pose)
{
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return false;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&WaypointNavigator::result_callback, this, std::placeholders::_1);

    nav_client_->async_send_goal(goal_msg, send_goal_options);

    return true;
}

void WaypointNavigator::result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
{
    goal_done_ = true;

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

    for (size_t i = 0; i < name_neighbours.size(); ++i) {
        for (const std::string& neigh_name : name_neighbours[i]) {
            auto it = std::find(names.begin(), names.end(), neigh_name);
            if (it != names.end()) {
                index_neighbours[i].push_back(std::distance(names.begin(), it));
            } else {
                std::cerr << "Neighbour name " << neigh_name << " not found in names list!" << std::endl;
            }
        }
    }
    return index_neighbours;
}


// Dijkstra's Algorithm to find the shortest path from start to goal.
std::vector<int> WaypointNavigator::dijkstra(
    const std::vector<std::vector<double>>& weights_,
    const std::vector<std::vector<int>>& neighbours_,
    int start, int goal) 
{
    int num_nodes = weights_.size();
    std::vector<double> dist(num_nodes, std::numeric_limits<int>::max());  // Distance array
    std::vector<int> prev(num_nodes, -1);          // Previous node for path reconstruction

    dist[start] = 0;  // Distance to the start node is 0
    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> pq;
    pq.push(PathNode{start, 0});

    while (!pq.empty()) {
    PathNode current = pq.top();
    pq.pop();

    // If we reached the goal, break out
    if (current.id == goal) {
    break;
    }

    // Explore neighbors of the current node
    for (size_t i = 0; i < neighbours_[current.id].size(); ++i) {
    int neighbor_id = neighbours_[current.id][i];
    double weight = weights_[current.id][i];

    // If a shorter path to the neighbor is found
    if (dist[current.id] + weight < dist[neighbor_id]) {
        dist[neighbor_id] = dist[current.id] + weight;
        prev[neighbor_id] = current.id;
        pq.push(PathNode{neighbor_id, dist[neighbor_id]});
    }
    }
    }

    // Reconstruct the shortest path from start to goal
    std::vector<int> path;
    for (int at = goal; at != -1; at = prev[at]) {
    path.push_back(at);
    }

    // Print the shortest path and its distance
    std::cout << "Shortest path from " << start << " to " << goal << " is: " << std::endl;
    for (int i = path.size() - 1; i >= 0; --i) {
    std::cout << path[i] << " ";
    }
    std::cout << "Total distance: " << dist[goal] << std::endl;

    return path;
}

void WaypointNavigator::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_location_.x = msg->pose.pose.position.x;
    current_location_.y = msg->pose.pose.position.y;
    odom_received_ = true;
}

void WaypointNavigator::wait_for_odometry()
{
    rclcpp::Rate rate(10);
    while (!odom_received_ && rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
}

double WaypointNavigator::distance(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}