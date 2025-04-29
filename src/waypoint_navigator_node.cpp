#include "rclcpp/rclcpp.hpp"

#include "waypoint_navigation/waypoint_navigator_node.hpp"

WaypointNavigator::WaypointNavigator()
: Node("waypoint_navigator")
{
    RCLCPP_INFO(this->get_logger(), "Waypoint navigator node started");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNavigator>());

    rclcpp::shutdown();
    
    return 0;
}