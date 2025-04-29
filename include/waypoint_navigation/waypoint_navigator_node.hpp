#ifndef WAYPOINT_NAVIGATION__WAYPOINT_NAVIGATOR_NODE_HPP
#define WAYPOINT_NAVIGATION__WAYPOINT__NAVIGATOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"

class WaypointNavigator : public rclcpp::Node
{
    public:
        WaypointNavigator();

    private:
        void callback();
};

#endif // WAYPOINT_NAVIGATION__WAYPOINT_NAVIGATOR_NODE_HPP