/*
 * ==============================================================================
 * aruco_follower.hpp
 * Author : Marcos Ferrando España
 * ==============================================================================
 */

#ifndef ARUCO_VISION__ARUCO_FOLLOWER_HPP_
#define ARUCO_VISION__ARUCO_FOLLOWER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"

typedef enum {
    ROBOT_IDLE,
    ROBOT_MOVE,
    ROBOT_PARK,
    ROBOT_SEARCH
} robot_state_t;

typedef struct {
    int16_t aruco_id;
    bool    is_destination;
    float   dist_to_next;
    float   search_timeout;
    int16_t next_id;        // -1 means there is no next marker.
} robot_route_t;

/* 
 * Class ArucoFollowerNode
 * It inherits from rclcpp::Node to get network communication abilities
 */
class ArucoFollowerNode : public rclcpp::Node {
public:
    ArucoFollowerNode();    // Setup function that will run automatically when the node boots

private:
    // Loop to control the robot`s movement
    void ctrlLoop();

    // Callbacks
    // void distanceCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void coordCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void idCallback(const std_msgs::msg::Int32::SharedPtr msg);

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // Subscribers
    // rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr coord_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr id_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables
    robot_state_t rs;
    // float dist;
    float target_x;
    float target_y;
    float target_angle;
    int target_id;

    rclcpp::Time time;
    rclcpp::Time search_start_time;

    std::map<int, robot_route_t> route;
};

#endif //ARUCO_VISION__ARUCO_FOLLOWER_HPP_