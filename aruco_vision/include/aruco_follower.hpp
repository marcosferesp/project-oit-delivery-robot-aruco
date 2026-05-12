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

typedef enum {
    ROBOT_IDLE = 0,
    ROBOT_MOVE,
    ROBOT_PARK
} robot_state_t;

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
    void distanceCallback(const std_msgs::msg::Float32::SharedPtr msg);

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // Subscriber
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dist_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables
    robot_state_t rs;
    float dist;
    rclcpp::Time time;
};

#endif //ARUCO_VISION__ARUCO_FOLLOWER_HPP_