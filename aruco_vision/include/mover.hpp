/*
 * ==============================================================================
 * mover.hpp
 * Author : Marcos Ferrando España
 * ==============================================================================
 */

#ifndef ARUCO_VISION__MOVER_HPP_
#define ARUCO_VISION__MOVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


/* 
 * Class MoverNode
 * It inherits from rclcpp::Node to get network communication abilities
 */
class MoverNode : public rclcpp::Node {
public:
    MoverNode();    // Setup function that will run automatically when the node boots

private:
    // Callbacks
    void moveCallback();

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // ARUCO_VISION__MOVER_HPP_