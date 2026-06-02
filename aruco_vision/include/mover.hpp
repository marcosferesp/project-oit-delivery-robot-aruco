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
 * Inherits from rclcpp::Node for basic motor debugging
 */
class MoverNode : public rclcpp::Node {
public:
    MoverNode();

private:
    // --- Callbacks ---
    void moveCallback();

    // --- Publishers ---
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // --- Timers ---
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // ARUCO_VISION__MOVER_HPP_