/*
 * ==============================================================================
 * mover.cpp
 * Contains the vision processing logic, translating camera images to find ArUco markers and calculating their physical distance
 * Author : Marcos Ferrando España
 * ==============================================================================
 */

#include "mover.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* 
 * Constructor MoverNode
 * Prepares the node to run by setting up the publisher and starting the timer
 * Inputs :
 * Output :
 */
MoverNode::MoverNode() : Node("mover_node") {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&MoverNode::moveCallback, this));

    RCLCPP_INFO(this->get_logger(), "Motor Ctrl is online and sending forward velocity...");
}

/* 
 * Function moveCallback
 * Creates a movement message and broadcasts it to the motors
 * Inputs :
 * Output :
 */
void MoverNode::moveCallback() {
    auto message = geometry_msgs::msg::Twist();

    message.linear.x = 0.1;
    message.angular.z = 0.0;

    cmd_pub_->publish(message);
}

/* 
 * Main
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoverNode>());
    rclcpp::shutdown();
    return 0;
}