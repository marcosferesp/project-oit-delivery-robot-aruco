/*
 * ==============================================================================
 * main.cpp
 * Initializes the ROS 2 system and keeps the ArUco detector node running continuously
 * Author : Marcos Ferrando España
 * ==============================================================================
 */

#include "rclcpp/rclcpp.hpp"
#include "aruco_detector.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}