#include "rclcpp/rclcpp.hpp"
#include "aruco_vision/aruco_detector.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}