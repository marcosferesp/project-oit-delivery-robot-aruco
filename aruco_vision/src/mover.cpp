/*
 * ==============================================================================
 * mover.cpp
 * Contains the vision processing logic, translating camera images to find ArUco markers and calculating their physical distance
 * Author : Marcos Ferrando España
 * ==============================================================================
 */

#include "mover.hpp"
#include "aruco_follower.hpp"
#include <chrono>

using namespace std::chrono_literals;

// #define DEBUG_SIMPLE_MOVE_PUB


#if DEBUG_SIMPLE_MOVE_PUB
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
#endif // DEBUG_SIMPLE_MOVE_PUB


/* 
 * Constructor ArucoFollowerNode
 * Init the node and sets up the publisher, subscriber and timer
 * Inputs :
 * Output :
 */
ArucoFollowerNode::ArucoFollowerNode() : Node("aruco_follower_node"), rs(ROBOT_IDLE), dist(0.0) {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    dist_sub_ = this->create_subscription<std_msgs::msg::Float32>( "/aruco_distance", 10, std::bind(&ArucoFollowerNode::distanceCallback, this, std::placeholders::_1) );
    timer_ = this->create_wall_timer(100ms, std::bind(&ArucoFollowerNode::ctrlLoop, this));

    time = this->now();

    RCLCPP_INFO(this->get_logger(), "Motor Ctrl is online and preparing to follow ArUco nearby...");
}

/* 
 * Function distanceCallback
 * Triggers whenever a new distance measurement is published by the camera
 * Inputs : msg (std_msgs::msg::Float32::SharedPtr) : the distance data
 * Output :
 */
void ArucoFollowerNode::distanceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    dist = msg->data;
    time = this->now();
}

/* 
 * Function ctrlLoop
 * Evaluates the robot's current state and sends motor commands
 * Inputs :
 * Output :
 */
void ArucoFollowerNode::ctrlLoop() {
    auto message = geometry_msgs::msg::Twist();
    auto now = this->now();

    // If 2 seconds pass with no new distance data we consider there is no marker
    bool aruco_visible = (now - time).seconds() < 2.0;

    if (!aruco_visible) {
        rs = ROBOT_IDLE;
    }
    
    switch (rs) {
        case ROBOT_IDLE:
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            if (dist <= 0.30) {
                rs = ROBOT_PARK;
            } else if (dist >= 0.35) {
                rs = ROBOT_MOVE;
            } else {
                rs = ROBOT_IDLE;
            }
            break;

        case ROBOT_MOVE:
            message.linear.x = 0.1;
            message.angular.z = 0.0;
            if (dist <= 0.30) {
                rs = ROBOT_PARK;
            }
            break;

        case ROBOT_PARK:
            message.linear.x = 0.1;
            message.angular.z = 0.0;
            if (dist >= 0.35) {
                rs = ROBOT_MOVE;
            }
    
        default:
            break;
    }

    cmd_pub_->publish(message);
}

/* 
 * Main
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

#if DEBUG_SIMPLE_MOVE_PUB
    rclcpp::spin(std::make_shared<MoverNode>());
#else
    rclcpp::spin(std::make_shared<ArucoFollowerNode>());
#endif // DEBUG_SIMPLE_MOVE_PUB

    rclcpp::shutdown();

    return 0;
}