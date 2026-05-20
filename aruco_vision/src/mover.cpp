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
ArucoFollowerNode::ArucoFollowerNode() : Node("aruco_follower_node"), rs(ROBOT_IDLE) {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    coord_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/aruco_coordinates", 10, std::bind(&ArucoFollowerNode::coordCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(100ms, std::bind(&ArucoFollowerNode::ctrlLoop, this));

    time = this->now();

    RCLCPP_INFO(this->get_logger(), "Motor Ctrl is online and preparing to follow ArUco nearby...");
}

// /* 
//  * Function distanceCallback
//  * Triggers whenever a new distance measurement is published by the camera
//  * Inputs : msg (std_msgs::msg::Float32::SharedPtr) : the distance data
//  * Output :
//  */
// void ArucoFollowerNode::distanceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
//     dist = msg->data;
//     time = this->now();
// }

/* 
 * Function coordCallback
 * Triggers whenever new x and y coordinates are published by the camera
 * Inputs : msg (std_msgs::msg::Float32::SharedPtr) : the distance data
 * Output :
 */
void ArucoFollowerNode::coordCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    target_x = msg->x;
    target_y = msg->y;
    target_angle = msg->z;
    time = this->now();
}

// Target values to park and uncertainties
#define X_PARK      960.0
#define Y_PARK      540.0
#define UNCERTAINTY 30.0
#define HYSTERESIS  80.0
#define ANGLE_PARK  0.0
#define ANGLE_UNCE  0.2
#define ANGLE_HYST  0.35

// Proportional gains
#define KP_Y        0.00037
#define KP_X        0.00021
#define KP_ANGLE    0.063

// Motor limits
#define SPEED_LINEAR    0.2
#define SPEED_ANGULAR   0.2
#define DEADBAND_LINEAR 0.02
#define DEADBAND_ANGLE  0.05

// Sensor time out
#define ARUCO_TIMEOUT   10

// The distance of the "Ghost Marker" in pixels
#define LOOKAHEAD_DIST 200.0

/* 
 * Function ctrlLoop
 * Evaluates the robot's current state and sends motor commands
 * Inputs :
 * Output :
 */
void ArucoFollowerNode::ctrlLoop() {
    auto message = geometry_msgs::msg::Twist();
    auto now = this->now();

    // 1. Data Evaluation
    bool aruco_visible = (now - time).seconds() < ARUCO_TIMEOUT;

    // A. Calculate Y Error normally
    float error_y = Y_PARK - target_y;

    // B. Calculate the Angle Error
    float error_angle = 0.0 - target_angle;
    error_angle = atan2(sin(error_angle), cos(error_angle));

    // C. --- PURE PURSUIT (VIRTUAL WAYPOINT) LOGIC ---
    // Shrink the lookahead distance as we get closer so the ghost marker merges with the real one
    float current_lookahead = LOOKAHEAD_DIST;
    if (std::abs(error_y) < LOOKAHEAD_DIST) {
        current_lookahead = std::abs(error_y);
    }

    // Project the virtual X coordinate using Trigonometry
    // NOTE: Depending on your camera's coordinate frame, if the robot curves the wrong way, 
    // change the '-' to a '+' in the line below.
    float virtual_x = target_x - (current_lookahead * sin(target_angle));

    // Calculate the X error used for STEERING (aiming at the ghost marker)
    float steering_error_x = X_PARK - virtual_x;
    
    // Calculate the REAL X error used for PARKING (checking if we physically reached the true center)
    float real_error_x = X_PARK - target_x;
    // ------------------------------------------------
    
    // Evaluate pure physical position for state transitions (Using REAL X)
    bool ready_to_move = (std::abs(error_y) > UNCERTAINTY || 
                          std::abs(real_error_x) > UNCERTAINTY || 
                          std::abs(error_angle) > ANGLE_UNCE);
                          
    bool ready_to_park = (std::abs(error_y) < UNCERTAINTY && 
                          std::abs(real_error_x) < UNCERTAINTY && 
                          std::abs(error_angle) < ANGLE_UNCE);
                          
    bool robot_drifted = (std::abs(error_y) > HYSTERESIS || 
                          std::abs(real_error_x) > HYSTERESIS || 
                          std::abs(error_angle) > ANGLE_HYST);

    if (!aruco_visible) {
        rs = ROBOT_IDLE;
    }

    // 2. State Machine
    switch (rs) {
        case ROBOT_IDLE:
            if (aruco_visible) {
                if (ready_to_move || robot_drifted) {
                    rs = ROBOT_MOVE;
                    RCLCPP_INFO(this->get_logger(), "Target acquired. Chasing virtual waypoint.");
                } else if (ready_to_park) {
                    rs = ROBOT_PARK;
                    RCLCPP_INFO(this->get_logger(), "Target acquired. Already perfectly aligned.");
                }
            }
            break;

        case ROBOT_MOVE:
            if (ready_to_park) {
                rs = ROBOT_PARK;
                RCLCPP_INFO(this->get_logger(), "Alignment complete. Parking.");
            }
            break;

        case ROBOT_PARK:
            if (robot_drifted) {
                rs = ROBOT_MOVE;
                RCLCPP_INFO(this->get_logger(), "Robot drifted. Waking up to adjust.");
            }
            break;

        default:
            break;
    }

    // 3. Motor Execution
    message.linear.x = 0.0;
    message.angular.z = 0.0;

    if (rs == ROBOT_MOVE) {
        // Drive forward based on Y distance
        message.linear.x = KP_Y * error_y;
        
        // Steer based on the Angle and the VIRTUAL X coordinate
        // (We keep the minus sign for KP_X because your camera is behind the wheels)
        message.angular.z = (KP_ANGLE * error_angle) - (KP_X * steering_error_x);

        // Hard safety limits
        if (message.linear.x > SPEED_LINEAR) message.linear.x = SPEED_LINEAR;
        if (message.linear.x < -SPEED_LINEAR) message.linear.x = -SPEED_LINEAR;
        if (message.angular.z > SPEED_ANGULAR) message.angular.z = SPEED_ANGULAR;
        if (message.angular.z < -SPEED_ANGULAR) message.angular.z = -SPEED_ANGULAR;

        // Deadband logic to prevent low-speed motor whining and twitching
        if (std::abs(message.linear.x) < DEADBAND_LINEAR) message.linear.x = 0.0;
        if (std::abs(message.angular.z) < DEADBAND_ANGLE) message.angular.z = 0.0;
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