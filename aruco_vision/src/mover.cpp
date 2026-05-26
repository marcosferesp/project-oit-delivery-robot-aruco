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

    // Init Robot Route Database
    route[0] = {0, false, 2.0, 15.0}; // ID 0: Pass through, max 15s search
    route[1] = {1, false, 1.5, 15.0}; // ID 1: Pass through, max 15s search
    route[2] = {2, true,  0.0, 0.0};  // ID 2: Destination. Park here.

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
 * Inputs : msg (geometry_msgs::msg::Point::SharedPtr) : the coordinates data
 * Output :
 */
void ArucoFollowerNode::coordCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    target_x = msg->x;
    target_y = msg->y;
    target_angle = msg->z;
    time = this->now();
}

/* 
 * Function coordCallback
 * Triggers whenever new x and y coordinates are published by the camera
 * Inputs : msg (std_msgs::msg::Int32::SharedPtr) : the ArUco ID data
 * Output :
 */
void ArucoFollowerNode::idCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    target_id = msg->data;
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
#define DEADBAND_LINEAR 0.01
#define DEADBAND_ANGLE  0.01

// Sensor time out
#define ARUCO_TIMEOUT   1.0

/* 
 * Function ctrlLoop
 * Evaluates the robot's current state and sends motor commands
 * Inputs :
 * Output :
 */
void ArucoFollowerNode::ctrlLoop() {
    auto message = geometry_msgs::msg::Twist();
    auto now = this->now();

    // If 1 second passes with no new data, we lost the marker
    bool aruco_visible = (now - time).seconds() < ARUCO_TIMEOUT;

    float error_y = Y_PARK - target_y;
    float error_x = X_PARK - target_x;
    float error_angle = ANGLE_PARK - target_angle;
    error_angle = atan2(sin(error_angle), cos(error_angle));

    robot_route_t rr = route[target_id];

    bool ready_to_move = ((std::abs(error_y) > UNCERTAINTY) || (std::abs(error_x) > UNCERTAINTY) || (std::abs(error_angle) > ANGLE_UNCE));
    bool ready_to_park = (rr.is_destination && ((std::abs(error_y) < UNCERTAINTY) && (std::abs(error_x) < UNCERTAINTY) && (std::abs(error_angle) < ANGLE_UNCE)));
    bool robot_drifted = ((std::abs(error_y) > HYSTERESIS) || (std::abs(error_x) > HYSTERESIS) || (std::abs(error_angle) > ANGLE_HYST));

    switch (rs) {
        case ROBOT_IDLE:
            if( aruco_visible ){
                if( ready_to_move || robot_drifted ){
                    rs = ROBOT_MOVE;
                    RCLCPP_INFO(this->get_logger(), "Target acquired. Moving to align.");
                }else if( ready_to_park ){
                    rs = ROBOT_PARK;
                    RCLCPP_INFO(this->get_logger(), "Target acquired. Parking.");
                }
                
            }
            break;

        case ROBOT_MOVE:
            if (!aruco_visible) {
                if (!rr.is_destination) {
                    rs = ROBOT_SEARCH;
                    search_start_time = now;
                    RCLCPP_INFO(this->get_logger(), "Marker passed. Entering SEARCH mode.");
                } else {
                    rs = ROBOT_IDLE;
                }
            } else if( ready_to_park ){
                rs = ROBOT_PARK;
                RCLCPP_INFO(this->get_logger(), "Aligning complete. Parking.");
            }
            break;

        case ROBOT_PARK:
            if (!aruco_visible) {
                rs = ROBOT_IDLE;
            } else if( robot_drifted ){
                rs = ROBOT_MOVE;
                RCLCPP_INFO(this->get_logger(), "Robot has drifted. Moving to align.");
            }
            break;

        case ROBOT_SEARCH:
            if (aruco_visible) {
                rs = ROBOT_MOVE;
                RCLCPP_INFO(this->get_logger(), "Target ID %d acquired from search. Moving.", target_id);
            } else if ((now - search_start_time).seconds() > rr.search_timeout) {
                rs = ROBOT_IDLE;
                RCLCPP_INFO(this->get_logger(), "Search timeout! Target lost. Idling.");
            }
            break;

        default:
            break;
    }

    message.linear.x = 0.0;
    message.angular.z = 0.0;

    float debug_rawa = 0.0;

    switch (rs) {
        case ROBOT_MOVE:
        {
            if (rr.is_destination) {
                message.linear.x = KP_Y * error_y;

                if( std::abs(error_y) <= UNCERTAINTY ){
                    debug_rawa = -(KP_ANGLE * error_angle);
                    message.angular.z = -(KP_ANGLE * error_angle);
                }else{
                    debug_rawa = -((KP_ANGLE * error_angle) + (KP_X * error_x));
                    message.angular.z = -((KP_ANGLE * error_angle) + (KP_X * error_x));
                }
            } else {
                message.linear.x = SPEED_LINEAR;
                message.angular.z = -((KP_ANGLE * error_angle) + (KP_X * error_x));
            }
            
            if (message.linear.x > SPEED_LINEAR)
                message.linear.x = SPEED_LINEAR;
            if (message.linear.x < -SPEED_LINEAR)
                message.linear.x = -SPEED_LINEAR;
            if (message.angular.z > SPEED_ANGULAR)
                message.angular.z = SPEED_ANGULAR;
            if (message.angular.z < -SPEED_ANGULAR)
                message.angular.z = -SPEED_ANGULAR;

            if (std::abs(message.linear.x) < DEADBAND_LINEAR) message.linear.x = 0.0;
            if (std::abs(message.angular.z) < DEADBAND_ANGLE) message.angular.z = 0.0;

            RCLCPP_INFO(this->get_logger(), "ErrY: %5.1f | ErrX: %5.1f | ErrAng: %6.3f || RawAngZ: %6.3f | OutAngZ: %6.3f", error_y, error_x, error_angle, debug_rawa, message.angular.z);

            break;
        }

        case ROBOT_SEARCH:
        {
            message.linear.x = SPEED_LINEAR;
            message.angular.z = 0.0;

            break;
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