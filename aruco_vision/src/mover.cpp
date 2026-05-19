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

#define X_PARK 960.0
#define Y_PARK 540.0
#define UNCERTAINTY 20.0
#define HYSTERESIS 60.0

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
    bool aruco_visible = (now - time).seconds() < 1.0;

    // The robot is perfectly parked ONLY if both X and Y are in the center zone
    bool y_fixed = (target_y >= Y_PARK-UNCERTAINTY && target_y <= Y_PARK+UNCERTAINTY);
    bool x_fixed = (target_x >= X_PARK-UNCERTAINTY && target_x <= X_PARK+UNCERTAINTY);

    // The robot moves again if it gets out of a larger zone to keep it from always moving
    bool y_hyst = (target_y >= Y_PARK-HYSTERESIS && target_y <= Y_PARK+HYSTERESIS);
    bool x_hyst = (target_x >= X_PARK-HYSTERESIS && target_x <= X_PARK+HYSTERESIS);

    // The robot is aligned if the angle is close to 0.0 radians
    bool angle_fixed = (target_angle >= -0.1 && target_angle <= 0.1);

    if (!aruco_visible) {
        rs = ROBOT_IDLE;
    }

    switch (rs) {
        case ROBOT_IDLE:
            if (aruco_visible) {
                if (!y_fixed) {
                    rs = ROBOT_FIX_Y;
                } else if (!x_fixed) {
                    rs = ROBOT_FIX_X;
                } else {
                    rs = ROBOT_PARK;
                }
            }
            break;

        case ROBOT_FIX_Y:
            if (y_fixed) {
                if (!x_fixed) rs = ROBOT_FIX_X;
                else rs = ROBOT_ALIGN;
            }
            break;

        case ROBOT_FIX_X:
            if (x_fixed) {
                if (!y_fixed) rs = ROBOT_FIX_Y;
                else rs = ROBOT_ALIGN;
            }
            break;

        case ROBOT_ALIGN:
            if (!y_fixed) {
                rs = ROBOT_FIX_Y;
            } else if (!x_fixed) {
                rs = ROBOT_FIX_X;
            } else if (angle_fixed) {
                rs = ROBOT_PARK;
            }
            break;

        case ROBOT_PARK:
            if (!y_hyst) {
                rs = ROBOT_FIX_Y;
            } else if (!x_hyst) {
                rs = ROBOT_FIX_X;
            } else {
                rs = ROBOT_PARK;
            }
            break;

        default:
            break;
    }

    message.linear.x = 0.0;
    message.angular.z = 0.0;

    float wanted_angle = 0.0, current_angle = 0.0, error_angle = 0.0;

    if( rs == ROBOT_FIX_Y ){
        if (target_y < Y_PARK) {
            message.linear.x = 0.1;     // Forward
        } else if (target_y > Y_PARK) {
            message.linear.x = -0.1;    // Backward
        }
    } else if( rs == ROBOT_FIX_X ){
        if (target_x < X_PARK) {
            message.angular.z = -0.2;   // Spin Right
        } else if (target_x > X_PARK) {
            message.angular.z = 0.2;    // Spin Left
        }
    } else if( rs == ROBOT_ALIGN ){
        wanted_angle = 0.0; 
        current_angle = target_angle; 
        error_angle = wanted_angle - current_angle;
        error_angle = atan2(sin(error_angle), cos(error_angle));
        if (error_angle > 0.1) {
            message.angular.z = 0.2;
            RCLCPP_INFO(this->get_logger(), "ALIGNING: Error = %.3f rad -> Orientating LEFT", error_angle);
        } else if (error_angle < -0.1) {
            message.angular.z = -0.2;
            RCLCPP_INFO(this->get_logger(), "ALIGNING: Error = %.3f rad -> Orientating RIGHT", error_angle);
        } else {
            RCLCPP_INFO(this->get_logger(), "ALIGNING: Error = %.3f rad -> ALIGNED!", error_angle);
        }
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