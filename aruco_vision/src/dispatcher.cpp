#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <limits>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    // Create a simple node purely for publishing
    auto node = rclcpp::Node::make_shared("terminal_input_node");
    auto publisher = node->create_publisher<std_msgs::msg::Int32>("/cmd_taxi", 10);
    
    RCLCPP_INFO(node->get_logger(), "\n"
        "=========================================\n"
        " Dynamic Terminal Input Node v1.0\n"
        "-----------------------------------------\n"
        " Osaka Institute of Technology\n"
        " Department of Real World Computing\n"
        " OIT ArUco Delivery Robot\n"
        " Developer : Marcos FERRANDO ESPAÑA\n"
        " Supervisor: Prof. Kenzaburo MIYAWAKI\n"
        "-----------------------------------------\n"
        " Publishing to : /cmd_taxi\n"
        " Message type  : std_msgs/msg/Int32\n"
        "-----------------------------------------\n"
        " Enter the ID of an ArUco available in the database and press ENTER to publish the command.\n"
        "=========================================\n"
        " Type 'help' for available commands.\n"
        " Waiting for user input...\n");
    
    while (rclcpp::ok()) {
        int target_id;
        std::cout << "\n[DISPATCHER] Enter next ArUco ID for the queue: ";
        
        if (std::cin >> target_id) {
            // If the user typed a valid number, publish it to the network
            auto msg = std_msgs::msg::Int32();
            msg.data = target_id;
            publisher->publish(msg);
            RCLCPP_INFO(node->get_logger(), "Dispatched ID %d to the robot's queue.", target_id);
        } else {
            // If the user accidentally typed letters, clear the error so the terminal doesn't crash
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "[ERROR] Invalid input. Please enter a valid number." << std::endl;
        }
    }

    rclcpp::shutdown();
    return 0;
}