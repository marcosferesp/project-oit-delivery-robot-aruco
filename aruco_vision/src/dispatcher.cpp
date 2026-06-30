#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>

void cmdHelp(const std::string& arg) {
    if (arg.empty()) {
        std::cout << "\n=== AVAILABLE COMMANDS ===\n";
        std::cout << "  taxi  : Manage destinations\n";
        std::cout << "  pkg   : Manage package status\n";
        std::cout << "\n==========================\n";
        std::cout << " Type 'help' <cmd> for more details on a command.\n";

    } else if (arg == "taxi") {
        std::cout << "Objective : Dispatches the robot to a known destinations or lists available targets.\n";
        std::cout << "  taxi list     : Show available destinations\n";
        std::cout << "  taxi <id> : Dispatch robot to a specific destination\n";

    } else if (arg == "pkg") {
        std::cout << "Objective : Simulates the web-server package retrieval signal.\n";
        std::cout << "  pkg true      : Set package as taken\n";
        std::cout << "  pkg false     : Set package as not taken\n";
        std::cout << "  pkg status    : Show package status and robot position\n";

    } else {
        std::cout << "\033[1;31m[ERROR] Unknown help argument.\033[0m\n";
    }
}

std::vector<int> valid_ids;

void cmdTaxi(const std::string& arg, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub, rclcpp::Logger logger) {
    if (valid_ids.empty()) {
        std::cout << "\033[1;31m[ERROR] Database empty. Is the Mover node running?\033[0m\n";
        return;
    }

    if (arg.empty()) {
        std::cout << "\033[1;31m[ERROR] Missing argument. Use 'taxi list' or 'taxi <number>'.\033[0m\n";
        return;
    }

    if (arg == "list") {
        std::cout << "\nAvailable Destinations: [ ";
        for (int id : valid_ids) { std::cout << id << " "; }
        std::cout << "]\n";

    } else {
        try {
            int target_id = std::stoi(arg);
            if (std::find(valid_ids.begin(), valid_ids.end(), target_id) != valid_ids.end()) {
                auto msg = std_msgs::msg::Int32();
                msg.data = target_id;
                pub->publish(msg);
                RCLCPP_INFO(logger, "\033[1;32mDispatched ID %d to the network.\033[0m", target_id);
            } else {
                std::cout << "\033[1;31m[ERROR] ID " << target_id << " is not in the active database.\033[0m\n";
            }
        } catch (...) {
            std::cout << "\033[1;31m[ERROR] Invalid argument. Use 'taxi list' or 'taxi <number>'.\033[0m\n";
        }
    }
}

bool pkg_status = false;
std::string robot_pos = "UNKNOWN (Requires Telemetry Node)";

void cmdPkg(const std::string& arg, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub) {
    if (arg == "true" || arg == "false") {
        pkg_status = (arg == "true");
        auto msg = std_msgs::msg::Bool();
        msg.data = pkg_status;
        pub->publish(msg);
        std::cout << "Package status forced to: " << (pkg_status ? "TRUE" : "FALSE") << "\n";

    } else if (arg == "status") {
        std::cout << "  Package Taken : " << (pkg_status ? "TRUE" : "FALSE") << "\n";
        std::cout << "  Robot Position: " << robot_pos << "\n";

    } else {
        std::cout << "\033[1;31m[ERROR] Invalid argument. Use 'pkg true', 'pkg false', or 'pkg status'.\033[0m\n";
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("dispatcher_node");
    auto taxi_pub = node->create_publisher<std_msgs::msg::Int32>("/cmd_taxi", 10);
    auto pkg_pub = node->create_publisher<std_msgs::msg::Bool>("/pkg_status", 10);
    
    // --- Database Synchronization ---
    rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
    latched_qos.transient_local();

    auto db_sub = node->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/active_database", latched_qos,
        [](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            valid_ids = msg->data;
            std::cout << "\n\033[1;32m[SYSTEM] Synced route database from Mover: [ \033[0m";
            for (int id : valid_ids) std::cout << "\033[1;32m" << id << " \033[0m";
            std::cout << "\033[1;32m]\033[0m\n[DISPATCHER] > " << std::flush;
        }
    );

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
    
    std::string line;
    while (rclcpp::ok()) {
        std::cout << "\n[DISPATCHER] > ";
        std::getline(std::cin, line);
        
        if (line.empty()) continue;

        // Parse user input
        std::istringstream iss(line);
        std::string command, argument;
        iss >> command;
        iss >> argument; // Will be empty if user only typed one word

        // Route to the appropriate command function
        if (command == "help") {
            cmdHelp(argument);
        } else if (command == "taxi") {
            cmdTaxi(argument, taxi_pub, node->get_logger());
        } else if (command == "pkg") {
            cmdPkg(argument, pkg_pub);
        } else {
            std::cout << "\033[1;31m[ERROR] Unknown command. Type 'help' for a list of commands.\033[0m\n";
        }
    }

    rclcpp::shutdown();
    return 0;
}