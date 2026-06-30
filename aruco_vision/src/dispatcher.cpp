#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <thread>
#include <readline/readline.h>
#include <readline/history.h>

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

void cmdTaxi(const std::string& arg, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr list_pub, rclcpp::Logger logger) {
    if (arg.empty()) {
        std::cout << "\033[1;31m[ERROR] Missing argument. Use 'taxi list' or 'taxi <number>'.\033[0m\n";
        return;
    }

    if (arg == "list") {
        // Send the boolean trigger to the Mover!
        auto list_msg = std_msgs::msg::Bool();
        list_msg.data = true;
        list_pub->publish(list_msg);
        std::cout << "\033[1;33mRequesting database from Mover...\033[0m\n";
        return;
    } 
    
    // Safety check: If they try to send a number before the database is synced
    if (valid_ids.empty()) {
        std::cout << "\033[1;31m[ERROR] Database empty. Requesting sync. Please wait a second and type your command again.\033[0m\n";
        auto list_msg = std_msgs::msg::Bool();
        list_msg.data = true;
        list_pub->publish(list_msg);
        return;
    }

    // Normal Dispatch Logic
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
    
    // --- The New Trigger Publisher ---
    auto db_req_pub = node->create_publisher<std_msgs::msg::Bool>("/req_db", 10);
    
    // --- The Network Callback (Prints the list when it arrives) ---
    auto db_sub = node->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/active_db", 10,
        [](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            valid_ids = msg->data;
            std::cout << "\n\033[1;32m[SYSTEM] Available Destinations: [ ";
            for (int id : valid_ids) std::cout << id << " ";
            std::cout << "]\033[0m\n[DISPATCHER] > " << std::flush;
        }
    );

    // --- FIX 1: Run the ROS 2 node in a background thread so the UI doesn't block the network ---
    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

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
        // --- FIX 2 & 3: readline automatically handles ANSI codes, arrow keys, and Ctrl+C crashes ---
        char* input = readline("\n[DISPATCHER] > ");
        
        // Safely catch Ctrl+C or Ctrl+D (EOF) to prevent infinite spam
        if (!input) {
            std::cout << "\nShutting down Dispatcher...\n";
            break; 
        }

        std::string line(input);
        
        // Add the typed command to the up-arrow history
        if (!line.empty()) {
            add_history(input); 
        }
        
        free(input); // Prevent memory leaks from readline allocation

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
            cmdTaxi(argument, taxi_pub, db_req_pub, node->get_logger());
        } else if (command == "pkg") {
            cmdPkg(argument, pkg_pub);
        } else {
            std::cout << "\033[1;31m[ERROR] Unknown command. Type 'help' for a list of commands.\033[0m\n";
        }
    }

    rclcpp::shutdown();
    
    // Safely collapse the background network thread before exiting
    spin_thread.join(); 
    return 0;
}