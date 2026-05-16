#include "rclcpp/rclcpp.hpp"
#include "udp_agent/CUDPAgentNode.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto ros_node = std::make_shared<CUDPAgent>();

    int exit_code = 0;
    try {
        rclcpp::spin(ros_node);
    }
    catch (const std::exception& e) {
    }
    ros_node.reset(); 

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    return exit_code;
}