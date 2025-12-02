#include "rclcpp/rclcpp.hpp"
#include "segwayrmp/robot.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the node with the name "SmartCar"
    auto node = std::make_shared<rclcpp::Node>("smartcar");

    // Declare the parameter (so it's visible in ROS2)
    node->declare_parameter<std::string>("segwaySmartCarSerial", "ttyUSB0");
    //node->declare_parameter<std::string>("segwaySmartCarSerial", "ttyTHS1");
    

    // Get the parameter value (default "ttyUSB0")
    std::string serial = node->get_parameter("segwaySmartCarSerial").as_string();

    // Print out the parameter value and pass it to your serial function
    RCLCPP_INFO(node->get_logger(), "segwaySmartCarSerial: %s", serial.c_str());
    set_smart_car_serial(serial.c_str());
    RCLCPP_INFO(node->get_logger(), "set_smart_car_serial DONE");

    // Before calling init_control_ctrl, set the communication interface type
    set_comu_interface(comu_serial);
    RCLCPP_INFO(node->get_logger(), "set_comu_interface DONE");
    
    int ctrl_output = init_control_ctrl();
    RCLCPP_INFO(node->get_logger(), "init_control_ctrl: %d", ctrl_output);
    
    if(ctrl_output == -1){
        RCLCPP_ERROR(node->get_logger(), "init_control failed");
    }else{
        RCLCPP_INFO(node->get_logger(), "init success!");
    }

    robot::Chassis sbv(node);

    // Spin the node
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
