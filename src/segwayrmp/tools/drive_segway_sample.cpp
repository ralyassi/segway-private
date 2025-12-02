#include <math.h>
#include <iostream>
#include <termios.h>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "segway_msgs/srv/ros_set_chassis_enable_cmd.hpp"
#include "segway_msgs/srv/ros_clear_chassis_error_code_cmd.hpp"
#include "segway_msgs/srv/ros_enable_chassis_rotate_cmd.hpp"
#include "segway_msgs/srv/ros_start_chassis_left_rotate_cmd.hpp"
#include "segway_msgs/srv/ros_stop_chassis_rotate_cmd.hpp"
#include "segway_msgs/srv/chassis_send_event.hpp"
#include "segway_msgs/action/ros_set_iap_cmd.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

#define PRINTHELP        'h'
#define ADDLINEVEL       'w'
#define DECLINEVEL       's'
#define ADDANGULARVEL    'a'
#define DECANGULARVEL    'd'
#define PRINTCURVEL      'f'
#define VELRESETZERO     'g'
#define ENABLECMD        'e'
#define CHASSISPAUSE     'q'
#define IAPCHASSIS       'v'
#define IAPMOTORFRONT    'b'
#define IAPMOTORREAR     'n'
#define CLEARERRORS      'm'
#define ENABLEROTATE     'j'
#define LEFTROTATE       'k'
#define STOPROTATE       'l'

using SetChassisEnable = segway_msgs::srv::RosSetChassisEnableCmd;
using ClearChassisError = segway_msgs::srv::RosClearChassisErrorCodeCmd;
using EnableChassisRotate = segway_msgs::srv::RosEnableChassisRotateCmd;
using StartChassisLeftRotate = segway_msgs::srv::RosStartChassisLeftRotateCmd;
using StopChassisRotate = segway_msgs::srv::RosStopChassisRotateCmd;
using ChassisSendEvent = segway_msgs::srv::ChassisSendEvent;
using SetIapCmd = segway_msgs::action::RosSetIapCmd;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
rclcpp::Client<SetChassisEnable>::SharedPtr ros_set_chassis_enable_cmd_client;
rclcpp::Client<ClearChassisError>::SharedPtr ros_clear_chassis_error_code_cmd_client;
rclcpp::Client<EnableChassisRotate>::SharedPtr ros_enable_chassis_rotate_cmd_client;
rclcpp::Client<StartChassisLeftRotate>::SharedPtr ros_start_chassis_left_rotate_cmd_client;
rclcpp::Client<StopChassisRotate>::SharedPtr ros_stop_chassis_rotate_cmd_client;
rclcpp::Service<ChassisSendEvent>::SharedPtr chassis_send_event_srv_server;
rclcpp_action::Client<SetIapCmd>::SharedPtr iap_action_client;

int rate = 10;

char const* print_help() {
    return
        "\t h : Displays the required keys and their meaning\n"
        "\t w : Increase forward speed by 0.1m/s\n"
        "\t s : Decrease forward speed by 0.1m/s\n"
        "\t a : Increase the angular velocity by 0.1rad/s\n"
        "\t d : Decrease the angular velocity by 0.1rad/s\n"
        "\t f : Displays current speed Settings\n"
        "\t g : Speed reset to zero\n"
        "\t e : Chassis enable switch\n"
        "\t q : Running pause. Click 'q'key again to resume running by the previous speed. W/S/A/D keys can also restore chassis running\n"
        "\t v : Iap the central board, please put the central.bin file in /sdcard/firmware/\n"
        "\t b : Iap the motor front board, please put the motor_front.bin file in /sdcard/firmware/\n"
        "\t n : Iap the motor rear board, please put the motor_rear.bin file in /sdcard/firmware/\n"
        "\t m : Clear the error code of the chassis, excluding warnings and exceptions\n"
        "\t j : Enable the chassis to rotate in place switch, 1: enable ; 0:disable\n"
        "\t k : Rotate left in place\n"
        "\t l : Stop rotation in place\n"
        "\t others : Do nothing\n";
}

// ... Keyboard scan functions stay the same ...

void changemode(int dir) { /* ... as before ... */ }
int kbhit (void) { /* ... as before ... */ }
static int scanKeyboard() { /* ... as before ... */ }
char get_keyboard() { return scanKeyboard(); }

void iapActionDoneCb(
    const rclcpp_action::ClientGoalHandle<SetIapCmd>::WrappedResult & result)
{
    RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"),
        "iap result:%d, iap_result_errorcode:%d",
        result.result->iap_result, result.result->error_code);
}
void iapActionFeedbackCb(
    rclcpp_action::ClientGoalHandle<SetIapCmd>::SharedPtr,
    const std::shared_ptr<const SetIapCmd::Feedback> feedback)
{
    RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"),
        "percent_complete : %d", feedback->iap_percent);
}

void drive_chassis_test(rclcpp::Node::SharedPtr node)
{
    static uint16_t set_enable_cmd = 1;
    uint8_t enable_switch = 0;
    static double set_line_speed; 
    static double set_angular_speed;
    static double send_line_speed;
    static double send_angular_speed;
    static uint8_t chassis_pause = 0;
    uint8_t iap_flag = 0;
    uint8_t clear_err_switch = 0;
    uint8_t enable_rotate_switch = 0;
    uint8_t left_rotate_switch = 0;
    uint8_t stop_rotate_switch = 0;

    auto enable_request = std::make_shared<SetChassisEnable::Request>();
    auto clear_err_request = std::make_shared<ClearChassisError::Request>();
    auto enable_rotate_request = std::make_shared<EnableChassisRotate::Request>();
    auto left_rotate_request = std::make_shared<StartChassisLeftRotate::Request>();
    auto stop_rotate_request = std::make_shared<StopChassisRotate::Request>();

    char keyvalue = get_keyboard();
    switch (keyvalue)
    {
        case PRINTHELP:
            printf("%s\n", print_help());
            break;
        case ADDLINEVEL:
            set_line_speed += 0.1;
            send_line_speed = set_line_speed;
            send_angular_speed = set_angular_speed;
            printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
            break;
        case DECLINEVEL:
            set_line_speed -= 0.1;
            send_line_speed = set_line_speed;
            send_angular_speed = set_angular_speed;
            printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
            break;
        case ADDANGULARVEL:
            set_angular_speed += 0.1;
            send_line_speed = set_line_speed;
            send_angular_speed = set_angular_speed;
            printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
            break;
        case DECANGULARVEL:
            set_angular_speed -= 0.1;
            send_line_speed = set_line_speed;
            send_angular_speed = set_angular_speed;
            printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
            break;
        case PRINTCURVEL:
            printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
            break;
        case VELRESETZERO:
            set_line_speed = 0; 
            set_angular_speed = 0;
            send_line_speed = 0;
            send_angular_speed = 0;
            printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
            break;
        case ENABLECMD:
            enable_switch = 1;
            enable_request->ros_set_chassis_enable_cmd = set_enable_cmd;
            ++set_enable_cmd;        
            set_enable_cmd %= 2;
            RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "enable chassis switch[%d]", enable_request->ros_set_chassis_enable_cmd);
            printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
            break;  
        case CHASSISPAUSE:
            ++chassis_pause;
            chassis_pause %= 2;
            if (chassis_pause) {
                send_line_speed = 0;
                send_angular_speed = 0;
                printf("Stop the chassis temporarily\n");
            } else {
                send_line_speed = set_line_speed;
                send_angular_speed = set_angular_speed;
                printf("current set_line_speed[%lf], set_angular_speed[%lf]\n", set_line_speed, set_angular_speed);
            }
            break;   
        case IAPCHASSIS:
            RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "iap central");
            iap_flag = 1;
            break;
        case IAPMOTORFRONT:
            RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"),  "iap motor front");
            iap_flag = 2;
            break;
        case IAPMOTORREAR:
            RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"),  "iap motor rear");
            iap_flag = 3;
            break;
        case CLEARERRORS:
            clear_err_request->clear_chassis_error_code_cmd = 1;
            RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "clear chassis errors[%d]", clear_err_request->clear_chassis_error_code_cmd);
            clear_err_switch = 1;
            break;
        case ENABLEROTATE:
            enable_rotate_request->ros_enable_chassis_rotate_cmd = 1;
            RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "enable chassis rotate[%d]", enable_rotate_request->ros_enable_chassis_rotate_cmd);
            enable_rotate_switch = 1;
            break;
        case LEFTROTATE:
            left_rotate_request->ros_start_chassis_left_rotate_cmd = 2.0; //rad/s
            RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "left rotate[%lf]", left_rotate_request->ros_start_chassis_left_rotate_cmd);
            left_rotate_switch = 1;
            break;
        case STOPROTATE:
            stop_rotate_request->ros_stop_chassis_rotate_cmd = 1;
            RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"), "stop rotate[%d]", stop_rotate_request->ros_stop_chassis_rotate_cmd);
            stop_rotate_switch = 1;
            break;
        default:
            break;
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = send_line_speed;
    cmd_vel.angular.z = send_angular_speed;
    cmd_pub->publish(cmd_vel);

    if (enable_switch){
        ros_set_chassis_enable_cmd_client->async_send_request(enable_request);
    }

    if (iap_flag & 3) {
        auto goal_msg = SetIapCmd::Goal();
        goal_msg.board_index_for_iap = iap_flag;
        iap_action_client->async_send_goal(goal_msg,
            rclcpp_action::Client<SetIapCmd>::SendGoalOptions()
                .feedback_callback = iapActionFeedbackCb,
                .result_callback = iapActionDoneCb
        );
    }

    if (clear_err_switch) {
        ros_clear_chassis_error_code_cmd_client->async_send_request(clear_err_request);
    }

    if (enable_rotate_switch) {
        ros_enable_chassis_rotate_cmd_client->async_send_request(enable_rotate_request);
    }

    if (left_rotate_switch) {
        ros_start_chassis_left_rotate_cmd_client->async_send_request(left_rotate_request);
    }

    if (stop_rotate_switch) {
        ros_stop_chassis_rotate_cmd_client->async_send_request(stop_rotate_request);
    }
}

bool ros_get_chassis_send_event_callback(
    const std::shared_ptr<ChassisSendEvent::Request> req,
    std::shared_ptr<ChassisSendEvent::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("drive_segway_sample"),
        "The ROS2 test node receives the event ID:%d", req->chassis_send_event_id);
    // ...event handling logic...
    res->ros_is_received = true;
    return true;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("drive_segway_sample");

    cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    ros_set_chassis_enable_cmd_client = node->create_client<SetChassisEnable>("ros_set_chassis_enable_cmd_srv");
    ros_clear_chassis_error_code_cmd_client = node->create_client<ClearChassisError>("ros_clear_chassis_error_code_cmd_srv");
    ros_enable_chassis_rotate_cmd_client = node->create_client<EnableChassisRotate>("ros_enable_chassis_rotate_cmd_srv");
    ros_start_chassis_left_rotate_cmd_client = node->create_client<StartChassisLeftRotate>("ros_start_chassis_left_rotate_cmd_srv");
    ros_stop_chassis_rotate_cmd_client = node->create_client<StopChassisRotate>("ros_stop_chassis_rotate_cmd_srv");

    chassis_send_event_srv_server = node->create_service<ChassisSendEvent>("chassis_send_event_srv", &ros_get_chassis_send_event_callback);

    iap_action_client = rclcpp_action::create_client<SetIapCmd>(node, "ros_set_iap_cmd_action");
    RCLCPP_INFO(node->get_logger(), "Waiting for action server to start.");
    while (!iap_action_client->wait_for_action_server(1s)) {
        RCLCPP_INFO(node->get_logger(), "Waiting for action server...");
    }
    RCLCPP_INFO(node->get_logger(), "Action server started.");

    rclcpp::Rate loop_rate(rate);
    printf("%s\n", print_help());
    while (rclcpp::ok()) {
        drive_chassis_test(node);
        loop_rate.sleep();
        rclcpp::spin_some(node);
    }
    
    rclcpp::shutdown();
    return 0;
}
