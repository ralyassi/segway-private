#include "segwayrmp/robot.h"
#include "sensor_msgs/msg/imu.hpp"
#include "Ge_encoder_odometry.h"


#define ODOM_BY_CHASSIS

#define IMU_ANGULAR_VEL_CONVERT_UINIT 0.0009288
#define IMU_LINEAR_VEL_CONVERT_UINIT 0.0023943
#define RAD_DEGREE_CONVER            57.2958

chassis_motors_speed_data_t motorsSpeedData;
chassis_car_speed_data_t    carSpeedData;
uint64_t Speed_TimeStamp;
uint8_t Speed_update;

front_motors_ticks_t frontTicksData;
rear_motors_ticks_t rearTicksData;
uint64_t Ticks_TimeStamp;
uint8_t Ticks_update;

imu_gyr_original_data_t ImuGyrData;
uint64_t ImuGyr_TimeStamp;
uint8_t ImuGyr_update;

imu_acc_original_data_t ImuAccData;
uint64_t ImuAcc_TimeStamp;
uint8_t ImuAcc_update;

odom_pos_xy_t OdomPoseXy;
odom_euler_xy_t OdomEulerXy;
odom_euler_z_t OdomEulerZ;
odom_vel_line_xy_t OdomVelLineXy;
uint64_t Odom_TimeStamp;
uint8_t Odom_update;

float FrontAxleAngle;
uint64_t FrontAxle_TimeStamp;
uint8_t FrontAxle_update;

int chassis_event_id = 0;
// ROS2: You’ll create the service client in a function/method, not as a global
rclcpp::Client<segway_msgs::srv::ChassisSendEvent>::SharedPtr chassis_send_event_srv_client = nullptr;
static robot::Chassis* g_chassis_ptr = nullptr;



static void PubData(StampedBasicFrame *frame)
{
    if (frame->type_id == Chassis_Data_Motors_Speed)
    {
        memcpy(&motorsSpeedData, frame->data, sizeof(motorsSpeedData)); //Speed data from chassis
        Speed_TimeStamp = frame->timestamp;
        Speed_update |= 1;
    }
    else if (frame->type_id == Chassis_Data_Car_Speed)
    {
        memcpy(&carSpeedData, frame->data, sizeof(carSpeedData)); //Speed data from chassis
        Speed_TimeStamp = frame->timestamp;
        Speed_update |= 2;
    }
    else if (frame->type_id == Chassis_Data_Front_Ticks)
    {
        memcpy(&frontTicksData, frame->data, sizeof(frontTicksData)); //Ticks data from chassis
        Ticks_TimeStamp = frame->timestamp;
        Ticks_update |= 1;
        //RCLCPP_INFO(g_chassis_ptr->get_logger(), "fLTICKS:%d, frticks:%d", frontTicksData.fl_ticks, frontTicksData.fr_ticks);
    }
    else if (frame->type_id == Chassis_Data_Rear_Ticks)
    {
        memcpy(&rearTicksData, frame->data, sizeof(rearTicksData)); //Ticks data from chassis
        Ticks_TimeStamp = frame->timestamp;
        Ticks_update |= 2;
        //RCLCPP_INFO(g_chassis_ptr->get_logger(), "rLTICKS:%d, rrticks:%d", rearTicksData.rl_ticks, rearTicksData.rr_ticks);
    }
    else if (frame->type_id == Chassis_Data_Imu_Gyr)
    {
        memcpy(&ImuGyrData, frame->data, sizeof(ImuGyrData)); //Ticks data from chassis
        ImuGyr_TimeStamp = frame->timestamp;
        ImuGyr_update = 1;
        //RCLCPP_INFO(g_chassis_ptr->get_logger(), "GYR0:%d, gyr1:%d, gyr2:%d", ImuGyrData.gyr[0], ImuGyrData.gyr[1], ImuGyrData.gyr[2]);
    }
    else if (frame->type_id == Chassis_Data_Imu_Acc)
    {
        memcpy(&ImuAccData, frame->data, sizeof(ImuAccData)); //Ticks data from chassis
        ImuAcc_TimeStamp = frame->timestamp;
        ImuAcc_update = 1;
        //RCLCPP_INFO(g_chassis_ptr->get_logger(), "ACC0:%d, acc1:%d, acc2:%d", ImuAccData.acc[0], ImuAccData.acc[1], ImuAccData.acc[2]);
    }
    else if (frame->type_id == Chassis_Data_Odom_Pose_xy)
    {
        memcpy(&OdomPoseXy, frame->data, sizeof(OdomPoseXy)); //Ticks data from chassis
        Odom_TimeStamp = frame->timestamp;
        Odom_update |= 1;
        //RCLCPP_INFO(g_chassis_ptr->get_logger(), "odomPosX:%f, odomPosY:%f", OdomPoseXy.pos_x, OdomPoseXy.pos_y);
    }
    else if (frame->type_id == Chassis_Data_Odom_Euler_xy)
    {
        memcpy(&OdomEulerXy, frame->data, sizeof(OdomEulerXy)); //Ticks data from chassis
        Odom_TimeStamp = frame->timestamp;
        Odom_update |= 2;
        //RCLCPP_INFO(g_chassis_ptr->get_logger(), "OdomEulerX:%f, OdomEulerY:%f", OdomEulerXy.euler_x, OdomEulerXy.euler_y);
    }
    else if (frame->type_id == Chassis_Data_Odom_Euler_z)
    {
        memcpy(&OdomEulerZ, frame->data, sizeof(OdomEulerZ)); //Ticks data from chassis
        Odom_TimeStamp = frame->timestamp;
        Odom_update |= 4;
        //RCLCPP_INFO(g_chassis_ptr->get_logger(), "OdomEulerZ:%f", OdomEulerZ.euler_z);
    }
    else if (frame->type_id == Chassis_Data_Odom_Linevel_xy)
    {
        memcpy(&OdomVelLineXy, frame->data, sizeof(OdomVelLineXy)); //Ticks data from chassis
        Odom_TimeStamp = frame->timestamp;
        Odom_update |= 8;
        //RCLCPP_INFO(g_chassis_ptr->get_logger(), "OdomVelLineX:%f, OdomVelLineY:%f", OdomVelLineXy.vel_line_x, OdomVelLineXy.vel_line_y);
    }
    else if (frame->type_id == Chassis_Data_Front_Encoder_Angle)
    {
        memcpy(&FrontAxleAngle, frame->data, sizeof(FrontAxleAngle));
        FrontAxle_TimeStamp = frame->timestamp;
        FrontAxle_update = 1;
    }


    else
    {
        // Unknown frame type: print a warning and some raw data for debugging
        RCLCPP_INFO(
            g_chassis_ptr->get_logger(),
            "Received unknown frame type: %d (timestamp: %ld)", 
            frame->type_id, frame->timestamp
        );

        int max_bytes = std::min((size_t)8, sizeof(frame->data)); // Print up to 8 bytes
        std::ostringstream oss;
        oss << "First bytes:";
        for (int i = 0; i < max_bytes; ++i) {
            oss << " " << std::hex << std::setw(2) << std::setfill('0')
                << (static_cast<int>(reinterpret_cast<uint8_t*>(frame->data)[i]) & 0xFF);
        }
        RCLCPP_INFO(g_chassis_ptr->get_logger(), "%s", oss.str().c_str());
    }

}

static void EvnetPubData(int event_no) 
{
    chassis_event_id = event_no;
    robot::Chassis::chassis_send_event_callback(event_no);
}

//The timestamp from the upper computer is the count of microseconds，
rclcpp::Time timestamp2rostime(int64_t timestamp)
{
    uint32_t sec_ = timestamp / 1000000;
    uint32_t nsec_ = (timestamp % 1000000) * 1000;
    return rclcpp::Time(sec_, nsec_);
}

double getOrientationX()
{
    float x = OdomEulerXy.euler_x / RAD_DEGREE_CONVER / 2.0;
    float y = OdomEulerXy.euler_y / RAD_DEGREE_CONVER / 2.0;
    float z = OdomEulerZ.euler_z / RAD_DEGREE_CONVER / 2.0;

    float fCosHRoll = cos(x);
    float fSinHRoll = sin(x);
    float fCosHPitch = cos(y);
    float fSinHPitch = sin(y);
    float fCosHYaw = cos(z);
    float fSinHYaw = sin(z);

    return (fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw);
}

double getOrientationY()
{
    float x = OdomEulerXy.euler_x / RAD_DEGREE_CONVER / 2.0;
    float y = OdomEulerXy.euler_y / RAD_DEGREE_CONVER / 2.0;
    float z = OdomEulerZ.euler_z / RAD_DEGREE_CONVER / 2.0;

    float fCosHRoll = cos(x);
    float fSinHRoll = sin(x);
    float fCosHPitch = cos(y);
    float fSinHPitch = sin(y);
    float fCosHYaw = cos(z);
    float fSinHYaw = sin(z);

    return (fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw);
}

double getOrientationZ()
{
    float x = OdomEulerXy.euler_x / RAD_DEGREE_CONVER / 2.0;
    float y = OdomEulerXy.euler_y / RAD_DEGREE_CONVER / 2.0;
    float z = OdomEulerZ.euler_z / RAD_DEGREE_CONVER / 2.0;

    float fCosHRoll = cos(x);
    float fSinHRoll = sin(x);
    float fCosHPitch = cos(y);
    float fSinHPitch = sin(y);
    float fCosHYaw = cos(z);
    float fSinHYaw = sin(z);

    return (fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw);
}

double getOrientationW()
{
    float x = OdomEulerXy.euler_x / RAD_DEGREE_CONVER / 2.0;
    float y = OdomEulerXy.euler_y / RAD_DEGREE_CONVER / 2.0;
    float z = OdomEulerZ.euler_z / RAD_DEGREE_CONVER / 2.0;

    float fCosHRoll = cos(x);
    float fSinHRoll = sin(x);
    float fCosHPitch = cos(y);
    float fSinHPitch = sin(y);
    float fCosHYaw = cos(z);
    float fSinHYaw = sin(z);

    return (fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw);
}

void ros_set_iap_cmd_callback(
    const std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<rclcpp_action::ServerGoalHandle<segway_msgs::action::RosSetIapCmd>> goal_handle)
{  //rclcpp_action::ServerGoalHandle<segway_msgs::action::RosSetIapCmd>::SharedPtr goal_handle)
    const auto goal = goal_handle->get_goal();
    switch (goal->board_index_for_iap)
    {
        case 1:
            RCLCPP_INFO(node->get_logger(), "iap for central board");
            iapCentralBoard();
            break;
        case 2:
            RCLCPP_INFO(node->get_logger(), "IAP for the front wheel board");
            iapMotorBoard(motor_front);
            break;
        case 3:
            RCLCPP_INFO(node->get_logger(), "IAP for the rear wheel board");
            iapMotorBoard(motor_rear);
            break;
        case 4:
            RCLCPP_INFO(node->get_logger(), "IAP for the brake board");
            iapBrakeBoard();
            break;
        case 5:
            RCLCPP_INFO(node->get_logger(), "IAP for all board: brake; motor_rear; motor_front; central");
            checkAndIapAllFw();
            break;
        default:
            RCLCPP_INFO(node->get_logger(),
                "board_index_for_iap[%d] value out of normal range [1,2,3,4]",
                goal->board_index_for_iap);
            break;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::Rate loop_rate(2.0);

    auto feedback = std::make_shared<segway_msgs::action::RosSetIapCmd::Feedback>();
    auto result = std::make_shared<segway_msgs::action::RosSetIapCmd::Result>();

    while (!isHostIapOver())
    {
        if (goal_handle->is_canceling() || !rclcpp::ok())
        {
            RCLCPP_INFO(node->get_logger(), "ros_set_iap_cmd_action: Preempted");
            goal_handle->canceled(result);
            setHostIapCanceled();
            return;
        }
        feedback->iap_percent = getAllIapProgress();
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    rclcpp::sleep_for(std::chrono::milliseconds(100));
    result->iap_result = getHostIapResult();
    result->error_code = getHostIapErrorCode();
    if (result->iap_result == IAP_STATE_COMPLETE)
    {
        goal_handle->succeed(result);
    }
    else
    {
        goal_handle->abort(result);
    }
}



namespace robot
{

rclcpp::Logger Chassis::get_logger() const { return node_->get_logger(); }

Chassis::Chassis(const std::shared_ptr<rclcpp::Node> & node) : node_(node)
{	
     g_chassis_ptr = this;
    timestamp_data.on_new_data = PubData;
    aprctrl_datastamped_jni_register(&timestamp_data);

    event_data.event_callback = EvnetPubData;
    
    aprctrl_eventcallback_jni_register(&event_data);

    velocity_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_segway", 1, std::bind(&Chassis::cmd_vel_callback, this, std::placeholders::_1));

    bms_fb_pub = node_->create_publisher<segway_msgs::msg::BmsFb>("bms_fb", 10);
    chassis_ctrl_src_fb_pub = node_->create_publisher<segway_msgs::msg::ChassisCtrlSrcFb>("chassis_ctrl_src_fb", 10);
    chassis_mileage_meter_fb_pub = node_->create_publisher<segway_msgs::msg::ChassisMileageMeterFb>("chassis_mileage_meter_fb", 10);
    chassis_mode_fb_pub = node_->create_publisher<segway_msgs::msg::ChassisModeFb>("chassis_mode_fb", 10);
    error_code_fb_pub = node_->create_publisher<segway_msgs::msg::ErrorCodeFb>("error_code_fb", 10);
    front_axle_angle_fb_pub = node_->create_publisher<segway_msgs::msg::FrontAxleAngleFb>("front_axle_angle_fb", 10);
    motor_work_mode_fb_pub = node_->create_publisher<segway_msgs::msg::MotorWorkModeFb>("motor_work_mode_fb", 10);
    speed_fb_pub = node_->create_publisher<segway_msgs::msg::SpeedFb>("speed_fb", 10);
    ticks_fb_pub = node_->create_publisher<segway_msgs::msg::TicksFb>("ticks_fb", 10);
    Odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    Imu_pub = node_->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node_);


    chassis_send_event_srv_client = node_->create_client<segway_msgs::srv::ChassisSendEvent>("chassis_send_event_srv");

    ros_enable_chassis_rotate_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosEnableChassisRotateCmd>(
            "ros_enable_chassis_rotate_cmd_srv",
            std::bind(&Chassis::ros_enable_chassis_rotate_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_get_chassis_rotate_switch_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosGetChassisRotateSwitchCmd>(
            "ros_get_chassis_rotate_switch_cmd_srv",
            std::bind(&Chassis::ros_get_chassis_rotate_switch_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_clear_chassis_error_code_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosClearChassisErrorCodeCmd>(
            "ros_clear_chassis_error_code_cmd_srv",
            std::bind(&Chassis::ros_clear_chassis_error_code_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_get_chassis_sn_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosGetChassisSnCmd>(
            "ros_get_chassis_sn_cmd_srv",
            std::bind(&Chassis::ros_get_chassis_sn_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_get_load_param_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosGetLoadParamCmd>(
            "ros_get_load_param_cmd_srv",
            std::bind(&Chassis::ros_get_load_param_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_get_sw_version_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosGetSwVersionCmd>(
            "ros_get_sw_version_cmd_srv",
            std::bind(&Chassis::ros_get_sw_version_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_get_vel_max_feedback_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosGetVelMaxFeedbackCmd>(
            "ros_get_vel_max_feedback_cmd_srv",
            std::bind(&Chassis::ros_get_vel_max_feedback_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_set_chassis_buzzer_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosSetChassisBuzzerCmd>(
            "ros_set_chassis_buzzer_cmd_srv",
            std::bind(&Chassis::ros_set_chassis_buzzer_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_set_chassis_enable_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosSetChassisEnableCmd>(
            "ros_set_chassis_enable_cmd_srv",
            std::bind(&Chassis::ros_set_chassis_enable_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_set_chassis_poweroff_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosSetChassisPoweroffCmd>(
            "ros_set_chassis_poweroff_cmd_srv",
            std::bind(&Chassis::ros_set_chassis_poweroff_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_set_load_param_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosSetLoadParamCmd>(
            "ros_set_load_param_cmd_srv",
            std::bind(&Chassis::ros_set_load_param_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_set_vel_max_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosSetVelMaxCmd>(
            "ros_set_vel_max_cmd_srv",
            std::bind(&Chassis::ros_set_vel_max_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_reset_host_power_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosResetHostPowerCmd>(
            "ros_reset_host_power_cmd_srv",
            std::bind(&Chassis::ros_reset_host_power_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_start_chassis_left_rotate_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosStartChassisLeftRotateCmd>(
            "ros_start_chassis_left_rotate_cmd_srv",
            std::bind(&Chassis::ros_start_chassis_left_rotate_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_start_chassis_right_rotate_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosStartChassisRightRotateCmd>(
            "ros_start_chassis_right_rotate_cmd_srv",
            std::bind(&Chassis::ros_start_chassis_right_rotate_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_stop_chassis_rotate_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosStopChassisRotateCmd>(
            "ros_stop_chassis_rotate_cmd_srv",
            std::bind(&Chassis::ros_stop_chassis_rotate_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_get_rotate_function_cfg_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosGetRotateFunctionCfgCmd>(
            "ros_get_rotate_function_cfg_cmd_srv",
            std::bind(&Chassis::ros_get_rotate_function_cfg_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_set_cfg_rotate_function_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosSetCfgRotateFunctionCmd>(
            "ros_set_cfg_rotate_function_cmd_srv",
            std::bind(&Chassis::ros_set_cfg_rotate_function_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    ros_get_host_and_chassis_match_cmd_srv_server =
        node_->create_service<segway_msgs::srv::RosGetHostAndChassisMatchCmd>(
            "ros_get_host_and_chassis_match_cmd_srv",
            std::bind(&Chassis::ros_get_host_and_chassis_match_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    update_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&Chassis::TimeUpdate1000Hz, this));
    update_timer2_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&Chassis::TimeUpdate1Hz, this));

    RCLCPP_INFO(node_->get_logger(), "ros_set_iap_cmd_action started");
}


/* code */
void Chassis::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_input)
{
    double angular_vel_ = cmd_input->angular.z; // get angular velocity, rad/s
    double linear_vel_ = cmd_input->linear.x;   // get linear velocity, m/s

    set_cmd_vel(linear_vel_, angular_vel_); // Configure coefficients according to chassis parameters

    //RCLCPP_INFO(node_->get_logger(), "angular_temp:%f  rad/s   linear_temp:%f  m/s", angular_vel_, linear_vel_);
}

void Chassis::ros_enable_chassis_rotate_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosEnableChassisRotateCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosEnableChassisRotateCmd::Response> res)
{
    if (!req->ros_enable_chassis_rotate_cmd) {
        res->chassis_enable_rotate_result = enable_rotate_switch(0);
        RCLCPP_INFO(node_->get_logger(), "chassis_enable_rotate_result disable: %d", res->chassis_enable_rotate_result);
    }
    else if (req->ros_enable_chassis_rotate_cmd) {
        res->chassis_enable_rotate_result = enable_rotate_switch(1);
        RCLCPP_INFO(node_->get_logger(), "chassis_enable_rotate_result enable: %d", res->chassis_enable_rotate_result);
    }
}

void Chassis::ros_get_chassis_rotate_switch_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosGetChassisRotateSwitchCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosGetChassisRotateSwitchCmd::Response> res)
{
    if (!req->ros_get_chassis_rotate_cmd) {
        // Optionally, you can log or handle invalid request
        return;
    }
    res->chassis_rotate_state = get_rotate_switch_stat();
    RCLCPP_INFO(node_->get_logger(), "chassis_rotate_state: %d", res->chassis_rotate_state);
}

            
bool Chassis::ros_clear_chassis_error_code_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosClearChassisErrorCodeCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosClearChassisErrorCodeCmd::Response> res)
{
    if (!req->clear_chassis_error_code_cmd)
    {
        return false;
    }
    res->clear_chassis_error_code_result = clear_chassis_error_code();
    RCLCPP_INFO(node_->get_logger(), "clear_chassis_error_code:%d ", res->clear_chassis_error_code_result);
    return true;
}


void Chassis::ros_get_load_param_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosGetLoadParamCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosGetLoadParamCmd::Response> res)
{
    if (!req->ros_get_load_param) {
        // Optionally log or handle invalid request
        return;
    }
    res->get_load_param = get_chassis_load_state();
    RCLCPP_INFO(node_->get_logger(), "get_load_param: %d", res->get_load_param);
}


void Chassis::ros_get_chassis_sn_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosGetChassisSnCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosGetChassisSnCmd::Response> res)
{
    if (!req->ros_get_chassis_sn) {
        // Optionally log or handle invalid request
        return;
    }

    char chassis_sn[30];
    if (get_chassis_central_SN(chassis_sn)) {
        res->chassis_sn = chassis_sn;
        RCLCPP_INFO(node_->get_logger(), "chassis_SN: %s", res->chassis_sn.c_str());
    }
    // If get_chassis_central_SN fails, response is left empty (optional: handle error)
}


void Chassis::ros_get_sw_version_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosGetSwVersionCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosGetSwVersionCmd::Response> res)
{
    if (!req->ros_get_sw_version_cmd) {
        // Optionally log or handle invalid request
        return;
    }
    res->host_version = get_host_version();
    res->host_pantch_version = get_host_patch_version();
    res->central_version = get_chassis_central_version();
    res->motor_version = get_chassis_motor_version();
    RCLCPP_INFO(node_->get_logger(), "req.ros_get_sw_version_cmd: %d", req->ros_get_sw_version_cmd);
    RCLCPP_INFO(node_->get_logger(), "res.host_version: %d, res.host_pantch_version: %d, res.central_version: %d, res.motor_version: %d",
        res->host_version, res->host_pantch_version, res->central_version, res->motor_version);
}


void Chassis::ros_get_vel_max_feedback_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosGetVelMaxFeedbackCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosGetVelMaxFeedbackCmd::Response> res)
{
    if (!req->ros_get_vel_max_fb_cmd) {
        // Optionally log or handle invalid request
        return;
    }
    res->forward_max_vel_fb = get_line_forward_max_vel_fb();
    res->backward_max_vel_fb = get_line_backward_max_vel_fb();
    res->angular_max_vel_fb = get_angular_max_vel_fb();
    RCLCPP_INFO(node_->get_logger(), "req.ros_get_vel_max_fb_cmd: %d", req->ros_get_vel_max_fb_cmd);
    RCLCPP_INFO(node_->get_logger(), "res.forward_max_vel_fb: %d, res.backward_max_vel_fb: %d, res.angular_max_vel_fb: %d",
       res->forward_max_vel_fb, res->backward_max_vel_fb, res->angular_max_vel_fb);
}


void Chassis::ros_set_chassis_buzzer_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosSetChassisBuzzerCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosSetChassisBuzzerCmd::Response> res)
{
    uint8_t ret;
    if (req->ros_set_chassis_buzzer_cmd == 0) {
        ret = set_chassis_buzzer(0);
    } else {
        ret = set_chassis_buzzer(1);
    }
    RCLCPP_INFO(node_->get_logger(), "req.ros_set_chassis_buzzer_cmd: %d, set_chassis_buzzer(): %d",
       req->ros_set_chassis_buzzer_cmd, ret);

    res->set_buzzer_result = ret;
}


void Chassis::ros_set_chassis_enable_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosSetChassisEnableCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosSetChassisEnableCmd::Response> res)
{
    uint8_t ret;
    if (!req->ros_set_chassis_enable_cmd) {
        ret = set_enable_ctrl(0);
    } else {
        ret = set_enable_ctrl(1);
    }
    RCLCPP_INFO(node_->get_logger(), "req.ros_set_chassis_enable_cmd: %d, set_enable_ctrl(): %d",
       req->ros_set_chassis_enable_cmd, ret);

    res->chassis_set_chassis_enable_result = ret;
}


void Chassis::ros_set_chassis_poweroff_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosSetChassisPoweroffCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosSetChassisPoweroffCmd::Response> res)
{
    if (!req->ros_set_chassis_poweroff_cmd) {
        // Optionally log or handle invalid request
        return;
    }
    uint8_t ret = set_chassis_poweroff();
    RCLCPP_INFO(node_->get_logger(), "req.ros_set_chassis_poweroff_cmd: %d, set_chassis_poweroff(): %d",
       req->ros_set_chassis_poweroff_cmd, ret);

    res->chassis_set_poweroff_result = ret;
}


void Chassis::ros_set_load_param_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosSetLoadParamCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosSetLoadParamCmd::Response> res)
{
    int16_t value = req->ros_set_load_param;
    if (value != 0 && value != 1) {
        // Optionally log or handle invalid value
        return;
    }
    uint8_t ret = set_chassis_load_state(value);
    RCLCPP_INFO(node_->get_logger(), "req.ros_set_load_param: %d, set_chassis_load_state(): %d", value, ret);

    res->chassis_set_load_param_result = ret;
}


void Chassis::ros_set_vel_max_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosSetVelMaxCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosSetVelMaxCmd::Response> res)
{
    double forward = req->ros_set_forward_max_vel;
    double backward = req->ros_set_backward_max_vel;
    double angular = req->ros_set_angular_max_vel;
    uint8_t ret_forw = set_line_forward_max_vel(forward);
    uint8_t ret_back = set_line_backward_max_vel(backward);
    uint8_t ret_angl = set_angular_max_vel(angular);
    RCLCPP_INFO(node_->get_logger(), "req.ros_set_forward_max_vel: %f, req.ros_set_backward_max_vel: %f, req.ros_set_angular_max_vel: %f",
        forward, backward, angular);
    RCLCPP_INFO(node_->get_logger(), "set_line_forward_max_vel(): %d, set_line_backward_max_vel(): %d, set_angular_max_vel(): %d",
        ret_forw, ret_back, ret_angl);

    res->chassis_set_max_vel_result = ret_forw | ret_back | ret_angl;
}


void Chassis::ros_reset_host_power_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosResetHostPowerCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosResetHostPowerCmd::Response> res)
{
    uint16_t reset_interval_time = req->reset_interval_time;
    res->reset_result = reset_host_power_time_s(reset_interval_time);
    RCLCPP_INFO(node_->get_logger(), "req.reset_interval_time: %d", reset_interval_time);
    RCLCPP_INFO(node_->get_logger(), "res.reset_result: %d", res->reset_result);
}


void Chassis::ros_start_chassis_left_rotate_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosStartChassisLeftRotateCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosStartChassisLeftRotateCmd::Response> res)
{
    double turn_vel = req->ros_start_chassis_left_rotate_cmd;
    set_vel_of_rotation(turn_vel);
    res->chassis_left_rotate_result = enable_chassis_in_situ_rotation(0);
    RCLCPP_INFO(node_->get_logger(), "req.ros_start_chassis_left_rotate_cmd: %lf", turn_vel);
    RCLCPP_INFO(node_->get_logger(), "res.chassis_left_rotate_result: %d", res->chassis_left_rotate_result);
}


void Chassis::ros_start_chassis_right_rotate_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosStartChassisRightRotateCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosStartChassisRightRotateCmd::Response> res)
{
    double turn_vel = req->ros_start_chassis_right_rotate_cmd;
    set_vel_of_rotation(turn_vel);
    res->chassis_right_rotate_result = enable_chassis_in_situ_rotation(1);
    RCLCPP_INFO(node_->get_logger(), "req.ros_start_chassis_right_rotate_cmd: %lf", turn_vel);
    RCLCPP_INFO(node_->get_logger(), "res.chassis_right_rotate_result: %d", res->chassis_right_rotate_result);
}


void Chassis::ros_stop_chassis_rotate_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosStopChassisRotateCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosStopChassisRotateCmd::Response> res)
{
    if (!req->ros_stop_chassis_rotate_cmd) {
        // Optionally log or handle invalid request
        return;
    }
    disable_chassis_in_situ_ratotion();
    RCLCPP_INFO(node_->get_logger(), "req.ros_stop_chassis_rotate_cmd: %d", req->ros_stop_chassis_rotate_cmd);
}


void Chassis::ros_get_rotate_function_cfg_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosGetRotateFunctionCfgCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosGetRotateFunctionCfgCmd::Response> res)
{
    (void)req;  // unused
    res->cfg_in_situ_rotation_cmd = get_rotate_scheme_cfg();
    RCLCPP_INFO(node_->get_logger(), "res.cfg_in_situ_rotation_cmd: %d", res->cfg_in_situ_rotation_cmd);
}

void Chassis::ros_set_cfg_rotate_function_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosSetCfgRotateFunctionCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosSetCfgRotateFunctionCmd::Response> res)
{
    if (!req->cfg_in_situ_rotation_function) {
        cfg_rotate_scheme_switch(0);
    } else {
        cfg_rotate_scheme_switch(1);
    }
    RCLCPP_INFO(node_->get_logger(), "req.cfg_in_situ_rotation_function: %d", req->cfg_in_situ_rotation_function);
    (void)res;  // unused
}

void Chassis::ros_get_host_and_chassis_match_cmd_callback(
    const std::shared_ptr<segway_msgs::srv::RosGetHostAndChassisMatchCmd::Request> req,
    std::shared_ptr<segway_msgs::srv::RosGetHostAndChassisMatchCmd::Response> res)
{
    (void)req;  // unused
    res->is_matched_with_firmware = check_version_matched_with_fw();
    RCLCPP_INFO(node_->get_logger(), "res.is_matched_with_firmware: %d", res->is_matched_with_firmware);
}


void Chassis::chassis_send_event_callback(int event_no)
{
    auto request = std::make_shared<segway_msgs::srv::ChassisSendEvent::Request>();
    request->chassis_send_event_id = static_cast<int16_t>(event_no);

    // Call the service asynchronously and wait for response (synchronously)
    auto future = chassis_send_event_srv_client->async_send_request(request);

    // Option 1: Wait for response synchronously (simple/old style)
    if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
    {
        auto response = future.get();
        //RCLCPP_INFO(node_->get_logger(),
        //    "send chassis_send_event_srv, chassis_send_event_id: %d", request->chassis_send_event_id);
        //RCLCPP_INFO(node_->get_logger(),
        //    "receive chassis_send_event_srv, ros_is_received: %d", response->ros_is_received);
    }
    else
    {
        //RCLCPP_ERROR(node_->get_logger(), "Failed to call service chassis_send_event_srv");
    }
}


void Chassis::PubOdomToRosOdom(Odometry odom_data)
{
    // nav_msgs::msg::Odometry ROS_odom; // Should be a local or member variable, initialized as ROS2 type
    ROS_odom.header.stamp = node_->get_clock()->now(); // timestamp2rostime(odom_data.TimeStamp);
    ROS_odom.header.frame_id = "odom";
    ROS_odom.pose.pose.position.x = odom_data.pose_.x;
    ROS_odom.pose.pose.position.y = odom_data.pose_.y;
    ROS_odom.pose.pose.position.z = 0;
    ROS_odom.pose.pose.orientation.x = 0;
    ROS_odom.pose.pose.orientation.y = 0;
    ROS_odom.pose.pose.orientation.z = sin(odom_data.pose_.orientation / 2);
    ROS_odom.pose.pose.orientation.w = cos(odom_data.pose_.orientation / 2);
    ROS_odom.twist.twist.linear.x = odom_data.twist_.v_x;
    ROS_odom.twist.twist.linear.y = odom_data.twist_.v_y;
    ROS_odom.twist.twist.linear.z = 0;
    ROS_odom.twist.twist.angular.x = 0;
    ROS_odom.twist.twist.angular.y = 0;
    ROS_odom.twist.twist.angular.z = odom_data.twist_.w_z;
    Odom_pub->publish(ROS_odom);  // Note the arrow for ROS2 publishers
}

void Chassis::PubImuToRosImu(void)
{
    uint64_t imu_stamp = ImuGyr_TimeStamp > ImuAcc_TimeStamp ? ImuGyr_TimeStamp : ImuAcc_TimeStamp;
    // sensor_msgs::msg::Imu ros_imu; // Should be a local or member variable, initialized as ROS2 type
    ros_imu.header.stamp = node_->get_clock()->now(); // timestamp2rostime(imu_stamp);
    ros_imu.header.frame_id = "robot_imu";
    ros_imu.angular_velocity.x = static_cast<double>(ImuGyrData.gyr[0]) / 900.0;
    ros_imu.angular_velocity.y = static_cast<double>(ImuGyrData.gyr[1]) / 900.0;
    ros_imu.angular_velocity.z = static_cast<double>(ImuGyrData.gyr[2]) / 900.0;
    ros_imu.linear_acceleration.x = static_cast<double>(ImuAccData.acc[0]) / 4000.0 * 9.8;
    ros_imu.linear_acceleration.y = static_cast<double>(ImuAccData.acc[1]) / 4000.0 * 9.8;
    ros_imu.linear_acceleration.z = static_cast<double>(ImuAccData.acc[2]) / 4000.0 * 9.8;
    
    // Rashid: add covariance (example values, not sensor specific)
    for (int i = 0; i < 9; ++i) {
        ros_imu.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
        ros_imu.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
    }
    
    Imu_pub->publish(ros_imu); // Note the arrow for ROS2 publishers
}


void Chassis::TimeUpdate1000Hz()
{
    if (Speed_update == 3)
    {
        speed_fb.car_speed = carSpeedData.car_speed;
        speed_fb.car_speed /= LINE_SPEED_TRANS_GAIN_MPS;
        speed_fb.turn_speed = carSpeedData.turn_speed;
        speed_fb.turn_speed /= ANGULAR_SPEED_TRANS_GAIN_RADPS;
        speed_fb.fl_speed = motorsSpeedData.fl_speed;
        speed_fb.fl_speed /= LINE_SPEED_TRANS_GAIN_MPS;
        speed_fb.fr_speed = motorsSpeedData.fr_speed;
        speed_fb.fr_speed /= LINE_SPEED_TRANS_GAIN_MPS;
        speed_fb.rl_speed = motorsSpeedData.rl_speed;
        speed_fb.rl_speed /= LINE_SPEED_TRANS_GAIN_MPS;
        speed_fb.rr_speed = motorsSpeedData.rr_speed;
        speed_fb.rr_speed /= LINE_SPEED_TRANS_GAIN_MPS;
        speed_fb.speed_timestamp = Speed_TimeStamp;
        Speed_update = 0;
        speed_fb_pub->publish(speed_fb);
    }

    if (Ticks_update == 3)
    {
        ticks_fb.fl_ticks = frontTicksData.fl_ticks;
        ticks_fb.fr_ticks = frontTicksData.fr_ticks;
        ticks_fb.rl_ticks = rearTicksData.rl_ticks;
        ticks_fb.rr_ticks = rearTicksData.rr_ticks;
        ticks_fb.ticks_timestamp = Ticks_TimeStamp;
        Ticks_update = 0;
        ticks_fb_pub->publish(ticks_fb);
#ifndef ODOM_BY_CHASSIS
        SensorData::Ticks tick_msg(Ticks_TimeStamp, frontTicksData.l_ticks, frontTicksData.r_ticks);
        if (robot::Chassis::m_ge_encoder.add_ticks(tick_msg))
        {
            Odometry odome = robot::Chassis::m_ge_encoder.GetOdometry();
            PubOdomToRosOdom(odome);
        }
#endif
    }

    if (ImuGyr_update == 1 && ImuAcc_update == 1)
    {
#ifndef ODOM_BY_CHASSIS
        double yaw_radps = 0;
        yaw_radps = static_cast<double>(ImuGyrData.gyr[2]) / 900;
        SensorData::BaseImu imu_msg(ImuAcc_TimeStamp, yaw_radps);
        if (robot::Chassis::m_ge_encoder.add_imubase(imu_msg))
        {
            Odometry odome = robot::Chassis::m_ge_encoder.GetOdometry();
            PubOdomToRosOdom(odome);
        }
#endif
        PubImuToRosImu();

        ImuGyr_update = 0;
        ImuAcc_update = 0;
    }

#ifdef ODOM_BY_CHASSIS
    static uint64_t time_pre = 0;
    if (Odom_update == 15)
    {
        Odom_update = 0;

        // geometry_msgs::msg::Quaternion odom_quat = ... // tf::createQuaternionMsgFromYaw is not available directly in ROS2. Use tf2.
        geometry_msgs::msg::Quaternion odom_quat;
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(0, 0, OdomEulerZ.euler_z / RAD_DEGREE_CONVER);
        odom_quat.x = tf2_quat.x();
        odom_quat.y = tf2_quat.y();
        odom_quat.z = tf2_quat.z();
        odom_quat.w = tf2_quat.w();

        ROS_odom.header.stamp = node_->get_clock()->now(); //timestamp2rostime(Odom_TimeStamp);
        ROS_odom.header.frame_id = "odom";
        ROS_odom.pose.pose.position.x = OdomPoseXy.pos_x;
        ROS_odom.pose.pose.position.y = OdomPoseXy.pos_y;
        ROS_odom.pose.pose.position.z = 0;
        ROS_odom.pose.pose.orientation.x = getOrientationX();
        ROS_odom.pose.pose.orientation.y = getOrientationY();
        ROS_odom.pose.pose.orientation.z = getOrientationZ();
        ROS_odom.pose.pose.orientation.w = getOrientationW();
        ROS_odom.child_frame_id = "base_link";
        ROS_odom.twist.twist.linear.x = static_cast<double>(carSpeedData.car_speed) / LINE_SPEED_TRANS_GAIN_MPS;
        ROS_odom.twist.twist.linear.y = 0;
        ROS_odom.twist.twist.linear.z = 0;
        ROS_odom.twist.twist.angular.x = static_cast<double>(ImuGyrData.gyr[0]) / 900.0;
        ROS_odom.twist.twist.angular.y = static_cast<double>(ImuGyrData.gyr[1]) / 900.0;
        ROS_odom.twist.twist.angular.z = static_cast<double>(ImuGyrData.gyr[2]) / 900.0;

        odom_trans.header.stamp = node_->get_clock()->now(); //timestamp2rostime(Odom_TimeStamp);
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = OdomPoseXy.pos_x;
        odom_trans.transform.translation.y = OdomPoseXy.pos_y;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster->sendTransform(odom_trans);

        if ((Odom_TimeStamp - time_pre) > 100000)
        {
            static uint8_t first = 1;
            if (first)
            {
                first = 0;
            }
            else
            {
                //uint64_t timegap = (Odom_TimeStamp - time_pre);
                //printf("!!!!!!!!!! timeout !!!timegap: %lu, curtime:%lu, pretime:%lu\n", timegap, Odom_TimeStamp, time_pre);
            }
        }
        time_pre = Odom_TimeStamp;
        Odom_pub->publish(ROS_odom);
    }
#endif

    if (FrontAxle_update == 1)
    {
        FrontAxle_update = 0;
        front_axle_angle_fb.chassis_front_axle_angle = FrontAxleAngle;
        front_axle_angle_fb.chassis_front_axle_angle_timestamp = FrontAxle_TimeStamp;
        front_axle_angle_fb_pub->publish(front_axle_angle_fb);
    }
}

void Chassis::TimeUpdate1Hz()
{
    bms_fb.bat_soc = get_bat_soc();
    bms_fb.bat_charging = get_bat_charging();
    bms_fb.bat_vol = get_bat_mvol();
    bms_fb.bat_current = get_bat_mcurrent();
    bms_fb.bat_temp = get_bat_temp();
    bms_fb_pub->publish(bms_fb);

    chassis_ctrl_src_fb.chassis_ctrl_cmd_src = get_ctrl_cmd_src();
    chassis_ctrl_src_fb_pub->publish(chassis_ctrl_src_fb);

    chassis_mileage_meter_fb.vehicle_meters = get_vehicle_meter();
    chassis_mileage_meter_fb_pub->publish(chassis_mileage_meter_fb);

    chassis_mode_fb.chassis_mode = get_chassis_mode(); //0: lock_mode, 1:ctrl_mode, 2:push_mode, 3:emergency mode, 4:error mode
    chassis_mode_fb_pub->publish(chassis_mode_fb);

    error_code_fb.host_error        = get_err_state(Host);
    error_code_fb.central_error     = get_err_state(Central);
    error_code_fb.front_left_motor_error   = get_err_state(Motor0);
    error_code_fb.front_right_motor_error  = get_err_state(Motor1);
    error_code_fb.rear_left_motor_error    = get_err_state(Motor2);
    error_code_fb.rear_right_motor_error   = get_err_state(Motor3);
    error_code_fb.bms_error         = get_err_state(BMS);
    error_code_fb.brake_error       = get_err_state(Brake);
    error_code_fb_pub->publish(error_code_fb);

    motor_work_mode_fb.motor_work_mode = get_chassis_work_model();
    motor_work_mode_fb_pub->publish(motor_work_mode_fb);
}

} // namespace robot

