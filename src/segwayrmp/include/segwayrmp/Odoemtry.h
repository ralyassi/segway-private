#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "segwayrmp/msg/sensor.hpp"  // Update with correct PascalCase if needed

#define pi 3.141593f

namespace odometry
{
    class Odometry
    {
        public:
            Odometry(const rclcpp::Node::SharedPtr& node);

        private:
            void Odometry_Cacl(double vx, double vy, double th);
            void Sensor_callback(const segwayrmp::msg::Sensor::SharedPtr msg);
            void TimeUpdate20Hz();

            rclcpp::Node::SharedPtr node_;

            rclcpp::Subscription<segwayrmp::msg::Sensor>::SharedPtr sensor_sub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

            rclcpp::TimerBase::SharedPtr update_timer_;
            rclcpp::Time current_time, last_time;

            std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
            nav_msgs::msg::Odometry odom; // Odometry object
            geometry_msgs::msg::Quaternion odom_quat; // Quaternion
            geometry_msgs::msg::TransformStamped odom_trans; // TransformStamped

            int16_t L_Ticks_;
            int16_t R_Ticks_;
            int16_t Pre_L_Ticks_;
            int16_t Pre_R_Ticks_;
    };

}

#endif
