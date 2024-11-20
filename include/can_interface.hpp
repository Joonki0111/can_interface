#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include <iostream>
#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float64.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"

#define RAD2DEG 57.3
#define STEERCMD2SIG 242.552
#define SPEEDCMD2SIG 255.0
#define STEERCMD_OFFSET 127.0

class AwToCan : public rclcpp::Node 
{
    public:
        AwToCan();

    private:
        // Pub
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_velocity_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_steer_angle_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_velocity_cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_velocity_status_pub_;

        // Sub
        rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr sub_aw_command_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TC_thro_cmd;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TC_brake_cmd;

        // CAN
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;    

        autoware_auto_vehicle_msgs::msg::VelocityReport velocity_msg_;
        autoware_auto_vehicle_msgs::msg::SteeringReport steering_msg_;
        
        std_msgs::msg::Float64 velocity_command_msg_;
        std_msgs::msg::Float64 steer_command_msg_;
        
        float speed_data_ = 0.0;
        float velocity_command_ = 0.0;
        
        float steering_angle_data_ = 0.0;
        float steer_command_ = 0.0;

        float TC_thro_output_cmd = 0.0;
        float TC_brake_output_cmd = 0.0;


        void can_data_callback(const can_msgs::msg::Frame::SharedPtr msg);
        void AwCmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);
        void TCthro_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void TCbrake_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void TimerCallback();
};

#endif