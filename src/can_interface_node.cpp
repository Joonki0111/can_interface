#include "can_interface.hpp"

using namespace std::chrono_literals;
std::chrono::steady_clock::time_point time_previous_;

AwToCan::AwToCan() : Node("Aw_to")
{
    // pub
    pub_velocity_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", rclcpp::QoS(1));
    pub_steer_angle_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", rclcpp::QoS(1));    
    TC_velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/velocity_cmd", rclcpp::QoS(1));
    TC_velocity_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/velocity_status", rclcpp::QoS(1));

    // sub  
    sub_aw_command_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", rclcpp::QoS(1), std::bind(&AwToCan::AwCmd_callback, this, std::placeholders::_1));
    TC_thro_cmd = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/output/throttle_cmd", rclcpp::QoS(1), std::bind(&AwToCan::TCthro_callback, this, std::placeholders::_1));
    TC_brake_cmd = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/output/brake_cmd", rclcpp::QoS(1), std::bind(&AwToCan::TCbrake_callback, this, std::placeholders::_1));

    // Can bridge
    sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
        "/from_can_bus", rclcpp::QoS(1), std::bind(&AwToCan::can_data_callback, this, std::placeholders::_1));
    pub_can_ = this->create_publisher<can_msgs::msg::Frame>(
        "/to_can_bus", rclcpp::QoS(1));

    // timer
    timer_ = this->create_wall_timer(10ms, std::bind(&AwToCan::TimerCallback, this));
}

void AwToCan::can_data_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if(msg->id == 513) // 201
    {
        speed_data_= msg->data[5] << 8 | msg->data[4];
        
        speed_data_ *= 0.01;

        velocity_msg_.header.stamp = this->now();
        velocity_msg_.header.frame_id = "base_link";
        velocity_msg_.longitudinal_velocity = speed_data_;
        pub_velocity_->publish(velocity_msg_);

        std_msgs::msg::Float64 TC_velocity_msg;
        TC_velocity_msg.data = speed_data_;
        TC_velocity_status_pub_->publish(TC_velocity_msg);
    }

    if(msg->id == 273) // 111
    {
        double calc_val;
        calc_val = msg->data[0] + (msg->data[1] << 8); 
                
        if(calc_val > 65535)
        {
            calc_val -= 65535;
        }
        
        calc_val -= 5200.f;
        calc_val *= 0.01071f;
        calc_val /= RAD2DEG;

        steering_msg_.stamp = this->now();
        steering_msg_.steering_tire_angle = calc_val;
        pub_steer_angle_->publish(steering_msg_);
    }    
}

void AwToCan::AwCmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
    velocity_command_ = msg->longitudinal.speed;
    velocity_command_msg_.data = velocity_command_;
    
    steer_command_ = msg->lateral.steering_tire_angle;
    
    TC_velocity_cmd_pub_->publish(velocity_command_msg_);
    
}

void AwToCan::TCthro_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    TC_thro_output_cmd = msg->data;
}

void AwToCan::TCbrake_callback(const std_msgs::msg::Float64::SharedPtr msg)
{   
    TC_brake_output_cmd = msg->data;
}


void AwToCan::TimerCallback()
{
    uint8_t steer_can = static_cast<uint8_t>(std::clamp((steer_command_* STEERCMD2SIG) + STEERCMD_OFFSET, 0.0, 255.0));
    uint8_t throttle_can = static_cast<uint8_t>(std::clamp(TC_thro_output_cmd * SPEEDCMD2SIG, 0.0, 255.0));
    uint8_t brake_can = static_cast<uint8_t>(std::clamp(TC_brake_output_cmd * SPEEDCMD2SIG, 0.0, 255.0));

    can_msgs::msg::Frame can_data;
    can_data.id = 320;
    can_data.dlc = 4;
    can_data.data[0] = throttle_can;
    can_data.data[1] = steer_can;
    can_data.data[2] = brake_can;
    can_data.data[3] = 1;
    
    pub_can_->publish(can_data);
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AwToCan>());
  rclcpp::shutdown();
  return 0;
}