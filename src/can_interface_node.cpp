#include "can_interface.hpp"

using namespace std::chrono_literals;

AwToCan::AwToCan() : Node("Aw_to_CAN")
{
    use_motor_revolution_ = this->declare_parameter("use_motor_revolution", false);
    
    // pub
    AW_pub_velocity_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", rclcpp::QoS(1));
    AW_pub_steer_angle_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", rclcpp::QoS(1));    
    TC_velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/velocity_cmd", rclcpp::QoS(1));
    TC_velocity_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/velocity_status", rclcpp::QoS(1));
    TC_motor_velocity_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/velocity_status", rclcpp::QoS(1));
    TC_steer_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/steer_cmd", rclcpp::QoS(1));
    TC_steer_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/steer_status", rclcpp::QoS(1));

    // sub  
    sub_aw_command_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", rclcpp::QoS(1), std::bind(&AwToCan::AwCmd_callback, this, std::placeholders::_1));
    TC_thro_cmd = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/output/throttle_cmd", rclcpp::QoS(1), std::bind(&AwToCan::TCthro_callback, this, std::placeholders::_1));
    TC_brake_cmd = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/output/brake_cmd", rclcpp::QoS(1), std::bind(&AwToCan::TCbrake_callback, this, std::placeholders::_1));
    TC_steer_cmd = this->create_subscription<std_msgs::msg::Int16>(
        "/twist_controller/output/steer_cmd", rclcpp::QoS(1), std::bind(&AwToCan::TCsteer_callback, this, std::placeholders::_1));

    // Can bridge
    Interface_sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
        "/socketcan/interface/from_can_bus", rclcpp::QoS(1), std::bind(&AwToCan::Interface_can_data_callback, this, std::placeholders::_1));
    Interface_pub_can_ = this->create_publisher<can_msgs::msg::Frame>(
        "/socketcan/interface/to_can_bus", rclcpp::QoS(1));
    Motor_sub_can_= this->create_subscription<can_msgs::msg::Frame>(
        "/socketcan/motor/from_can_bus", rclcpp::QoS(1), std::bind(&AwToCan::Motor_can_data_callback, this, std::placeholders::_1));

    // timer
    timer_ = this->create_wall_timer(10ms, std::bind(&AwToCan::TimerCallback, this));
}

void AwToCan::Interface_can_data_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if(msg->id == 513) // 201
    {
        float speed_data = msg->data[5] << 8 | msg->data[4];
        
        speed_data *= 0.01;

        autoware_auto_vehicle_msgs::msg::VelocityReport AW_velocity_status_msg;
        AW_velocity_status_msg.header.stamp = this->now();
        AW_velocity_status_msg.header.frame_id = "base_link";
        AW_velocity_status_msg.longitudinal_velocity = speed_data;
        AW_pub_velocity_->publish(AW_velocity_status_msg);

        std_msgs::msg::Float64 TC_velocity_msg;
        TC_velocity_msg.data = speed_data;
        if(!use_motor_revolution_)
        {
            TC_velocity_status_pub_->publish(TC_velocity_msg);
        }
    }

    if(msg->id == 273) // 111
    {
        float steer_data = msg->data[0] + (msg->data[1] << 8); 

        if(steer_data > 65535)
        {
            steer_data -= 65535;
        }
        
        steer_data -= 5200.f;
        steer_data *= 0.01071f;
        steer_data /= RAD2DEG;
        
        autoware_auto_vehicle_msgs::msg::SteeringReport steering_msg;
        steering_msg.stamp = this->now();
        steering_msg.steering_tire_angle = steer_data;
        AW_pub_steer_angle_->publish(steering_msg);

        std_msgs::msg::Float64 steering_status_msg;
        steering_status_msg.data = steer_data;
        TC_steer_status_pub_->publish(steering_status_msg);
    }    
}

void AwToCan::Motor_can_data_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if(msg->id == 402724847) //  Motor_control id 180117EF
    {
        double motor_rpm_raw = msg->data[7] << 8 | msg->data[6];
        double motor_rpm = motor_rpm_raw - 32000;
        
        double vehicle_speed_data = motor_rpm * MY_PI * WHEEL_DIAMETER / GEAR_RATIO;
        vehicle_speed_data /= 60; //m/s

        std_msgs::msg::Float64 TC_motor_velocity_msg;
        TC_motor_velocity_msg.data = vehicle_speed_data;
        if(use_motor_revolution_)
        {
            TC_motor_velocity_status_pub_->publish(TC_motor_velocity_msg);
        }
    }
}

void AwToCan::AwCmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
    std_msgs::msg::Float64 TC_velocity_cmd_msg;
    std_msgs::msg::Float64 TC_steer_cmd_msg;

    TC_velocity_cmd_msg.data = msg->longitudinal.speed;
    TC_steer_cmd_msg.data = msg->lateral.steering_tire_angle;

    TC_velocity_cmd_pub_->publish(TC_velocity_cmd_msg);
    TC_steer_cmd_pub_->publish(TC_steer_cmd_msg);
}

void AwToCan::TCthro_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    TC_thro_output_cmd_ = msg->data;
}

void AwToCan::TCbrake_callback(const std_msgs::msg::Float64::SharedPtr msg)
{   
    TC_brake_output_cmd_ = msg->data;
}

void AwToCan::TCsteer_callback(const std_msgs::msg::Int16::SharedPtr msg)
{   
    TC_steer_output_cmd_ = msg->data;
}

void AwToCan::TimerCallback()
{
    uint8_t throttle_can = static_cast<uint8_t>(std::clamp(TC_thro_output_cmd_ * SPEEDCMD2SIG, 0.0, 255.0));
    uint8_t brake_can = static_cast<uint8_t>(std::clamp(TC_brake_output_cmd_ * SPEEDCMD2SIG, 0.0, 255.0));

    can_msgs::msg::Frame can_data;
    can_data.id = 320;
    can_data.dlc = 4;
    can_data.data[0] = throttle_can;
    can_data.data[1] = 0;
    can_data.data[2] = brake_can;
    can_data.data[3] = 1;
    
    Interface_pub_can_->publish(can_data);

    can_msgs::msg::Frame can_data1;

    const int8_t TC_steer_output_cmd_1 = (int8_t)((TC_steer_output_cmd_ & 0xFF00) >> 8);
    const int8_t TC_steer_output_cmd_2 = (int8_t)(TC_steer_output_cmd_ & 0x00FF);

    can_data1.id = 666;
    can_data1.dlc = 2;
    can_data1.data[0] = TC_steer_output_cmd_1;
    can_data1.data[1] = TC_steer_output_cmd_2;
    Interface_pub_can_->publish(can_data1);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AwToCan>());
  rclcpp::shutdown();
  return 0;
}