#include <autoware_control_msgs/msg/control.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"

#include <memory>
#include <iostream>
#include <cmath>
#include <chrono> // 包含 chrono 库

float WHEEL_TREAD = 0.394; // wheel_distance
// float WHEEL_BASE = 0.498;

using namespace std::chrono_literals;

class AutoMotorpub : public rclcpp::Node
{
public:
    AutoMotorpub() : Node("AutoMotorpub"),count(0){
        RCLCPP_INFO(this->get_logger(),"PUB Node 启动");
        
        Autosub_ = this->create_subscription<autoware_control_msgs::msg::Control>("/control/command/control_cmd", 10, std::bind(&AutoMotorpub::callback_control_cmd, this, std::placeholders::_1));//订阅ctrl_cmd 来自autoware的指示
        Autopub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS{1});
        
        timer_ = this->create_wall_timer(0.02s,std::bind(&AutoMotorpub::on_timer,this));//定时器
    };

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Autopub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr Autosub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvaturesub_;
    // autoware command messages定义一些存放的信息
    autoware_control_msgs::msg::Control::ConstSharedPtr control_cmd_ptr_;
    //要发布的速度指令
    geometry_msgs::msg::Twist twister;

    //控制指令的回调函数

    
    void callback_control_cmd(const autoware_control_msgs::msg::Control::ConstSharedPtr msg){
      
        control_cmd_ptr_ = msg;
        double velocity_of_veh = control_cmd_ptr_->longitudinal.velocity;
        double angle = control_cmd_ptr_->lateral.steering_tire_angle;
        double acc = control_cmd_ptr_->longitudinal.acceleration;
        double jerk = control_cmd_ptr_->longitudinal.jerk;
        double w,v;
        w = velocity_of_veh * tan(angle) / WHEEL_TREAD;
        // v = velocity_of_veh + acc * 0.02 + jerk * 0.02 * 0.02;
        v = velocity_of_veh;
        twister.angular.z = w;
        twister.linear.x = v;
    }

    //发布器和定时器
    size_t count;
    void on_timer(){
    Autopub_->publish(twister);
  }

};

int main(int argc, char ** argv)
{
  //初始化客户端
  rclcpp::init(argc,argv);

  // 调用回旋函数
  rclcpp::spin(std::make_shared<AutoMotorpub>());
  // 释放资源
  rclcpp::shutdown();

  return 0;
}