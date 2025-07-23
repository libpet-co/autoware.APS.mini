#include <autoware_control_msgs/msg/control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

class TestControlPublisher : public rclcpp::Node
{
public:
    TestControlPublisher() : Node("test_control_publisher")
    {
        RCLCPP_INFO(this->get_logger(), "测试控制指令发布器启动");
        
        // 创建发布器
        publisher_ = this->create_publisher<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", 10);
        
        // 创建定时器，每1秒发布一次
        timer_ = this->create_wall_timer(
            0.1s, std::bind(&TestControlPublisher::publishZeroCommand, this));
    }

private:
    void publishZeroCommand()
    {
        auto message = autoware_control_msgs::msg::Control();
        
        // 设置时间戳
        // 注意：Control 消息可能没有 header 字段，因此直接设置纵向和横向控制参数
        
        // 纵向控制 - 所有参数设为0
        message.longitudinal.velocity = 0.0;     // 速度
        message.longitudinal.acceleration = 0.0; // 加速度
        message.longitudinal.jerk = 0.0;         // 加加速度
        
        // 横向控制 - 所有参数设为0
        message.lateral.steering_tire_angle = 0.0;          // 转向角
        message.lateral.steering_tire_rotation_rate = 0.0;  // 转向角速度
        
        // 发布消息
        publisher_->publish(message);
        
        // 打印发布信息
        RCLCPP_INFO(this->get_logger(), "发布零速指令: [速度=%.1f] [加速度=%.1f] [加加速度=%.1f] [转向角=%.1f]",
                    message.longitudinal.velocity,
                    message.longitudinal.acceleration,
                    message.longitudinal.jerk,
                    message.lateral.steering_tire_angle);
    }

    rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestControlPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}