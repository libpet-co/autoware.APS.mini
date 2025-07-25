#include <rclcpp/rclcpp.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <iomanip>
#include <sstream>
#include <mutex>

using namespace std::chrono_literals;

// 存储所有需要打印信息的结构体
struct MonitorData {
    // 控制指令信息
    autoware_control_msgs::msg::Control::ConstSharedPtr last_control;
    rclcpp::Time last_control_time;
    
    // 手柄信息
    sensor_msgs::msg::Joy::ConstSharedPtr last_joy;
    rclcpp::Time last_joy_time;
    
    // 速度指令信息
    geometry_msgs::msg::Twist::ConstSharedPtr last_cmd_vel;
    rclcpp::Time last_cmd_vel_time;
    
    // 系统状态
    bool manual_mode;
    int speed_level;
    int mode_switch_button;
    
    // 用于按钮状态检测
    std::vector<int> last_buttons;
    
    // 构造函数初始化
    MonitorData() : 
        manual_mode(true),
        speed_level(0),
        mode_switch_button(0),
        last_buttons(20, 0)  // 假设最多20个按钮
    {}
};

class SimplifiedModeMonitor : public rclcpp::Node
{
public:
    SimplifiedModeMonitor() : Node("simplified_mode_monitor")
    {
        // 声明参数
        this->declare_parameter<double>("wheel_tread", 0.394); // 轴距默认值
        this->declare_parameter<int>("mode_switch_button", 0); // 模式切换按钮
        
        // 获取参数
        wheel_tread_ = this->get_parameter("wheel_tread").as_double();
        monitor_data_.mode_switch_button = this->get_parameter("mode_switch_button").as_int();
        
        RCLCPP_INFO(this->get_logger(), "启动简化模式监听节点");
        RCLCPP_INFO(this->get_logger(), "配置参数: 轴距=%.3fm, 模式切换按钮=%d", 
                   wheel_tread_, monitor_data_.mode_switch_button);

        // 创建订阅器
        control_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", 10,
            [this](const autoware_control_msgs::msg::Control::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                monitor_data_.last_control = msg;
                monitor_data_.last_control_time = this->now();
            });
            
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            [this](const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                
                // 处理按钮状态变化
                processButtons(msg);
                
                monitor_data_.last_joy = msg;
                monitor_data_.last_joy_time = this->now();
            });
            
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                monitor_data_.last_cmd_vel = msg;
                monitor_data_.last_cmd_vel_time = this->now();
            });
            
        // 创建定时器（每100毫秒执行一次打印）
        timer_ = this->create_wall_timer(
            100ms, 
            [this]() {
                std::lock_guard<std::mutex> lock(mutex_);
                printMessages();
            });
    }

private:
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 线程安全
    std::mutex mutex_;
    
    // 参数
    double wheel_tread_;
    
    // 存储所有监控数据的结构体
    MonitorData monitor_data_;
    
    // 时间戳字符串转换
    std::string timeToStr(const rclcpp::Time& time) {
        auto nanoseconds = std::chrono::nanoseconds(time.nanoseconds());
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(nanoseconds);
        nanoseconds -= seconds;
        
        std::ostringstream oss;
        oss << seconds.count() << "." << std::setfill('0') << std::setw(9) << nanoseconds.count();
        return oss.str();
    }
    
    // 处理按钮状态变化
    void processButtons(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg)
    {
        // 确保按钮数组大小匹配
        if (monitor_data_.last_buttons.size() != joy_msg->buttons.size()) {
            monitor_data_.last_buttons.resize(joy_msg->buttons.size(), 0);
        }
        
        // 模式切换按钮 (点动)
        if (joy_msg->buttons.size() > monitor_data_.mode_switch_button &&
            joy_msg->buttons[monitor_data_.mode_switch_button] == 1 && 
            monitor_data_.last_buttons[monitor_data_.mode_switch_button] == 0) {
            monitor_data_.manual_mode = !monitor_data_.manual_mode;
        }
        // 速度档位按钮处理（三个独立按钮）
        if (joy_msg->buttons[7] == 1 && !monitor_data_.last_buttons[7]) {
            monitor_data_.speed_level = 1;
            RCLCPP_INFO(this->get_logger(), "速度档位: 低速");
        }
        else if (joy_msg->buttons[2] == 1 && !monitor_data_.last_buttons[2]) {
            monitor_data_.speed_level = 2;
            RCLCPP_INFO(this->get_logger(), "速度档位: 中速");
        }
        else if (joy_msg->buttons[3] == 1 && !monitor_data_.last_buttons[3]) {
            monitor_data_.speed_level = 3;
            RCLCPP_INFO(this->get_logger(), "速度档位: 高速");
        }
        
        // 保存当前按钮状态
        monitor_data_.last_buttons = joy_msg->buttons;
    }

    double get_current_linear_scale()
    {
        switch (monitor_data_.speed_level) {
            case 1: return 0.35;
            case 2: return 0.65;
            case 3: return 1.2;
            default: return 2;
        }
    }
    
    // 打印消息
    void printMessages()
    {
        // 确保必要消息已收到
        if (!monitor_data_.last_cmd_vel || 
            (!monitor_data_.manual_mode && !monitor_data_.last_control) || 
            (monitor_data_.manual_mode && !monitor_data_.last_joy)) {
            return;
        }
        
        // 清屏并打印完整状态
        RCLCPP_INFO(this->get_logger(), "\033[2J\033[1;1H"); // ANSI清屏
        
        if (monitor_data_.manual_mode) {
            printManualMode();
        } else {
            printAutonomousMode();
        }
        
        // 打印最终执行的cmd_vel
        printCmdVel();
    }
    
    // 打印自动模式信息
    void printAutonomousMode()
    {
        // 计算理论角速度
        const double expected_angular = (std::abs(monitor_data_.last_control->longitudinal.velocity) < 0.001) ? 0.0 :
            monitor_data_.last_control->longitudinal.velocity * 
            std::tan(monitor_data_.last_control->lateral.steering_tire_angle) / wheel_tread_;
        
        // 计算消息接收时间差
        const double delta_time = (monitor_data_.last_cmd_vel_time - monitor_data_.last_control_time).seconds();
        
        RCLCPP_INFO(this->get_logger(), "┌┌────────────────── \033[1;32mAUTO MODE\033[0m ───────────────────┐┐");
        RCLCPP_INFO(this->get_logger(), "│ \033[1;34m/control/command/control_cmd\033[0m");
        RCLCPP_INFO(this->get_logger(), "│  接收时间: %s", timeToStr(monitor_data_.last_control_time).c_str());
        RCLCPP_INFO(this->get_logger(), "│  速度: %6.3f m/s | 转向角: %6.4f rad", 
                  monitor_data_.last_control->longitudinal.velocity,
                  monitor_data_.last_control->lateral.steering_tire_angle);
        RCLCPP_INFO(this->get_logger(), "│ \033[1;34m/joy\033[0m");
        
        // 显示模式按钮状态
        RCLCPP_INFO(this->get_logger(), "│  模式按钮[%d]: %s", monitor_data_.mode_switch_button,
                   (monitor_data_.last_joy->buttons.size() > monitor_data_.mode_switch_button && 
                    monitor_data_.last_joy->buttons[monitor_data_.mode_switch_button] == 1) ? "按下" : "释放");
        
    }
    
    // 打印手动模式信息
    void printManualMode()
    {
        // 计算消息接收时间差
        const double delta_time = (monitor_data_.last_cmd_vel_time - monitor_data_.last_joy_time).seconds();
        
        RCLCPP_INFO(this->get_logger(), "┌┌────────────────── \033[1;33mMANUAL MODE\033[0m ──────────────────┐┐");
        RCLCPP_INFO(this->get_logger(), "│ \033[1;34m/control/command/control_cmd\033[0m");
        RCLCPP_INFO(this->get_logger(), "│  接收时间: %s", timeToStr(monitor_data_.last_joy_time).c_str());
        RCLCPP_INFO(this->get_logger(), "│  速度: %6.3f m/s | 转向角: %6.4f rad", 
                  monitor_data_.last_control->longitudinal.velocity,
                  monitor_data_.last_control->lateral.steering_tire_angle);
        RCLCPP_INFO(this->get_logger(), "│ \033[1;34m/joy\033[0m");
        
        // 显示模式按钮状态
        RCLCPP_INFO(this->get_logger(), "│  模式按钮[%d]: %s", monitor_data_.mode_switch_button,
                   (monitor_data_.last_joy->buttons.size() > monitor_data_.mode_switch_button && 
                    monitor_data_.last_joy->buttons[monitor_data_.mode_switch_button] == 1) ? "按下" : "释放");
    }
    
    // 打印最终执行的cmd_vel
    void printCmdVel()
    {
        double linear_speed, angular_speed;
        double linear_scale = get_current_linear_scale();
        double delta_time = 0.0;
        
        if (!monitor_data_.manual_mode && monitor_data_.last_control) {
            delta_time = (monitor_data_.last_cmd_vel_time - monitor_data_.last_control_time).seconds();
            linear_speed = monitor_data_.last_cmd_vel->linear.x;
            angular_speed = monitor_data_.last_cmd_vel->angular.z;
        } else if (monitor_data_.manual_mode && monitor_data_.last_joy) {
            delta_time = (monitor_data_.last_cmd_vel_time - monitor_data_.last_joy_time).seconds();
            linear_speed = monitor_data_.last_joy->axes[1] * linear_scale;
            angular_speed = monitor_data_.last_joy->axes[3] * 0.6;
        }
        
        RCLCPP_INFO(this->get_logger(), "│ \033[1;34m/cmd_vel\033[0m");
        RCLCPP_INFO(this->get_logger(), "│  接收时间: %s (延迟: %.1fms)", 
                  timeToStr(monitor_data_.last_cmd_vel_time).c_str(), delta_time * 1000);
        RCLCPP_INFO(this->get_logger(), "│  线速度: %6.3f m/s | 角速度: %6.4f rad/s", 
                  linear_speed, angular_speed);
                  
        RCLCPP_INFO(this->get_logger(), "└└───────────────────────────────────────────────┘┘\n");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplifiedModeMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}