#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "motor_controller.hpp" 
#include <sensor_msgs/msg/joy.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>


namespace ControlMode {
constexpr uint8_t MANUAL = 0;          // 手动模式
constexpr uint8_t AUTONOMOUS = 1;      // 自动模式
}
class IntegratedController : public rclcpp::Node
{
public:
    IntegratedController() : Node("integrated_controller"),
        manual_mode_(true),
        motor_enabled_(false),
        enable_pressed_(false),
        speed_level_(2)  // 默认中速
    {
        // 初始化参数（手柄+电机）
        initParameters();
        
        // 创建电机控制器
        motor_controller_ = std::make_unique<MotorController>(this);
        motor_controller_->initialize();
        
        // 订阅手柄消息
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&IntegratedController::joyCallback, this, std::placeholders::_1));
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&IntegratedController::command_callback, this, std::placeholders::_1));
    
        // 创建状态发布器
        velocity_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", 1);
        steerang_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
            "/vehicle/status/steering_status", 1);       
        vehicle_odom_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "vehicle_velocity_converter/twist_with_covariance", 1);
        control_mode_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
            "/vehicle/status/control_mode", 1); 
        
        // 控制循环定时器
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&IntegratedController::controlLoop, this));
    }

private:
    void initParameters()
    {
        // 手柄参数
        declare_parameter<int>("linear_axis", 1);
        declare_parameter<int>("angular_axis", 3);
        declare_parameter<double>("deadzone", 0.1);

        // 声明并获取参数 - 按键映射
        declare_parameter<int>("mode_switch_button", 0);
        declare_parameter<int>("motor_lock_button", 1);
        declare_parameter<int>("enable_button", 5);

        declare_parameter<int>("low_speed_button", 7);    
        declare_parameter<int>("medium_speed_button", 2);  
        declare_parameter<int>("high_speed_button", 3);    
        
        // 速度档位
        declare_parameter<double>("low_linear_scale", 0.35);
        declare_parameter<double>("low_angular_scale", 0.6);
        declare_parameter<double>("medium_linear_scale", 0.65);
        declare_parameter<double>("medium_angular_scale", 0.6);
        declare_parameter<double>("high_linear_scale", 1.2);
        declare_parameter<double>("high_angular_scale", 0.6);
        
        // 获取参数值
        linear_axis_ = get_parameter("linear_axis").as_int();
        angular_axis_ = get_parameter("angular_axis").as_int();
        deadzone_ = get_parameter("deadzone").as_double();
        mode_switch_button_ = get_parameter("mode_switch_button").as_int();
        motor_lock_button_ = get_parameter("motor_lock_button").as_int();
        enable_button_ = get_parameter("enable_button").as_int();
        
        low_linear_scale_ = get_parameter("low_linear_scale").as_double();
        low_angular_scale_ = get_parameter("low_angular_scale").as_double();
        medium_linear_scale_ = get_parameter("medium_linear_scale").as_double();
        medium_angular_scale_ = get_parameter("medium_angular_scale").as_double();
        high_linear_scale_ = get_parameter("high_linear_scale").as_double();
        high_angular_scale_ = get_parameter("high_angular_scale").as_double();

        low_speed_button_ = get_parameter("low_speed_button").as_int();
        medium_speed_button_ = get_parameter("medium_speed_button").as_int();
        high_speed_button_ = get_parameter("high_speed_button").as_int();

        // 电机参数
        declare_parameter("wheel_distance", 0.46);
        declare_parameter("linear_acceleration_max", 0.5);
        declare_parameter("angular_acceleration_max", 0.5);
        declare_parameter("max_angular_speed", 0.5);

        wheel_distance = get_parameter("wheel_distance").as_double();
        max_angular_speed = get_parameter("max_angular_speed").as_double();
        
    }

    void command_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto twist = geometry_msgs::msg::Twist();
        if( motor_enabled_ && !manual_mode_){
            twist.linear.x = msg->linear.x;
            twist.angular.z = msg->angular.z;
            // RCLCPP_INFO(this->get_logger(), "[Auto Mode: cmd_vel]linear speed: %f", twist.linear.x);
            // RCLCPP_INFO(this->get_logger(), "[Auto Mode: cmd_vel]angular speed: %f", twist.linear.z);
            setMotorSpeeds(twist);
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {

        // 确保last_buttons_大小正确
        if (last_buttons_.size() != joy_msg->buttons.size()) {
            last_buttons_.resize(joy_msg->buttons.size(), 0);
        }
        
        // 处理按键事件
        process_buttons(joy_msg);
        
        auto twist = geometry_msgs::msg::Twist();

        // RCLCPP_INFO_THROTTLE(
        //     this->get_logger(), 
        //     *this->get_clock(), 
        //     500,  // 每0.5秒发布一次
        //     "状态: 模式=%s, 电机=%s, 档位=%d",
        //     manual_mode_ ? "手动" : "自动",
        //     motor_enabled_ ? "锁轴" : "解锁",
        //     speed_level_
        // );
        if (can_move() && manual_mode_) {
            // 获取当前档位的速度缩放因子
            double linear_scale = get_current_linear_scale();
            double angular_scale = get_current_angular_scale();
            
            // 处理左摇杆 - 线速度（X方向）
            if (std::abs(joy_msg->axes[linear_axis_]) > deadzone_) {
                twist.linear.x = joy_msg->axes[linear_axis_] * linear_scale;
            }
            // 处理右摇杆 - 角速度（Z方向）
            if (std::abs(joy_msg->axes[angular_axis_]) > deadzone_) {
                twist.angular.z = joy_msg->axes[angular_axis_] * angular_scale;
            }
            RCLCPP_INFO(this->get_logger(), "[Manual Mode cmd_vel]linear speed: %f", twist.linear.x);
            RCLCPP_INFO(this->get_logger(), "[Manual Mode cmd_vel]angular speed: %f", twist.linear.z);
            // 设置电机速度
            setMotorSpeeds(twist);
        }
        else{
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            setMotorSpeeds(twist);
        }

        // 保存当前按键状态用于下一次比较
        last_buttons_ = joy_msg->buttons;
    }

    void controlLoop()
    {
        // geometry_msgs::msg::Twist cmd_vel;
        
        // 发布状态信息
        publishStatus();
    }

    void setMotorSpeeds(const geometry_msgs::msg::Twist& cmd_vel)
    {

        // 运动学逆解
        double target_linear = cmd_vel.linear.x;
        double target_angular = cmd_vel.angular.z;
        // double target_angular = std::clamp(cmd_vel.angular.z, -max_angular_speed, max_angular_speed);

        // if (target_angular != 0) {
        //     target_linear = std::clamp(target_linear, -0.5, 0.85);
        // }
        
        double left_rpm = (target_linear - (target_angular * wheel_distance) / 2.0) * 
                          motor_controller_->rpm_per_mps_;
        double right_rpm = (target_linear + (target_angular * wheel_distance) / 2.0) * 
                           motor_controller_->rpm_per_mps_;
        
        // 设置电机速度
        motor_controller_->setMotorSpeeds(
            static_cast<int>(-left_rpm), 
            static_cast<int>(right_rpm)
        );
    }

    void publishStatus()
    {
        double linear_speed,angular_speed;
        // 获取当前速度
        motor_controller_->getCurrentSpeeds(
            linear_speed, 
            angular_speed,
            wheel_distance
        );

        linear_velocity = linear_speed;
        angular_velocity = angular_speed;

        autoware_vehicle_msgs::msg::VelocityReport velocity_report_msg;
        convert_velocity_to_autoware_msg(velocity_report_msg);
        
        autoware_vehicle_msgs::msg::SteeringReport steer_report_msg;
        convert_steering_to_autoware_msg(steer_report_msg);
        
        geometry_msgs::msg::TwistWithCovarianceStamped vehicle_odom_msg;
        send_odom_data_to_autoware(vehicle_odom_msg);

        publish_control_mode();
    }

    void publish_control_mode()
    {
        autoware_vehicle_msgs::msg::ControlModeReport msg;
        msg.stamp = this->get_clock()->now();
        // msg.mode = manual_mode_ ? ControlMode::MANUAL : ControlMode::AUTONOMOUS;
        msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;  // 1
        control_mode_pub_->publish(msg);
    }

    void convert_velocity_to_autoware_msg(autoware_vehicle_msgs::msg::VelocityReport msg)
    {   
        msg.longitudinal_velocity = linear_velocity;
        msg.lateral_velocity = 0.0;
        msg.heading_rate = angular_velocity;
        msg.header.frame_id = "base_link";//不明白为什么这里要设定为base_link
        velocity_pub_->publish(msg);
    }

    void convert_steering_to_autoware_msg(autoware_vehicle_msgs::msg::SteeringReport msg)
    {
        msg.stamp = this->get_clock()->now();
        double v,w,angle;
        v = linear_velocity;
        w = angular_velocity;
        if(w==0||(abs(v)<0.01))
        {
            angle = 0;
        }
        else angle = atan2(wheel_distance * w,v);
        msg.steering_tire_angle = angle;
        steerang_pub_->publish(msg);
    }

    void send_odom_data_to_autoware(geometry_msgs::msg::TwistWithCovarianceStamped msg)
    {
        msg.header.frame_id = "base_link";
        msg.header.stamp = this->get_clock()->now();
        // msg.twist.covariance = {0,0,0,0,0,0,
        //                         0,0,0,0,0,0,
        //                         0,0,0,0,0,0,
        //                         0,0,0,0,0,0,
        //                         0,0,0,0,0,0,
        //                         0,0,0,0,0,0};
        msg.twist.covariance = {1000,1000,1000,1000,1000,1000,
                                1000,1000,1000,1000,1000,1000,
                                1000,1000,1000,1000,1000,1000,
                                1000,1000,1000,1000,1000,1000,
                                1000,1000,1000,1000,1000,1000,
                                1000,1000,1000,1000,1000,1000,};
        msg.twist.twist.linear.x = linear_velocity;
        msg.twist.twist.angular.z = angular_velocity;
        vehicle_odom_pub_->publish(msg);
    }

    void process_buttons(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        // 模式切换按钮 (点动)
        if (joy_msg->buttons[mode_switch_button_] == 1 && !last_buttons_[mode_switch_button_]) {
            manual_mode_ = !manual_mode_;
            RCLCPP_INFO(this->get_logger(), "模式切换: %s", manual_mode_ ? "手动" : "自动");
        }
        
        // 电机锁按钮 (点动)
        if (joy_msg->buttons[motor_lock_button_] == 1 && !last_buttons_[motor_lock_button_] && manual_mode_) {
            motor_enabled_ = !motor_enabled_;
            if(motor_enabled_){motor_controller_->enableMotors();}
            else{motor_controller_->disableMotors();}
            
            RCLCPP_INFO(this->get_logger(), "电机状态: %s", motor_enabled_ ? "锁轴" : "解锁");
        }
        
        // 使能按钮 (需要持续按住)
        enable_pressed_ = (joy_msg->buttons[enable_button_] == 1);
        
        // 速度档位按钮处理（三个独立按钮）
        if (joy_msg->buttons[low_speed_button_] == 1 && !last_buttons_[low_speed_button_]) {
            speed_level_ = 1;
            RCLCPP_INFO(this->get_logger(), "速度档位: 低速");
        }
        else if (joy_msg->buttons[medium_speed_button_] == 1 && !last_buttons_[medium_speed_button_]) {
            speed_level_ = 2;
            RCLCPP_INFO(this->get_logger(), "速度档位: 中速");
        }
        else if (joy_msg->buttons[high_speed_button_] == 1 && !last_buttons_[high_speed_button_]) {
            speed_level_ = 3;
            RCLCPP_INFO(this->get_logger(), "速度档位: 高速");
        }
        
    }
    
    double get_current_linear_scale()
    {
        switch (speed_level_) {
            case 1: return low_linear_scale_;
            case 2: return medium_linear_scale_;
            case 3: return high_linear_scale_;
            default: return medium_linear_scale_;
        }
    }
    
    double get_current_angular_scale()
    {
        switch (speed_level_) {
            case 1: return low_angular_scale_;
            case 2: return medium_angular_scale_;
            case 3: return high_angular_scale_;
            default: return medium_angular_scale_;
        }
    }
    
    bool can_move()
    {

        // 必须锁轴使能,否则不允许运动
        if (!motor_enabled_) {  
            return false;
        }
        
        // 必须按住使能按钮才能运动
        if (!enable_pressed_) {
            return false;
        }
        
        return true;
    }

    std::unique_ptr<MotorController> motor_controller_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_pub_;//速度报告
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steerang_pub_;//“转向角”报告
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vehicle_odom_pub_;//里程计报告
    rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;  

    rclcpp::TimerBase::SharedPtr control_timer_;
    sensor_msgs::msg::Joy::SharedPtr last_joy_;
    
    // 状态变量
    bool manual_mode_;
    bool motor_enabled_;
    bool enable_pressed_;
    int speed_level_;
    
    // 按键状态记录 (用于检测按键按下事件)
    std::vector<int> last_buttons_;
    
    // 参数 - 摇杆轴
    int linear_axis_;
    int angular_axis_;
    double deadzone_;
    
    // 参数 - 按键映射
    int mode_switch_button_;
    int motor_lock_button_;
    int enable_button_;

    int low_speed_button_;
    int medium_speed_button_;
    int high_speed_button_;
    
    // 参数 - 速度档位缩放因子
    double low_linear_scale_;
    double low_angular_scale_;
    double medium_linear_scale_;
    double medium_angular_scale_;
    double high_linear_scale_;
    double high_angular_scale_;
    double linear_velocity;
    double angular_velocity;

    double wheel_distance;
    double max_angular_speed;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IntegratedController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}