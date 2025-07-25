#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm> 

class ScanFilter : public rclcpp::Node {
public:
    ScanFilter() : Node("scan_filter") {
        // 声明参数
        this->declare_parameter("min_angle", 0.0);  
        this->declare_parameter("max_angle", 0.0); 
        this->declare_parameter("input_topic", "/scan");
        this->declare_parameter("output_topic", "/filtered_scan");
        this->declare_parameter("publish_rate", 10.0);  
        
        min_angle_ = this->get_parameter("min_angle").as_double();
        max_angle_ = this->get_parameter("max_angle").as_double();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "Filtering scan between %.1f° and %.1f°", 
                    min_angle_, max_angle_);
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f Hz", publish_rate);

        rclcpp::QoS scan_qos(rclcpp::KeepLast(10));
        scan_qos.best_effort(); 
        scan_qos.durability_volatile();  

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            input_topic, 
            scan_qos, 
            std::bind(&ScanFilter::scan_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic, 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&ScanFilter::timer_callback, this));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_msg_ = msg;
    }
    
    void timer_callback() {
        if (!latest_msg_) {
            return;
        }
        
        auto filtered_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*latest_msg_);
        
        int min_index = static_cast<int>((min_angle_ * M_PI/180.0 - latest_msg_->angle_min) / latest_msg_->angle_increment);
        int max_index = static_cast<int>((max_angle_ * M_PI/180.0 - latest_msg_->angle_min) / latest_msg_->angle_increment);
        
        min_index = std::max(0, min_index);
        max_index = std::min(static_cast<int>(latest_msg_->ranges.size())-1, max_index);
        
        filtered_msg->angle_min = min_angle_ * M_PI/180.0;
        filtered_msg->angle_max = max_angle_ * M_PI/180.0;

        filtered_msg->ranges = std::vector<float>(
            latest_msg_->ranges.begin() + min_index,
            latest_msg_->ranges.begin() + max_index + 1);
            
        if (!latest_msg_->intensities.empty()) {
            filtered_msg->intensities = std::vector<float>(
                latest_msg_->intensities.begin() + min_index,
                latest_msg_->intensities.begin() + max_index + 1);
        }
        
        std::reverse(filtered_msg->ranges.begin(), filtered_msg->ranges.end());
        if (!filtered_msg->intensities.empty()) {
            std::reverse(filtered_msg->intensities.begin(), filtered_msg->intensities.end());
        }
        
        filtered_msg->header.stamp = this->now();
        
        publisher_->publish(*filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_msg_;
    double min_angle_;
    double max_angle_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanFilter>());
    rclcpp::shutdown();
    return 0;
}