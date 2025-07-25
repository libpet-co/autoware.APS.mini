#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <limits>
#include <iomanip>

class ScanSubscriber : public rclcpp::Node {
public:
    ScanSubscriber() : Node("scan_subscriber") {
        // 设置 QoS 配置（匹配 LiDAR 驱动的 BEST_EFFORT）
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort);
        
        // 订阅 /scan 话题
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/filtered_scan",
            qos_profile,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                this->scan_callback(msg);
            }
        );
        RCLCPP_INFO(this->get_logger(), "订阅 /scan 话题成功 (QoS: BEST_EFFORT)，等待数据...");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 打印消息头信息
        RCLCPP_INFO(this->get_logger(), "\n--- 接收到 LaserScan 消息 ---");
        RCLCPP_INFO(this->get_logger(), "时间戳: %d.%09ld", msg->header.stamp.sec, msg->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "坐标系: %s", msg->header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "角度范围: [%.2f, %.2f] rad", msg->angle_min, msg->angle_max);
        RCLCPP_INFO(this->get_logger(), "距离范围: [%.2f, %.2f] m", msg->range_min, msg->range_max);
        RCLCPP_INFO(this->get_logger(), "角度增量: %.4f rad", msg->angle_increment);
        RCLCPP_INFO(this->get_logger(), "数据点数: %zu", msg->ranges.size());

        // 打印所有 ranges 数据（带索引和角度）
        RCLCPP_INFO(this->get_logger(), "\n所有距离值 (ranges):");
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float angle = msg->angle_min + i * msg->angle_increment;
            float range = msg->ranges[i];

            // 标注无效值
            std::string status;
            if (std::isinf(range)) {
                status = " (INF)";
            } else if (std::isnan(range)) {
                status = " (NAN)";
            } else if (range < msg->range_min || range > msg->range_max) {
                status = " (OUT OF RANGE)";
            }

            // 打印格式：索引 + 角度（度） + 距离值
            RCLCPP_INFO(
                this->get_logger(),
                "[%4zu] 角度: %6.2f° 距离: %7.3f m%s",
                i,
                angle * 180.0 / M_PI,  // 弧度转角度
                range,
                status.c_str()
            );
        }

        // 计算并打印最近有效距离
        float min_range = std::numeric_limits<float>::infinity();
        for (const auto& r : msg->ranges) {
            if (r >= msg->range_min && r <= msg->range_max && !std::isnan(r)) {
                min_range = std::min(min_range, r);
            }
        }
        if (std::isfinite(min_range)) {
            RCLCPP_INFO(this->get_logger(), "\n最近有效障碍物距离: %.3f m", min_range);
        } else {
            RCLCPP_WARN(this->get_logger(), "无有效距离数据！");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}