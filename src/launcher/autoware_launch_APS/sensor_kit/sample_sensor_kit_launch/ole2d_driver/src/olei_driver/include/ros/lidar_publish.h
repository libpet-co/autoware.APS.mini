#pragma once

#include <deque>
#include <mutex>

#include <sensor_msgs/msg/laser_scan.hpp>
#include "olei/olei_packet_reader.h" 
#include <rclcpp/rclcpp.hpp>

class LidarPublisher: public OleiPacketReader
{
public:
  LidarPublisher(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<ScanConfig> config,
                     std::shared_ptr<ScanParameters> params, const std::string& scan_topic,
                     const std::string& frame_id);

  virtual void read(OleiVFPacket_A& packet);
  virtual void read(OleiVFPacket_B& packet);
  virtual void read(OleiVFPacket_C& packet);
  virtual void read(Olei2Packet_A& packet);


  virtual bool start();

  virtual bool stop();

protected:
  std::string frame_id_;
  std::deque<sensor_msgs::msg::LaserScan::SharedPtr> d_queue_;
  std::mutex q_mutex_;

  std::shared_ptr<ScanConfig> config_;
  std::shared_ptr<ScanParameters> params_;

  bool check_status(uint32_t status_flags);

  template <typename T>
  void to_msg_queue(T& packet, uint16_t layer_idx = 0, int layer_inclination = 0);

    template <typename T>
  void to_msg_queue2(T& packet, uint16_t layer_idx = 0, int layer_inclination = 0);

  virtual void resetCurrentScans()
  {
  }
private:
    std::shared_ptr<rclcpp::Node> node_;
    uint16_t current_id;
    int idx = 0;
    int num = 0;
    int current_num = 0;
    rclcpp::Publisher<olei_interfaces::msg::VFHeader>::SharedPtr header_publisher_;
    rclcpp::Publisher<olei_interfaces::msg::V2Header>::SharedPtr header_publisher_2;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    virtual void publish_header(olei_interfaces::msg::VFHeader& header);
    void publish_scan(sensor_msgs::msg::LaserScan::SharedPtr msg);
};
