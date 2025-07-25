#include "ros/lidar_publish.h"
#include"olei/olei_packet/olei_packet.h"
#include "olei/olei_packet/olei_vf_packet.h"
#include "olei/olei_packet/olei_vf_packet_a.h"
#include "olei/olei_packet/olei_vf_packet_b.h"
#include "olei/olei_packet/olei_vf_packet_c.h"
#include "olei/olei_packet/olei_version2.h"
#include "olei/olei_packet/olei_version2_a.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LidarPublisher::LidarPublisher(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<ScanConfig> config,
    std::shared_ptr<ScanParameters> params, const std::string& scan_topic,
    const std::string& frame_id)
     : node_(node),config_(config),params_(params),frame_id_(frame_id)
{
  scan_publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS());
  header_publisher_ = node_->create_publisher<olei_interfaces::msg::VFHeader>("/v3_header", 1);
  header_publisher_2 = node_->create_publisher<olei_interfaces::msg::V2Header>("/v2_header", 1);
  frame_id_ = frame_id;
}

template <typename T>
void LidarPublisher::to_msg_queue(T &packet, uint16_t layer_idx, int layer_inclination)
{
  sensor_msgs::msg::LaserScan::SharedPtr msg;
  int actual_num = 0;  
  actual_num = packet.header.end_index - packet.header.start_index;
  
  RCLCPP_INFO(node_->get_logger(), "actual number: %d", actual_num);

  if (d_queue_.empty())
    d_queue_.emplace_back();
  else if (d_queue_.size() > 5)
    d_queue_.pop_front();
  if (current_id > packet.header.first_index)
  {
    uint16_t frequency = packet.header.scan_frequency & 32767;
    const auto scan_time = rclcpp::Duration::from_seconds((1.0 / frequency)*60);
    msg.reset(new sensor_msgs::msg::LaserScan());
    msg->header.frame_id.assign(frame_id_);
    // msg->header.seq = packet.header.header.scan_number;
    msg->header.stamp = packet.last_acquired_point_stamp - scan_time;
    msg->scan_time = static_cast<float>(scan_time.seconds());
    msg->angle_increment = (360.0/packet.header.num_points_scan) * M_PI / 180.0;
    {
      msg->time_increment = msg->scan_time/(float)packet.header.num_points_scan;

      msg->angle_min = packet.header.start_index * msg->angle_increment - M_PI;
      msg->angle_max = packet.header.end_index * msg->angle_increment - M_PI;

      msg->range_min = config_->range_min;
      msg->range_max = config_->range_max;
    }   
    msg->ranges.resize(actual_num);
    if (!packet.intensity.empty())
      msg->intensities.resize(actual_num);
    d_queue_.push_back(msg);
  }
  current_id = packet.header.first_index;
  msg = d_queue_.back();
  if (!msg)
    return;
  // errors in scan_number - not in sequence sometimes
  /*if (msg->header.seq != packet.header.header.scan_number)
    return;*/
  //int idx = packet.header.first_index;
  for (int i = 0; i < packet.header.num_points_packet; i++)
  {
    if(idx>=actual_num)
      break;
    float data;
    if (packet.distance[i] == 0xFFFFFFFF)
      data = std::numeric_limits<std::uint32_t>::quiet_NaN();
    else
      data = packet.distance[i] / 1000.0;
    msg->ranges[idx] = std::move(data);
    if (!packet.intensity.empty())
      msg->intensities[idx] = packet.intensity[i];
    idx++;
  }
  if (actual_num == idx)
  {
    if (msg)
    {      
      publish_scan(msg);
      d_queue_.pop_back();
      idx = 0;
    }
  }
}

template <typename T>
void LidarPublisher::to_msg_queue2(T &packet, uint16_t layer_idx, int layer_inclination)
{
  sensor_msgs::msg::LaserScan::SharedPtr msg;
  if (d_queue_.empty())
    d_queue_.emplace_back();
  else if (d_queue_.size() > 5)
    d_queue_.pop_front(); 
  if (current_id > packet.angle[0])
  {  
    uint16_t frequency = packet.header.scan_frequency & 32767;
    const auto scan_time = rclcpp::Duration::from_seconds((1.0 / frequency)*60);
    msg.reset(new sensor_msgs::msg::LaserScan());
    msg->header.frame_id.assign(frame_id_);
    // msg->header.seq = packet.header.header.scan_number;
    msg->header.stamp = packet.last_acquired_point_stamp - scan_time;
    msg->scan_time = static_cast<float>(scan_time.seconds());
    msg->angle_increment = (packet.angle[10] - packet.angle[0]) * M_PI / 180000.0;
    // msg->angle_increment = (msg->angle_max - msg->angle_min) * M_PI / 180.0 / num;  // 动态计算间隔
    // msg->angle_increment = (config_->end_angle - config_->start_angle) * M_PI / 180.0 / num;  // 动态计算间隔
    {
      msg->time_increment = msg->scan_time / num;
      msg->angle_min = packet.angle[0]* M_PI / 18000.0 - M_PI;
      msg->angle_max = packet.angle[0]* M_PI / 18000.0 - M_PI + msg->angle_increment*num;

      // msg->angle_min = packet.angle[0]* M_PI / 18000.0 - config_->end_angle * M_PI / 180.0;
      // msg->angle_max = packet.angle[0]* M_PI / 18000.0 - config_->end_angle * M_PI / 180.0 + msg->angle_increment*num;

      // // // 转换为弧度
      // msg->angle_min = config_->start_angle * M_PI / 180.0;
      // msg->angle_max = config_->end_angle * M_PI / 180.0;
      
      // msg->angle_min = packet.header.start_index * msg->angle_increment - M_PI;
      // msg->angle_max = packet.header.end_index * msg->angle_increment - M_PI;

      // RCLCPP_INFO(node_->get_logger(), "angle_min: %f", msg->angle_min);
      // RCLCPP_INFO(node_->get_logger(), "angle_max: %f", msg->angle_max);

      // RCLCPP_INFO(node_->get_logger(), "packet angle[0]: %d", packet.angle[0]);
      // RCLCPP_INFO(node_->get_logger(), "packet angle[1]: %d", packet.angle[1]);
      // RCLCPP_INFO(node_->get_logger(), "packet angle[2]: %d", packet.angle[2]);
      // RCLCPP_INFO(node_->get_logger(), "packet angle[3]: %d", packet.angle[3]);
      // RCLCPP_INFO(node_->get_logger(), "packet angle[4]: %d", packet.angle[4]);
      // RCLCPP_INFO(node_->get_logger(), "packet angle[10]: %d", packet.angle[10]);
      // RCLCPP_INFO(node_->get_logger(), "packet angle[100]: %d", packet.angle[100]);
      // RCLCPP_INFO(node_->get_logger(), "num: %d", num);
      // RCLCPP_INFO(node_->get_logger(), "scan_time: %f", msg->scan_time);
      // RCLCPP_INFO(node_->get_logger(), "angle_increment: %f", msg->angle_increment);

      msg->range_min = config_->range_min;
      msg->range_max = config_->range_max;
    } 
    msg->ranges.resize(num);
    current_num = num;
    if (!packet.intensity.empty())
      msg->intensities.resize(num);
    d_queue_.push_back(msg);
    num = 0;
  }
  current_id = packet.angle[0];
  num += packet.distance.size();
  msg = d_queue_.back();
  if (!msg)
    return;
  // errors in scan_number - not in sequence sometimes
  /*if (msg->header.seq != packet.header.header.scan_number)
    return;*/
  //int idx = packet.header.first_index;
  for (int i = 0; i < packet.distance.size(); i++)
  {
    if(idx>=msg->ranges.size())
      break;
    float data;
    if (packet.distance[i] == 0xFFFFFFFF)
      data = std::numeric_limits<std::uint32_t>::quiet_NaN();
    else
      data = packet.distance[i] / 1000.0;
    msg->ranges[idx] = std::move(data); // fill the data into topic 
    if (!packet.intensity.empty())
      msg->intensities[idx] = packet.intensity[i];
    idx++;
  }
  if (current_num == idx)
  {
    if (msg)
    {      
      publish_scan(msg);
      d_queue_.pop_back();
      idx = 0;
      current_num = 0;
    }
  }
}

void LidarPublisher::read(OleiVFPacket_A &packet)
{
    publish_header(packet.header);
    to_msg_queue<OleiVFPacket_A>(packet);
}

void LidarPublisher::read(OleiVFPacket_B &packet)
{
    publish_header(packet.header);
    to_msg_queue<OleiVFPacket_B>(packet);
}

void LidarPublisher::read(OleiVFPacket_C &packet)
{
    publish_header(packet.header);
    to_msg_queue<OleiVFPacket_C>(packet);
}


void LidarPublisher::read(Olei2Packet_A &packet)
{   
    // RCLCPP_INFO(node_->get_logger(), "I am here LidarPublisher");
    // RCLCPP_INFO(node_->get_logger(), "Raw packet angles: start=%d, end=%d", 
    //         config_->start_angle, config_->end_angle);
    // RCLCPP_INFO(node_->get_logger(), "Raw packet angles: start=%d, end=%d", 
    //         packet.header.start_index, packet.header.end_index);

    header_publisher_2->publish(packet.header);
    to_msg_queue2<Olei2Packet_A>(packet);
}

bool LidarPublisher::start()
{
    return true;
}

bool LidarPublisher::stop()
{
    return true;
}

void LidarPublisher::publish_header(olei_interfaces::msg::VFHeader &header)
{
  header_publisher_->publish(header);
}

void LidarPublisher::publish_scan(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scan_publisher_->publish(*msg);
}
