#pragma once

#include <boost/smart_ptr.hpp>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>

#include "olei_interfaces/msg/vf_header.hpp"
#include "olei_interfaces/msg/v2_header.hpp"
class OleiPacketReader;

class OleiPacket
{
public:
  rclcpp::Time last_acquired_point_stamp;
  std::vector<uint16_t> angle; 
  std::vector<uint16_t> distance; 
  std::vector<uint16_t> intensity;
  rclcpp::Serialization<olei_interfaces::msg::VFHeader> serialization;
  virtual void read_with(OleiPacketReader& reader)
  {
  }

  int find_packet_start(uint8_t* buf, size_t buf_len);
  bool parse_buf(uint8_t* buf, size_t buf_len, size_t& remainder, size_t& packet_size);

  virtual ~OleiPacket() = default;

protected:
  size_t header_size;
  virtual size_t get_size() = 0;
  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(boost::shared_array<uint8_t> buffer) = 0;
  virtual void read_data(uint8_t* buf, size_t num) = 0;
};
