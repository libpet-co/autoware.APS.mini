#pragma once
#include "olei/olei_packet/olei_packet.h"
#include "olei/olei_packet/olei_vf_packet_a.h"
#include "olei/olei_packet/olei_vf_packet_b.h"
#include "olei/olei_packet/olei_vf_packet_c.h"
#include "olei/olei_packet/olei_version2_a.h"
#include "parser.h"

template <typename T>
class OleiParser : public Parser<OleiPacket>
{
public:
  virtual bool parse(uint8_t* buf, size_t buf_len, std::vector<std::unique_ptr<OleiPacket>>& results, size_t& used,
                     rclcpp::Logger logger) override
  {
    std::unique_ptr<T> packet = std::make_unique<T>();
    uint32_t serial_size = packet->get_size();
    uint8_t* orig_buf = buf;
    int count = 0;
    used = 0;
    while (buf_len >= serial_size)
    {
      int start = packet->find_packet_start(buf, buf_len);
      if (start == -1)
      {
        RCLCPP_DEBUG(logger, "No magic number found. Invalid packet.");
        break;
      }
      buf += start;
      buf_len -= start;
      if (buf_len < serial_size)
        break;
      size_t remainder = 0;
      size_t p_size = 0;
      if (!packet->parse_buf(buf, buf_len, remainder, p_size))
        break;
      packet->last_acquired_point_stamp = rclcpp::Clock().now();
      results.push_back(std::move(packet));
      ++count;
      buf += p_size;
      buf_len -= p_size;
      used = buf - orig_buf;
      packet = std::make_unique<T>();
    }

    if (count == 0)
      RCLCPP_DEBUG(logger, "Received data smaller than header size");

    return count > 0;
  }
};

using OleiVFPacket_A_Parser = OleiParser<OleiVFPacket_A>;
using OleiVFPacket_B_Parser = OleiParser<OleiVFPacket_B>;
using OleiVFPacket_C_Parser = OleiParser<OleiVFPacket_C>;
using Olei2Packet_A_Parser = OleiParser<Olei2Packet_A>;
