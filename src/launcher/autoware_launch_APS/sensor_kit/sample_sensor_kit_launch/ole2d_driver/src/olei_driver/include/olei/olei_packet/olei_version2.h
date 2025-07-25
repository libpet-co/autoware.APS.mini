#pragma once

#include "olei/olei_packet/olei_packet.h"


class Olei2Packet : public OleiPacket
{
public:
  Olei2Packet()
  {
    header_size = sizeof(olei_interfaces::msg::V2Header);
  }

  virtual size_t get_size();
  olei_interfaces::msg::V2Header header;
  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(boost::shared_array<uint8_t> buffer);
};