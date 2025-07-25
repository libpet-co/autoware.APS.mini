#pragma once

#include "olei/olei_packet/olei_packet.h"


class OleiVFPacket : public OleiPacket
{
public:
  OleiVFPacket()
  {
    header_size = sizeof(olei_interfaces::msg::VFHeader);
  }

  virtual size_t get_size();
  olei_interfaces::msg::VFHeader header;
  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(boost::shared_array<uint8_t> buffer);
};
