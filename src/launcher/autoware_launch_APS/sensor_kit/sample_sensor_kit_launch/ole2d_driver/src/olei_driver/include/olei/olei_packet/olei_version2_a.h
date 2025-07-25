#pragma once

#include "olei/olei_packet/olei_version2.h"

class Olei2Packet_A : public Olei2Packet
{
protected:
#pragma pack(push, pfA, 1)
  struct Data
  {
    uint16_t angle;
    uint16_t distance;
    uint16_t intensity;
    uint16_t saved;
  };
#pragma pack(pop, pfA)

  virtual void read_data(uint8_t* buf, size_t num);
  virtual void read_with(OleiPacketReader& reader);
};